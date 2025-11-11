#include <PS2X_lib.h>
#include <Servo.h>
#include <NewPing.h>

// --- 1. KHAI BÁO CHUNG ---
PS2X ps2x;
Servo myservo;
enum Mode { MANUAL, AUTONOMOUS };
Mode currentMode = MANUAL;

// --- 2. KHAI BÁO PIN (Theo code L298N) ---
// PS2
int error = 0;
byte type = 0;
byte vibrate = 0;

// Laser/Còi
const int pin_laze = 7;
bool is_fire = false;
bool is_buzz = false;
unsigned long motorPreviousMillis = 0;
unsigned long motorInterval = 1000; 

// Cảm biến (NewPing)
#define TRIG_PIN 11 
#define ECHO_PIN 12 
#define MAX_DISTANCE 150 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
int distance = 100;

// L298N
//Hai bánh phải
#define ENA 3 
#define IN1 4 
#define IN2 5
//Hai bánh trái
#define ENB 2 
#define IN3 6 
#define IN4 7 
#define MAX_SPEED 140
#define FORWARD_SPEED MAX_SPEED
#define TURN_SPEED 100
#define MAX_SPEED_OFFSET 20 

// Servo
int servo_pin = 10; 
int pos = 90; // Vị trí servo (90 là ở giữa)

// --- 3. LOGIC TỰ HÀNH (STATE MACHINE) ---
enum AutoState { 
  AUTO_FORWARD, 
  AUTO_STOP_1,
  AUTO_REVERSE, 
  AUTO_STOP_2,
  AUTO_LOOK_LEFT, 
  AUTO_LOOK_RIGHT,
  AUTO_CENTER_SERVO,
  AUTO_TURN_LEFT, 
  AUTO_TURN_RIGHT 
};
AutoState currentAutoState = AUTO_FORWARD; 
unsigned long autoStateMillis = 0; 
int distLeft = 0;
int distRight = 0;

// ===========================================
// HÀM DI CHUYỂN (L298N)
// ===========================================

void setMotorSpeed(int spdL, int spdR) {
  analogWrite(ENA, abs(spdL));
  analogWrite(ENB, abs(spdR));
  
  digitalWrite(IN1, (spdL > 0) ? HIGH : LOW);
  digitalWrite(IN2, (spdL > 0) ? LOW : HIGH);
  digitalWrite(IN3, (spdR > 0) ? HIGH : LOW);
  digitalWrite(IN4, (spdR > 0) ? LOW : HIGH);
}
void moveStop()   { setMotorSpeed(0, 0); }
void moveForward(){ setMotorSpeed(FORWARD_SPEED, FORWARD_SPEED); }
void moveBackward(){ setMotorSpeed(-FORWARD_SPEED, -FORWARD_SPEED); }
void turnRight()  { setMotorSpeed(TURN_SPEED, -TURN_SPEED); } // Xoay phải tại chỗ
void turnLeft()   { setMotorSpeed(-TURN_SPEED, TURN_SPEED); } // Xoay trái tại chỗ

// ===========================================
// HÀM PHỤ TRỢ (LAZE/RUNG)
// ===========================================
void fire_and_buzz_logic() {
  unsigned long currentMillis = millis();
  if (is_fire || is_buzz) {
    digitalWrite(pin_laze, HIGH); 
    vibrate = 1; 
    if (currentMillis - motorPreviousMillis >= motorInterval) {
      is_fire = false;
      is_buzz = false;
    }
  } else {
    digitalWrite(pin_laze, LOW);
    vibrate = 0;
  }
}

// ===========================================
//       LOGIC CÁC CHẾ ĐỘ
// ===========================================

// --- LOGIC CHẾ ĐỘ TỰ HÀNH (KHÔNG THAY ĐỔI) ---
void autonomous_logic() {
  unsigned long currentMillis = millis();
  
  switch(currentAutoState) {
    
    case AUTO_FORWARD:
      // 1. Đi thẳng và liên tục kiểm tra
      moveForward();
      distance = sonar.ping_cm();
      if (distance > 0 && distance <= 20) { // Nếu thấy vật cản
        moveStop();
        autoStateMillis = currentMillis; // Đặt mốc thời gian
        currentAutoState = AUTO_REVERSE; // Chuyển sang bước Lùi
      }
      break;
      
    case AUTO_REVERSE:
      // 2. Lùi trong 300ms
      moveBackward();
      if (currentMillis - autoStateMillis > 300) {
        moveStop();
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_LOOK_LEFT; // Chuyển sang bước Nhìn Trái
      }
      break;
      
    case AUTO_LOOK_LEFT:
      // 3. Quay servo sang trái (170 độ) và chờ 0.5s
      myservo.write(170); 
      if (currentMillis - autoStateMillis > 500) { // Chờ servo quay
        distLeft = sonar.ping_cm(); // Đo khoảng cách bên trái
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_LOOK_RIGHT; // Chuyển sang bước Nhìn Phải
      }
      break;
      
    case AUTO_LOOK_RIGHT:
      // 4. Quay servo sang phải (10 độ) và chờ 1s (quay từ 170->10)
      myservo.write(10);
      if (currentMillis - autoStateMillis > 1000) { // Chờ servo quay
        distRight = sonar.ping_cm(); // Đo khoảng cách bên phải
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_CENTER_SERVO; // Chuyển sang bước Quyết định
      }
      break;
      
    case AUTO_CENTER_SERVO:
      // 5. Quay servo về giữa
      myservo.write(90); 
      if (distLeft > distRight) {
        currentAutoState = AUTO_TURN_LEFT; // Quyết định Rẽ Trái
      } else {
        currentAutoState = AUTO_TURN_RIGHT; // Quyết định Rẽ Phải
      }
      autoStateMillis = currentMillis;
      break;
      
    case AUTO_TURN_LEFT:
      // 6. Rẽ trái trong 400ms
      turnLeft();
      if (currentMillis - autoStateMillis > 400) {
        moveStop();
        currentAutoState = AUTO_FORWARD; // Quay lại bước Đi thẳng
      }
      break;
      
    case AUTO_TURN_RIGHT:
      // 7. Rẽ phải trong 400ms
      turnRight();
      if (currentMillis - autoStateMillis > 400) {
        moveStop();
        currentAutoState = AUTO_FORWARD; // Quay lại bước Đi thẳng
      }
      break;
  }
}

// --- LOGIC CHẾ ĐỘ BẰNG TAY CẦM PS2 ---
void manual_logic() {
  // 1. Lái xe
  int leftY = ps2x.Analog(PSS_LY); // 0 (lên) -> 128 (giữa) -> 255 (xuống)
  int leftX = ps2x.Analog(PSS_LX); // 0 (trái) -> 128 (giữa) -> 255 (phải)

  if (leftX < 100) { // Rẽ trái
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  } 
  else if (leftX > 150) { // Rẽ phải
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  }
  else if (abs(leftY - 128) > MAX_SPEED_OFFSET) {
    int spd = map(leftY, 0, 255, FORWARD_SPEED, -FORWARD_SPEED);
    setMotorSpeed(spd, spd);
  }
  else {
    moveStop();
  }
}

// ===========================================
//       SETUP VÀ LOOP
// ===========================================

void setup() {
  Serial.begin(115200); 
  
  // 1. Cài đặt PS2
  pinMode(13, OUTPUT);
  pinMode(pin_laze, OUTPUT);
  do {
    digitalWrite(13, !digitalRead(13));
    delay(500); 
    error = ps2x.config_gamepad(A2, A1, A3, A0, false, false);
  } while(error != 0);
  Serial.println("PS2 Connected.");
  digitalWrite(13, LOW);
  
  // 2. Cài đặt L298N
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  // 3. Cài đặt Servo
  myservo.attach(servo_pin); 
  myservo.write(pos);
  
  Serial.println("Robot Ready!");
}

void loop(){
  // --- 1. ĐỌC TAY CẦM ---
  ps2x.read_gamepad(false, vibrate); 
  
  // --- 2. XỬ LÝ LAZE/RUNG ---
  fire_and_buzz_logic(); 
  if (ps2x.ButtonPressed(PSB_CROSS) && !is_fire) {
    is_fire = true;
    is_buzz = true;
    motorPreviousMillis = millis(); 
  }

  // --- 3. CHUYỂN CHẾ ĐỘ (Nút SELECT) ---
  if(ps2x.ButtonPressed(PSB_SELECT)) {
    if (currentMode == MANUAL) {
      currentMode = AUTONOMOUS;
      currentAutoState = AUTO_FORWARD; // Đặt lại trạng thái tự hành
      myservo.write(90); // Hướng servo về phía trước
      Serial.println("MODE: AUTONOMOUS");
    } else {
      currentMode = MANUAL;
      moveStop(); // Dừng xe khi chuyển về manual
      Serial.println("MODE: MANUAL");
    }
  }
  
  // --- 4. THỰC THI CHẾ ĐỘ ---
  if (currentMode == MANUAL) {
    manual_logic();
  } else {
    autonomous_logic();
  }

  // --- 5. DELAY NHỎ ---
  delay(15);
}