#include <PS2X_lib.h>
#include <Servo.h>
#include <NewPing.h>
//KHAI BÁO CHUNG & CẤU HÌNH
PS2X ps2x;
Servo myservo;
enum Mode { MANUAL, AUTONOMOUS };
Mode currentMode = MANUAL;
bool isArmEnabled = true; // Luôn cho phép điều khiển tay quay trong chế độ MANUAL

//KHAI BÁO PIN VÀ CẤU HÌNH CÁC MODULE
// PS2
const int PS2_DAT = A2; 
const int PS2_CMD = A1; 
const int PS2_ATT = A3; 
const int PS2_CLK = A0; 
int error = 0;
byte type = 0;
byte vibrate = 0;

// Laser/Còi (Giữ nguyên)
const int pin_laze = 7;
bool is_fire = false;
bool is_buzz = false;
unsigned long motorPreviousMillis = 0;
unsigned long motorInterval = 1000; 

// Cảm biến Siêu âm (NewPing) (Giữ nguyên)
#define TRIG_PIN 11 
#define ECHO_PIN 12 
#define MAX_DISTANCE 150 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
int distance = 100;

// Servo (Giữ nguyên)
int servo_pin = 10; 
int pos = 90; 

// L298N 1: MOTOR BÁNH XE (M1)
//Bánh trái
#define ENA_WHEEL 3   // PWM
#define IN1_WHEEL 9
#define IN2_WHEEL 8
//Bánh phải
#define ENB_WHEEL 5   // PWM
#define IN3_WHEEL 4
#define IN4_WHEEL 2
#define MAX_SPEED 140
#define FORWARD_SPEED MAX_SPEED
#define TURN_SPEED 100

// L298N 2: MOTOR TAY QUAY
#define ENA_ARM 6     
#define IN5_ARM1_FORW 13 
#define IN6_ARM1_BACK A4  
#define ENB_ARM A5    
#define IN7_ARM2_FORW 0   
#define IN8_ARM2_BACK 1 
#define ARM_SPEED 255 

//LOGIC TỰ HÀNH (STATE MACHINE)
enum AutoState { 
  AUTO_FORWARD, 
  AUTO_REVERSE, 
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

// HÀM DI CHUYỂN BÁNH XE (L298N 1)
void setMotorWheelSpeed(int spdL, int spdR) {
  // Đặt tốc độ
  analogWrite(ENA_WHEEL, abs(spdL));
  analogWrite(ENB_WHEEL, abs(spdR));
    
  // Đặt hướng (Trái)
  digitalWrite(IN1_WHEEL, (spdL > 0) ? HIGH : LOW);
  digitalWrite(IN2_WHEEL, (spdL > 0) ? LOW : HIGH);
  
  // Đặt hướng (Phải)
  digitalWrite(IN3_WHEEL, (spdR > 0) ? HIGH : LOW);
  digitalWrite(IN4_WHEEL, (spdR > 0) ? LOW : HIGH);
}
void moveStop()  { setMotorWheelSpeed(0, 0); }
void moveForward(){ setMotorWheelSpeed(FORWARD_SPEED, FORWARD_SPEED); }
void moveBackward(){ setMotorWheelSpeed(-FORWARD_SPEED, -FORWARD_SPEED); }
void turnRight() { setMotorWheelSpeed(TURN_SPEED, -TURN_SPEED); } 
void turnLeft(){ setMotorWheelSpeed(-TURN_SPEED, TURN_SPEED); } 

// HÀM ĐIỀU KHIỂN TAY QUAY

// --- Cả hai Motor cùng quay thuận chiều (R2) ---
void armBothForward() {
  // Motor 1 (Trục 1) Forward
  digitalWrite(ENA_ARM, HIGH);
  digitalWrite(IN5_ARM1_FORW, HIGH);
  digitalWrite(IN6_ARM1_BACK, LOW);
  
  // Motor 2 (Trục 2) Forward (Cùng chiều với M1)
  digitalWrite(ENB_ARM, HIGH);
  digitalWrite(IN7_ARM2_FORW, HIGH);
  digitalWrite(IN8_ARM2_BACK, LOW);
}

// --- Cả hai Motor cùng quay nghịch chiều (L2) ---
void armBothBackward() {
  // Motor 1 (Trục 1) Backward
  digitalWrite(ENA_ARM, HIGH);
  digitalWrite(IN5_ARM1_FORW, LOW);
  digitalWrite(IN6_ARM1_BACK, HIGH);
  
  // Motor 2 (Trục 2) Backward (Cùng chiều với M1)
  digitalWrite(ENB_ARM, HIGH);
  digitalWrite(IN7_ARM2_FORW, LOW);
  digitalWrite(IN8_ARM2_BACK, HIGH);
}

// Hàm dừng Motor 1
void arm1Stop() {
  digitalWrite(ENA_ARM, LOW); // Tắt motor 1
  digitalWrite(IN5_ARM1_FORW, LOW);
  digitalWrite(IN6_ARM1_BACK, LOW);
}

// Hàm dừng Motor 2
void arm2Stop() {
  digitalWrite(ENB_ARM, LOW); // Tắt motor 2
  digitalWrite(IN7_ARM2_FORW, LOW);
  digitalWrite(IN8_ARM2_BACK, LOW);
}

// Hàm dừng tất cả các motor tay quay
void armStopAll() {
  arm1Stop();
  arm2Stop();
}

// HÀM PHỤ TRỢ (LAZE/RUNG)
void fire_and_buzz_logic() {
  unsigned long currentMillis = millis();
  if (is_fire || is_buzz) {
    digitalWrite(pin_laze, HIGH); 
    vibrate = 1; 
    if (currentMillis - motorPreviousMillis >= motorInterval) {
      is_fire = false;
      is_buzz = false;
      vibrate = 0;
    }
  } else {
    digitalWrite(pin_laze, LOW);
    vibrate = 0;
  }
}
//LOGIC CÁC CHẾ ĐỘ
//LOGIC CHẾ ĐỘ TỰ HÀNH
void autonomous_logic() {
  armStopAll();

  unsigned long currentMillis = millis();
    
  switch(currentAutoState) {
    // Logic tự hành giữ nguyên (kiểm tra vật cản, lùi, quét, rẽ)
    case AUTO_FORWARD:
      moveForward();
      distance = sonar.ping_cm();
      if (distance > 0 && distance <= 20) { 
        moveStop();
        autoStateMillis = currentMillis; 
        currentAutoState = AUTO_REVERSE; 
      }
      break;
        
    case AUTO_REVERSE:
      moveBackward();
      if (currentMillis - autoStateMillis > 300) {
        moveStop();
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_LOOK_LEFT; 
      }
      break;
        
    case AUTO_LOOK_LEFT:
      myservo.write(170); 
      if (currentMillis - autoStateMillis > 500) { 
        distLeft = sonar.ping_cm(); 
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_LOOK_RIGHT; 
      }
      break;
        
    case AUTO_LOOK_RIGHT:
      myservo.write(10);
      if (currentMillis - autoStateMillis > 1000) { 
        distRight = sonar.ping_cm(); 
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_CENTER_SERVO; 
      }
      break;
    
    case AUTO_CENTER_SERVO:
      myservo.write(90); 
      if (distLeft > distRight) {
        currentAutoState = AUTO_TURN_LEFT; 
      } else {
        currentAutoState = AUTO_TURN_RIGHT; 
      }
      autoStateMillis = currentMillis;
      break;
        
    case AUTO_TURN_LEFT:
      turnLeft();
      if (currentMillis - autoStateMillis > 400) {
        moveStop();
        currentAutoState = AUTO_FORWARD; 
      }
      break;
        
    case AUTO_TURN_RIGHT:
      turnRight();
      if (currentMillis - autoStateMillis > 400) {
        moveStop();
        currentAutoState = AUTO_FORWARD; 
      }
      break;
  }
}

//LOGIC CHẾ ĐỘ BẰNG TAY (MANUAL)
void manual_logic() {
  if (ps2x.Button(PSB_PAD_UP)) {
    Serial.println("Đi thẳng");
    moveForward();
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.println("Lùi");
    moveBackward();
  }
  else if (ps2x.Button(PSB_PAD_LEFT)) {
    Serial.println("Rẽ trái");
    turnLeft();
  }
  else if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.println("Rẽ phải");
    turnRight();
  }
  else {
    moveStop();
  }

  //ĐIỀU KHIỂN MOTOR TAY QUAY
  if (isArmEnabled) {
    
    if (ps2x.Button(PSB_R2)) {
      Serial.println("Quay thuận chiều");
      armBothForward();
    }
    else if (ps2x.Button(PSB_L2)) {
      Serial.println("Quay ngược chiều");
      armBothBackward();
    }
    else {
      armStopAll(); 
    }
  } else {
    armStopAll();
  }
}
//        SETUP VÀ LOOP
void setup() {
  Serial.begin(115200); 
    
  // 1. Cài đặt PS2
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_laze, OUTPUT);
  do {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500); 
    error = ps2x.config_gamepad(PS2_DAT, PS2_CMD, PS2_ATT, PS2_CLK, false, false);
  } while(error != 0);
  Serial.println("PS2 Connected.");
  digitalWrite(LED_BUILTIN, LOW);
    
  // 2. Cài đặt L298N 1 (BÁNH XE)
  pinMode(ENA_WHEEL, OUTPUT); pinMode(IN1_WHEEL, OUTPUT); pinMode(IN2_WHEEL, OUTPUT);
  pinMode(ENB_WHEEL, OUTPUT); pinMode(IN3_WHEEL, OUTPUT); pinMode(IN4_WHEEL, OUTPUT);

  // 3. Cài đặt L298N 2 (TAY QUAY)
  pinMode(ENA_ARM, OUTPUT); pinMode(IN5_ARM1_FORW, OUTPUT); pinMode(IN6_ARM1_BACK, OUTPUT);
  pinMode(ENB_ARM, OUTPUT); pinMode(IN7_ARM2_FORW, OUTPUT); pinMode(IN8_ARM2_BACK, OUTPUT);
  // Khai báo các chân Analog là Output Digital
  pinMode(A4, OUTPUT); 
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  // 4. Cài đặt Servo
  myservo.attach(servo_pin); 
  myservo.write(pos);
    
  Serial.println("Robot Ready!");
}

void loop(){
  ps2x.read_gamepad(false, vibrate); 
    
  fire_and_buzz_logic(); 
  if (ps2x.ButtonPressed(PSB_CROSS) && !is_fire) {
    is_fire = true;
    is_buzz = true;
    motorPreviousMillis = millis(); 
    Serial.println("FIRE!");
  }

  //CHUYỂN CHẾ ĐỘ (Nút SELECT)
  if(ps2x.ButtonPressed(PSB_SELECT)) {
    if (currentMode == MANUAL) {
      currentMode = AUTONOMOUS;
      currentAutoState = AUTO_FORWARD; 
      myservo.write(90); 
      moveStop(); 
      armStopAll();
      Serial.println("MODE: AUTONOMOUS");
    } else {
      currentMode = MANUAL;
      moveStop(); 
      armStopAll();
      Serial.println("MODE: MANUAL");
    }
  }
  if (currentMode == MANUAL) {
    manual_logic();
  } else {
    autonomous_logic();
  }
  delay(15);
}