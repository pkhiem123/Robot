#include <PS2X_lib.h>
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <PCF8574.h>

//KHAI BÁO PCF8574
PCF8574 pcf(0x27);

//KHAI BÁO CHUNG
PS2X ps2x;
Servo myservo;
enum Mode { MANUAL,
            AUTONOMOUS };
Mode currentMode = MANUAL;

//CẤU HÌNH PS2
#define PS2_DAT A2
#define PS2_CMD A1
#define PS2_ATT A3
#define PS2_CLK A0
int error = 0;
byte type = 0;
byte vibrate = 0;

// --- SIÊU ÂM ---
#define TRIG_PIN 11
#define ECHO_PIN 12
#define MAX_DISTANCE 50
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
int distance = 100;

// --- SERVO ---
int servo_pin = 10;
int pos = 90;

//MOTOR BÁNH XE
#define ENA_WHEEL_PWM 3
#define ENB_WHEEL_PWM 5
#define PCF_IN1_L 0
#define PCF_IN2_L 1
#define PCF_IN3_R 2
#define PCF_IN4_R 3

#define MAX_SPEED 160
#define FORWARD_SPEED MAX_SPEED
#define TURN_SPEED 120

//MOTOR TAY QUAY
#define ENB_ARM_PWM 6
#define PCF_IN3_ARM 4
#define PCF_IN4_ARM 5
#define ARM_SPEED 255

// --- LOGIC TỰ HÀNH ---
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

//HÀM ĐIỀU KHIỂN MOTOR 

// Hàm điều khiển bánh xe qua PCF8574 và PWM
void setMotorWheelSpeed(int spdL, int spdR) {
  analogWrite(ENA_WHEEL_PWM, abs(spdL));
  analogWrite(ENB_WHEEL_PWM, abs(spdR));

  // Điều khiển hướng (Logic - PCF8574 Pin)
  // Bánh Trái
  if (spdL > 0) {
    pcf.write(PCF_IN1_L, HIGH);
    pcf.write(PCF_IN2_L, LOW);
  } else if (spdL < 0) {
    pcf.write(PCF_IN1_L, LOW);
    pcf.write(PCF_IN2_L, HIGH);
  } else {
    pcf.write(PCF_IN1_L, LOW);
    pcf.write(PCF_IN2_L, LOW);
  }

  // Bánh Phải
  if (spdR > 0) {
    pcf.write(PCF_IN3_R, HIGH);
    pcf.write(PCF_IN4_R, LOW);
  } else if (spdR < 0) {
    pcf.write(PCF_IN3_R, LOW);
    pcf.write(PCF_IN4_R, HIGH);
  } else {
    pcf.write(PCF_IN3_R, LOW);
    pcf.write(PCF_IN4_R, LOW);
  }
}

void moveStop() {
  setMotorWheelSpeed(0, 0);
}
void moveForward() {
  setMotorWheelSpeed(FORWARD_SPEED, FORWARD_SPEED);
}
void moveBackward() {
  setMotorWheelSpeed(-FORWARD_SPEED, -FORWARD_SPEED);
}
void turnRight() {
  setMotorWheelSpeed(TURN_SPEED, -TURN_SPEED);
}
void turnLeft() {
  setMotorWheelSpeed(-TURN_SPEED, TURN_SPEED);
}

//HÀM ĐIỀU KHIỂN TAY QUAY

//Quay thuận
void armSpinForward() {
  analogWrite(ENB_ARM_PWM, ARM_SPEED);
  pcf.write(PCF_IN3_ARM, HIGH);
  pcf.write(PCF_IN4_ARM, LOW);
}

//Quay ngược
void armSpinBackward() {
  analogWrite(ENB_ARM_PWM, ARM_SPEED);
  pcf.write(PCF_IN3_ARM, LOW);
  pcf.write(PCF_IN4_ARM, HIGH);
}

// Dừng tay quay
void armStop() {
  analogWrite(ENB_ARM_PWM, 0);
  pcf.write(PCF_IN3_ARM, LOW);
  pcf.write(PCF_IN4_ARM, LOW);
}
//LOGIC TỰ HÀNH 
void autonomous_logic() {
  armSpinForward();

  unsigned long currentMillis = millis();

  switch (currentAutoState) {
    case AUTO_FORWARD:
      moveForward();
      distance = sonar.ping_cm();
      Serial.print("Khoảng cách: ");
      Serial.println(distance);
      if (distance > 0 && distance <= 25) {
        moveStop();
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_REVERSE;
      }
      break;

    case AUTO_REVERSE:
      moveBackward();
      if (currentMillis - autoStateMillis > 800) {  // Lùi lâu hơn chút
        moveStop();
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_LOOK_LEFT;
      }
      break;

    case AUTO_LOOK_LEFT:
      myservo.write(170);
      if (currentMillis - autoStateMillis > 600) {
        distLeft = sonar.ping_cm();
        autoStateMillis = currentMillis;
        currentAutoState = AUTO_LOOK_RIGHT;
      }
      break;

    case AUTO_LOOK_RIGHT:
      myservo.write(10);
      if (currentMillis - autoStateMillis > 600) {  // Tăng delay để servo kịp quay
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
      if (currentMillis - autoStateMillis > 500) {
        moveStop();
        currentAutoState = AUTO_FORWARD;
      }
      break;

    case AUTO_TURN_RIGHT:
      turnRight();
      if (currentMillis - autoStateMillis > 500) {
        moveStop();
        currentAutoState = AUTO_FORWARD;
      }
      break;
  }
}

//LOGIC MANUAL
void manual_logic() {
  // Di chuyển
  if (ps2x.Button(PSB_PAD_UP)) {
    Serial.println("Hướng lên");
    moveForward() ;
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.println("Hướng xuống");
    moveBackward();
  } else if (ps2x.Button(PSB_PAD_LEFT)) {
    Serial.println("Rẽ trái");
    turnLeft();
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.println("Rẽ phải");
    turnRight();
  } else {
    moveStop();
  }

  // Điều khiển tay quay
  if (ps2x.Button(PSB_R2)) {
    armSpinForward();  // Quay thuận
  } else if (ps2x.Button(PSB_L2)) {
    armSpinBackward();  
  } else {
    armStop();
  }
}

//SETUP & LOOP 
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // 1. Cài đặt PCF8574
  Serial.print("Đang khởi tạo PCF8574");
  if (!pcf.begin()) {
    Serial.println("Không thể khởi tạo...");
  }
  if (!pcf.isConnected()) {
    Serial.println("=> Không kết nối");
  } else {
    Serial.println("=> ổn");
  }
  for (int i = 0; i < 8; i++) {
    pcf.write(i, LOW);
  }

  // 2. Cài đặt PS2
  // Thử kết nối PS2 nhiều lần
  delay(300);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if (error == 0) {
    Serial.println("Tìm thấy tín hiệu, kết nối thành công");
  } else {
    Serial.println("Không tìm thấy tín hiệu");
  }

  // 3. Cài đặt PWM Pins (L298N)
  pinMode(ENA_WHEEL_PWM, OUTPUT);
  pinMode(ENB_WHEEL_PWM, OUTPUT);
  pinMode(ENB_ARM_PWM, OUTPUT);

  // 4. Cài đặt Servo
  myservo.attach(servo_pin);
  myservo.write(pos);

  Serial.println("Sẵn sàng");
}

void loop() {
  ps2x.read_gamepad(false, vibrate);

  // CHUYỂN CHẾ ĐỘ (Nút SELECT)
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    if (currentMode == MANUAL) {
      currentMode = AUTONOMOUS;
      currentAutoState = AUTO_FORWARD;
      myservo.write(90);
      moveStop();
      Serial.println("MODE: TỰ ĐỘNG");
    } else {
      currentMode = MANUAL;
      moveStop();
      armStop();
      Serial.println("MODE: THỦ CÔNG");
    }
  }

  if (currentMode == MANUAL) {
    manual_logic();
  } else {
    autonomous_logic();
  }

  delay(10);
}