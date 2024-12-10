#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

// ตั้งค่าพินเอ็นโค้ดเดอร์
#define ENCODER1_PIN_A 26
#define ENCODER1_PIN_B 27

#define ENCODER2_PIN_A 28
#define ENCODER2_PIN_B 29


// ตั้งค่าพินมอเตอร์เอ็นโค้ตเดอร์
#define motor_EN1_PWM 34
#define motor_EN1_DirA A0
#define motor_EN1_DirB A1

#define motor_EN2_PWM 36
#define motor_EN2_DirA A3
#define motor_EN2_DirB A4

// ตั้งค่าพินมอเตอร์เบรก
#define motor_BRAKE1_PWM 38
#define motor_BRAKE1_DirA A4
#define motor_BRAKE1_DirB A5

#define motor_BRAKE2_PWM 440
#define motor_BRAKE2_DirA A6
#define motor_BRAKE2_DirB A7

// ตั้งค่าพินมอเตอร์ปั๊ม
#define motor_PUMP_PinA 46    // กำหนดขา A   46
#define motor_PUMP_PinB 47    // กำหนดขา B   47
#define motor_PUMP_pwmPin 42  // กำหนดขา PWM  42

// ตั้งค่าพินเสต็ปมอเตอร์
#define STEP_dirPin A11   //Steping
#define STEP_stepPin A10  //Steping


// ตั้งค่าพินพร็อกซิมิตี้
#define PROXIMITY1_PIN 6
#define PROXIMITY2_PIN 7

//ตั้งค่าพินอัลตร้าโซนิค
#define Ultrasonic1_TRIG_PIN 4  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define Ultrasonic1_ECHO_PIN 8  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04

#define Ultrasonic2_TRIG_PIN 5  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define Ultrasonic2_ECHO_PIN 9  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04

//ตั้งค่าพินรีเลย์
#define relayPin 32

// //ตั้งค่าพินลิมิตสวิตช์
#define Limit1 50
#define Limit2 49
#define Limit3 1  //น่าจะต้องเปลี่ยน
#define Limit4 0  //น่าจะต้องเปลี่ยน
#define Limit5 10
#define Limit6 11
#define Limit7 12
#define Limit8 13



Servo myservo;
Encoder myEncoder1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder myEncoder2(ENCODER2_PIN_A, ENCODER2_PIN_B);

String servo, PROXIMITY1, PROXIMITY2, Relay, clockwise, counterclockwise, Pump_clockwise, Pump_counterclockwise, mortor_clockwise;
String mortor_EN1_clockwise, mortor_EN1_counterclockwise, mortor_EN2_clockwise, mortor_EN2_counterclockwise;
String mortor_Brake1_clockwise, mortor_Brake2_clockwise, mortor_Brake1_counterclockwise, mortor_Brake2_counterclockwise;
long oldPosition = -999;
int motorStatus = 0;  // สถานะมอเตอร์ (เปิดหรือปิด)
int threshold = 500;
bool proximityActive = false;

long duration, distance1, distance2;  // ประกาศตัวแปรเก็บค่าระยะ
unsigned long previousMillis = 0;
const long interval = 500;  // ระยะเวลาระหว่างการวัด (500 ms)
int stepsPerSecond = 1000;  // กำหนดจำนวนสเต็ปต่อวินาที
bool isMoving = false;      // ตัวแปรเพื่อตรวจสอบสถานะการหมุนของมอเตอร์
int motor1Status = 0;
int motor2Status = 0;

void setup() {
  pinMode(motor_EN1_PWM, OUTPUT);
  pinMode(motor_EN1_DirA, OUTPUT);
  pinMode(motor_EN1_DirB, OUTPUT);

  pinMode(motor_EN2_PWM, OUTPUT);
  pinMode(motor_EN2_DirA, OUTPUT);
  pinMode(motor_EN2_DirB, OUTPUT);

  pinMode(relayPin, OUTPUT);

  pinMode(PROXIMITY1_PIN, INPUT);
  pinMode(PROXIMITY2_PIN, INPUT);

  pinMode(Ultrasonic1_TRIG_PIN, OUTPUT);
  pinMode(Ultrasonic1_ECHO_PIN, INPUT);

  pinMode(Ultrasonic2_TRIG_PIN, OUTPUT);
  pinMode(Ultrasonic2_ECHO_PIN, INPUT);

  pinMode(motor_PUMP_PinA, OUTPUT);
  pinMode(motor_PUMP_PinB, OUTPUT);
  pinMode(motor_PUMP_pwmPin, OUTPUT);

  pinMode(motor_BRAKE1_PWM, OUTPUT);
  pinMode(motor_BRAKE1_DirA, OUTPUT);
  pinMode(motor_BRAKE1_DirB, OUTPUT);

  pinMode(motor_BRAKE2_PWM, OUTPUT);
  pinMode(motor_BRAKE2_DirA, OUTPUT);
  pinMode(motor_BRAKE2_DirB, OUTPUT);

  pinMode(STEP_stepPin, OUTPUT);
  pinMode(STEP_dirPin, OUTPUT);

  pinMode(Limit1, INPUT), pinMode(Limit2, INPUT), pinMode(Limit3, INPUT);
  pinMode(Limit4, INPUT), pinMode(Limit5, INPUT), pinMode(Limit6, INPUT);
  pinMode(Limit7, INPUT), pinMode(Limit8, INPUT);


  Serial.begin(9600);
  Serial.println("Arduino is ready");
}

void loop() {
  // MortorEN1_clockwise();
  // MortorEN1_counterclockwise();
  // Servo_motor();  // เรียกใช้งานฟังก์ชัน Servo_motor
  // Proximity_sensor_1();
  // Proximity_sensor_2();
  // Ultrasonic_1();
  // Ultrasonic_2();
  //Relay_SolenoidValve();
  // Servo_motor();
  //STEP_clockwise();
  //STEP_counterclockwise();
  // PUMP_clockwise();
  // Mortor_Brake1_clockwise();
  // Mortor_Brake2_clockwise();
  // Mortor_Brake1_counterclockwise();
  Mortor_Brake2_counterclockwise();
}

//---------------------------INPUT------------------------------------

void Proximity_sensor_1() {
  if (proximityActive) {
    Proximity_sensor_1();
  }

  if (Serial.available() > 0) {
    PROXIMITY1 = Serial.readStringUntil('\n');  // รับคำสั่งจาก Python
    PROXIMITY1.trim();

    if (PROXIMITY1 == "PROXIMITY ON") {
      proximityActive = true;
      Serial.println("Proximity sensor activated.");
    } else if (PROXIMITY1 == "PROXIMITY OFF") {
      proximityActive = false;
      Serial.println("Proximity sensor deactivated.");
    } else {
      Serial.println("Invalid command.");
    }
  }

  // อ่านค่าจากเซ็นเซอร์ดิจิตอล
  int sensorValue1 = digitalRead(PROXIMITY1_PIN);
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue1);

  // ตรวจสอบสถานะโลหะตามค่า HIGH หรือ LOW
  if (sensorValue1 == HIGH) {
    Serial.println("No Metal Detected!");
  } else {
    Serial.println("Metal Detected");
  }

  delay(200);  // หน่วงเวลาเพื่อความเสถียร
}

void Proximity_sensor_2() {
  if (proximityActive) {
    Proximity_sensor_2();
  }

  if (Serial.available() > 0) {
    PROXIMITY2 = Serial.readStringUntil('\n');  // รับคำสั่งจาก Python
    PROXIMITY1.trim();

    if (PROXIMITY2 == "PROXIMITY ON") {
      proximityActive = true;
      Serial.println("Proximity sensor activated.");
    } else if (PROXIMITY2 == "PROXIMITY OFF") {
      proximityActive = false;
      Serial.println("Proximity sensor deactivated.");
    } else {
      Serial.println("Invalid command.");
    }
  }

  // อ่านค่าจากเซ็นเซอร์ดิจิตอล
  int sensorValue2 = digitalRead(PROXIMITY1_PIN);
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue2);

  // ตรวจสอบสถานะโลหะตามค่า HIGH หรือ LOW
  if (sensorValue2 == HIGH) {
    Serial.println("No Metal Detected!");
  } else {
    Serial.println("Metal Detected");
  }

  delay(200);  // หน่วงเวลาเพื่อความเสถียร
}

void Ultrasonic_1() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    digitalWrite(Ultrasonic1_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(Ultrasonic1_TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(Ultrasonic1_TRIG_PIN, LOW);

    duration = pulseIn(Ultrasonic1_ECHO_PIN, HIGH, 30000);
    if (duration > 0) {
      distance1 = (duration / 2.0) / 29.1;
      Serial.print("Distance1: ");
      Serial.print(distance1);
      Serial.println(" cm");
    } else {
      Serial.println("Error: No signal detected");
    }
  }
}

void Ultrasonic_2() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    digitalWrite(Ultrasonic2_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(Ultrasonic2_TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(Ultrasonic2_TRIG_PIN, LOW);

    duration = pulseIn(Ultrasonic2_ECHO_PIN, HIGH, 30000);
    if (duration > 0) {
      distance2 = (duration / 2.0) / 29.1;
      Serial.print("Distance2: ");
      Serial.print(distance2);
      Serial.println(" cm");
    } else {
      Serial.println("Error: No signal detected");
    }
  }
}

//--------------------------- OUTPUT------------------------------------

void Servo_motor() {
  if (Serial.available() > 0) {            // ตรวจสอบว่ามีข้อมูลเข้ามา
    servo = Serial.readStringUntil('\n');  // อ่านข้อความจนถึง newline
    servo.trim();                          // ตัดช่องว่างและ newline ออก

    if (servo == "ON") {
      myservo.write(180);
      Serial.println("ON");  // ส่งข้อความกลับไปที่ Python

    } else if (servo == "OFF") {
      myservo.write(0);
      Serial.println("OFF");  // ส่งข้อความกลับไปที่ Python
    } else {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }
  }
  delay(100);
}

void Relay_SolenoidValve() {
  if (Serial.available() > 0) {            // ตรวจสอบว่ามีข้อมูลเข้ามา
    Relay = Serial.readStringUntil('\n');  // อ่านข้อความจนถึง newline
    Relay.trim();                          // ตัดช่องว่างและ newline ออก

    if (Relay == "ON") {
      digitalWrite(relayPin, HIGH);
      Serial.println("ON");  // ส่งข้อความกลับไปที่ Python

    } else if (Relay == "OFF") {
      digitalWrite(relayPin, LOW);
      Serial.println("OFF");  // ส่งข้อความกลับไปที่ Python
    } else {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }
  }
  delay(100);
}

void STEP_clockwise() {
  if (Serial.available() > 0) {                       // ตรวจสอบว่ามีข้อมูลเข้ามาจาก Serial
    String clockwise = Serial.readStringUntil('\n');  // อ่านข้อมูลจนถึง newline
    clockwise.trim();                                 // ตัดช่องว่างและ newline ออก

    if (clockwise == "ON" && !isMoving) {         // หากคำสั่งเป็น ON และยังไม่หมุน
      digitalWrite(STEP_dirPin, HIGH);            // HIGH = หมุนไปข้างหน้า
      Serial.println("ON");                       // ส่งข้อความกลับไปยัง Serial
      isMoving = true;                            // ตั้งค่าตัวแปร isMoving เป็น true
    } else if (clockwise == "OFF" && isMoving) {  // หากคำสั่งเป็น OFF และกำลังหมุน
      isMoving = false;                           // ตั้งค่าตัวแปร isMoving เป็น false เพื่อหยุดการหมุน
      Serial.println("OFF");                      // ส่งข้อความกลับไปยัง Serial
    } else if (clockwise != "ON" && clockwise != "OFF") {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }
  }

  // หมุนมอเตอร์หาก isMoving เป็น true
  if (isMoving) {
    for (int i = 0; i < stepsPerSecond; i++) {
      digitalWrite(STEP_stepPin, HIGH);
      delayMicroseconds(1000);  // ควบคุมความเร็วโดยการปรับเวลานี้
      digitalWrite(STEP_stepPin, LOW);
      delayMicroseconds(1000);
    }
  }
  delay(100);
}

void STEP_counterclockwise() {
  if (Serial.available() > 0) {                       // ตรวจสอบว่ามีข้อมูลเข้ามาจาก Serial
    counterclockwise = Serial.readStringUntil('\n');  // อ่านข้อมูลจนถึง newline
    counterclockwise.trim();                          // ตัดช่องว่างและ newline ออก

    if (counterclockwise == "ON" && !isMoving) {         // หากคำสั่งเป็น ON และยังไม่หมุน
      digitalWrite(STEP_dirPin, LOW);                    // HIGH = หมุนไปข้างหน้า
      Serial.println("ON");                              // ส่งข้อความกลับไปยัง Serial
      isMoving = true;                                   // ตั้งค่าตัวแปร isMoving เป็น true
    } else if (counterclockwise == "OFF" && isMoving) {  // หากคำสั่งเป็น OFF และกำลังหมุน
      isMoving = false;                                  // ตั้งค่าตัวแปร isMoving เป็น false เพื่อหยุดการหมุน
      Serial.println("OFF");                             // ส่งข้อความกลับไปยัง Serial
    } else if (counterclockwise != "ON" && counterclockwise != "OFF") {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }
  }

  // หมุนมอเตอร์หาก isMoving เป็น true
  if (isMoving) {
    for (int i = 0; i < stepsPerSecond; i++) {
      digitalWrite(STEP_stepPin, HIGH);
      delayMicroseconds(1000);  // ควบคุมความเร็วโดยการปรับเวลานี้
      digitalWrite(STEP_stepPin, LOW);
      delayMicroseconds(1000);
    }
  }
  delay(100);
}

void PUMP_clockwise() {
  if (Serial.available() > 0) {
    Pump_clockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    Pump_clockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(Pump_clockwise);  // Debug คำสั่งที่เข้ามา

    if (Pump_clockwise == "ON") {
      motor1Status = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (Pump_clockwise == "OFF") {
      motor1Status = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }

    // ควบคุมมอเตอร์ตามสถานะ
    if (motor1Status == 1) {
      digitalWrite(motor_PUMP_PinA, HIGH);  // ขา A HIGH
      digitalWrite(motor_PUMP_PinB, LOW);   // ขา B LOW
      analogWrite(motor_PUMP_pwmPin, 128);  // ความเร็ว PWM
    } else {
      digitalWrite(motor_PUMP_PinA, LOW);  // หยุดมอเตอร์
      digitalWrite(motor_PUMP_PinB, LOW);
      analogWrite(motor_PUMP_pwmPin, 0);
    }
  }
  delay(100);  // เลื่อนการทำงานเล็กน้อย
}

void PUMP_counterclockwise() {
  if (Serial.available() > 0) {                            // ตรวจสอบว่ามีข้อมูลเข้ามาจาก Serial
    Pump_counterclockwise = Serial.readStringUntil('\n');  // อ่านคำสั่งจาก Serial จนถึง newline
    Pump_counterclockwise.trim();                          // ตัดช่องว่างและ newline ออก

    if (Pump_counterclockwise == "ON") {
      motor1Status = 1;  // ตั้งค่าสถานะมอเตอร์ให้เป็น ON
      Serial.println("Motor ON");
    } else if (Pump_counterclockwise == "OFF") {
      motor1Status = 0;  // ตั้งค่าสถานะมอเตอร์ให้เป็น OFF
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }

    if (motor1Status == 1) {
      digitalWrite(motor_PUMP_PinA, LOW);   // ขา A HIGH
      digitalWrite(motor_PUMP_PinB, HIGH);  // ขา B LOW
      analogWrite(motor_PUMP_pwmPin, 128);  // กำหนดความเร็ว PWM (ค่า 0-255)
    } else {
      // ถ้ามอเตอร์ปิด (OFF), ให้หยุดการหมุน
      digitalWrite(motor_PUMP_PinA, LOW);  // ขา A LOW
      digitalWrite(motor_PUMP_PinB, LOW);  // ขา B LOW
      analogWrite(motor_PUMP_pwmPin, 0);   // หยุดการหมุน
    }
  }
  delay(100);  // เลื่อนการทำงานของ loop ให้มีความช้าลงเล็กน้อย
}


void MortorEN1_clockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_EN1_clockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_EN1_clockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_EN1_clockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_EN1_clockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_EN1_clockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_EN1_DirA, HIGH);
    digitalWrite(motor_EN1_DirB, LOW);
    analogWrite(motor_EN1_PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motor_EN1_DirA, LOW);
    digitalWrite(motor_EN1_DirB, LOW);
    analogWrite(motor_EN1_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void MortorEN1_counterclockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_EN1_counterclockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_EN1_counterclockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_EN1_counterclockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_EN1_counterclockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_EN1_counterclockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_EN1_DirA, LOW);
    digitalWrite(motor_EN1_DirB, HIGH);
    analogWrite(motor_EN1_PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motor_EN1_DirA, LOW);
    digitalWrite(motor_EN1_DirB, LOW);
    analogWrite(motor_EN1_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void MortorEN2_clockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_EN2_clockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_EN2_clockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_EN2_clockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_EN2_clockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_EN2_clockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_EN2_DirA, HIGH);
    digitalWrite(motor_EN2_DirB, LOW);
    analogWrite(motor_EN2_PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motor_EN2_DirA, LOW);
    digitalWrite(motor_EN2_DirB, LOW);
    analogWrite(motor_EN2_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void MortorEN2_counterclockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_EN2_counterclockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_EN2_counterclockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_EN2_counterclockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_EN2_counterclockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_EN2_counterclockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_EN2_DirA, LOW);
    digitalWrite(motor_EN2_DirB, HIGH);
    analogWrite(motor_EN2_PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motor_EN2_DirA, LOW);
    digitalWrite(motor_EN2_DirB, LOW);
    analogWrite(motor_EN2_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void Mortor_Brake1_clockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_Brake1_clockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_Brake1_clockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_Brake1_clockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_Brake1_clockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_Brake1_clockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_BRAKE1_DirA, HIGH);
    digitalWrite(motor_BRAKE1_DirB, LOW);
    analogWrite(motor_BRAKE1_PWM, 150);  // กำหนดความเร็ว (0-255)

  } else {
    digitalWrite(motor_BRAKE1_DirA, LOW);
    digitalWrite(motor_BRAKE1_DirB, LOW);
    analogWrite(motor_BRAKE1_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void Mortor_Brake1_counterclockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_Brake1_counterclockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_Brake1_counterclockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_Brake1_counterclockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_Brake1_counterclockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_Brake1_counterclockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_BRAKE1_DirA, LOW);
    digitalWrite(motor_BRAKE1_DirB, HIGH);
    analogWrite(motor_BRAKE1_PWM, 150);  // กำหนดความเร็ว (0-255)

  } else {
    digitalWrite(motor_BRAKE1_DirA, LOW);
    digitalWrite(motor_BRAKE1_DirB, LOW);
    analogWrite(motor_BRAKE1_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}
void Mortor_Brake2_clockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_Brake2_clockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_Brake2_clockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_Brake2_clockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_Brake2_clockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_Brake2_clockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_BRAKE2_DirA, HIGH);
    digitalWrite(motor_BRAKE2_DirB, LOW);
    analogWrite(motor_BRAKE2_PWM, 150);  // กำหนดความเร็ว (0-255)

  } else {
    digitalWrite(motor_BRAKE2_DirA, LOW);
    digitalWrite(motor_BRAKE2_DirB, LOW);
    analogWrite(motor_BRAKE2_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void Mortor_Brake2_counterclockwise() {
  // เช็คคำสั่งจาก Serial Monitor
  if (Serial.available() > 0) {
    mortor_Brake2_counterclockwise = Serial.readStringUntil('\n');  // อ่านคำสั่ง
    mortor_Brake2_counterclockwise.trim();                          // ตัดช่องว่างและ newline

    Serial.print("Received Command: ");
    Serial.println(mortor_Brake2_counterclockwise);  // Debug คำสั่งที่เข้าม

    // กำหนดสถานะมอเตอร์ตามคำสั่ง
    if (mortor_Brake2_counterclockwise == "ON") {
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (mortor_Brake2_counterclockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }
  }

  // ควบคุมมอเตอร์ตามสถานะ
  if (motorStatus == 1) {
    digitalWrite(motor_BRAKE2_DirA, LOW);
    digitalWrite(motor_BRAKE2_DirB, HIGH);
    analogWrite(motor_BRAKE2_PWM, 150);  // กำหนดความเร็ว (0-255)

  } else {
    digitalWrite(motor_BRAKE2_DirA, LOW);
    digitalWrite(motor_BRAKE2_DirB, LOW);
    analogWrite(motor_BRAKE2_PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}
