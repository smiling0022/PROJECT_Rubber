#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

#define PROXIMITY1_PIN 6
#define PROXIMITY2_PIN 7
#define TRIG_PIN1 5  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define ECHO_PIN1 9  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04
#define TRIG_PIN2 4  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define ECHO_PIN2 8  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04


#define relayPin 32

#define mortorD A0  //มอเตอร์เทส
#define mortorP A1

#define dirPin A11  //Steping

#define stepPin A10  //Steping
#define dirPin A11   //Steping

#define motorPinA 46  // กำหนดขา A
#define motorPinB 47  // กำหนดขา B
#define pwmPin 42     // กำหนดขา PWM

// ตั้งค่าพินเอ็นโค้ดเดอร์
#define encoder1PinA 26
#define encoder1PinB 27

#define encoder2PinA 26 //อย่าลืมเปลี่ยนขา
#define encoder2PinB 27 //อย่าลืมเปลี่ยนขา

// ตั้งค่าพินมอเตอร์
#define motorEN1PWM 44
#define motorEN1DirA 43
#define motorEN1DirB 42

#define motorEN2PWM 44  //อย่าลืมเปลี่ยนขา
#define motorEN2DirA 43 //อย่าลืมเปลี่ยนขา
#define motorEN2DirB 42 //อย่าลืมเปลี่ยนขา

int threshold = 500;
bool proximityActive = false;

Servo myservo;
String servo, PROXIMITY1, PROXIMITY2, Relay, clockwise, counterclockwise, Pump_clockwise, Pump_counterclockwise, mortor_clockwise, mortor_EN1_clockwise, mortor_EN1_counterclockwise,mortor_EN2_clockwise,
mortor_EN2_counterclockwise;

long duration, distance1, distance2;  // ประกาศตัวแปรเก็บค่าระยะ
unsigned long previousMillis = 0;
const long interval = 500;  // ระยะเวลาระหว่างการวัด (500 ms)
int stepsPerSecond = 1000;  // กำหนดจำนวนสเต็ปต่อวินาที
bool isMoving = false;      // ตัวแปรเพื่อตรวจสอบสถานะการหมุนของมอเตอร์
int motorStatus = 0;

Encoder encoder1(encoder1PinA, encoder1PinB);
Encoder encoder2(encoder2PinA, encoder2PinB);


long oldPosition = -999;

void setup() {

  Serial.begin(9600);
  Serial.println("Arduino is ready");  // แจ้งสถานะเมื่อเริ่มต้น

  myservo.attach(31);  // servo motor ขาเอาท์พุต 31

  pinMode(relayPin, OUTPUT);
  pinMode(PROXIMITY1_PIN, INPUT);
  pinMode(PROXIMITY2_PIN, INPUT);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(motorEN1PWM, OUTPUT);
  pinMode(motorEN1DirA, OUTPUT);
  pinMode(motorEN1DirB, OUTPUT);
}

void loop() {
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
  //PUMP_clockwise();
  //Mortor_clockwise();
 MortorEN1_clockwise();
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

    digitalWrite(TRIG_PIN1, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN1, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN1, LOW);

    duration = pulseIn(ECHO_PIN1, HIGH, 30000);
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

    digitalWrite(TRIG_PIN2, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN2, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN2, LOW);

    duration = pulseIn(ECHO_PIN2, HIGH, 30000);
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
      digitalWrite(dirPin, HIGH);                 // HIGH = หมุนไปข้างหน้า
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
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);  // ควบคุมความเร็วโดยการปรับเวลานี้
      digitalWrite(stepPin, LOW);
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
      digitalWrite(dirPin, LOW);                         // HIGH = หมุนไปข้างหน้า
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
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);  // ควบคุมความเร็วโดยการปรับเวลานี้
      digitalWrite(stepPin, LOW);
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
      motorStatus = 1;  // เปิดมอเตอร์
      Serial.println("Motor ON");
    } else if (Pump_clockwise == "OFF") {
      motorStatus = 0;  // ปิดมอเตอร์
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");
    }

    // ควบคุมมอเตอร์ตามสถานะ
    if (motorStatus == 1) {
      digitalWrite(motorPinA, HIGH);  // ขา A HIGH
      digitalWrite(motorPinB, LOW);   // ขา B LOW
      analogWrite(pwmPin, 128);       // ความเร็ว PWM
    } else {
      digitalWrite(motorPinA, LOW);  // หยุดมอเตอร์
      digitalWrite(motorPinB, LOW);
      analogWrite(pwmPin, 0);
    }
  }
  delay(100);  // เลื่อนการทำงานเล็กน้อย
}

void PUMP_counterclockwise() {
  if (Serial.available() > 0) {                            // ตรวจสอบว่ามีข้อมูลเข้ามาจาก Serial
    Pump_counterclockwise = Serial.readStringUntil('\n');  // อ่านคำสั่งจาก Serial จนถึง newline
    Pump_counterclockwise.trim();                          // ตัดช่องว่างและ newline ออก

    if (Pump_counterclockwise == "ON") {
      motorStatus = 1;  // ตั้งค่าสถานะมอเตอร์ให้เป็น ON
      Serial.println("Motor ON");
    } else if (Pump_counterclockwise == "OFF") {
      motorStatus = 0;  // ตั้งค่าสถานะมอเตอร์ให้เป็น OFF
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }

    if (motorStatus == 1) {
      digitalWrite(motorPinA, LOW);   // ขา A HIGH
      digitalWrite(motorPinB, HIGH);  // ขา B LOW
      analogWrite(pwmPin, 128);       // กำหนดความเร็ว PWM (ค่า 0-255)
    } else {
      // ถ้ามอเตอร์ปิด (OFF), ให้หยุดการหมุน
      digitalWrite(motorPinA, LOW);  // ขา A LOW
      digitalWrite(motorPinB, LOW);  // ขา B LOW
      analogWrite(pwmPin, 0);        // หยุดการหมุน
    }
  }
  delay(100);  // เลื่อนการทำงานของ loop ให้มีความช้าลงเล็กน้อย
}


// มอเตอร์เอ็นโค้ตเดอร์
void MortorEN1_clockwise(){
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
    digitalWrite(motorEN1DirA, HIGH);
    digitalWrite(motorEN1DirB, LOW);
    analogWrite(motorEN1PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motorEN1DirA, LOW);
    digitalWrite(motorEN1DirB, LOW);
    analogWrite(motorEN1PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void MortorEN1_counterclockwise(){
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
    digitalWrite(motorEN1DirA, LOW);
    digitalWrite(motorEN1DirB, HIGH);
    analogWrite(motorEN1PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motorEN1DirA, LOW);
    digitalWrite(motorEN1DirB, LOW);
    analogWrite(motorEN1PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void MortorEN2_clockwise(){
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
    digitalWrite(motorEN2DirA, HIGH);
    digitalWrite(motorEN2DirB, LOW);
    analogWrite(motorEN2PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motorEN2DirA, LOW);
    digitalWrite(motorEN2DirB, LOW);
    analogWrite(motorEN2PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

void MortorEN2_counterclockwise(){
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
    digitalWrite(motorEN2DirA, LOW);
    digitalWrite(motorEN2DirB, HIGH);
    analogWrite(motorEN2PWM, 150);  // กำหนดความเร็ว (0-255)

    // อัพเดตตำแหน่งของเอ็นโค้ดเดอร์ตลอดเวลา
    // long newPosition = myEncoder.read();

    // if (newPosition != oldPosition) {
    //   oldPosition = newPosition;
    //   Serial.print("Position: ");
    //   Serial.println(newPosition);  // แสดงตำแหน่งที่อัปเดต
    // }

  } else {
    digitalWrite(motorEN2DirA, LOW);
    digitalWrite(motorEN2DirB, LOW);
    analogWrite(motorEN2PWM, 0);  // ปิดมอเตอร์
  }

  delay(100);  // เลื่อนการทำงานเล็กน้อย (ปรับเวลาให้เหมาะสมตามต้องการ)
}

