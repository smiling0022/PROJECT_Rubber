#include <Arduino.h>
#include <Servo.h>
#define PROXIMITY1_PIN 6
#define PROXIMITY2_PIN 7
#define TRIG_PIN1 5  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define ECHO_PIN1 9  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04
#define TRIG_PIN2 4  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define ECHO_PIN2 8  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04
#define relayPin 32

#define stepPin A10   //Steping
#define dirPin A11    //Steping

const int motorPinA = A8;  // กำหนดขา A
const int motorPinB = A9;  // กำหนดขา B
const int pwmPin = 42;     // กำหนดขา PWM

int threshold = 500;
bool proximityActive = false;

Servo myservo;
String servo, PROXIMITY1, PROXIMITY2, Relay, clockwise, counterclockwise, Pump_clockwise;

long duration, distance1, distance2;  // ประกาศตัวแปรเก็บค่าระยะ
unsigned long previousMillis = 0;
const long interval = 500;  // ระยะเวลาระหว่างการวัด (500 ms)
int stepsPerSecond = 1000;  // กำหนดจำนวนสเต็ปต่อวินาที
bool isMoving = false;      // ตัวแปรเพื่อตรวจสอบสถานะการหมุนของมอเตอร์
int motorStatus = 0;


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
}

void loop() {
  //Servo_motor();  // เรียกใช้งานฟังก์ชัน Servo_motor
  // Proximity_sensor_1();
  // Proximity_sensor_2();
  // Ultrasonic_1();
  // Ultrasonic_2();
  //Relay_SolenoidValve();
  //Servo_motor();
  //STEP_clockwise();
  //STEP_counterclockwise();
  PUMP_clockwise();
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
  if (Serial.available() > 0) {           // ตรวจสอบว่ามีข้อมูลเข้ามาจาก Serial
    Pump_clockwise = Serial.readStringUntil('\n');  // อ่านคำสั่งจาก Serial จนถึง newline
    Pump_clockwise.trim();                          // ตัดช่องว่างและ newline ออก
    

    if (Pump_clockwise == "ON") {
      motorStatus = 1;  // ตั้งค่าสถานะมอเตอร์ให้เป็น ON
      Serial.println("Motor ON");
      Serial.println("Arduino: "+ motorStatus);
    } else if (Pump_clockwise == "OFF") {
      motorStatus = 0; // ตั้งค่าสถานะมอเตอร์ให้เป็น OFF
      Serial.println("Motor OFF");
    } else {
      Serial.println("Invalid Command");  // ส่งข้อความเมื่อคำสั่งไม่ถูกต้อง
    }

    // ถ้ามอเตอร์เปิด (ON), ให้หมุนมอเตอร์
    Serial.println("Arduino: "+motorStatus);

    if (motorStatus == 1) {
      digitalWrite(motorPinA, HIGH);  // ขา A HIGH
      digitalWrite(motorPinB, LOW);   // ขา B LOW
      analogWrite(pwmPin, 128);       // กำหนดความเร็ว PWM (ค่า 0-255)
    } else {
      // ถ้ามอเตอร์ปิด (OFF), ให้หยุดการหมุน
      digitalWrite(motorPinA, LOW);   // ขา A LOW
      digitalWrite(motorPinB, LOW);   // ขา B LOW
      analogWrite(pwmPin, 0);         // หยุดการหมุน
    }
  }
  delay(100); // เลื่อนการทำงานของ loop ให้มีความช้าลงเล็กน้อย
}
