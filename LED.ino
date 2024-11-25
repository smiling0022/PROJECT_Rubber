#include <Servo.h>
#define PROXIMITY1_PIN A6
#define PROXIMITY2_PIN A7

int threshold = 500;
bool proximityActive = false;

Servo myservo;
String servo;
String PROXIMITY1;
String PROXIMITY2;




void setup() {

  Serial.begin(9600);
  Serial.println("Arduino is ready");  // แจ้งสถานะเมื่อเริ่มต้น

  myservo.attach(31);  // servo motor ขาเอาท์พุต 31

  pinMode(PROXIMITY1_PIN, INPUT);  // ตั้งค่าขาเซนเซอร์เป็นอินพุต
  pinMode(PROXIMITY2_PIN, INPUT);

  Serial.println("Analog Proximity Sensor Ready");
}

void loop() {
  //Servo_motor();  // เรียกใช้งานฟังก์ชัน Servo_motor
  //Proximity_sensor_1();
  Proximity_sensor_2();
}




//---------------------------INPUT------------------------------------

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
  int sensorValue1 = analogRead(PROXIMITY1_PIN);
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue1);

  // ตรวจสอบสถานะโลหะตามค่า threshold
  if (sensorValue1 > threshold) {
    Serial.println("Metal Detected!");
  } else {
    Serial.println("No Metal Detected");
  }

  delay(200);  // หน่วงเวลาเพื่อความเสถียร
}

void Proximity_sensor_2() {
  if (proximityActive) {
    Proximity_sensor_2();
  }

  if (Serial.available() > 0) {
    PROXIMITY2 = Serial.readStringUntil('\n');  // รับคำสั่งจาก Python
    PROXIMITY2.trim();

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
  int sensorValue2 = analogRead(PROXIMITY1_PIN);
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue2);

  // ตรวจสอบสถานะโลหะตามค่า threshold
  if (sensorValue2 > threshold) {
    Serial.println("Metal Detected!");
  } else {
    Serial.println("No Metal Detected");
  }

  delay(200);  // หน่วงเวลาเพื่อความเสถียร
}