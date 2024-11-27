#include <Arduino.h>
#include <Servo.h>
#define PROXIMITY1_PIN 6
#define PROXIMITY2_PIN 7
#define TRIG_PIN1 5  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define ECHO_PIN1 9  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04
#define TRIG_PIN2 4  // ขาที่เชื่อมต่อกับขา Trig ของ HC-SR04
#define ECHO_PIN2 8  // ขาที่เชื่อมต่อกับขา Echo ของ HC-SR04
#define relayPin  32

int threshold = 500;
bool proximityActive = false;

Servo myservo;
String servo,PROXIMITY1,PROXIMITY2,Relay;

long duration, distance1,distance2; // ประกาศตัวแปรเก็บค่าระยะ
unsigned long previousMillis = 0;
const long interval = 500; // ระยะเวลาระหว่างการวัด (500 ms)


void setup() {

  Serial.begin(9600);
  Serial.println("Arduino is ready");  // แจ้งสถานะเมื่อเริ่มต้น

  myservo.attach(31);  // servo motor ขาเอาท์พุต 31

  pinMode(relayPin, OUTPUT);
  pinMode(PROXIMITY1_PIN, INPUT);  // ตั้งค่าขาเซนเซอร์เป็นอินพุต
  pinMode(PROXIMITY2_PIN, INPUT);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);


}

void loop() {
  //Servo_motor();  // เรียกใช้งานฟังก์ชัน Servo_motor
  // Proximity_sensor_1();
  // Proximity_sensor_2();
  // Ultrasonic_1();
  // Ultrasonic_2();
  Relay_SolenoidValve();
  //Servo_motor();
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