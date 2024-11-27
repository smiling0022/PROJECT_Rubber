#include <Arduino.h>

// กำหนดขา
#define stepPin  10
#define dirPin   11

// กำหนดความเร็ว (จำนวนสเต็ป/วินาที)
int stepsPerSecond = 1000;  // กำหนดความเร็วที่ต้องการ

void setup() {
  // ตั้งค่าขาเป็น OUTPUT
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // กำหนดทิศทางการหมุน
  digitalWrite(dirPin, HIGH);  // HIGH = หมุนไปข้างหน้า, LOW = หมุนถอยหลัง
}

void loop() {
  // การหมุนสเต็ปปิ้งมอเตอร์ในทิศทางที่กำหนด
  for (int i = 0; i < stepsPerSecond; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);  // ควบคุมความเร็วโดยการปรับเวลา (1,000us = 1ms ระหว่างการหมุนแต่ละสเต็ป)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}
