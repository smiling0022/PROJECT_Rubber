import serial
import threading
import time

# ตั้งค่าการเชื่อมต่อ Serial กับ Arduino
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=1)

def read_from_arduino():
    while True:
        try:
            data = arduino.readline().decode('utf-8').strip()
            if data:
                print(f"Arduino: {data}")  # แสดงข้อมูลที่ได้รับ
        except Exception as e:
            print(f"Error reading from Arduino: {e}")


def write_to_arduino_servo():
    while True:
        servo = input("Enter command (ON/OFF/exit): ").strip()
        if servo.lower() == 'exit':
            print("Exiting...")
            arduino.close()
            break
        elif servo in ["ON", "OFF"]:
            arduino.write(f"{servo}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

def Relay_SolenoidValve():
    while True:
        Relay = input("Enter command (ON/OFF/exit): ").strip()
        if Relay.lower() == 'exit':
            print("Exiting...")
            arduino.close()
            break
        elif Relay in ["ON", "OFF"]:
            arduino.write(f"{Relay}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

def write_to_arduino_Proximity1():
    while True:
        PROXIMITY1 = input("Enter proximity command (ON/OFF/exit): ").strip()
        if PROXIMITY1.lower() == 'exit':
            print("Exiting proximity control...")
            arduino.write("PROXIMITY OFF\n".encode('utf-8'))
            break
        elif PROXIMITY1 in ["ON", "OFF"]:
            arduino.write(f"PROXIMITY {PROXIMITY1}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

def write_to_arduino_Proximity2():
    while True:
        PROXIMITY2 = input("Enter proximity command (ON/OFF/exit): ").strip()
        if PROXIMITY2.lower() == 'exit':
            print("Exiting proximity control...")
            arduino.write("PROXIMITY OFF\n".encode('utf-8'))
            break
        elif PROXIMITY2 in ["ON", "OFF"]:
            arduino.write(f"PROXIMITY {PROXIMITY2}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

def write_to_arduino_Ultrasonic1():
    while True:
        try:
            # อ่านข้อมูลจาก Arduino และแสดงผล
            Ultrasonic1 = arduino.readline().decode('utf-8').strip()
            if Ultrasonic1:
                print(f"Distance1: {Ultrasonic1} cm")
        except Exception as e:
            print(f"Error reading from Arduino: {e}")

def write_to_arduino_Ultrasonic2():
    while True:
        try:
            # อ่านข้อมูลจาก Arduino และแสดงผล
            Ultrasonic2 = arduino.readline().decode('utf-8').strip()
            if Ultrasonic2:
                print(f"Distance2: {Ultrasonic2} cm")
        except Exception as e:
            print(f"Error reading from Arduino: {e}")

# สร้าง Thread สำหรับการอ่านข้อมูล
read_thread = threading.Thread(target=read_from_arduino, daemon=True)
read_thread.start()

# เรียกใช้งานการส่งคำสั่งใน Thread หลัก
print("Reading data from Arduino. Press Ctrl+C to stop.")
try:
    while True:
        pass  # รอให้ Thread ทำงาน
except KeyboardInterrupt:
    print("\nExiting...")
    arduino.close()

    
# write_to_arduino_Ultrasonic1()
# write_to_arduino_Ultrasonic2()
# write_to_arduino_Proximity1()
# write_to_arduino_Proximity2()
# Relay_SolenoidValve()
# write_to_arduino_servo()
