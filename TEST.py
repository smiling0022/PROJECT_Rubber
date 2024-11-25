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


# สร้าง Thread สำหรับการอ่านข้อมูล
read_thread = threading.Thread(target=read_from_arduino, daemon=True)
read_thread.start()

# เรียกใช้งานการส่งคำสั่งใน Thread หลัก
# write_to_arduino_servo()
write_to_arduino_Proximity1()
write_to_arduino_Proximity2()



