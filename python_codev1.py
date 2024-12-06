import serial
import threading
import time

# ตั้งค่าการเชื่อมต่อ Serial กับ Arduino
arduino = serial.Serial(port='COM5', baudrate=9600, timeout=1)

def read_from_arduino():
    while True:
        try:
            data = arduino.readline().decode('utf-8').strip()
            if data:
                print(f"Arduino: {data}")  # แสดงข้อมูลที่ได้รับ
        except Exception as e:
            print(f"Error reading from Arduino: {e}")

# ปั๊มดูดหมุนตามเข็ม 
def write_to_arduino_PUMP_clockwise():
    while True:
        Pump_clockwise = input("Enter command (ON/OFF/exit): ").strip()
        if Pump_clockwise.lower() == 'exit':
            print("Exiting...")
            arduino.close()
            break
        elif Pump_clockwise in ["ON", "OFF"]:
            arduino.write(f"{Pump_clockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")
# ปั๊มดูดหมุนทวนเข็ม
def write_to_arduino_PUMP_counterclockwise():
    while True:
        PUMP_counterclockwise = input("Enter command (ON/OFF/exit): ").strip()
        if PUMP_counterclockwise.lower() == 'exit':
            print("Exiting...")
            arduino.close()
            break
        elif PUMP_counterclockwise in ["ON", "OFF"]:
            arduino.write(f"{PUMP_counterclockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

# สเต็ปมอเเตอร์หมุนตามเข็ม
def write_to_arduino_STEP_clockwise():
    while True:
        clockwise = input("Enter command (ON/OFF/exit): ").strip()
        if clockwise.lower() == 'exit':
            print("Exiting...")
            arduino.close()
            break
        elif clockwise in ["ON", "OFF"]:
            arduino.write(f"{clockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

# สเต็ปมอเเตอร์หมุนทวนเข็ม
def write_to_arduino_STEP_counterclockwise():
    while True:
        counterclockwise = input("Enter command (ON/OFF/exit): ").strip()
        if counterclockwise.lower() == 'exit':
            print("Exiting...")
            arduino.close()
            break
        elif counterclockwise in ["ON", "OFF"]:
            arduino.write(f"{counterclockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

# เซอร์โว
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

# รีเลย์
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

# พร็อกซิมิตี้1
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

# พร็อกซิมิตี้2
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

# อัลตร้าโซนิค1
def write_to_arduino_Ultrasonic1():
    while True:
        try:
            # อ่านข้อมูลจาก Arduino และแสดงผล
            Ultrasonic1 = arduino.readline().decode('utf-8').strip()
            if Ultrasonic1:
                print(f"Distance1: {Ultrasonic1} cm")
        except Exception as e:
            print(f"Error reading from Arduino: {e}")

# อัลตร้าโซนิค2
def write_to_arduino_Ultrasonic2():
    while True:
        try:
            # อ่านข้อมูลจาก Arduino และแสดงผล
            Ultrasonic2 = arduino.readline().decode('utf-8').strip()
            if Ultrasonic2:
                print(f"Distance2: {Ultrasonic2} cm")
        except Exception as e:
            print(f"Error reading from Arduino: {e}")

# มอเตอร์เอ็นโค้ตเดอร์1
def write_to_arduino_mortor_EN1_clockwise():
    while True:
        mortor_EN1_clockwise = input("Enter command (ON/OFF/exit): ").strip()
        if mortor_EN1_clockwise == 'exit':  # แก้ไขตรงนี้
            print("Exiting...")
            arduino.close()
            break
        elif mortor_EN1_clockwise in ["ON", "OFF"]:
            arduino.write(f"{mortor_EN1_clockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

def write_to_arduino_mortor_EN1_counterclockwise():
    while True:
        mortor_EN1_counterclockwise = input("Enter command (ON/OFF/exit): ").strip()
        if  mortor_EN1_counterclockwise == 'exit':  # แก้ไขตรงนี้
            print("Exiting...")
            arduino.close()
            break
        elif  mortor_EN1_counterclockwise in ["ON", "OFF"]:
            arduino.write(f"{ mortor_EN1_counterclockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

# มอเตอร์เอ็นโค้ตเดอร์2
def write_to_arduino_mortor_EN2_clockwise():
    while True:
        mortor_EN2_clockwise = input("Enter command (ON/OFF/exit): ").strip()
        if mortor_EN2_clockwise == 'exit':  
            print("Exiting...")
            arduino.close()
            break
        elif mortor_EN2_clockwise in ["ON", "OFF"]:
            arduino.write(f"{mortor_EN2_clockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

def write_to_arduino_mortor_EN2_counterclockwise():
    while True:
        mortor_EN2_counterclockwise = input("Enter command (ON/OFF/exit): ").strip()
        if  mortor_EN2_counterclockwise == 'exit':  
            print("Exiting...")
            arduino.close()
            break
        elif  mortor_EN2_counterclockwise in ["ON", "OFF"]:
            arduino.write(f"{ mortor_EN2_counterclockwise}\n".encode('utf-8'))
        else:
            print("Invalid command. Please enter ON, OFF, or exit.")

# สร้าง Thread สำหรับการอ่านข้อมูล
read_thread = threading.Thread(target=read_from_arduino, daemon=True)
read_thread.start()

# ถ้าจะใช้อลตร้าต้องเปิดคอมเมนต์ตรงนี้ด้วย
# เรียกใช้งานการส่งคำสั่งใน Thread หลัก
# print("Reading data from Arduino. Press Ctrl+C to stop.")
# try:
#     while True:
#         pass  # รอให้ Thread ทำงาน
# except KeyboardInterrupt:
#     print("\nExiting...")
#     arduino.close()

    
# write_to_arduino_Ultrasonic1()
# write_to_arduino_Ultrasonic2()
# write_to_arduino_Proximity1()
# write_to_arduino_Proximity2()
# Relay_SolenoidValve()
# write_to_arduino_servo()
# write_to_arduino_STEP_clockwise()
# write_to_arduino_STEP_counterclockwise()
# write_to_arduino_PUMP_clockwise()
# write_to_arduino_PUMP_counterclockwise()
# write_to_arduino_mortor_EN1_clockwise()
write_to_arduino_mortor_EN1_counterclockwise()

