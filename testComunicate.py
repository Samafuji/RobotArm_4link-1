import serial
import time
import random

arduino = serial.Serial(port='COM8', baudrate=9600, timeout=1)

def write_read(x):
    arduino.write(bytes(str(x) + '\n', 'utf-8'))
    time.sleep(0.1)
    data = arduino.readline().decode('utf-8').rstrip()
    return data

while True:
    data = arduino.readline().decode('utf-8').rstrip()
    if data == "REQUEST":
        random_number = random.randint(1, 10)
        print(f"Sending random number: {random_number}")
        ack = write_read(random_number)
        print(f"Arduino ACK: {ack}")

    elif data.startswith("COUNTDOWN:"): # COUNTDOWN識別子で始まるかを確認
        remaining_seconds = int(data.split(":")[1]) # 残り秒数を抽出
        print(f"Remaining seconds: {remaining_seconds}")

    time.sleep(0.1)