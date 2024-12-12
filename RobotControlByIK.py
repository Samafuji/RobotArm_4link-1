import serial
import numpy as np
import time

arduino = serial.Serial(port='COM7', baudrate=9600, timeout=1)

def get_angles():
    arduino.write(b"GET\n")
    response = arduino.readline().decode().strip()
    if response.startswith("ANGLES"):
        _, q1, q2, q3 = map(int, response.split())
        return q1, q2, q3
    return None

def set_angles(q1, q2, q3):
    command = f"SET {q1} {q2} {q3}\n"
    arduino.write(command.encode())
    response = arduino.readline().decode().strip()
    return response == "OK"

def jacobian(theta):
    q1, q2, q3 = theta
    # 以下にJacobian行列の計算式を記述
    J = np.zeros((6, 3))
    # Jを計算（省略）
    return J

def forward_kinematics(theta):
    q1, q2, q3 = theta
    # 以下に順運動学の計算式を記述
    x = ...  # 省略
    y = ...
    z = ...
    return np.array([x, y, z])

def pid_control(target, current, kp, ki, kd, integral, prev_error):
    error = target - current
    integral += error
    derivative = error - prev_error
    control = kp * error + ki * integral + kd * derivative
    return control, integral, error

target_pos = np.array([10, 10, 10])  # 目標位置
integral = np.zeros(3)
prev_error = np.zeros(3)

while True:
    angles = get_angles()
    if angles is None:
        continue
    current_pos = forward_kinematics(angles)
    controls = []
    for i in range(3):
        control, integral[i], prev_error[i] = pid_control(
            target_pos[i], current_pos[i], kp=1.0, ki=0.1, kd=0.05, 
            integral=integral[i], prev_error=prev_error[i]
        )
        controls.append(control)
        
    angles = np.clip(np.array(angles) + controls, 0, 180).astype(int)
    set_angles(*angles)
    time.sleep(0.1)