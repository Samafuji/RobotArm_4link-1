
import serial
import numpy as np
# from scipy.optimize import fsolve
import time

arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)

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

def send_angles(q1, q2, q3):
    """
    sedn joints to Arduino
    (q1, q2, q3)
    """
    command = f"{q1:.2f},{q2:.2f},{q3:.2f}\n"
    arduino.write(command.encode())
    
def jacobian_simulink(theta):
    """
    Compute the Jacobian matrix and its derived forms.
    Args:
        theta: List or array of joint angles [q1, q2, q3].
    Returns:
        J: Full Jacobian matrix.
        J_inv: Pseudo-inverse of the full Jacobian.
        J_inv_small: Pseudo-inverse of the positional part of the Jacobian.
        J_T_small: Transpose of the positional part of the Jacobian.
    """
    q1, q2, q3 = theta

    J = compute_jacobian(q1, q2, q3)

    J_inv = np.linalg.pinv(J)

    J_Sub = J[:3, :]

    J_inv_small = np.linalg.pinv(J_Sub)
    J_T_small = J_Sub.T

    return J, J_inv, J_inv_small, J_T_small

def compute_jacobian(q1, q2, q3):
    """
    Compute the full 6x3 Jacobian matrix.
    Args:
        q1, q2, q3: Joint angles in radians.
    Returns:
        J: 6x3 Jacobian matrix.
    """
    J = np.array([
        [
            (1739 * np.sin(q1) * np.sin(q2) * np.sin(q3)) / 20
            - (1739 * np.cos(q2) * np.sin(q1)) / 20
            - (7271 * np.cos(q1)) / 100
            - (1739 * np.cos(q2) * np.cos(q3) * np.sin(q1)) / 20,
            - (1739 * np.cos(q1) * np.sin(q2)) / 20
            - (1739 * np.cos(q1) * np.cos(q2) * np.sin(q3)) / 20
            - (1739 * np.cos(q1) * np.cos(q3) * np.sin(q2)) / 20,
            - (1739 * np.cos(q1) * np.cos(q2) * np.sin(q3)) / 20
            - (1739 * np.cos(q1) * np.cos(q3) * np.sin(q2)) / 20
        ],
        [
            (1739 * np.cos(q1) * np.cos(q2)) / 20
            - (7271 * np.sin(q1)) / 100
            + (1739 * np.cos(q1) * np.cos(q2) * np.cos(q3)) / 20
            - (1739 * np.cos(q1) * np.sin(q2) * np.sin(q3)) / 20,
            - (1739 * np.sin(q1) * np.sin(q2)) / 20
            - (1739 * np.cos(q2) * np.sin(q1) * np.sin(q3)) / 20
            - (1739 * np.cos(q3) * np.sin(q1) * np.sin(q2)) / 20,
            - (1739 * np.cos(q2) * np.sin(q1) * np.sin(q3)) / 20
            - (1739 * np.cos(q3) * np.sin(q1) * np.sin(q2)) / 20
        ],
        [
            0,
            (1739 * np.sin(q2) * np.sin(q3)) / 20
            - (1739 * np.cos(q2) * np.cos(q3)) / 20
            - (1739 * np.cos(q2)) / 20,
            (1739 * np.sin(q2) * np.sin(q3)) / 20
            - (1739 * np.cos(q2) * np.cos(q3)) / 20
        ],
        [
            -np.sin(q1),
            -np.sin(q1),
            -np.sin(q1)
        ],
        [
            np.cos(q1),
            np.cos(q1),
            np.cos(q1)
        ],
        [
            0,
            0,
            0
        ]
    ])
    return J

def forward_kinematics(q1, q2, q3):
    """
    Compute the forward kinematics (position of the end-effector).
    Args:
        q1, q2, q3: Joint angles in radians.
    Returns:
        posi: Array of [x, y, z] positions.
    """
    x = (1739 * np.cos(q1) * np.cos(q2)) / 20 \
        - (7271 * np.sin(q1)) / 100 \
        + (1739 * np.cos(q1) * np.cos(q2) * np.cos(q3)) / 20 \
        - (1739 * np.cos(q1) * np.sin(q2) * np.sin(q3)) / 20

    y = (7271 * np.cos(q1)) / 100 \
        + (1739 * np.cos(q2) * np.sin(q1)) / 20 \
        - (1739 * np.sin(q1) * np.sin(q2) * np.sin(q3)) / 20 \
        + (1739 * np.cos(q2) * np.cos(q3) * np.sin(q1)) / 20

    z = 79 \
        - (1739 * np.cos(q2) * np.sin(q3)) / 20 \
        - (1739 * np.cos(q3) * np.sin(q2)) / 20 \
        - (1739 * np.sin(q2)) / 20

    R = np.array([
        [
            np.cos(q1) * np.cos(q2) * np.cos(q3)
            - np.cos(q1) * np.sin(q2) * np.sin(q3),
            - np.cos(q1) * np.cos(q2) * np.sin(q3)
            - np.cos(q1) * np.cos(q3) * np.sin(q2),
            -np.sin(q1)
        ],
        [
            np.cos(q2) * np.cos(q3) * np.sin(q1)
            - np.sin(q1) * np.sin(q2) * np.sin(q3),
            - np.cos(q2) * np.sin(q1) * np.sin(q3)
            - np.cos(q3) * np.sin(q1) * np.sin(q2),
            np.cos(q1)
        ],
        [
            - np.cos(q2) * np.sin(q3) - np.cos(q3) * np.sin(q2),
            np.sin(q2) * np.sin(q3) - np.cos(q2) * np.cos(q3),
            0
        ]
    ])

    # Uncomment to include roll, pitch, yaw
    # yaw = np.arctan2(R[1, 0], R[0, 0])
    # pitch = np.arcsin(-R[2, 0])
    # roll = np.arctan2(R[2, 1], R[2, 2])
    # posi = np.array([x, y, z, roll, pitch, yaw])

    posi = np.array([x, y, z])
    return posi

# def inverse_kinematics(x_target, y_target, z_target, q_init=(0.0, 0.0, 0.0)):
    """
    x_target, y_target, z_target: 求めたいエンドエフェクタ位置
    q_init: (q1, q2, q3) の初期推定値
    """
    def equations(q):
        q1, q2, q3 = q
        x, y, z = forward_kinematics(q1, q2, q3)
        return [
            x - x_target,
            y - y_target,
            z - z_target
        ]
    
    # fsolveを用いて非線形方程式系を解く
    solution = fsolve(equations, q_init)
    return solution

def pid_control(target, current, kp, ki, kd, integral, prev_error):
    error = target - current
    integral += error
    derivative = error - prev_error
    control = kp * error + ki * integral + kd * derivative
    return control, integral, error

target_pos = np.array([10, 10, 10])
integral = np.zeros(3)
prev_error = np.zeros(3)

q = [0.0, 0.0, 0.0]  # q1, q2, q3 の初期角度

tol = 1e-3
alpha = 1 # update property

while True:
    data = arduino.readline().decode('utf-8').strip()
    if data:
        try:
            x, y, z = map(float, data.split(','))
            target_pos = np.array([x, y, z])
            print(f"Target Position: x={x}, y={y}, z={z}")

            x_current, y_current, z_current = forward_kinematics(q[0], q[1], q[2])
            current_pos = np.array([x_current, y_current, z_current])
            
            e = target_pos - current_pos  # 誤差ベクトル
            J = compute_jacobian(q[0], q[1], q[2])

            J_pos = J[0:3, :]

            J_pinv = np.linalg.pinv(J_pos)

            dq = alpha * (J_pinv @ e)
            print(f"Joint Angles: dq={dq}")
            q = q + dq
            
            x_current, y_current, z_current = forward_kinematics(q[0], q[1], q[2])
            print(f"Joint Angles: q1={q[0]}, q2={q[1]}, q3={q[2]}")
            print(f"Target Position: x={x}, y={y}, z={z}")
            print(f"previous pos: x={current_pos[0]}, y={current_pos[1]}, z={current_pos[2]}")
            print(f"current pos: x={x_current}, y={y_current}, z={z_current}")

            # Arduinoに角度を送信
            send_angles(q[0], q[1], q[2])

        except Exception as e:
            print(f"Error: {e}")
    else:
        print("No data received from Arduino.")