
import serial
import numpy as np
# from scipy.optimize import fsolve
import time

arduino = serial.Serial(port='COM8', baudrate=9600, timeout=1)

def get_angles():
    arduino.write(b"GET\n")
    response = arduino.readline().decode('utf-8').strip()
    if response.startswith("POS"):
        try:
            _, q1, q2, q3 = map(float, response.split(","))
            return q1, q2, q3
        except ValueError as e:
            print(f"Parsing Error: {e} - Received data: {data}")
    return None

def send_angles(q1, q2, q3):
    command = f"SET {q1},{q2},{q3}\n"
    arduino.write(command.encode())
    response = arduino.readline().decode('utf-8').strip()
    print(response)
    return response

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

def Clamp(angles):
    """
    角度の制限（0度～180度）
    """
    return np.clip(angles, 0, np.pi)

def get_position():
    arduino.write(b"GET\n")
    try:
        data = arduino.readline().decode('utf-8').strip()
        if data.startswith("POS,"):  # Check if it starts with "POS,"
            try:
                parts = data[4:].split(",") # Remove "POS," and then split
                if len(parts) == 3:  # Expecting 3 values (x, y, z)
                    x, y, z = map(float, parts)
                    return np.array([x, y, z])
                else:
                    print(f"Invalid POS format: Expected 3 values, got {len(parts)}: {data}")
                    return None
            except ValueError as parse_error:
                print(f"Parsing Error (float conversion): {parse_error} - Received data: {data}")
                return None
        elif data == "": #タイムアウト時に空の文字列が帰ってくる場合があるのでその処理を追加
            print("Timeout or empty data received.")
            return None
        else:
            print(f"Unexpected data from Arduino: {data}")
            return None
    except serial.SerialTimeoutException:
        print("Serial read timed out")
        return None
    except UnicodeDecodeError as decode_error:
      print(f"Decoding Error: {decode_error} - Received bytes: {arduino.readline()}")
      return None


def main():
    target_pos = np.array([100.0, 100.0, 0.0])  # 初期目標位置を適切に設定
    q = np.array([30.0, 30.0, 30.0])/180*np.pi  # q1, q2, q3 の初期角度 (ラジアン)
    alpha = 0.1  # ステップサイズ

    waitACKflip = 0
    while True:
        try:
            data = arduino.readline().decode('utf-8').strip()
            # print(data)

            if waitACKflip:
                print("here")
                if  data.startswith("ACK"):
                    waitACKflip = 0
                continue
            
            target_pos = get_position()

            if target_pos is None:
              time.sleep(1)
              continue # データ取得に失敗した場合はスキップ
            waitACKflip = 1

            current_pos = forward_kinematics(q[0], q[1], q[2])
            # print(f"Target Position : x={target_pos[0]}, y={target_pos[1]}, z={target_pos[2]}")
            # print(f"Current Position : x={current_pos[0]}, y={current_pos[1]}, z={current_pos[2]}")
                        
            e = target_pos - current_pos  # 誤差ベクトル
            J = compute_jacobian(q[0], q[1], q[2])

            J_pos = J[0:3, :]
            J_pinv = np.linalg.pinv(J_pos)
            dq = alpha * (J_pinv @ e)
            q = q + dq
            q = Clamp(q)

            angles_deg = np.array(q) * 180.0 / np.pi

            print(f"Joint Angles: q1={angles_deg[0]:.2f}, q2={angles_deg[1]:.2f}, q3={angles_deg[2]:.2f}")
            success = send_angles(int(angles_deg[0]), int(angles_deg[1]), int(angles_deg[2]))

            if not success:
                print("Failed to set angles!")

            time.sleep(0.5)

        except serial.SerialException as serial_error:
            print(f"Serial Error: {serial_error}")
            try:
                arduino.close()
                arduino.open()
                print("Serial port reconnected.")
            except serial.SerialException as e:
                print(f"Failed to reconnect serial port: {e}")
                break  # 再接続に失敗したらループを抜ける
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break

    if arduino.is_open:
        arduino.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()