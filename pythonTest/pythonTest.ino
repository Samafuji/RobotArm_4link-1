#include <Servo.h>

// ジョイスティックのピン
int x1Pin = A0; // 左ジョイスティック X  左x++　　右x--
int y1Pin = A1; // 左ジョイスティック Y  下z++   上z--
int z1Pin = A2; // 左ジョイスティック ボタン 

int x2Pin = A3; // 右ジョイスティック X   左y++  右y--
int y2Pin = A4; // 右ジョイスティック Y
int z2Pin = A5; // 右ジョイスティック ボタン

int x1, y1, z1, x2, y2, z2;

// サーボモータのピン
int joint1 = 6;
int joint2 = 5;
int joint3 = 9;
int clip = 11;

Servo j1_servo, j2_servo, j3_servo, cl_servo;

// 目標位置と前回の位置
float x = 85.0, y = 70.0, z = 100.0;
float step_size = 1.0;

int q0 = 110; //20
int q1 = 150; //-30
int q2 = 40; //90

void PressedButtom(int z1, int z2) {
    // ボタン処理（変更なし）
    if (z1 < 20) {
        for (int pos = 60; pos >= 0; pos -= 1) {
            cl_servo.write(pos);
            delay(15);
        }
    } else if (z2 < 20) {
        for (int pos = 0; pos <= 60; pos += 1) {
            cl_servo.write(pos);
            delay(15);
        }
    }
}

void JoyStick(int x1, int x2, int y1, float *x, float *y, float *z) {
    // ジョイスティック処理（変更なし）
    if (y1 > 600) {
        *x -= step_size;
    } else if (y1 < 400) {
        *x += step_size;
    }

    if (x1 > 600) {
        *y += step_size;
    } else if (x1 < 400) {
        *y -= step_size;
    }

    if (y2 > 600) {
        *z -= step_size;
    } else if (y2 < 400) {
        *z += step_size;
    }
}

void Clamp(float *x, float *y, float *z)
{
  float rRange = 170;
  if (*x > rRange) {
        *x = rRange;
  } else if (*x < 0) {
      *x = 0;
  }

  if (*y > rRange) {
        *y = rRange;
  } else if (*y < 0) {
      *y = 0;
  }

  if (*z > rRange) {
        *z = rRange;
  } else if (*z < 20) {
      *z = 20;
  }
}

void setup() {
    Serial.begin(9600);

    j1_servo.attach(joint1);
    j2_servo.attach(joint2);
    j3_servo.attach(joint3);
    cl_servo.attach(clip);
    j1_servo.write(q0);
    j2_servo.write(q1);
    j3_servo.write(q2);
    cl_servo.write(70); // 初期位置
}

void loop() {
    x1 = analogRead(y1Pin);
    y1 = analogRead(x1Pin);
    z1 = analogRead(z1Pin);
    x2 = analogRead(y2Pin);
    y2 = analogRead(x2Pin);
    z2 = analogRead(z2Pin);

    JoyStick(x1, x2, y1, &x, &y, &z);
    PressedButtom(z1, z2);
    Clamp(&x, &y, &z);
    // Serial.print("Sensor Value: ");
    // Serial.print(x); // 改行付きで値を表示
    // Serial.print(" ");
    // Serial.print(y); // 改行付きで値を表示
    // Serial.print(" ");
    // Serial.println(z); // 改行付きで値を表示

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // 空白文字を削除

            Serial.println(command);
        if (command == "GET") {
            Serial.print("POS,");
            Serial.print(x);
            Serial.print(",");
            Serial.print(y);
            Serial.print(",");
            Serial.println(z);
            Serial.println("GetArduinoDone");
        } else if (command.startsWith("SET")) {
            int angle1, angle2, angle3;
            if (sscanf(command.c_str(), "SET %d,%d,%d", &angle1, &angle2, &angle3) == 3) {
                j1_servo.write(angle1);
                j2_servo.write(angle2);
                j3_servo.write(angle3);
                Serial.println("ACK"); // 受信確認を送信
            } else {
                Serial.print("Invalid angle data received: ");
                Serial.println(command);
            }
        } else {
          Serial.print("Unknown command received: ");
          Serial.println(command);
        }
    }
    delay(50);
}