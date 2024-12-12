#include <Servo.h>

// jotsitck setup
int x1Pin = 0; // 左ジョイスティック X
int y1Pin = 1; // 左ジョイスティック Y
int z1Pin = 2; // 左ジョイスティック ボタン

int x2Pin = 3; // 右ジョイスティック X
int y2Pin = 4; // 右ジョイスティック Y
int z2Pin = 5; // 右ジョイスティック ボタン

int x1, y1, z1, x2, y2, z2;

int joint1 = 6;
int joint2 = 5;
int joint3 = 9;
int clip = 11;

Servo j1_servo, j2_servo, j3_servo, cl_servo;

// Properties

float x = 100.0, y = 100.0, z = 100.0;                // 初期目標位置
float prev_x = 100.0, prev_y = 100.0, prev_z = 100.0; // 前回の位置
float step_size = 1.0;

int angle1 = 30, angle2 = 30, angle3 = 30; // 初期角度

void PressedButtom(int z1, int z2)
{
  if (z1 < 20)
  { // 按下按鈕時
    for (int pos = 60; pos >= 0; pos -= 1)
    {
      cl_servo.write(pos); // 告訴 servo 走到 'pos' 的位置
      delay(15);           // 等待 15ms 讓 servo 走到指定位置
    }
  }
  else if (z2 < 20)
  {
    for (int pos = 0; pos <= 60; pos += 1)
    {
      cl_servo.write(pos); // 告訴 servo 走到 'pos' 的位置
      delay(15);           // 等待 15ms 讓 servo 走到指定位置
    }
  }
}

bool JoyStick(int x1, int x2, int y1)
{
  bool Isupdated = false;

  if (x1 > 600)
  {
    x += step_size;
    Isupdated = true;
  }
  else if (x1 < 400)
  {
    x -= step_size;
    Isupdated = true;
  }

  if (x2 > 600)
  {
    y += step_size;
    Isupdated = true;
  }
  else if (x2 < 400)
  {
    y -= step_size;
    Isupdated = true;
  }

  if (y1 > 600)
  {
    z += step_size;
    Isupdated = true;
  }
  else if (y1 < 400)
  {
    z -= step_size;
    Isupdated = true;
  }

  return Isupdated;
}

void setup()
{
  // Serial.begin(115200);
  Serial.begin(9600);

  j1_servo.attach(joint1);
  j2_servo.attach(joint2);
  j3_servo.attach(joint3);
  cl_servo.attach(clip);

  j1_servo.write(angle1);
  j2_servo.write(angle2);
  j3_servo.write(angle3);
  cl_servo.write(70);
}

void loop()
{
  x1 = analogRead(y1Pin); // 左ジョイスティック X
  y1 = analogRead(x1Pin); // 左ジョイスティック Y
  z1 = analogRead(z1Pin); // 左ジョイスティック ボタン
  x2 = analogRead(y2Pin); // 右ジョイスティック X
  y2 = analogRead(x2Pin); // 右ジョイスティック Y
  z2 = analogRead(z2Pin); // 右ジョイスティック ボタン

  bool IsUpdated = JoyStick(x1, x2, y1);

  if (IsUpdated && (x != prev_x || y != prev_y || z != prev_z))
  {
    Serial.println("50,60,100");
    prev_x = x;
    prev_y = y;
    prev_z = z;
  }
  else
  {
    Serial.println("NOT_UPDATED");
  }

  PressedButtom(z1, z2);

  if (Serial.available() > 0)
  {
    String data = Serial.readString();
    if (data.startsWith("SET"))
    {
      sscanf(data.c_str(), "SET %d %d %d", &angle1, &angle2, &angle3);
      j1_servo.write(angle1);
      j2_servo.write(angle2);
      j3_servo.write(angle3);
      Serial.println("OK");
    }
    else if (data == "GET")
    {
      char buffer[50]; // バッファを用意
      snprintf(buffer, sizeof(buffer), "ANGLES %d %d %d", angle1, angle2, angle3);
      Serial.println(buffer); // フォーマットした文字列を送信
    }
  }
  delay(50);
}
