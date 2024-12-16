#include <Servo.h>
#include <math.h>

#define M_PI 3.14159265358979323846

int x1Pin = A0; // 左ジョイスティック X
int y1Pin = A1; // 左ジョイスティック Y
int z1Pin = A2; // 左ジョイスティック ボタン

int x2Pin = A3; // 右ジョイスティック X
int y2Pin = A4; // 右ジョイスティック Y
int z2Pin = A5; // 右ジョイスティック ボタン

int x1, y1, z1, x2, y2, z2;

int joint1 = 6;
int joint2 = 5;
int joint3 = 9;
int clip = 11;

Servo j1_servo, j2_servo, j3_servo, cl_servo;

// 初期位置・目標位置
float x = 100.0, y = 100.0, z = 100.0;  
float step_size = 5.0;

// initial condition in j spcae
float q[3] = {20.0 * M_PI/180.0, -30.0 * M_PI/180.0, 60.0 * M_PI/180.0};

// real angle to control
int q0 = 110; // 20
int q1 = 150; // -30
int q2 = 60;  // 90

bool PressedGet = false;
bool ChangeMode = true; // Mode true->cartian, mode false->angular
bool z2_pressed = false;

void PressedButton(int z1, bool *PressGet) {
  if (z1 < 20) {
      for (int pos = 45; pos >= 0; pos -= 1) {
          cl_servo.write(pos);
          delay(15);
      }
      *PressGet = true;
  }
}

void JoyStick(int x1, int x2, int y1, float *x, float *y, float *z) {
    // ここはユーザー指定ロジックに従う。y1でx軸、x1でy軸移動など
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

void Joint_control(int in, float *angle_i){
  float a;

  if(in > 850){
    a = 7.0;
  }
  else if(in > 600){
    a = 3.0;
  }
  else if(in < 200){
    a = -7.0;
  }
  else if(in < 400){
    a = -3.0;
  }
  else{
    a = 0.0;
  }
  *angle_i = *angle_i + a / 180.0 * M_PI;
}
// fk
void forward_kinematics(float q1, float q2, float q3, float &X, float &Y, float &Z) {
    float cq1=cos(q1), sq1=sin(q1);
    float cq2=cos(q2), sq2=sin(q2);
    float cq3=cos(q3), sq3=sin(q3);

    X = (1739 * cq1 * cq2) / 20.0
        - (7271 * sq1) / 100.0
        + (1739 * cq1 * cq2 * cq3) / 20.0
        - (1739 * cq1 * sq2 * sq3) / 20.0;

    Y = (7271 * cq1) / 100.0
        + (1739 * cq2 * sq1) / 20.0
        - (1739 * sq1 * sq2 * sq3) / 20.0
        + (1739 * cq2 * cq3 * sq1) / 20.0;

    Z = 79.0
        - (1739 * cq2 * sq3) / 20.0
        - (1739 * cq3 * sq2) / 20.0
        - (1739 * sq2) / 20.0;
}

// jaco in pos (3*3)
void compute_jacobian(float q1, float q2, float q3, float J[3][3]) {
    float cq1=cos(q1), sq1=sin(q1);
    float cq2=cos(q2), sq2=sin(q2);
    float cq3=cos(q3), sq3=sin(q3);

    J[0][0] = (1739 * sq1 * sq2 * sq3)/20.0 - (1739 * cq2 * sq1)/20.0 - (7271 * cq1)/100.0 - (1739 * cq2 * cq3 * sq1)/20.0;
    J[0][1] = - (1739 * cq1 * sq2)/20.0 - (1739 * cq1 * cq2 * sq3)/20.0 - (1739 * cq1 * cq3 * sq2)/20.0;
    J[0][2] = - (1739 * cq1 * cq2 * sq3)/20.0 - (1739 * cq1 * cq3 * sq2)/20.0;

    J[1][0] = (1739 * cq1 * cq2)/20.0 - (7271 * sq1)/100.0 + (1739 * cq1 * cq2 * cq3)/20.0 - (1739 * cq1 * sq2 * sq3)/20.0;
    J[1][1] = - (1739 * sq1 * sq2)/20.0 - (1739 * cq2 * sq1 * sq3)/20.0 - (1739 * cq3 * sq1 * sq2)/20.0;
    J[1][2] = - (1739 * cq2 * sq1 * sq3)/20.0 - (1739 * cq3 * sq1 * sq2)/20.0;

    J[2][0] = 0.0;
    J[2][1] = (1739 * sq2 * sq3)/20.0 - (1739 * cq2 * cq3)/20.0 - (1739 * cq2)/20.0;
    J[2][2] = (1739 * sq2 * sq3)/20.0 - (1739 * cq2 * cq3)/20.0;
}

// 3x3 J inverse
bool invert3x3(float A[3][3], float A_inv[3][3]) {
    float det = A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
               -A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
               +A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabs(det) < 1e-10) return false;

    float invdet = 1.0/det;
    A_inv[0][0] = (A[1][1]*A[2][2]-A[1][2]*A[2][1])*invdet;
    A_inv[0][1] = (A[0][2]*A[2][1]-A[0][1]*A[2][2])*invdet;
    A_inv[0][2] = (A[0][1]*A[1][2]-A[0][2]*A[1][1])*invdet;

    A_inv[1][0] = (A[1][2]*A[2][0]-A[1][0]*A[2][2])*invdet;
    A_inv[1][1] = (A[0][0]*A[2][2]-A[0][2]*A[2][0])*invdet;
    A_inv[1][2] = (A[0][2]*A[1][0]-A[0][0]*A[1][2])*invdet;

    A_inv[2][0] = (A[1][0]*A[2][1]-A[1][1]*A[2][0])*invdet;
    A_inv[2][1] = (A[0][1]*A[2][0]-A[0][0]*A[2][1])*invdet;
    A_inv[2][2] = (A[0][0]*A[1][1]-A[0][1]*A[1][0])*invdet;

    return true;
}

// clip
float clipf(float val, float minv, float maxv) {
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}

// clamp
void clampDeg(float &a) {
    if (a < 0) a=0;
    if (a > 180) a=180;
}

void setup() {
    Serial.begin(115200);

    j1_servo.attach(joint1);
    j2_servo.attach(joint2);
    j3_servo.attach(joint3);
    cl_servo.attach(clip);
    j1_servo.write(q0);
    j2_servo.write(q1);
    j3_servo.write(q2);
    cl_servo.write(70); // 初期位置
}

float previous_error[3] = {0.0, 0.0, 0.0}; // 前回のエラーを保持
unsigned long previous_time = 0;           // 前回の時間を記録

void loop() {
  x1 = analogRead(y1Pin);
  y1 = analogRead(x1Pin);
  z1 = analogRead(z1Pin);
  x2 = analogRead(y2Pin);
  y2 = analogRead(x2Pin);
  z2 = analogRead(z2Pin);

  PressedButton(z1, &PressedGet);

  if (z2 < 20) { 
      if (!z2_pressed) {
          ChangeMode = !ChangeMode;
          z2_pressed = true; 
          delay(100);
      }
  } else {
      z2_pressed = false;
  }

  if(PressedGet)
  {
    float gapX = (q[0] - 20.0 * M_PI/180.0) / 10;
    float gapY = (q[1] + 30.0 * M_PI/180.0) / 10;
    float gapZ = (q[2] - 60.0 * M_PI/180.0) / 10; 
    for( int i = 0;i < 10;i++)
    {
      q[0] -= gapX;
      q[1] -= gapY;
      q[2] -= gapZ; 

      // To degreee
      float q1_deg = q[0]*180.0/M_PI + 90.0;  
      float q2_deg = q[1]*180.0/M_PI + 180.0;
      float q3_deg = q[2]*180.0/M_PI;

      clampDeg(q1_deg);
      clampDeg(q2_deg);
      clampDeg(q3_deg);

      j1_servo.write((int)q1_deg);
      j2_servo.write((int)q2_deg);
      j3_servo.write((int)q3_deg);

      delay(100);
    }
    

    forward_kinematics(q[0], q[1], q[2], x, y, z);
    for (int pos = 0; pos <= 45; pos += 1) {
      cl_servo.write(pos);
      delay(15);
    }
    delay(500);
    PressedGet = false;

    return;
  }

  if (ChangeMode)
  {
    JoyStick(x1, x2, y1, &x, &y, &z);

    float Xc, Yc, Zc;
    forward_kinematics(q[0], q[1], q[2], Xc, Yc, Zc);

    float ex = x - Xc;
    float ey = y - Yc;
    float ez = z - Zc;
    float err_norm = sqrt(ex*ex + ey*ey + ez*ez);

    if (err_norm > 1.0) {
        float J[3][3];
        compute_jacobian(q[0], q[1], q[2], J);

        float J_inv[3][3];
        if (invert3x3(J, J_inv)) {
            float alpha = 1.3;
            // dq = alpha * J_inv * e + beta * J_inv * d
            float dq[3];
            float beta = 0.01; // Dゲインの調整値

            // 微分制御の計算
            float derivative[3];
            derivative[0] = (ex - previous_error[0]);
            derivative[1] = (ey - previous_error[1]);
            derivative[2] = (ez - previous_error[2]);
            
            dq[0] = alpha * (J_inv[0][0] * ex + J_inv[0][1] * ey + J_inv[0][2] * ez)
                  + beta * (J_inv[0][0] * derivative[0] + J_inv[0][1] * derivative[1] + J_inv[0][2] * derivative[2]);
            dq[1] = alpha * (J_inv[1][0] * ex + J_inv[1][1] * ey + J_inv[1][2] * ez)
                  + beta * (J_inv[1][0] * derivative[0] + J_inv[1][1] * derivative[1] + J_inv[1][2] * derivative[2]);
            dq[2] = alpha * (J_inv[2][0] * ex + J_inv[2][1] * ey + J_inv[2][2] * ez)
                  + beta * (J_inv[2][0] * derivative[0] + J_inv[2][1] * derivative[1] + J_inv[2][2] * derivative[2]);

            // // dq = alpha * J_inv * e
            // float dq[3];
            // dq[0] = alpha*(J_inv[0][0]*ex + J_inv[0][1]*ey + J_inv[0][2]*ez);
            // dq[1] = alpha*(J_inv[1][0]*ex + J_inv[1][1]*ey + J_inv[1][2]*ez);
            // dq[2] = alpha*(J_inv[2][0]*ex + J_inv[2][1]*ey + J_inv[2][2]*ez);

            // clamp
            float max_dq = 20.0 * M_PI/180.0;
            if (fabs(dq[0])>max_dq) dq[0]= (dq[0]>0)?max_dq:-max_dq;
            if (fabs(dq[1])>max_dq) dq[1]= (dq[1]>0)?max_dq:-max_dq;
            if (fabs(dq[2])>max_dq) dq[2]= (dq[2]>0)?max_dq:-max_dq;

            q[0] += dq[0];
            q[1] += dq[1];
            q[2] += dq[2];
        }
    }
  }
  else{
    Joint_control(x2, &q[0]);
    Joint_control(x1, &q[1]);
    Joint_control(y1, &q[2]);

    forward_kinematics(q[0], q[1], q[2], x, y, z);
  }

  // To degreee
  float q1_deg = q[0]*180.0/M_PI + 90.0;  
  float q2_deg = q[1]*180.0/M_PI + 180.0;
  float q3_deg = q[2]*180.0/M_PI;

  clampDeg(q1_deg);
  clampDeg(q2_deg);
  clampDeg(q3_deg);

  j1_servo.write((int)q1_deg);
  j2_servo.write((int)q2_deg);
  j3_servo.write((int)q3_deg);

  delay(50);
}
