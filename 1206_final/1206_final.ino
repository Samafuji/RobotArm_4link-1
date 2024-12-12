#include <Servo.h>

//jotsitck setup
int x1Pin = 0;  //X軸 接類比A0
int y1Pin = 1;  //Y軸 接類比A1
int z1Pin = 2;  //Z軸 接類比A2
  
int x1=0;    //Ｘ軸變數
int y1=0;    //Ｙ軸變數
int z1=0;    //Ｚ軸變數

int x2Pin = 3;  //X軸 接類比A3
int y2Pin = 4;  //Y軸 接類比A4
int z2Pin = 5;  //Z軸 接類比A5
  
int x2=0;    //Ｘ軸變數
int y2=0;    //Ｙ軸變數
int z2=0;    //Ｚ軸變數

int buttom_l = 12;
int buttom_r = 13;
bool hasTriggered = false;

int joint1 = 6;
int joint2 = 5;
int joint3 = 9;
int clip = 11;

Servo j1_servo;
Servo j2_servo;
Servo j3_servo;
Servo cl_servo;

int minangle = 0;
int maxangle = 180;
int angle1 = 90;
int angle2 = 90;
int angle3 = 90;

int pos = 0;

int Joint_control(int in, int angle_i){
  int a;

  if(in > 850){
    a = 7;
  }
  else if(in > 600){
    a = 3;
  }
  else if(in < 200){
    a = -7;
  }
  else if(in < 400){
    a = -3;
  }
  else{
    a = 0;
  }
  angle_i = angle_i + a;
  if (angle_i < 0){
    angle_i = 0;
  }
  else if(angle_i > 180){
    angle_i = 180;
  }

  return angle_i;

}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
	Serial.setTimeout(1); 
  
  j1_servo.attach(joint1);
  j2_servo.attach(joint2);
  j3_servo.attach(joint3);
  cl_servo.attach(clip);
  cl_servo.write(70);
}

void loop() {
  // put your main code here, to run repeatedly:
 
  char buf[100];
  
  x1 = analogRead(y1Pin);   //左邊左右
  y1 = analogRead(x1Pin);   //左邊左右
  z1 = analogRead(z1Pin);   //左邊左右
  x2 = analogRead(y2Pin);   //左邊左右
  y2 = analogRead(x2Pin);   //左邊左右
  z2 = analogRead(z2Pin);   //左邊左右
  sprintf(buf, "x1=%d, y1=%d, z1=%d, x2=%d, y2=%d, z2=%d", x1, y1, z1, x2, y2, z2);
  Serial.println(buf);



  angle1 = Joint_control(x2, angle1);
  j1_servo.write(angle1); // 馬達1跟隨 X 軸
  angle2 = Joint_control(x1, angle2);
  j2_servo.write(angle2); // 馬達1跟隨 X 軸
  angle3 = Joint_control(y1, angle3);
  j3_servo.write(angle3); // 馬達1跟隨 X 軸

  delay(50);

  if (z1 < 20){//按下按鈕時
    for (pos = 60; pos >= 0; pos -= 1){ 
      cl_servo.write(pos);   // 告訴 servo 走到 'pos' 的位置
      delay(15);   // 等待 15ms 讓 servo 走到指定位置
    }    
  }
  else if(z2 < 20){
    for (pos = 0; pos <= 60; pos += 1){     
      cl_servo.write(pos);    // 告訴 servo 走到 'pos' 的位置
      delay(15);        // 等待 15ms 讓 servo 走到指定位置
    }
  }
}
/*

*/


