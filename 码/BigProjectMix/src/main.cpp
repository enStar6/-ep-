#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include <string.h>
#include <Stream.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

Servo servo1;
Servo servo2;
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

String str_in,mode,str_vy = "",str_vx = "", str_w = "",
str_ax, str_ay, str_ox, str_oy,
str_cn;
double angle2,angle1,
rotate_d = 0.95,
gX,gY,gZ,gXb,gYb,gZb;
int vx, vy, w,
ax, ay, axn = 140, ayn = 30, ox, oy, t,
w1, w2, w3, w4,
len, y_pos,
i = 0, ccm = 0,
servo1Pin = 10, servo2Pin = 9, pulsewidth, 
cs = 0, cn = 0, cPin = 24, ct = 0;
const int L = 79, l = 77, r = 1;

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//自定义函数

void prt_sgl(String key, double val, bool end = false){
  if(end){
    Serial.print(key);
    Serial.print(":");
    Serial.print(val);
    Serial.println(" ");
  }else {
    Serial.print(key);
    Serial.print(":");
    Serial.print(val);
    Serial.print(" ");
  } 
}

void prt()    //打印关键信息函数 
{                                                       
  prt_sgl("vx", vx);
  prt_sgl("vy", vy);
  prt_sgl("w", w);
  prt_sgl("ax", ax);
  prt_sgl("ay", ay);
  prt_sgl("ox", ox);
  prt_sgl("oy", oy);
  prt_sgl("angle1", angle1);
  prt_sgl("angle2", angle2);
  prt_sgl("gX", gX);
  prt_sgl("gY", gY);
  prt_sgl("cs",cs);
  prt_sgl("cn",cn);
  prt_sgl("ccm", ccm, true);
}

void servoPulse(int servoPin,double angle)    //定义一个脉冲函数
{
  pinMode(servoPin, OUTPUT);        //定义引脚模式
  digitalWrite(servoPin, LOW);      //引脚拉低重置
  //发送10个脉冲
  pulsewidth = map(angle, 0, 180, 500, 2500); //将角度转化为500-2500的脉宽值
  digitalWrite(servoPin, HIGH);   //将舵机接口电平至高
  delayMicroseconds(pulsewidth);  //延时脉宽值的微秒数
  digitalWrite(servoPin, LOW);    //将舵机接口电平至低
  delayMicroseconds(2500 - pulsewidth);     //低电平延时填满周期剩下部分
}

void arm(double x, double y, double arm1 = 130, double arm2 = 130)    //机械臂运动逆解
{
  // 逆解所需知识:1.勾股定理
  //             2.基本三角函数
  //             3.余弦定理
  //             4.角度制与弧度制转换
  //constrain函数的作用是限制舵机角度在一定范围内
  if(y >= 0){
    y = abs(y);
    angle2 = constrain((acos((x*x+y*y)/(260*sqrt(x*x+y*y))) + atan(y/x))*(180/M_PI),0,130);
  }else{
    y = abs(y);
    angle2 = constrain((acos((x*x+y*y)/(260*sqrt(x*x+y*y))) - atan(y/x))*(180/M_PI),0,130);   
  }
  angle1 = constrain(angle2 + (acos((33800-x*x-y*y)/33800))*(180/M_PI),90,165);
  //舵机角度输出
  servoPulse(servo1Pin, angle1);
  servoPulse(servo2Pin, 180-angle2);
}

void mec(int vx, int vy, int w, int L = L, int l = l, int r = r)    //因为底盘电机是TT电机，控制方式是pwm，控制精度不达标，所以没有带入轴距、孔距、麦轮半径等计算
{
  w1 = vy - vx - w;
  w2 = vx + vy - w;
  w3 = vx + vy + w;
  w4 = vy - vx + w;
  w1 = constrain(w1, -255, 255);
  w2 = constrain(w2, -255, 255);
  w3 = constrain(w3, -255, 255);
  w4 = constrain(w4, -255, 255);
  motor1.setSpeed(abs(w1));
  motor2.setSpeed(abs(w2));
  motor3.setSpeed(abs(w3));
  motor4.setSpeed(abs(w4));
  motor1.run(w1 >= 0 ? FORWARD : BACKWARD);
  motor2.run(w2 >= 0 ? FORWARD : BACKWARD);
  motor3.run(w3 >= 0 ? FORWARD : BACKWARD);
  motor4.run(w4 >= 0 ? FORWARD : BACKWARD);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  //串口波特率初始化
  Serial.begin(115200);
  Serial1.begin(9600);
  //! 左上↖电机为3,右上↗电机为1
  //! 左下↙电机为4,右下↘电机为2
  motor1.run(RELEASE);
  motor1.setSpeed(0);
  motor2.run(RELEASE);
  motor2.setSpeed(0);
  motor3.run(RELEASE);
  motor3.setSpeed(0);
  motor4.run(RELEASE);
  motor4.setSpeed(0);
  //机械臂位置初始化
  ox = 130;
  oy = 30;
  arm(ox,oy);
  //机械爪位置初始化
  servoPulse(cPin, 0);
}

void loop() { 
  //蓝牙信息接收
  //蓝牙发送格式为：数据标记符（作用于底盘 or 机械臂？）+ 数据信息 + 结束位
  //例："m255," 指控制TT电机以pwm255（满速）旋转
  if(Serial1.available()){
    str_in = Serial1.readStringUntil('*');                 //速度较快的接收字符串信息的方法
    mode = str_in.substring(0,1);                          //分割数据标记符
    str_in = str_in.substring(1,str_in.length()+1);        //分割数据信息    
  }

  //蓝牙信息处理
  if(mode == "r" && ccm == 0){                             //! 平移
    len = str_in.length();                                 //获取实质信息的长度
    y_pos = str_in.indexOf("Y");                           //获取字符"Y"的索引
    str_vy = str_in.substring(y_pos+1,len);                //截取底盘y速度字符串
    if(i == 0 ||i == 1){
      vy = atoi(str_vy.c_str());
      vy -= 2*vy;                                          //因为蓝牙软件的奇怪设置所以要将vy反一下   
      i = 1;
    }  
    str_vx = str_in.substring(1,y_pos);
    vx = atoi(str_vx.c_str());  
  }else if(mode == "d" && ccm == 0){                       //! 旋转
    len = str_in.length();
    y_pos = str_in.indexOf("Y");                           //获取字符"Y"的索引
    str_vy = str_in.substring(y_pos+1,len);                //? 为啥是len?
    if(i == 0 || i == 2){                                  //! 优先由左摇杆控制vy,若手指离开左摇杆(vy等于0),才由右摇杆控制vy
      vy = atoi(str_vy.c_str());
      vy -= 2*vy;                                          //因为蓝牙软件的奇怪设置所以要将vy反一下 
      i = 2;
    }
    str_w = str_in.substring(1,y_pos);
    w = ((double)atoi(str_w.c_str())) / rotate_d;
  }else if(mode == "a"){                                   //! 机械臂
    len = str_in.length();
    y_pos = str_in.indexOf("Y");                           //获取字符"Y"的索引
    str_ay = str_in.substring(y_pos+1,len);                //? 为啥是len?
    ay = atoi(str_ay.c_str());
    ay -= 2*ay;                                            //因为蓝牙软件的奇怪设置所以要将vy反一下   
    str_ax = str_in.substring(1,y_pos);
    ax = atoi(str_ax.c_str());
    ax -= 2*ax;  
  }else if(mode == "s"){                                   //机械臂随便设了个归零值
    ox = 130;
    oy = 30;
    arm(ox, oy);
  }else if(mode == "A"){                                   //手机X方向陀螺仪角度（横屏前后翻转）            
    gX = atoi(str_in.c_str());
    gX -= 2*gX;
  }else if (mode == "B"){                                  //手机Y方向陀螺仪角度（横屏左右翻转）
    gY = atoi(str_in.c_str());
  }else if(mode == "G"){                                   //切换陀螺仪控制/摇杆控制
    if(ccm == 0){                                          //每次切换模式底盘制动
      ccm = 1;                                             
      vx = 0;
      vy = 0;
      w = 0;
    }else{                              
      ccm = 0;
      vx = 0;
      vy = 0;
      w = 0;
    }
  }
  else if(mode == "C"){                             //机械爪控制信息接收
    cn = -2;
    ct = 1;
  }else if(mode == "O"){
    cn = 2;
    ct = 1;
  }else if(mode == "T"){
    cn = 0;
    ct = 1;
  }
  
  if(vy == 0){                                             //左右摇杆优先级系统
    i = 0;                                                 //0：平等  1：左  2：右
  }

  //陀螺仪角度值与底盘控制值换算
  if(ccm == 1){
    if(gX >= 2){                                           //前后移动没有做调速
      vy = 255;
    }else if (gX <= -2){
      vy = -255;
    }else{
      vy = 0;
    }
    if(gY >= 0){                                           //左右转可调速，并映射至较高速度，防止堵转
      w = map(gY, 1, 10, 130, 255);
    }else{
      w = map(gY, -1, -10, -130, -255);
    }
    if(abs(w) < 130){
      w = 0;
    }
  }
  
  //底盘控制函数
  mec(vx, vy, w);

  //机械臂控制
  if(t >= 2 && (ax != 0 || ay != 0)){                       //当机械臂摇杆有动静才更新舵机角度
    ox += ax;
    oy += ay;
    ox = constrain(ox, 20, 230);                            //限制机械臂沿x轴移动范围
    oy = constrain(oy, -60, 180);                           //限制机械臂沿y轴移动范围
    arm(ox,oy);
    t = 0;
  }
                                                        //控制机械臂位置更新频率
  //机械爪控制
  servoPulse(cPin, cs);
  cs+=cn;
  cs = constrain(cs, 0, 70);
  
  t++;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  prt();                                                    //串口调试输出
  delay(1);                                                 //主循环延迟
} 