
#include <SoftwareSerial.h>
#include <Servo.h>

#define PI 3.14159265
#define MotorL1countA 21 //编码器A
#define MotorL1countB 45 //编码器B

#define MotorR1countA 20 //编码器A
#define MotorR1countB 44 //编码器B

#define MotorL2countA 18 //编码器A
#define MotorL2countB 43 //编码器B

#define MotorR2countA 2 //编码器A
#define MotorR2countB 34 //编码器B

#define MotorL3countA 19 //编码器A
#define MotorL3countB 42 //编码器B


#define MotorR3countA 3 //编码器A
#define MotorR3countB 33 //编码器B

const int  R3A =23;
const int  R3B =24;
const int  R3S =13;
const int  L3S =12;
const int  L3A =25;
const int  L3B =26;
const int  L2A =36;
const int  L2B =35;
const int  L2S =9;
const int  L1S =6;
const int  L1A =39;
const int  L1B =38;
const int  R1A =28;
const int  R1B =29;
const int  R1S =8;
const int  R2S =7;
const int  R2A =31;
const int  R2B =30;

unsigned long lastTime,lastTime2;
int SampleTime = 10;
unsigned long right_cd = 0;
unsigned long left_cd = 0;
unsigned long back_cd = 0;
unsigned long cross_cd = 0;
int cross_num = 0;
int cross_num_limit = 3000;
float add_speed = 0;
int begin_turn = 0;
float avr[5];
int ai = 0;
// float Balance_Kp = 0;
// float Balance_Kd = 0;
int now_car_speed = 0;
int H_L1_B = 0;
int H_R1_B = 0;
int H_L2_B = 0;
int H_R2_B = 0;
int H_L3_B = 0;
int H_R3_B = 0;
int run_l = 100;
int run_r = 100;
int car_room;
int l_red = 0;
int forWard_speed = 80;
int r_red = 0;
int times = 0;
int now_block = 0;
unsigned long now_time = 0;
int ser_dir = 105;
// int ser_dir = 90;
int ser_max = 140;//135对应实际角度17.11
int ser_min = 50;//对应实际角度27.41
float kp = 4, ki = 1.4, kd = 0;  //PID参数
//float kp = 1.4, ki = 0.4, kd = 0.5; //PID参数
int reset_pid = 0;
int MID_flag = 0;
int LL = 0; //光电
int L = 0;
int M = 0;
int R = 0;
int RR = 0;
//int craft = [0,0,0,0];
int craft_num = 0;
int start = 0;
float carsp = 0;//小车速度
float carle = 0;
int error = 0;
// Servo myser; //默认90度
float serial_angle = 0;
int  serial_speed = 0;
int  serial_turn = 4;
// Servo myser1;
// Servo myser2;
int turn = 0;//转向模式开启与关闭
volatile float motorL1 = 0; //中断变量，左轮子脉冲计数
volatile float motorR1 = 0; //中断变量，右轮子脉冲计数
volatile float motorL2 = 0; //中断变量，左轮子脉冲计数
volatile float motorR2 = 0; //中断变量，右轮子脉冲计数
volatile float motorL3 = 0; //中断变量，左轮子脉冲计数
volatile float motorR3 = 0; //中断变量，右轮子脉冲计数
float vL;
int i;
int now_state = -1;
float vR;

float v1 = 0; //单位cm/s
float v2 = 0; //单位cm/s
float v3 = 0; //单位cm/s
float v4 = 0; //单位cm/s
float v5 = 0; //单位cm/s
float v6 = 0; //单位cm/s
float Target_V_L = 8, Target_V_R = 8; //目标速度，单位cm/s
int Pwm_L = 0, Pwm_R = 0; //左右轮PWM
String command; //串口接收数据
float  p_out, i_out, d_out, t_out ; //浮点数
float L_out, R_out, M_out;
float P_out, D_out, I_out ,T_out;
int now_cross = 0;
int now_cross2 = 0;
int now_pwm_l;
int now_pwm_r;
int speed_level = 0;
int led_two[3];
int speed_set = 0;

int now_room = 0;
char val;
int now_catch = 0;
SoftwareSerial BT(14, 15);
//使用终端方法读取左右轮速度
int bal_car ;
int receive_data = 8;
// Servo myservo;

  /**
     函数作用：读取左右电机的速度
     返回值：  无
     
    r3------l3
    l2------l1
    r2------r1

   * */
void Read_Moto_V() {

  //这是读取电机速度的函数
  unsigned long nowtime = 0;
  motorL1 = 0;
  motorR1 = 0;
  motorL2 = 0;
  motorR2 = 0;
  motorL3 = 0;
  motorR3 = 0;
  nowtime = millis() + 50; //读50毫秒
  attachInterrupt(digitalPinToInterrupt(MotorL1countA), Read_Moto_L1, RISING); //左轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorR1countA), Read_Moto_R1, RISING); //右轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorL2countA), Read_Moto_L2, RISING); //左轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorR2countA), Read_Moto_R2, RISING); //右轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorL3countA), Read_Moto_L3, RISING); //左轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorR3countA), Read_Moto_R3, RISING); //右轮脉冲开中断计数
  while (millis() < nowtime); //达到50毫秒关闭中断
  detachInterrupt(digitalPinToInterrupt(MotorL1countA));//左轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorR1countA));//右轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorL2countA));//左轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorR2countA));//右轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorL3countA));//左轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorR3countA));//右轮脉冲关中断计数
float V_L1,V_R1,V_L2,V_R2,V_L3,V_R3;
V_L1=V_R1=V_L2=V_R2=V_L3=V_R3 = 0;
  //390来自转速比1:30中的30乘以编码器线数13
  V_L1 = ((motorL1 / 390) * 6.5 * PI) / 0.05; //单位cm/s
  V_R1 = ((motorR1 / 390) * 6.5 * PI) / 0.05; //单位cm/s
  V_L2 = ((motorL2 / 130) * 6.5 * PI) / 0.05; //单位cm/s
  V_R2 = ((motorR2 / 130) * 6.5 * PI) / 0.05; //单位cm/s
  V_L3 = ((motorL3 / 390) * 6.5 * PI) / 0.05; //单位cm/s
  V_R3 = ((motorR3 / 390) * 6.5 * PI) / 0.05; //单位cm/s
  v1 = V_L1;
  v2 = V_R1;
  v3 = V_L2;
  v4 = V_R2;
  v5 = V_L3;
  v6 = V_R3;
  Serial.print("L1:");
  Serial.print(v1);
  Serial.print("\tL2:");
  Serial.print(v3);
  Serial.print("\tL3:");
  Serial.println(v5);

  Serial.print("R1:");
  Serial.print(v2);
  Serial.print("\tR2:");
  Serial.print(v4);
  Serial.print("\tR3:");
  Serial.println(v6);
  Serial.println();
// 变压器一边为车头

}

void Read_Moto_L1() {
  /**
     函数作用：读取左电机的编码器
     返回值：  无
   * */
  H_L1_B = digitalRead(MotorL1countB);//0是倒转 1是正转
  if (H_L1_B == 1)
  {
    motorL1++;
  } else
  {
    motorL1--;
  }
  //  Serial.print("Moto LB = ");
  //  Serial.print(motorL);
}
void Read_Moto_L2() {
  /**
     函数作用：读取左电机的编码器
     返回值：  无
   * */
  H_L2_B = digitalRead(MotorL2countB);//0是倒转 1是正转
  if (H_L2_B == 0)
  {
    motorL2++;
  } else
  {
    motorL2--;
  }
  //  Serial.print("Moto LB = ");
  //  Serial.print(motorL);
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
void Read_Moto_L3() {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
  /**
     函数作用：读取左电机的编码器
     返回值：  无
   * */
  H_L3_B = digitalRead(MotorL3countB);//0是倒转 1是正转
  if (H_L3_B == 1)
  {
    motorL3++;
  } else
  {
    motorL3--;
  }
  //  Serial.print("Moto LB = ");
  //  Serial.print(motorL);
}
//读取右轮脉冲
void Read_Moto_R1() {
  /**
     函数作用：读取右电机的编码器
     返回值：  无
   * */
  H_R1_B = digitalRead(MotorR1countB);//0是倒转 1是正转
  if (H_R1_B == 0)
  {
    motorR1++;
  } else
  {
    motorR1--;
  }


  //  Serial.print("\tMoto RB = ");
  //  Serial.println(motorR);
}
void Read_Moto_R3() {
  /**
     函数作用：读取右电机的编码器
     返回值：  无
   * */
  H_R3_B = digitalRead(MotorR3countB);//0是倒转 1是正转
  if (H_R3_B == 1)
  {
    motorR3++;
  } else
  {
    motorR3--;
  }


  //  Serial.print("\tMoto RB = ");
  //  Serial.println(motorR);
}
void Read_Moto_R2() {
  /**
     函数作用：读取右电机的编码器
     返回值：  无
   * */
  H_R2_B = digitalRead(MotorR2countB);//0是倒转 1是正转
  if (H_R2_B == 1)
  {
    motorR2++;
  } else
  {
    motorR2--;
  }


  //  Serial.print("\tMoto RB = ");
  //  Serial.println(motorR);
}

  /**
     函数作用：电机行走，传入的L_v R_v为左右轮速度，范围-255~255，对应倒转和正转
     返回值：  无
      L      R
      1------1
      2------2
      3------3

   * */
void Run_Moto_F(int L_v, int R_v, int L_Moto,int R_Moto) {
  int L_Motor_A,R_Motor_A,L_Motor_B,R_Motor_B,L_Motor_SPEED,R_Motor_SPEED;
  switch (L_Moto)
  {
  case 1:
    L_Motor_A = L1A;
    L_Motor_B = L1B;
    L_Motor_SPEED = L1S;
    break;
  case 2:
    L_Motor_A = L2A;
    L_Motor_B = L2B;
    L_Motor_SPEED = L2S;
    break;
  case 3:
    L_Motor_A = L3A;
    L_Motor_B = L3B;
    L_Motor_SPEED = L3S;
    break;
  default:
    break;
  }
switch (R_Moto)
{
  case 1:
    R_Motor_A = R1A;
    R_Motor_B = R1B;
    R_Motor_SPEED = R1S;
    break;
  case 2:
    R_Motor_A = R2A;
    R_Motor_B = R2B;
    R_Motor_SPEED = R2S;
    break;
  case 3:
    R_Motor_A = R3A;
    R_Motor_B = R3B;
    R_Motor_SPEED = R3S;
    break;
  default:
    break;
  }
  if (L_v > 250)
  {
    L_v = 250;
  }
  else if ( L_v < -250 )
  {
    L_v = -250;
  }
  if (R_v > 250)
  {
    R_v = 250;
  }
  else if ( R_v < -250 )
  {
    R_v = -250;
  }
  //前进，可设置左右轮速度
  if (L_v > 0) {
    digitalWrite(L_Motor_A, 0);
    digitalWrite(L_Motor_B, HIGH);
    analogWrite(L_Motor_SPEED, L_v);
  } else if (L_v < 0) {
    digitalWrite(L_Motor_A, HIGH);
    digitalWrite(L_Motor_B, 0);
    analogWrite(L_Motor_SPEED, -L_v);
  } else
  {
    digitalWrite(L_Motor_A, 0);
    digitalWrite(L_Motor_B, 0);
    analogWrite(L_Motor_SPEED, 0);
  }
  if (R_v > 0) {
    digitalWrite(R_Motor_A, HIGH);
    digitalWrite(R_Motor_B, 0);
    analogWrite(R_Motor_SPEED, R_v);
  } else if (R_v < 0) {
    digitalWrite(R_Motor_A, 0);
    digitalWrite(R_Motor_B, HIGH);
    analogWrite(R_Motor_SPEED, -R_v);
  } else
  {
    digitalWrite(R_Motor_A, 0);
    digitalWrite(R_Motor_B, 0);
    analogWrite(R_Motor_SPEED, 0);
    // delay(2000);
  }
}

int Position_Servo (float Encoder, float Target) { //小车转向pid，输入为转向角度，以及目标度数，返回的是目标速度的叠加
//   float Position_KP = 0.15*(now_car_speed/10)*0.5, Position_KI = 0, Position_KD = 0;
    // float Position_KP = 0.08, Position_KI = 0, Position_KD = 0;
        float Position_KP = 0.4, Position_KI = 0, Position_KD  = 2;
  static float err, angle, I_err, Last_err;
  err = Encoder - Target;
  I_err += err;
  angle = Position_KP * err + Position_KI * I_err + Position_KD * (err - Last_err);
  Last_err = err;

  return angle;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(L1A, OUTPUT);
  pinMode(L1B, OUTPUT);
  pinMode(R1A, OUTPUT);
  pinMode(R1B, OUTPUT);
  pinMode(L1S, OUTPUT);
  pinMode(R1S, OUTPUT);
 
  pinMode(L2A, OUTPUT);
  pinMode(L2B, OUTPUT);
  pinMode(R2A, OUTPUT);
  pinMode(R2B, OUTPUT);
  pinMode(L2S, OUTPUT);
  pinMode(R2S, OUTPUT);

    pinMode(L3A, OUTPUT);
  pinMode(L3B, OUTPUT);
  pinMode(R3A, OUTPUT);
  pinMode(R3B, OUTPUT);
  pinMode(L3S, OUTPUT);
  pinMode(R3S, OUTPUT);

  delay(1000);

  Serial.begin(9600);
  Serial.println("BRGIN");
  // HC-05默认，9600
  Serial.begin(9600);
  //   P_out = Balance_Kp;
  //  D_out = Balance_Kd;
  //  myservo.attach(10);
  //  myservo.write(ser_dir);
  now_car_speed = 15;
  now_time = millis();
  left_cd = millis();
  right_cd = millis();
  back_cd = millis();
  T_out = 60;
        //   myser1.write(180);
        // myser2.write(0);
        

}

void Rec_serial() {
  /**
     函数作用：读取串口，读取格式为“a偏移角度s速度t状态@”
     返回值：  目标速度的叠加，用于在正常速度上进行加减从而使小车转向的数值
   * */
  int angle_index = 0;
  int speed_index = 0;
  int turn_index = 0;
  int end_index = 0;
  String angle_str;
  String speed_str;
  String turn_str;

  // while (Serial.available()>0&&command.length()<=8)
  // {
  //     command += char(Serial.read());
  //         // command += Serial.readStringUntil('@');
  //     delay(1); //去掉之后不太好用
  // }
  command += Serial.readStringUntil('@\n');
  // delay(1); //去掉之后不太好用
  if (command.length() >= 6)
  {

    angle_index = command.indexOf('a');
    speed_index = command.indexOf('s');
    turn_index = command.indexOf('t');
    end_index = command.indexOf('@');
    angle_str = command.substring(angle_index + 1, speed_index);
    speed_str = command.substring(speed_index + 1, turn_index);
    turn_str = command.substring(turn_index + 1, end_index);

    serial_angle = angle_str.toInt();
    serial_speed = speed_str.toInt();
    serial_turn = turn_str.toInt();
    // Serial.print("command:");
    // Serial.println(command);
    // Serial.print("serial_angle:");
    // Serial.println(serial_angle);
    // Serial.print("\tserial_speed:");
    // Serial.print(serial_speed);
    // Serial.print("\tserial_turn:");
    // Serial.println(serial_turn);
    command = "";
    while (Serial.read() >= 0)
    {
      /* code */
    }

  }
}

void Rec_SPID() {

  while (Serial.available() > 0)
  {
    command += char(Serial.read());
    delay(1); //去掉之后不太好用
  }
  //  Serial.print("command:");
  //  Serial.println(command);
  int P_index = 0;
  int D_index = 0;
  int I_index = 0;
  int T_index = 0;

  String P_command;
  String I_command;
  String D_command;
  String T_command;
  if (command.length() > 0)
  {
    P_index = command.indexOf('p');//变量1
    I_index = command.indexOf('i');
    D_index = command.indexOf('d');//变量2
    T_index = command.indexOf('t');//变量2
    P_command = command.substring(P_index + 1, I_index);
    I_command = command.substring(I_index + 1, D_index);
    D_command = command.substring(D_index + 1, T_index);
    T_command = command.substring(T_index + 1);
    P_out = P_command.toFloat();
    I_out = I_command.toFloat();
    D_out = D_command.toFloat();
    T_out = T_command.toFloat();
    // Serial.print("Kp:");
    // Serial.print(P_out);
    // Serial.print("\tKi:");
    // Serial.print(I_out);
    // Serial.print("\tKd:");
    // Serial.println(D_out);
    command = "";
    MID_flag = 1;
    if (kp != P_out || ki != I_out || kd != D_out) {

      kp = P_out;
      ki = I_out;
      kd = D_out;
      delay(2000);
    }
  }


}

int Incremental_Pi_L(int current_speed, int target_speed) {
  /**
    函数作用：左轮pid，使得左轮在有负载的情况下保持速度
    返回值：  速度数值
  * */

   static float pwm, bias, last_bias, prev_bias; //静态变量存在程序全周期：pwm:增量输出,bias:本次偏差,last_bias:上次偏差,prev_bais_:上上次偏差
   static float lastInput;
   bias = current_speed - target_speed; //计算本次偏差e(k)
   float dInput;
   dInput = (current_speed - lastInput);
   if (bias < 18&&bias>5)
   {
     dInput = dInput*2;
   }
   
   
    

  // pwm -= (kp * (bias - last_bias) + ki * bias + kd * (bias - 2 * last_bias + prev_bias)); //增量式PID控制器
  // p0.08i1d0t60

    pwm -= (kp * (dInput)+ ki * bias + kd * (bias - 2 * last_bias + prev_bias)); //增量式PID控制器
  prev_bias = last_bias; //保存上上次偏差
  last_bias = bias;   //保存上一次偏差
  //PWM 限幅度  Arduino的PWM 最高为255  限制在250
  if (pwm < -250) {
    pwm = -250;
  }
  if (pwm > 250) {
    pwm = 250;
  }

  lastInput = current_speed;

     return pwm; 
          //增量输出
}
int Incremental_Pi_R(int current_speed, int target_speed) {
  /**
    函数作用：左轮pid，使得左轮在有负载的情况下保持速度
    返回值：  速度数值
  * */
   unsigned long now = millis();
   int timeChange = (now - lastTime2);
   static float pwm, bias, last_bias, prev_bias; //静态变量存在程序全周期：pwm:增量输出,bias:本次偏差,last_bias:上次偏差,prev_bais_:上上次偏差
   static float lastInput;
   bias = current_speed - target_speed; //计算本次偏差e(k)
   float dInput;
   dInput = (current_speed - lastInput);
   if (bias < 18&&bias>5)
   {
     dInput = dInput*2;
   }
   
   

  // pwm -= (kp * (bias - last_bias) + ki * bias + kd * (bias - 2 * last_bias + prev_bias)); //增量式PID控制器
  // p0.08i1d0t60

    pwm -= (kp * (dInput)+ ki * bias + kd * (bias - 2 * last_bias + prev_bias)); //增量式PID控制器
  prev_bias = last_bias; //保存上上次偏差
  last_bias = bias;   //保存上一次偏差
  //PWM 限幅度  Arduino的PWM 最高为255  限制在250
  if (pwm < -250) {
    pwm = -250;
  }
  if (pwm > 250) {
    pwm = 250;
  }

  lastInput = current_speed;

     return pwm; 
          //增量输出
}


                                                                           
void loop() { 
Read_Moto_V();
// Run_Moto_F(100,0,L1A,R1A,L1B,R1B,L1S,R1S);
//  Run_Moto_F(100,0,L2A,R2A,L2B,R2B,L2S,R2S);
Run_Moto_F(100,100,1,2);
delay(1000);
Run_Moto_F(250,-100,2,1);
}
