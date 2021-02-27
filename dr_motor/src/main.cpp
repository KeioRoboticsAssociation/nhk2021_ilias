#include <mbed.h>
#include "motordriver.h"
#include "PID.h"
#include "controller.h"
#include "main.h"
#include "mbedserial.h"

#define SUMPLING_TIME_US 100000 //タイマー割り込み周期 kHzオーダーつまり1000が理想
#define PERIOD 1000 //モーターのMAX値

Serial pc(USBTX, USBRX, 115200);
InterruptIn switch1(USER_BUTTON);
DigitalIn target_speed(PA_5);
Ticker ticker1, ticker2;
CAN can1(PA_11, PA_12, 500000);
Mbedserial Ms(pc);
Controller controller(can1, 0x334);

//モーター
PwmOut PWM_a(PA_8);
PwmOut PWM_b(PA_0);
DigitalOut PHASE_a(PC_11);
DigitalOut PHASE_b(PD_2);
Motor motor_a(PWM_a, PHASE_a, PERIOD, true);
Motor motor_b(PWM_b, PHASE_b, PERIOD, true);

//エンコーダ
InterruptIn enA_a(PA_9);
DigitalIn enB_a(PB_4);
InterruptIn enA_b(PA_9);
DigitalIn enB_b(PB_4);

// これから出てくる[2]は全て[0]がmotor_a, [1]がmotor_bに対応している
double v[2] = {250, 250}; // 最終的なモーターへの指令値

//エンコーダ
int en_count[2] = {0, 0};
int old_en_count[2] = {0, 0};
int d_en_count[2] = {0, 0};

//PID
double Kp = 1.0, Ki = 0, Kd = 0;
double diff[4];
static double integral[2];
double delta_v=0;

//角速度を計測する
void speed_calc(void) {
  //pc.printf("encount;%d  oldencount;%d\n", en_count,old_en_count);//d_en_countは現在値
  d_en_count[0] = en_count[0] - old_en_count[0];
  old_en_count[0] = en_count[0]; //en_countは古くなったので保管しておく
  d_en_count[0] *= 1000000 / SUMPLING_TIME_US;
  pc.printf("Anglular Velocity(°/s):%d\n", d_en_count[0]);//d_en_countは現在値

  //pc.printf("encount;%d  oldencount;%d\n", en_count,old_en_count);//d_en_countは現在値
  d_en_count[1] = en_count[1] - old_en_count[1];
  old_en_count[1] = en_count[1]; //en_countは古くなったので保管しておく
  d_en_count[1] *= 1000000 / SUMPLING_TIME_US;
  pc.printf("Anglular Velocity(°/s):%d\n", d_en_count[1]);//d_en_countは現在値
}

//エンコーダ
void encoder(void) {
  if(enB_a == 1) en_count[0] += 1;
  else en_count[0] -= 1;

  if(enB_b == 1) en_count[1] += 1;
  else en_count[1] -= 1;
}

void limit(double &a){
  if(a>=PERIOD*0.8) a=PERIOD*0.8;
  else if(a<=-1*PERIOD*0.8) a=-1*PERIOD*0.8;
}

//PID
void PID(void) {
  double p[2], i[2], d[2];
  diff[0] = diff[1];
  diff[1] = target_speed - (double)d_en_count[0];
  //diff[1]は目標速度と現在の速度の差分
  integral[0] += (diff[1] + diff[0]) / 2.0 * (SUMPLING_TIME_US/(double)1000000);
  p[0] = Kp * diff[1];
  i[0] = Ki * integral[0];
  d[0] = (diff[0] - diff[1]) / (SUMPLING_TIME_US/(double)1000000);
  v[0] += p[0] + i[0] + d[0];//エンコーダーの差分から車輪の差分に変換し、足す
  limit(v[0]);

  diff[2] = diff[3];
  diff[3] = target_speed - (double)d_en_count[1];
  //diff[1]は目標速度と現在の速度の差分
  integral[1] += (diff[3] + diff[2]) / 2.0 * (SUMPLING_TIME_US/(double)1000000);
  p[1] = Kp * diff[3];
  i[1] = Ki * integral[1];
  d[1] = (diff[2] - diff[3]) / (SUMPLING_TIME_US/(double)1000000);
  v[1] += p[1] + i[1] + d[1];//エンコーダーの差分から車輪の差分に変換し、足す
  limit(v[1]);
  //vの値が増え(減り)すぎないようにセッターを設ける
  //pc.printf("%d\n",d_en_count);
}

void motor_run() {
  PID();
  if(v[0] > 0) {
    PHASE_a = 1;
    PWM_a.write(v[0]);
  }
  else {
    PHASE_a = 0;
    PWM_a.write(-v[0]);
  }
  if(v[1] > 0) {
    PHASE_b = 1;
    PWM_b.write(v[1]);
  }
  else {
    PHASE_b = 0;
    PWM_b.write(-v[1]);
  }
}

int main() {
  while(switch1);
  ticker1.attach_us(&encoder, SUMPLING_TIME_US);
  ticker2.attach_us(&speed_calc, SUMPLING_TIME_US);
  while(1) motor_run();
}
