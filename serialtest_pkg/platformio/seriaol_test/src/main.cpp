#include "mbedserial.h"
#include <chrono>

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);
Ticker ticker;
#define SUMPLING_TIME_US 100000 //タイマー割り込み周期 kHzオーダーつまり1000が理想

float v, w;
int lost_time_threshold = 1500;
std::chrono::system_clock::time_point last_rcv_time;


bool isRecieved() {
    auto current_time = std::chrono::system_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 current_time - last_rcv_time)
                                 .count();

    if (elapsed < lost_time_threshold) {
        return true;
    } else {
        //pc.printf("%f\n", elapsed);
        return false;
    }
}

void CallBack(){// Callbackで取得しないと誤差が蓄積してタイミングがずれて死ぬ

    v = Ms.getfloat[0];
    w = Ms.getfloat[1];
    
    last_rcv_time = std::chrono::system_clock::now();
}

void Timer_interrupt()
{
    if(!isRecieved()){
      v = 0;
      w = 0;
    }
    pc.printf("v: %f, w: %f\n", v, w);
}

DigitalOut myled(LED1);

int main()
{
  myled = 1;
  last_rcv_time = std::chrono::system_clock::now();
  Ms.float_attach(CallBack);
  ticker.attach_us(&Timer_interrupt, SUMPLING_TIME_US); //タイマー割り込みの設定(speed_calc()を SUMPLING_TIME_US ごとに実行)

  while (1)
  {
    myled = !myled;
    wait(0.1);
  }
}

