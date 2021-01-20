#include <mbed.h>
#include "motordriver.h"
#include "PID.h"
#include "controller.h"
#include "main.h"
#include "mbedserial.h"

Serial pc(USBTX, USBRX, 115200);
InterruptIn switch1(USER_BUTTON); //青ボタン
CAN can1(PA_11, PA_12, 500000);
Mbedserial Ms(pc);
Controller controller(can1, 0x334);

int main() {
  while(switch1);
}