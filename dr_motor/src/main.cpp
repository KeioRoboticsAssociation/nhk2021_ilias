#include <mbed.h>

Serial pc(USBTX, USBRX, 115200);
InterruptIn switch1(USER_BUTTON);
DigitalOut led(LED1);

int main() {
  while(1) {
    led = true;
    wait_ms(1000);
    led = false;
    wait_ms(1000);
  }
}