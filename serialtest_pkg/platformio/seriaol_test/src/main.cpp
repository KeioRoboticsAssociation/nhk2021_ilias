#include "mbedserial.h"

Serial pc(USBTX, USBRX, 115200);
Mbedserial Ms(pc);

float v, w;

int main()
{
  while (1)
  {
    v = Ms.getfloat[0];
    w = Ms.getfloat[1];

    pc.printf("v: %f, w: %f\n", v, w);

    wait(0.1);
  }
}

