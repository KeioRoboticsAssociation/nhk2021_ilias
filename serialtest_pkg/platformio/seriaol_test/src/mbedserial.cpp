// mbedserial.cpp
// serial communication : mbed <-> ROS

#include "mbedserial.h"

/***********************init***********************/
void _nullfunc() { ; };

Mbedserial::Mbedserial(Serial &pc) : rospc(pc)
{
    msg_buf = new char[256];
    bufsize = 0;
    endmsg = '\n';
    floatarraysize = 0;
    intarraysize = 0;
    chararraysize = 0;
    for (int i = 0; i < 3; i++)
    {
        pfunccb[i] = _nullfunc;
    }
    rospc.attach(callback(this, &Mbedserial::rcv_callback), Serial::RxIrq);
}

/**********************receive**********************/

void Mbedserial::rcv_callback()
{
    msg_buf[bufsize] = rospc.getc();
    if (msg_buf[bufsize] == endmsg)
    {
        switch (msg_buf[0])
        {
        case 'f':
            floatarraysize = *(int *)(&msg_buf[1]);
            //memcpy(&floatarraysize, &msg_buf[1], 4);
            if (bufsize == floatarraysize * 4 + 5)
            {
                for (int i = 0; i < floatarraysize; i++)
                {
                    getfloat[i] = *(float *)(&msg_buf[i * 4 + 5]);
                    //memcpy(&getfloat[i], &msg_buf[i * 4 + 5], 4);
                }
                pfunccb[0]();
            }
            break;
        case 'i':
            intarraysize = *(int *)(&msg_buf[1]);
            //memcpy(&intarraysize, &msg_buf[1], 4);
            if (bufsize == intarraysize * 4 + 5)
            {
                for (int i = 0; i < intarraysize; i++)
                {
                    getint[i] = *(int *)(&msg_buf[i * 4 + 5]);
                    //memcpy(&getint[i], &msg_buf[i * 4 + 5], 4);
                }
                pfunccb[1]();
            }
            break;
        case 'c':
            chararraysize = *(int *)(&msg_buf[1]);
            //memcpy(&chararraysize, &msg_buf[1], 4);
            if (bufsize == chararraysize + 5)
            {
                memcpy(&getchar[0], &msg_buf[5], chararraysize);
                pfunccb[2]();
            }
            break;
        }
        delete[] msg_buf;
        bufsize = 0;
        msg_buf = new char[256];
    }
    else
    {
        bufsize++;
    }
}

/***********************send***********************/

int pub_wait_time = 8; //ms

void Mbedserial::float_write(float array[], int arraysize)
{
    // send data type
    char msg_type = 'f';
    rospc.putc(msg_type);

    // send array size
    char arraysize_c[4];
    *(int *)arraysize_c = arraysize;
    //memcpy(arraysize_c, &arraysize, 4);
    for (int i = 0; i < 4; i++)
    {
        rospc.putc(arraysize_c[i]);
    }

    // send float data
    char array_c[4];
    for (int i = 0; i < arraysize; i++)
    {
        *(float *)array_c = array[i];
        //memcpy(array_c, &array[i], 4);
        for (int j = 0; j < 4; j++)
        {
            rospc.putc(array_c[j]);
        }
    }

    // send end message
    rospc.putc(endmsg);
    //wait_ms(pub_wait_time);
}

void Mbedserial::int_write(int *array, int arraysize)
{
    // send data type
    char msg_type = 'i';
    rospc.putc(msg_type);

    // send array size
    char arraysize_c[4];
    *(int *)arraysize_c = arraysize;
    //memcpy(arraysize_c, &arraysize, 4);
    for (int i = 0; i < 4; i++)
    {
        rospc.putc(arraysize_c[i]);
    }

    // send int data
    char array_c[4];
    for (int i = 0; i < arraysize; i++)
    {
        *(int *)array_c = array[i];
        //memcpy(array_c, &array[i], 4);
        for (int j = 0; j < 4; j++)
        {
            rospc.putc(array_c[j]);
        }
    }

    // send end message
    rospc.putc(endmsg);
    //wait_ms(pub_wait_time);
}

void Mbedserial::char_write(char *array, int arraysize)
{
    // send data type
    char msg_type = 'c';
    rospc.putc(msg_type);

    // send array size
    char arraysize_c[4];
    *(int *)arraysize_c = arraysize;
    //memcpy(arraysize_c, &arraysize, 4);
    for (int i = 0; i < 4; i++)
    {
        rospc.putc(arraysize_c[i]);
    }

    // send char data
    for (int i = 0; i < arraysize; i++)
    {
        rospc.putc(array[i]);
    }

    // send end message
    rospc.putc(endmsg);
    //wait_ms(pub_wait_time);
}