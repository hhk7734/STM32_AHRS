#include <Arduino.h>
#include <Wire.h>
#include "RF24_STM.h"
#include <math.h>

#define BOARD_NRF24_CE_PIN_01 PA8
#define BOARD_NRF24_CE_PIN_02 PA15
#define BOARD_NRF24_CE_PIN_03 PB3
#define BOARD_NRF24_CE_PIN_04 PB4
#define BOARD_NRF24_CE_PIN_05 PB5

#define BOARD_NRF24_CSN_PIN_01 PB12
#define BOARD_NRF24_CSN_PIN_02 PB6
#define BOARD_NRF24_CSN_PIN_03 PB7
#define BOARD_NRF24_CSN_PIN_04 PB8
#define BOARD_NRF24_CSN_PIN_05 PB9

#define NRF_ADDRESS_01 0xF0F0F0F0A1ULL
#define NRF_ADDRESS_02 0xF0F0F0F0B2ULL
#define NRF_ADDRESS_03 0xF0F0F0F0C3ULL
#define NRF_ADDRESS_04 0xF0F0F0F0D4ULL
#define NRF_ADDRESS_05 0xF0F0F0F0E5ULL

#define NRF_CHANNEL_01 30
#define NRF_CHANNEL_02 35
#define NRF_CHANNEL_03 40
#define NRF_CHANNEL_04 45
#define NRF_CHANNEL_05 50

RF24 radio[2]={ RF24(BOARD_NRF24_CE_PIN_01 ,BOARD_NRF24_CSN_PIN_01),
                RF24(BOARD_NRF24_CE_PIN_02 ,BOARD_NRF24_CSN_PIN_02) };

void print_ang(int16_t *data)
{
    float ang[3], q[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
        q[i] = float(data[i])/10000.0;
    }

    ang[0] = atan2( 2*(q[1]*q[2]+q[0]*q[3]) , 1-2*(q[0]*q[0]+q[1]*q[1]) ) * 57.2975;
    ang[1] = asin ( 2*(q[1]*q[3]-q[0]*q[2]) ) * 57.2975;
    ang[2] = atan2( 2*(q[0]*q[1]+q[2]*q[3]) , 1-2*(q[1]*q[1]+q[2]*q[2]) ) * 57.2975;

    Serial1.print((int16_t)ang[0]);
    Serial1.print(',');
    Serial1.print((int16_t)ang[1]);
    Serial1.print(',');
    Serial1.print((int16_t)ang[2]);
    Serial1.print(',');
    // Serial1.print(data[0]);
    // Serial1.print(',');
    // Serial1.print(data[1]);
    // Serial1.print(',');
    // Serial1.print(data[2]);
    // Serial1.print(',');
    // Serial1.print(data[3]);
    // Serial1.print(',');
    Serial1.print(data[4]);
    Serial1.print(',');
    Serial1.println(data[5]);
}

void print_fin(int16_t *data)
{
    Serial1.print(data[0]>>7);
    Serial1.print(',');
    Serial1.print(data[1]>>7);
    Serial1.print(',');
    Serial1.print(data[2]>>7);
    Serial1.print(',');
    Serial1.print(data[3]>>7);
    Serial1.print(',');
    Serial1.print(data[4]>>7);
    Serial1.print(',');
    Serial1.println(data[5]);
}

void setup()
{
    Serial1.begin(115200);
    Serial1.println("\n\rRF24/examples/GettingStarted/");

    radio[0].begin();
    radio[0].setChannel(NRF_CHANNEL_01);
    radio[0].openReadingPipe(1,NRF_ADDRESS_01);
    radio[0].startListening();
    
    radio[1].begin();
    radio[1].setChannel(NRF_CHANNEL_02);
    radio[1].openReadingPipe(1,NRF_ADDRESS_02);
    radio[1].startListening();
}

void loop()
{
    int16_t _data[6];
    for (uint8_t i = 0; i < 2; ++i)
    {
        if(radio[i].available())
        {
            bool done = false;
            while (!done)
            {
                done = radio[i].read( _data, 12 );
                delayMicroseconds(200);
            }
            if(_data[5] == 11)
            {
                print_fin(_data);
            }
            else
            {
                print_ang(_data);
            }
        }
    }
}