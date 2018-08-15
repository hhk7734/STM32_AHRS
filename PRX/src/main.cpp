#include <Arduino.h>
#include <Wire.h>
#include "RF24_STM.h"
#include <math.h>

#define BOARD_NRF24_CE_PIN_01 PA8
#define BOARD_NRF24_CE_PIN_02 PA15
#define BOARD_NRF24_CE_PIN_03 PB1
#define BOARD_NRF24_CE_PIN_04 PB4
#define BOARD_NRF24_CE_PIN_05 PB5

#define BOARD_NRF24_CSN_PIN_01 PB12
#define BOARD_NRF24_CSN_PIN_02 PB6
#define BOARD_NRF24_CSN_PIN_03 PB0
#define BOARD_NRF24_CSN_PIN_04 PB8
#define BOARD_NRF24_CSN_PIN_05 PB9

#define NRF_ADDRESS_01 0xF0F0F0F001ULL
#define NRF_ADDRESS_02 0xF0F0F0F002ULL
#define NRF_ADDRESS_03 0xF0F0F0F003ULL
#define NRF_ADDRESS_04 0xF0F0F0F004ULL
#define NRF_ADDRESS_05 0xF0F0F0F005ULL

#define NRF_CHANNEL_01 66
#define NRF_CHANNEL_02 70
#define NRF_CHANNEL_03 74
#define NRF_CHANNEL_04 78
#define NRF_CHANNEL_05 82

RF24 *radio_01 = new RF24(BOARD_NRF24_CE_PIN_01, BOARD_NRF24_CSN_PIN_01);
RF24 *radio_02 = new RF24(BOARD_NRF24_CE_PIN_02, BOARD_NRF24_CSN_PIN_02);
RF24 *radio_03 = new RF24(BOARD_NRF24_CE_PIN_03, BOARD_NRF24_CSN_PIN_03);

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
    Serial1.println("\n\rRF24");
    
    radio_01->begin();
    radio_01->setChannel(NRF_CHANNEL_01);
    radio_01->openReadingPipe(1,NRF_ADDRESS_01);
    radio_01->startListening();
    delay(1);

    radio_02->begin();
    radio_02->setChannel(NRF_CHANNEL_02);
    radio_02->openReadingPipe(1,NRF_ADDRESS_02);
    radio_02->startListening();
    delay(1);

    radio_03->begin();
    radio_03->setChannel(NRF_CHANNEL_03);
    radio_03->openReadingPipe(1,NRF_ADDRESS_03);
    radio_03->startListening();
    delay(1);
}

void loop()
{
    int16_t _data[6];

    if(radio_01->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_01->read( _data, 12 );
            delayMicroseconds(200);
        }
        if(_data[5] == 11)
        {
            print_fin(_data);
        }
        else if(_data[5] > 0)
        {
            print_ang(_data);
        }
    }
    delayMicroseconds(200);

    if(radio_02->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_02->read( _data, 12 );
            delayMicroseconds(200);
        }
        if(_data[5] == 11)
        {
            print_fin(_data);
        }
        else if(_data[5] > 0)
        {
            print_ang(_data);
        }
    }
    delayMicroseconds(200);

    if(radio_03->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_03->read( _data, 12 );
            delayMicroseconds(200);
        }
        if(_data[5] == 11)
        {
            print_fin(_data);
        }
        else if(_data[5] > 0)
        {
            print_ang(_data);
        }
    }
    delayMicroseconds(200);
}