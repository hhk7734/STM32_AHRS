#include <Arduino.h>
#include <Wire.h>
#include "RF24_STM.h"
#include <math.h>

// STM32F103C8T6
//                     VB      3.3V
//                     PC13    GND          Li-Po (-)
//                     PC14    5V           Li-Po (+)
//                     PC15    PB9
//                     PA0     PB8
//                     PA1     PB7/SDA1     MPU_SDA, AT24C_SDA
//                 TX2/PA2     PB6/SCL1     MPU_SCL, AT24C_SCL
//                 RX2/PA3     PB5
//                     PA4     PB4
//                     PA5     PB3
//                     PA6     PA15
//                     PA7     PA12
//                     PB0     PA11
//                     PB1     PA10/RX1     Xbee_DOUT
//                     PB10    PA9 /TX1     Xbee_DIN
//                     PB11    PA8          NRF_CE      3
//                     R       PB15/MOSI2   NRF_MOSI    6
//                     3.3V    PB14/MISO2   NRF_MISO    7
//                     GND     PB13/SCK2    NRF_SCK     5
//                     GND     PB12/NSS2    NRF_CSN     4

#define BOARD_NRF24_CE_PIN_01 PA8
#define BOARD_NRF24_CE_PIN_02 PB1
#define BOARD_NRF24_CE_PIN_03 PA3
#define BOARD_NRF24_CE_PIN_04 PB6
#define BOARD_NRF24_CE_PIN_05 PB5

#define BOARD_NRF24_CSN_PIN_01 PB12
#define BOARD_NRF24_CSN_PIN_02 PB0
#define BOARD_NRF24_CSN_PIN_03 PA2
#define BOARD_NRF24_CSN_PIN_04 PB7
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

#define NRF_READ_DELAY 100
#define NRF_READ_INTERVAL 100

// #define ANG_TEST
#define CALCULATE_ARM

#define QX 0
#define QY 1
#define QZ 2
#define QW 3

RF24 *radio_01 = new RF24(BOARD_NRF24_CE_PIN_01, BOARD_NRF24_CSN_PIN_01);
RF24 *radio_02 = new RF24(BOARD_NRF24_CE_PIN_02, BOARD_NRF24_CSN_PIN_02);
RF24 *radio_03 = new RF24(BOARD_NRF24_CE_PIN_03, BOARD_NRF24_CSN_PIN_03);
RF24 *radio_04 = new RF24(BOARD_NRF24_CE_PIN_04, BOARD_NRF24_CSN_PIN_04);

int16_t _data[5][6] = {0};
double _q[4][4];
double c_quat[13];

uint16_t step = 0;

void print_ang(int16_t *data)
{
#ifdef ANG_TEST
    float ang[3], q[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
        q[i] = float(data[i]) / 20000.0;
    }
    ang[0] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[0] * q[0] + q[1] * q[1])) * 57.2975;
    ang[1] = asin(2 * (q[1] * q[3] - q[0] * q[2])) * 57.2975;
    ang[2] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * 57.2975;

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
#else
    Serial1.print(data[0]);
    Serial1.print(',');
    Serial1.print(data[1]);
    Serial1.print(',');
    Serial1.print(data[2]);
    Serial1.print(',');
    Serial1.print(data[3]);
    Serial1.print(',');
    // Serial1.print(data[4]);
    // Serial1.print(',');
    Serial1.println(data[5]);
#endif // ANG_TEST
}

void print_fin(int16_t *data)
{
    Serial1.print(data[0] >> 6);
    Serial1.print(',');
    Serial1.print(data[1] >> 6);
    Serial1.print(',');
    Serial1.print(data[2] >> 6);
    Serial1.print(',');
    Serial1.print(data[3] >> 6);
    Serial1.print(',');
    Serial1.print(data[4] >> 6);
    Serial1.print(',');
    Serial1.println(data[5]);
}

void read_nrf(void)
{
    if (radio_01->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_01->read(_data[0], 12);
            delayMicroseconds(NRF_READ_DELAY);
        }
#ifndef CALCULATE_ARM
        print_ang(_data[0]);
#endif // CALCULATE_ARM
    }
    delayMicroseconds(NRF_READ_INTERVAL);

    if (radio_02->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_02->read(_data[1], 12);
            delayMicroseconds(NRF_READ_DELAY);
        }
#ifndef CALCULATE_ARM
        print_ang(_data[1]);
#endif // CALCULATE_ARM
    }
    delayMicroseconds(NRF_READ_INTERVAL);

    if (radio_03->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_03->read(_data[2], 12);
            delayMicroseconds(NRF_READ_DELAY);
        }
#ifndef CALCULATE_ARM
        print_ang(_data[2]);
#endif // CALCULATE_ARM
    }
    delayMicroseconds(NRF_READ_INTERVAL);

    if (radio_04->available())
    {
        bool done = false;
        while (!done)
        {
            done = radio_04->read(_data[3], 12);
            delayMicroseconds(NRF_READ_DELAY);
        }

        if (_data[3][5] == 11)
        {
            for (uint8_t i = 0; i < 6; ++i)
            {
                _data[4][i] = _data[3][i];
            }
#ifndef CALCULATE_ARM
            print_fin(_data[4]);
#endif // CALCULATE_ARM
        }
#ifndef CALCULATE_ARM
        else
        {
            print_ang(_data[3]);
        }
#endif // CALCULATE_ARM
    }
    delayMicroseconds(NRF_READ_INTERVAL);
}

void setup()
{
    Serial1.begin(115200);
    Serial1.println("\n\rRF24");

    radio_01->begin();
    radio_01->setChannel(NRF_CHANNEL_01);
    radio_01->openReadingPipe(1, NRF_ADDRESS_01);
    radio_01->startListening();
    delay(1);

    radio_02->begin();
    radio_02->setChannel(NRF_CHANNEL_02);
    radio_02->openReadingPipe(1, NRF_ADDRESS_02);
    radio_02->startListening();
    delay(1);

    radio_03->begin();
    radio_03->setChannel(NRF_CHANNEL_03);
    radio_03->openReadingPipe(1, NRF_ADDRESS_03);
    radio_03->startListening();
    delay(1);

    radio_04->begin();
    radio_04->setChannel(NRF_CHANNEL_04);
    radio_04->openReadingPipe(1, NRF_ADDRESS_04);
    radio_04->startListening();
    delay(1);
}

void loop()
{
    read_nrf();
#ifdef CALCULATE_ARM
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            _q[i][j] = _data[i][j];
            _q[i][j] /= 20000.0;
        }
    }
    switch (step)
    {
    case 0:
        c_quat[0] = 2 * _q[0][QX] * _q[0][QY] - 2 * _q[0][QW] * _q[0][QZ];
        c_quat[1] = 2 * _q[0][QW] * _q[0][QY] + 2 * _q[0][QX] * _q[0][QZ];

        Serial1.print((int16_t)(atan2(c_quat[0], c_quat[1]) * 10000));
        Serial1.print(',');

        ++step;
        break;
    case 6:

        c_quat[2] = -(2 * _q[0][QY] * _q[0][QY] + 2 * _q[0][QZ] * _q[0][QZ] - 1) * (2 * _q[1][QY] * _q[1][QY] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[0][QW] * _q[0][QY] - 2 * _q[0][QX] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QY] - 2 * _q[1][QX] * _q[1][QZ]) - (2 * _q[0][QW] * _q[0][QZ] + 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QW] * _q[1][QZ] + 2 * _q[1][QX] * _q[1][QY]);

        c_quat[3] = (2 * _q[0][QW] * _q[0][QY] + 2 * _q[0][QX] * _q[0][QZ]) * (2 * _q[1][QY] * _q[1][QY] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[1][QW] * _q[1][QY] - 2 * _q[1][QX] * _q[1][QZ]) * (2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QY] - 1) + (2 * _q[0][QW] * _q[0][QX] - 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QZ] + 2 * _q[1][QX] * _q[1][QY]);

        Serial1.print((int16_t)(atan2(c_quat[2], c_quat[3]) * 10000));
        Serial1.print(',');

        ++step;
        break;

    case 12:
        c_quat[4] = (2 * _q[0][QW] * _q[0][QZ] - 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QY] * _q[1][QY] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[1][QW] * _q[1][QZ] + 2 * _q[1][QX] * _q[1][QY]) * (2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QZ] * _q[0][QZ] - 1) - (2 * _q[0][QW] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QY] - 2 * _q[1][QX] * _q[1][QZ]);

        Serial1.print((int16_t)(acos(c_quat[4]) * 10000));
        Serial1.print(',');

        ++step;
        break;

    case 18:

        c_quat[5] = -(2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QZ] * _q[0][QZ] - 1) * (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[0][QW] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QZ]) - (2 * _q[0][QW] * _q[0][QZ] - 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QW] * _q[1][QZ] - 2 * _q[1][QX] * _q[1][QY]);

        c_quat[6] = (2 * _q[1][QW] * _q[1][QX] - 2 * _q[1][QY] * _q[1][QZ]) * (2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QZ] * _q[0][QZ] - 1) - (2 * _q[0][QW] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QY] - 1) - (2 * _q[0][QW] * _q[0][QZ] - 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QW] * _q[1][QY] + 2 * _q[1][QX] * _q[1][QZ]);

        Serial1.print((int16_t)(atan2(c_quat[5], c_quat[6]) * 10000));
        Serial1.print(',');

        ++step;
        break;

    case 24:
        c_quat[7] = (2 * _q[2][QW] * _q[2][QY] - 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QY] - 1) - (2 * _q[1][QW] * _q[1][QY] + 2 * _q[1][QX] * _q[1][QZ]) * (2 * _q[2][QY] * _q[2][QY] + 2 * _q[2][QZ] * _q[2][QZ] - 1) - (2 * _q[1][QW] * _q[1][QX] - 2 * _q[1][QY] * _q[1][QZ]) * (2 * _q[2][QW] * _q[2][QZ] + 2 * _q[2][QX] * _q[2][QY]);

        c_quat[8] = (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QY] - 1) * (2 * _q[2][QX] * _q[2][QX] + 2 * _q[2][QY] * _q[2][QY] - 1) + (2 * _q[1][QW] * _q[1][QX] - 2 * _q[1][QY] * _q[1][QZ]) * (2 * _q[2][QW] * _q[2][QX] - 2 * _q[2][QY] * _q[2][QZ]) + (2 * _q[1][QW] * _q[1][QY] + 2 * _q[1][QX] * _q[1][QZ]) * (2 * _q[2][QW] * _q[2][QY] + 2 * _q[2][QX] * _q[2][QZ]);

        Serial1.print((int16_t)(atan2(c_quat[7], c_quat[8]) * 10000));
        Serial1.print(',');

        ++step;
        break;

    case 30:
        c_quat[9] = (2 * _q[2][QW] * _q[2][QX] - 2 * _q[2][QY] * _q[2][QZ]) * (2 * _q[3][QX] * _q[3][QX] + 2 * _q[3][QZ] * _q[3][QZ] - 1) - (2 * _q[3][QW] * _q[3][QX] + 2 * _q[3][QY] * _q[3][QZ]) * (2 * _q[2][QX] * _q[2][QX] + 2 * _q[2][QY] * _q[2][QY] - 1) - (2 * _q[2][QW] * _q[2][QY] + 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[3][QW] * _q[3][QZ] - 2 * _q[3][QX] * _q[3][QY]);

        c_quat[10] = (2 * _q[2][QX] * _q[2][QX] + 2 * _q[2][QZ] * _q[2][QZ] - 1) * (2 * _q[3][QX] * _q[3][QX] + 2 * _q[3][QZ] * _q[3][QZ] - 1) + (2 * _q[2][QW] * _q[2][QX] + 2 * _q[2][QY] * _q[2][QZ]) * (2 * _q[3][QW] * _q[3][QX] + 2 * _q[3][QY] * _q[3][QZ]) + (2 * _q[2][QW] * _q[2][QZ] - 2 * _q[2][QX] * _q[2][QY]) * (2 * _q[3][QW] * _q[3][QZ] - 2 * _q[3][QX] * _q[3][QY]);

        Serial1.print((int16_t)(atan2(c_quat[9], c_quat[10]) * 10000));
        Serial1.print(',');

        ++step;
        break;

    case 36:
        c_quat[11] = (2 * _q[2][QY] * _q[2][QY] + 2 * _q[2][QZ] * _q[2][QZ] - 1) * (2 * _q[3][QY] * _q[3][QY] + 2 * _q[3][QZ] * _q[3][QZ] - 1) + (2 * _q[2][QW] * _q[2][QY] - 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[3][QW] * _q[3][QY] - 2 * _q[3][QX] * _q[3][QZ]) + (2 * _q[2][QW] * _q[2][QZ] + 2 * _q[2][QX] * _q[2][QY]) * (2 * _q[3][QW] * _q[3][QZ] + 2 * _q[3][QX] * _q[3][QY]);

        c_quat[12] = (2 * _q[3][QW] * _q[3][QY] + 2 * _q[3][QX] * _q[3][QZ]) * (2 * _q[2][QY] * _q[2][QY] + 2 * _q[2][QZ] * _q[2][QZ] - 1) - (2 * _q[2][QW] * _q[2][QY] - 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[3][QX] * _q[3][QX] + 2 * _q[3][QY] * _q[3][QY] - 1) + (2 * _q[2][QW] * _q[2][QZ] + 2 * _q[2][QX] * _q[2][QY]) * (2 * _q[3][QW] * _q[3][QX] - 2 * _q[3][QY] * _q[3][QZ]);

        Serial1.print((int16_t)(atan2(c_quat[11], c_quat[12]) * 10000));
        Serial1.print(',');

        ++step;
        break;
    case 42:
        for (uint8_t i = 0; i < 4; ++i)
        {
            Serial1.print(_data[4][i] >> 6);
            Serial1.print(',');
        }
        Serial1.println(_data[4][4] >> 6);
        step = 0;
        break;

    default:
        ++step;
        break;
    }
#endif // CALCULATE_ARM
}