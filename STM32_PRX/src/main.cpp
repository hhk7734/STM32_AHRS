#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "RF24_STM.h"

RF24 *radio_01 = new RF24(BOARD_NRF24_CE_PIN_01, BOARD_NRF24_CSN_PIN_01);
RF24 *radio_02 = new RF24(BOARD_NRF24_CE_PIN_02, BOARD_NRF24_CSN_PIN_02);
RF24 *radio_03 = new RF24(BOARD_NRF24_CE_PIN_03, BOARD_NRF24_CSN_PIN_03);
RF24 *radio_04 = new RF24(BOARD_NRF24_CE_PIN_04, BOARD_NRF24_CSN_PIN_04);

int16_t _data[5][6] = {0};

#ifdef CALCULATE_ARM
int16_t step = 0;
int16_t arm_angle[7] = {0};
#endif // CALCULATE_ARM

void print_ang(int16_t *data);
void print_fin(int16_t *data);
void print_arm_ang(void);
void read_nrf(void);

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
    print_arm_ang();
#endif // CALCULATE_ARM
}

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
        int16_t _temp[6];
        bool done = false;
        while (!done)
        {
            done = radio_04->read(_temp, 12);
            delayMicroseconds(NRF_READ_DELAY);
        }

        if (_temp[5] == 11)
        {
            for (uint8_t i = 0; i < 6; ++i)
            {
                _data[4][i] = _temp[i];
            }
#ifndef CALCULATE_ARM
            print_fin(_data[4]);
#endif // CALCULATE_ARM
        }
        else
        {
            for (uint8_t i = 0; i < 6; ++i)
            {
                _data[3][i] = _temp[i];
            }
#ifndef CALCULATE_ARM
            print_ang(_data[3]);
#endif // CALCULATE_ARM
        }
    }
    delayMicroseconds(NRF_READ_INTERVAL);
}

#ifdef CALCULATE_ARM

void ang_constrain(int16_t new_ang, int16_t *old_ang)
{
    if (new_ang - *old_ang > 3141)
    {
        new_ang -= 6282;
        *old_ang = constrain(new_ang, *old_ang - MAX_ERROR, *old_ang + MAX_ERROR);
        if (*old_ang < -3141)
            *old_ang += 6282;
    }
    else if (new_ang - *old_ang < -3141)
    {
        new_ang += 6282;
        *old_ang = constrain(new_ang, *old_ang - MAX_ERROR, *old_ang + MAX_ERROR);
        if (*old_ang > 3141)
            *old_ang -= 6282;
    }
    else
        *old_ang = constrain(new_ang, *old_ang - MAX_ERROR, *old_ang + MAX_ERROR);
}

void print_arm_ang(void)
{
    double _q[4][4];
    double c_quat[13];
    int16_t ang_temp;
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

        ang_temp = atan2(c_quat[0], c_quat[1]) * 1000;
        ang_constrain(ang_temp, &arm_angle[0]);

        Serial1.print(arm_angle[0]);
        Serial1.print(',');

        ++step;
        break;

    case 6:

        c_quat[2] = -(2 * _q[0][QY] * _q[0][QY] + 2 * _q[0][QZ] * _q[0][QZ] - 1) * (2 * _q[1][QY] * _q[1][QY] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[0][QW] * _q[0][QY] - 2 * _q[0][QX] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QY] - 2 * _q[1][QX] * _q[1][QZ]) - (2 * _q[0][QW] * _q[0][QZ] + 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QW] * _q[1][QZ] + 2 * _q[1][QX] * _q[1][QY]);

        c_quat[3] = (2 * _q[0][QW] * _q[0][QY] + 2 * _q[0][QX] * _q[0][QZ]) * (2 * _q[1][QY] * _q[1][QY] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[1][QW] * _q[1][QY] - 2 * _q[1][QX] * _q[1][QZ]) * (2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QY] - 1) + (2 * _q[0][QW] * _q[0][QX] - 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QZ] + 2 * _q[1][QX] * _q[1][QY]);

        ang_temp = atan2(c_quat[2], c_quat[3]) * 1000;
        ang_constrain(ang_temp, &arm_angle[1]);

        Serial1.print(arm_angle[1]);
        Serial1.print(',');

        ++step;
        break;

    case 12:
        c_quat[4] = (2 * _q[0][QW] * _q[0][QZ] - 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QY] * _q[1][QY] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[1][QW] * _q[1][QZ] + 2 * _q[1][QX] * _q[1][QY]) * (2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QZ] * _q[0][QZ] - 1) - (2 * _q[0][QW] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QY] - 2 * _q[1][QX] * _q[1][QZ]);

        ang_temp = acos(c_quat[4]) * 1000;
        arm_angle[2] = constrain(ang_temp, arm_angle[2] - MAX_ERROR, arm_angle[2] + MAX_ERROR);

        Serial1.print(arm_angle[2]);
        Serial1.print(',');

        ++step;
        break;

    case 18:

        c_quat[5] = -(2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QZ] * _q[0][QZ] - 1) * (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QZ] * _q[1][QZ] - 1) - (2 * _q[0][QW] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QW] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QZ]) - (2 * _q[0][QW] * _q[0][QZ] - 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QW] * _q[1][QZ] - 2 * _q[1][QX] * _q[1][QY]);

        c_quat[6] = (2 * _q[1][QW] * _q[1][QX] - 2 * _q[1][QY] * _q[1][QZ]) * (2 * _q[0][QX] * _q[0][QX] + 2 * _q[0][QZ] * _q[0][QZ] - 1) - (2 * _q[0][QW] * _q[0][QX] + 2 * _q[0][QY] * _q[0][QZ]) * (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QY] - 1) - (2 * _q[0][QW] * _q[0][QZ] - 2 * _q[0][QX] * _q[0][QY]) * (2 * _q[1][QW] * _q[1][QY] + 2 * _q[1][QX] * _q[1][QZ]);

        ang_temp = atan2(c_quat[5], c_quat[6]) * 1000;
        ang_constrain(ang_temp, &arm_angle[3]);

        Serial1.print(arm_angle[3]);
        Serial1.print(',');

        ++step;
        break;

    case 24:
        c_quat[7] = (2 * _q[2][QW] * _q[2][QY] - 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QY] - 1) - (2 * _q[1][QW] * _q[1][QY] + 2 * _q[1][QX] * _q[1][QZ]) * (2 * _q[2][QY] * _q[2][QY] + 2 * _q[2][QZ] * _q[2][QZ] - 1) - (2 * _q[1][QW] * _q[1][QX] - 2 * _q[1][QY] * _q[1][QZ]) * (2 * _q[2][QW] * _q[2][QZ] + 2 * _q[2][QX] * _q[2][QY]);

        c_quat[8] = (2 * _q[1][QX] * _q[1][QX] + 2 * _q[1][QY] * _q[1][QY] - 1) * (2 * _q[2][QX] * _q[2][QX] + 2 * _q[2][QY] * _q[2][QY] - 1) + (2 * _q[1][QW] * _q[1][QX] - 2 * _q[1][QY] * _q[1][QZ]) * (2 * _q[2][QW] * _q[2][QX] - 2 * _q[2][QY] * _q[2][QZ]) + (2 * _q[1][QW] * _q[1][QY] + 2 * _q[1][QX] * _q[1][QZ]) * (2 * _q[2][QW] * _q[2][QY] + 2 * _q[2][QX] * _q[2][QZ]);

        ang_temp = atan2(c_quat[7], c_quat[8]) * 1000;
        ang_constrain(ang_temp, &arm_angle[4]);

        Serial1.print(arm_angle[4]);
        Serial1.print(',');

        ++step;
        break;

    case 30:
        c_quat[9] = (2 * _q[2][QW] * _q[2][QX] - 2 * _q[2][QY] * _q[2][QZ]) * (2 * _q[3][QX] * _q[3][QX] + 2 * _q[3][QZ] * _q[3][QZ] - 1) - (2 * _q[3][QW] * _q[3][QX] + 2 * _q[3][QY] * _q[3][QZ]) * (2 * _q[2][QX] * _q[2][QX] + 2 * _q[2][QY] * _q[2][QY] - 1) - (2 * _q[2][QW] * _q[2][QY] + 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[3][QW] * _q[3][QZ] - 2 * _q[3][QX] * _q[3][QY]);

        c_quat[10] = (2 * _q[2][QX] * _q[2][QX] + 2 * _q[2][QZ] * _q[2][QZ] - 1) * (2 * _q[3][QX] * _q[3][QX] + 2 * _q[3][QZ] * _q[3][QZ] - 1) + (2 * _q[2][QW] * _q[2][QX] + 2 * _q[2][QY] * _q[2][QZ]) * (2 * _q[3][QW] * _q[3][QX] + 2 * _q[3][QY] * _q[3][QZ]) + (2 * _q[2][QW] * _q[2][QZ] - 2 * _q[2][QX] * _q[2][QY]) * (2 * _q[3][QW] * _q[3][QZ] - 2 * _q[3][QX] * _q[3][QY]);

        ang_temp = atan2(c_quat[9], c_quat[10]) * 1000;
        ang_constrain(ang_temp, &arm_angle[5]);

        Serial1.print(arm_angle[5]);
        Serial1.print(',');

        ++step;
        break;

    case 36:
        c_quat[11] = (2 * _q[2][QY] * _q[2][QY] + 2 * _q[2][QZ] * _q[2][QZ] - 1) * (2 * _q[3][QY] * _q[3][QY] + 2 * _q[3][QZ] * _q[3][QZ] - 1) + (2 * _q[2][QW] * _q[2][QY] - 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[3][QW] * _q[3][QY] - 2 * _q[3][QX] * _q[3][QZ]) + (2 * _q[2][QW] * _q[2][QZ] + 2 * _q[2][QX] * _q[2][QY]) * (2 * _q[3][QW] * _q[3][QZ] + 2 * _q[3][QX] * _q[3][QY]);

        c_quat[12] = (2 * _q[3][QW] * _q[3][QY] + 2 * _q[3][QX] * _q[3][QZ]) * (2 * _q[2][QY] * _q[2][QY] + 2 * _q[2][QZ] * _q[2][QZ] - 1) - (2 * _q[2][QW] * _q[2][QY] - 2 * _q[2][QX] * _q[2][QZ]) * (2 * _q[3][QX] * _q[3][QX] + 2 * _q[3][QY] * _q[3][QY] - 1) + (2 * _q[2][QW] * _q[2][QZ] + 2 * _q[2][QX] * _q[2][QY]) * (2 * _q[3][QW] * _q[3][QX] - 2 * _q[3][QY] * _q[3][QZ]);

        ang_temp = atan2(c_quat[11], c_quat[12]) * 1000;
        ang_constrain(ang_temp, &arm_angle[6]);

        Serial1.print(arm_angle[6]);
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
        step = -3;
        break;

    default:
        ++step;
        break;
    }
}
#endif // CALCULATE_ARM