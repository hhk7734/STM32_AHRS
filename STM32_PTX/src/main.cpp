#include <Arduino.h>

#include "config.h"
#include "MPU9250.h"
#include "EKF.h"
#include "RF24_STM.h"
#include "AT24C.h"

RF24 radio(BOARD_NRF24_CE_PIN, BOARD_SPI2_NSS_PIN);

MPU9250_ mpu;

EKF_ ekf;

AT24C_ eep;

void setup()
{
    int16_t temp1[3], temp2[3];

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial1.begin(115200);
    Serial1.println("begin");
    // MPU
    ////////////////////////////////////////////////////////////////////////////////

    mpu.setup();

    eep.read_i16A3(EEPROM_ACC_OFF_X, temp1);
    mpu.set_acc_offset(temp1);

    eep.read_i16A3(EEPROM_GYRO_OFF_X, temp1);
    mpu.set_gyro_offset(temp1);

    eep.read_i16A3(EEPROM_MAG_OFF_X, temp1);
    eep.read_i16A3(EEPROM_MAG_SEN_X, temp2);
    mpu.set_mag_offset(temp1, temp2);

    Serial1.println("are you ready?");
    Serial1.println("1 : ACC_CALIBRATION");
    Serial1.println("2 : GYRO_CALIBRATION");
    Serial1.println("3 : MAG_CALIBRATION");

    uint32_t st_time = millis();
    Serial1.flush();
    while ((!Serial1.available()) && (millis() - st_time < 3000))
    {
    }
    digitalWrite(LED_BUILTIN, LOW);
    int8_t status = Serial1.read();
    Serial1.println(status);

    switch (status)
    {
    case ACC_CALIBRATION:
        mpu.acc_calibration();
        mpu.get_acc_offset(temp1);
        eep.write_i16A3(EEPROM_ACC_OFF_X, temp1);
        eep.read_i16A3(EEPROM_ACC_OFF_X, temp1);
        Serial1.print(temp1[0]);
        Serial1.print(',');
        Serial1.print(temp1[1]);
        Serial1.print(',');
        Serial1.println(temp1[2]);
        while (!Serial1.available())
        {
        }
        Serial1.flush();
        break;
    case GYRO_CALIBRATION:
        mpu.gyro_calibration();
        mpu.get_gyro_offset(temp1);
        eep.write_i16A3(EEPROM_GYRO_OFF_X, temp1);
        eep.read_i16A3(EEPROM_GYRO_OFF_X, temp1);
        Serial1.print(temp1[0]);
        Serial1.print(',');
        Serial1.print(temp1[1]);
        Serial1.print(',');
        Serial1.println(temp1[2]);
        while ((!Serial1.available()))
        {
        }
        Serial1.flush();
        break;
    case MAG_CALIBRATION:
        mpu.mag_calibration();
        mpu.get_mag_offset(temp1, temp2);
        eep.write_i16A3(EEPROM_MAG_OFF_X, temp1);
        eep.write_i16A3(EEPROM_MAG_SEN_X, temp2);
        eep.read_i16A3(EEPROM_MAG_OFF_X, temp1);
        eep.read_i16A3(EEPROM_MAG_SEN_X, temp2);
        Serial1.print(temp1[0]);
        Serial1.print(',');
        Serial1.print(temp1[1]);
        Serial1.print(',');
        Serial1.println(temp1[2]);
        Serial1.print(temp2[0]);
        Serial1.print(',');
        Serial1.print(temp2[1]);
        Serial1.print(',');
        Serial1.println(temp2[2]);
        while ((!Serial1.available()))
        {
        }
        Serial1.flush();
        break;
    }
    digitalWrite(LED_BUILTIN, HIGH);

    // EKF
    ////////////////////////////////////////////////////////////////////////////////

    // NRF24
    ////////////////////////////////////////////////////////////////////////////////
    radio.begin();
    radio.setChannel(SYSTEM_NRF_CHANNEL);
    radio.openWritingPipe(SYSTEM_NRF_ADDRESS);
    radio.stopListening();

    Serial1.println("start");

// Flexible sensor
////////////////////////////////////////////////////////////////////////////////
#ifdef SYSTEM_FINGER
    pinMode(FIR_FINGER, INPUT_ANALOG);
    pinMode(SEC_FINGER, INPUT_ANALOG);
    pinMode(THI_FINGER, INPUT_ANALOG);
    pinMode(FOU_FINGER, INPUT_ANALOG);
    pinMode(FIF_FINGER, INPUT_ANALOG);
#endif
}

void loop()
{
    uint32_t old_time, new_time, delta_time;
    old_time = micros();

    int16_t gyro[3], acc[3], mag[3];
    float uacc[3], umag[3];
    float norm;

    int32_t lpf_acc[3] = {0};
    int32_t lpf_mag[3] = {0};

#ifdef SYSTEM_FINGER
    uint8_t step = 0;
#endif

    for (;;)
    {
        new_time = micros();
        delta_time = new_time - old_time;
        old_time = new_time;

        mpu.get_all(acc, gyro, mag);

        float half_delta_ang[3];
        for (uint8_t j = 0; j < 3; ++j)
        {
            half_delta_ang[j] = (float)gyro[j] * delta_time * GYRO_SCALER;
        }

        ekf.predict(half_delta_ang);

        for (uint8_t j = 0; j < 3; ++j)
        {
            lpf_acc[j] -= (lpf_acc[j] >> 4);
            lpf_acc[j] += acc[j];
            acc[j] = lpf_acc[j] >> 4;

            lpf_mag[j] -= (lpf_mag[j] >> 4);
            lpf_mag[j] += mag[j];
            mag[j] = lpf_mag[j] >> 4;
        }

        norm = sqrt((int32_t)acc[0] * acc[0] + (int32_t)acc[1] * acc[1] + (int32_t)acc[2] * acc[2]);
        for (uint8_t j = 0; j < 3; ++j)
            uacc[j] = (float)acc[j] / norm;

        umag[0] = uacc[1] * mag[2] - uacc[2] * mag[1];
        umag[1] = uacc[2] * mag[0] - uacc[0] * mag[2];
        umag[2] = uacc[0] * mag[1] - uacc[1] * mag[0];
        norm = sqrt(umag[0] * umag[0] + umag[1] * umag[1] + umag[2] * umag[2]);
        for (uint8_t j = 0; j < 3; ++j)
            umag[j] /= norm;

        ekf.update(uacc, umag);

        float q[4];

        ekf.get_quat(q);

        int16_t data[6];
#ifdef SYSTEM_FINGER
        if (step == 0)
        {
#endif
            data[0] = q[0] * 20000;
            data[1] = q[1] * 20000;
            data[2] = q[2] * 20000;
            data[3] = q[3] * 20000;
            data[4] = (uint16_t)(delta_time / 100);
            data[5] = SYSTEM_NUMBER;
#ifndef SYSTEM_FINGER
            radio.write(data, 12);
#else
        if (radio.write(data, 12))
        {
            step = 1;
        }
    }
    else
    {
        data[0] = analogRead(FIR_FINGER);
        data[1] = analogRead(SEC_FINGER);
        data[2] = analogRead(THI_FINGER);
        data[3] = analogRead(FOU_FINGER);
        data[4] = analogRead(FIF_FINGER);
        data[5] = 11;
        if (radio.write(data, 12))
        {
            step = 0;
        }
    }
#endif
        }
    }