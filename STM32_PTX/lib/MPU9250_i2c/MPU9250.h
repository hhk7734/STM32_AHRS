/**
 * filename : MPU9250.h
 *
 * created  : 2018/08/08
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : acc, gyro, mag 9axis MPU9250
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

//#include 
#include <Arduino.h>

class MPU9250_
{
public :
    MPU9250_();
    ~MPU9250_();

    void setup(void);
    
    void get_acc_offset(int16_t *acc_off);
    void get_gyro_offset(int16_t *gyro_off);
    void get_mag_offset(int16_t *mag_off, int16_t *mag_sens);
    void set_acc_offset(int16_t *acc_off);
    void set_gyro_offset(int16_t *gyro_off);
    void set_mag_offset(int16_t *mag_off, int16_t *mag_sens);
    
    void acc_calibration(void);
    void gyro_calibration(void);
    void mag_calibration(void);
    
    void get_acc(int16_t *xyz);
    void get_gyro(int16_t *xyz);
    void get_mag(int16_t *xyz);
    void get_all(int16_t *acc_xyz, int16_t *gyro_xyz, int16_t *mag_xyz);
protected :

private :
    int16_t acc_offset[3];
    int16_t gyro_offset[3];
    int16_t mag_offset[3];
    int16_t mag_sensitivity[3];

    uint8_t write_reg(uint8_t reg, uint8_t data);
    uint8_t read_3axis(uint8_t reg, int16_t *xyz);
    void    read_raw_acc(int16_t *xyz);
    void    read_raw_gyro(int16_t *xyz);
    void    read_raw_mag(int16_t *xyz);
}; // MPU9250_

#endif // _MPU9250_H_