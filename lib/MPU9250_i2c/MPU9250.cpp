/**
 * filename : MPU9250.cpp
 *
 * created  : 2018/08/08
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : acc, gyro, mag 9axis MPU9250
 */

#include "MPU9250.h"

#include <Wire.h>
#include <math.h>

#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09
#define AK8963_CNTL1     0x0A
#define AK8963_CNTL2     0x0B
#define AK8963_ASTC      0x0C
#define AK8963_I2CDIS    0x0F
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12

#define MPU9250_AD0 0

#if MPU9250_AD0
    #define MPU9250_ADDRESS 0x69
#else
    #define MPU9250_ADDRESS 0x68
#endif

#define MPU9250_SELF_TEST_X_GYRO  0x00
#define MPU9250_SELF_TEST_Y_GYRO  0x01
#define MPU9250_SELF_TEST_Z_GYRO  0x02
#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F
#define MPU9250_XG_OFFSET_H      0x13
#define MPU9250_XG_OFFSET_L      0x14
#define MPU9250_YG_OFFSET_H      0x15
#define MPU9250_YG_OFFSET_L      0x16
#define MPU9250_ZG_OFFSET_H      0x17
#define MPU9250_ZG_OFFSET_L      0x18
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_LP_ACCEL_ODR     0x1E
#define MPU9250_WOM_THR          0x1F
#define MPU9250_FIFO_EN          0x23
#define MPU9250_I2C_MST_CTRL     0x24
#define MPU9250_I2C_SLV0_ADDR    0x25
#define MPU9250_I2C_SLV0_REG     0x26
#define MPU9250_I2C_SLV0_CTRL    0x27
#define MPU9250_I2C_SLV1_ADDR    0x28
#define MPU9250_I2C_SLV1_REG     0x29
#define MPU9250_I2C_SLV1_CTRL    0x2A
#define MPU9250_I2C_SLV2_ADDR    0x2B
#define MPU9250_I2C_SLV2_REG     0x2C
#define MPU9250_I2C_SLV2_CTRL    0x2D
#define MPU9250_I2C_SLV3_ADDR    0x2E
#define MPU9250_I2C_SLV3_REG     0x2F
#define MPU9250_I2C_SLV3_CTRL    0x30
#define MPU9250_I2C_SLV4_ADDR    0x31
#define MPU9250_I2C_SLV4_REG     0x32
#define MPU9250_I2C_SLV4_DO      0x33
#define MPU9250_I2C_SLV4_CTRL    0x34
#define MPU9250_I2C_SLV4_DI      0x35
#define MPU9250_I2C_MST_STATUS   0x36
#define MPU9250_INT_PIN_CFG      0x37
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_INT_STATUS       0x3A
#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_ACCEL_XOUT_L     0x3C
#define MPU9250_ACCEL_YOUT_H     0x3D
#define MPU9250_ACCEL_YOUT_L     0x3E
#define MPU9250_ACCEL_ZOUT_H     0x3F
#define MPU9250_ACCEL_ZOUT_L     0x40
#define MPU9250_TEMP_OUT_H       0x41
#define MPU9250_TEMP_OUT_L       0x42
#define MPU9250_GYRO_XOUT_H      0x43
#define MPU9250_GYRO_XOUT_L      0x44
#define MPU9250_GYRO_YOUT_H      0x45
#define MPU9250_GYRO_YOUT_L      0x46
#define MPU9250_GYRO_ZOUT_H      0x47
#define MPU9250_GYRO_ZOUT_L      0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_I2C_SLV0_DO      0x63
#define MPU9250_I2C_SLV1_DO      0x64
#define MPU9250_I2C_SLV2_DO      0x65
#define MPU9250_I2C_SLV3_DO      0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL  0x69
#define MPU9250_USER_CTRL        0x6A
#define MPU9250_PWR_MGMT_1       0x6B
#define MPU9250_PWR_MGMT_2       0x6C
#define MPU9250_FIFO_COUNTH      0x72
#define MPU9250_FIFO_COUNTL      0x73
#define MPU9250_FIFO_R_W         0x74
#define MPU9250_WHO_AM_I_MPU9250 0x75
#define MPU9250_XA_OFFSET_H      0x77
#define MPU9250_XA_OFFSET_L      0x78
#define MPU9250_YA_OFFSET_H      0x7A
#define MPU9250_YA_OFFSET_L      0x7B
#define MPU9250_ZA_OFFSET_H      0x7D
#define MPU9250_ZA_OFFSET_L      0x7E

//
// Static Variables initialization
// type MPU9250_::variables = init;
//

//
// Constructor & Destructor
//

MPU9250_::MPU9250_()
{
   // variables initialization
}

MPU9250_::~MPU9250_() {}

//
// Public
//

void MPU9250_::setup(void)
{
    Wire.begin();
    Wire.setClock(400000);

    write_reg(MPU9250_PWR_MGMT_1   , 0b10000000); // reset
    delay(100);
    
    write_reg(MPU9250_PWR_MGMT_1   , 0b00000001); // auto selects the clock source
    write_reg(MPU9250_CONFIG       , 0b00000000); // gyro DLPF 0
    write_reg(MPU9250_GYRO_CONFIG  , 0b00011011); // FS_SEL +-2000 dps, Filter choice gyro delay 0.97ms
    write_reg(MPU9250_ACCEL_CONFIG , 0b00010000); // FS_SEL +-8 g
    write_reg(MPU9250_ACCEL_CONFIG2, 0b00001000); // Fchoice 1, DLPF 0, delay 1.88ms

    write_reg(MPU9250_I2C_MST_CTRL , 0b00001101); // i2c master clock 400kHz

    write_reg(MPU9250_I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS); // read AK8963
    write_reg(MPU9250_I2C_SLV0_REG , AK8963_YOUT_L);
    write_reg(MPU9250_I2C_SLV0_CTRL, 0b11010010); // i2c_slv0 enable, swap byte, group of 2 bytes, read 2 bytes
    write_reg(MPU9250_I2C_SLV1_ADDR, 0x80 | AK8963_ADDRESS);
    write_reg(MPU9250_I2C_SLV1_REG , AK8963_XOUT_L);
    write_reg(MPU9250_I2C_SLV1_CTRL, 0b11010010);
    write_reg(MPU9250_I2C_SLV2_ADDR, 0x80 | AK8963_ADDRESS);
    write_reg(MPU9250_I2C_SLV2_REG , AK8963_ZOUT_L); // mag z-axis == acc, gyro (-)z-axis
    write_reg(MPU9250_I2C_SLV2_CTRL, 0b11010011); // read 3 bytes; when ST2 register is read, AK8963 judges that data reading is finished

    write_reg(MPU9250_I2C_SLV4_ADDR, AK8963_ADDRESS); // write AK8963
    write_reg(MPU9250_I2C_SLV4_REG , AK8963_CNTL1);
    write_reg(MPU9250_I2C_SLV4_DO  , 0b00010110); // mag output 16bit, 100Hz rate
    write_reg(MPU9250_I2C_SLV4_CTRL, 0b10000000); // i2c_slv4 enable, this bit auto clears

    write_reg(MPU9250_USER_CTRL    , 0b00100000); // i2c master enable
}

void MPU9250_::get_acc_offset(int16_t *acc_off)
{
    for (uint8_t i = 0; i<3 ; ++i)
    {
        acc_off[i]  = acc_offset[i];
    }
}

void MPU9250_::get_gyro_offset(int16_t *gyro_off)
{
    for (uint8_t i = 0; i<3 ; ++i)
    {
        gyro_off[i] = gyro_offset[i];
    }
}

void MPU9250_::get_mag_offset(int16_t *mag_off, int16_t *mag_sens)
{
    for (uint8_t i = 0; i<3 ; ++i)
    {
        mag_off[i]  = mag_offset[i];
        mag_sens[i] = mag_sensitivity[i];
    }
}

void MPU9250_::set_acc_offset(int16_t *acc_off)
{
    for (uint8_t i = 0; i<3 ; ++i)
    {
        acc_offset[i] = acc_off[i];
    }
}

void MPU9250_::set_gyro_offset(int16_t *gyro_off)
{
    for (uint8_t i = 0; i<3 ; ++i)
    {
        gyro_offset[i] = gyro_off[i];
    }
}

void MPU9250_::set_mag_offset(int16_t *mag_off, int16_t *mag_sens)
{
    for (uint8_t i = 0; i<3 ; ++i)
    {
        mag_offset[i]      = mag_off[i];
        mag_sensitivity[i] = mag_sens[i];
    }
}

void MPU9250_::acc_calibration(void)
{
    int16_t raw_acc[3];
    int32_t raw_acc_sum[3] = {0};

    for (uint8_t i = 0; i<128 ; ++i)
    {
        read_raw_acc(raw_acc);
        for (uint8_t j = 0; j<3 ; ++j)
        {
            raw_acc_sum[j] += raw_acc[j];
        }
        delay(3);
    }

    for (uint8_t i = 0; i<3 ; ++i)
    {
        acc_offset[i] = raw_acc_sum[i]>>7;
    }

    acc_offset[2] -= 4096;
}

void MPU9250_::gyro_calibration(void)
{
    int16_t raw_gyro[3];
    int32_t raw_gyro_sum[3] = {0};

    for (uint8_t i = 0; i<128 ; ++i)
    {
        read_raw_gyro(raw_gyro);
        for (uint8_t j = 0; j<3 ; ++j)
        {
            raw_gyro_sum[j] += raw_gyro[j];
        }
        delay(3);
    }

    for (uint8_t i = 0; i<3 ; ++i)
    {
        gyro_offset[i] = raw_gyro_sum[i]>>7;
    } 
}

void MPU9250_::mag_calibration(void)
{
    double parameter[6] = {0};
    double b[6] = {0};
    double A[6][6] = {0};


    for (uint8_t k = 0; k < 80 ; ++k)
    {
        int16_t raw_mag[3];
        double f_raw_mag[3];
        read_raw_mag(raw_mag);
        for (uint8_t i = 0; i< 3; ++i) f_raw_mag[i] = raw_mag[i];
        double component[6];

        component[0] = f_raw_mag[0]*f_raw_mag[0];
        component[1] = f_raw_mag[0];
        component[2] = f_raw_mag[1]*f_raw_mag[1];
        component[3] = f_raw_mag[1];
        component[4] = f_raw_mag[2]*f_raw_mag[2];
        component[5] = f_raw_mag[2];

        for (uint8_t i = 0; i<6 ; i++)
        {
            for (uint8_t j = 0; j<6 ; j++)
            {
                A[i][j] +=component[i]*component[j];
            }
            b[i] += component[i];
        }
        delay(190);
    }

    for (uint8_t i = 0; i<6 ; ++i) b[i] *=10000.0;


    for (uint8_t k = 0; k<5 ; k++)
    {
        // pivoting
        uint8_t max = k;
        for (uint8_t i = k+1; i<6 ; i++)
        {
            if(abs(A[i][k])>abs(A[max][k])) max = i;
        }
        if (max != k)
        {
            for (uint8_t i = k; i<6 ; i++)
            {
                double aTemp = A[k][i];
                A[k][i] = A[max][i];
                A[max][i] = aTemp;
            }

            double bTemp = b[k];
            b[k]=b[max];
            b[max]=bTemp;
        }

        // Gauss elimination
        for (uint8_t i = k+1 ; i<6 ; i++)
        {
            for (uint8_t j = k+1 ; j<6 ; j++)
            {
                A[i][j] = A[i][j] - A[k][j]*(A[i][k]/A[k][k]);
            }
            b[i] = b[i] - b[k]*(A[i][k]/A[k][k]);
            A[i][k] = 0;
        }
    }

    // backward substitution
    for (uint8_t i = 6; i; i--)
    {
        uint8_t k = i-1;
        parameter[k] = b[k];
        for (uint8_t j = i; j<6 ; j++)
        {
            parameter[k] -= A[k][j]*parameter[j];
        }
        parameter[k] /= A[k][k];
    }

    mag_sensitivity[0] = 32;
    mag_sensitivity[1] = 32.0/sqrt(parameter[0]/parameter[2]);
    mag_sensitivity[2] = 32.0/sqrt(parameter[0]/parameter[4]);
    mag_offset[0] = -parameter[1]/(2.0*parameter[0]);
    mag_offset[1] = -parameter[3]/(2.0*parameter[2]);
    mag_offset[2] = -parameter[5]/(2.0*parameter[4]);
}

void MPU9250_::get_acc(int16_t *xyz)
{
    read_raw_acc(xyz);
    for (uint8_t i = 0; i < 3; ++i)
    {
        xyz[i] -= acc_offset[i];
    }
}

void MPU9250_::get_gyro(int16_t *xyz)
{
    read_raw_gyro(xyz);
    for (uint8_t i = 0; i < 3; ++i)
    {
        xyz[i] -= gyro_offset[i];
    }
}

void MPU9250_::get_mag(int16_t *xyz)
{
    read_raw_mag(xyz);
    for (uint8_t i = 0; i < 3; ++i)
    {
        xyz[i] = ((int32_t)(xyz[i] - mag_offset[i])*mag_sensitivity[i])>>5;
    }
}
//
// Private
//

uint8_t MPU9250_::write_reg(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission(); // i2c communication state
}

uint8_t MPU9250_::read_3axis(uint8_t reg, int16_t *xyz)
{
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false); // repeated start
    uint8_t rx_buff_length = Wire.requestFrom(MPU9250_ADDRESS,6);
    xyz[0] = (Wire.read() << 8) | Wire.read();
    xyz[1] = (Wire.read() << 8) | Wire.read();
    xyz[2] = (Wire.read() << 8) | Wire.read();
    return rx_buff_length;
}

void MPU9250_::read_raw_acc(int16_t *xyz)
{
    read_3axis(MPU9250_ACCEL_XOUT_H,xyz);
}

void MPU9250_::read_raw_gyro(int16_t *xyz)
{
    read_3axis(MPU9250_GYRO_XOUT_H,xyz);
}

void MPU9250_::read_raw_mag(int16_t *xyz)
{
    read_3axis(MPU9250_EXT_SENS_DATA_00,xyz);
    xyz[2] = -xyz[2]; // mag z-axis is reversed
}