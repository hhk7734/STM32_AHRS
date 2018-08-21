/**
 * filename : AT24C.h
 *
 * created  : 2018/05/18
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : AT24C02B EEPROM
 */

// AT24C02B
// Write Cycle Time 5ms!!!!!!
// 8-byte Page Write, 0x00, 0x08... 0x08*n address

// 0x00 |     1acc      |       |       |
// 0x04 |       |       |       |       |
// 0x08 |     1gyro     |       |       |
// 0x0C |       |       |       |       |
// 0x10 |     1magoff   |       |       |
// 0x14 |       |       |       |       |
// 0x18 |     1magsen   |       |       |
// 0x1C |       |       |       |       |
//
// 0x80 |     4acc      |       |       |
// 0x84 |       |       |       |       |
//
// 0x98 |     4magsen   |       |       |
// 0x9C |       |       |       |       |
//
// 0xA0 |       |       |       |       |

// if(!eep.write_i16(0xF0,-321))
// {
//     int16_t xyz;
//     if(eep.read_i16(0xF0,&xyz)) Serial2.println(xyz);
// }

#ifndef _AT24C_H_
#define _AT24C_H_

//#include 
#include <Arduino.h>

//#define
#define AT24C_A0 0
#define AT24C_A1 0
#define AT24C_A2 0
#define AT24C_ADDRESS (0B01010000 | (AT24C_A2<<2)\
                        | (AT24C_A1<<1) | AT24C_A0)

typedef union
{
    uint16_t data;
    uint8_t eep_data[2];
}u16_u8;

typedef union
{
    int16_t data;
    uint8_t eep_data[2];
}i16_u8;

typedef union
{
    //
    // little endian
    // 0    1    2    3    4    5
    // 0x34 0x12 0x78 0x56 0x01 0x91
    // data[0]   data[1]   data[2]
    // e[0] e[1] e[2] e[3] e[4] e[5]
    //
    uint16_t data[3];
    uint8_t eep_data[6];
}u16A3_u8;

typedef union
{
    int16_t data[3];
    uint8_t eep_data[6];
}i16A3_u8;

class AT24C_
{
public :
    AT24C_();
    ~AT24C_();
    
    uint8_t write_u8(uint8_t address, uint8_t data);
    uint8_t write_u16(uint8_t address, uint16_t data);
    uint8_t write_i16(uint8_t address, int16_t data);
    uint8_t write_u16A3(uint8_t address, uint16_t *data);
    uint8_t write_i16A3(uint8_t address, int16_t *data);

    uint8_t read_u8(uint8_t address, uint8_t *data);
    uint8_t read_u16(uint8_t address, uint16_t *data);
    uint8_t read_i16(uint8_t address, int16_t *data);
    uint8_t read_u16A3(uint8_t address, uint16_t *data);
    uint8_t read_i16A3(uint8_t address, int16_t *data);
protected :

private :
}; // AT24C_

#endif // _AT24C_H_