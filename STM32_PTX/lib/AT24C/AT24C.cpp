/**
 * filename : AT24C.cpp
 *
 * created  : 2018/05/18
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : AT24C02B EEPROM
 */

#include "AT24C.h"

#include <Wire.h>

//#include 

//
// Static Variables initialization
// type AT24C_::variables = init;
//

//
// Constructor & Destructor
//
AT24C_::AT24C_()
{
    // variables initialization
}
AT24C_::~AT24C_() {}

//
// Public
//
uint8_t AT24C_::write_u8(uint8_t address, uint8_t data)
{
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    uint8_t temp = Wire.endTransmission();
    delay(10);
    return temp;
}

uint8_t AT24C_::write_u16(uint8_t address, uint16_t data)
{
    u16_u8 u16;
    u16.data = data;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.write(u16.eep_data[0]);
    Wire.write(u16.eep_data[1]);
    uint8_t temp = Wire.endTransmission();
    delay(10);
    return temp;
}

uint8_t AT24C_::write_i16(uint8_t address, int16_t data)
{
    i16_u8 i16;
    i16.data = data;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.write(i16.eep_data[0]);
    Wire.write(i16.eep_data[1]);
    uint8_t temp = Wire.endTransmission();
    delay(10);
    return temp;
}

uint8_t AT24C_::write_u16A3(uint8_t address, uint16_t *data)
{
    u16A3_u8 u16A3;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    for (uint8_t i = 0; i < 3; ++i)
    {
        u16A3.data[i]=data[i];
        Wire.write(u16A3.eep_data[2*i]);
        Wire.write(u16A3.eep_data[2*i+1]);
    }
    uint8_t temp = Wire.endTransmission();
    delay(10);
    return temp;
}

uint8_t AT24C_::write_i16A3(uint8_t address, int16_t *data)
{
    i16A3_u8 i16A3;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    for (uint8_t i = 0; i < 3; ++i)
    {
        i16A3.data[i]=data[i];
        Wire.write(i16A3.eep_data[2*i]);
        Wire.write(i16A3.eep_data[2*i+1]);
    }
    uint8_t temp = Wire.endTransmission();
    delay(10);
    return temp;
}

uint8_t AT24C_::read_u8(uint8_t address, uint8_t *data)
{
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    uint8_t temp = Wire.requestFrom(AT24C_ADDRESS,1);
    *data = Wire.read();
    return temp;
}

uint8_t AT24C_::read_u16(uint8_t address, uint16_t *data)
{
    u16_u8 u16;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    uint8_t temp = Wire.requestFrom(AT24C_ADDRESS,2);
    u16.eep_data[0] = Wire.read();
    u16.eep_data[1] = Wire.read();
    *data = u16.data;
    return temp;
}

uint8_t AT24C_::read_i16(uint8_t address, int16_t *data)
{
    i16_u8 i16;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    uint8_t temp = Wire.requestFrom(AT24C_ADDRESS,2);
    i16.eep_data[0] = Wire.read();
    i16.eep_data[1] = Wire.read();
    *data = i16.data;
    return temp;
}

uint8_t AT24C_::read_u16A3(uint8_t address, uint16_t *data)
{
    u16A3_u8 u16A3;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    uint8_t temp = Wire.requestFrom(AT24C_ADDRESS,6);
    for (uint8_t i = 0; i < 3; ++i)
    {
        u16A3.eep_data[2*i] = Wire.read();
        u16A3.eep_data[2*i + 1] = Wire.read();
        data[i] = u16A3.data[i];
    }
    return temp;
}

uint8_t AT24C_::read_i16A3(uint8_t address, int16_t *data)
{
    i16A3_u8 i16A3;
    Wire.beginTransmission(AT24C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    uint8_t temp = Wire.requestFrom(AT24C_ADDRESS,6);
    for (uint8_t i = 0; i < 3; ++i)
    {
        i16A3.eep_data[2*i] = Wire.read();
        i16A3.eep_data[2*i + 1] = Wire.read();
        data[i] = i16A3.data[i];
    }
    return temp;
}
//
// Protected
//

//
// Private
//