#ifndef _CONFIG_H_
#define _CONFIG_H_

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
//                     PB1     PA10/RX1
//                     PB10    PA9 /TX1
//                     PB11    PA8          NRF_CE
//                     R       PB15/MOSI2   NRF_MOSI
//                     3.3V    PB14/MISO2   NRF_MISO
//                     GND     PB13/SCK2    NRF_SCK
//                     GND     PB12/NSS2    NRF_CSN

#define EEPROM_ACC_OFF_X 0x20
#define EEPROM_ACC_OFF_Y (EEPROM_ACC_OFF_X + 0x2)
#define EEPROM_ACC_OFF_Z (EEPROM_ACC_OFF_X + 0x4)

#define EEPROM_GYRO_OFF_X 0x28
#define EEPROM_GYRO_OFF_Y (EEPROM_GYRO_OFF_X + 0x2)
#define EEPROM_GYRO_OFF_Z (EEPROM_GYRO_OFF_X + 0x4)

#define EEPROM_MAG_OFF_X 0x30
#define EEPROM_MAG_OFF_Y (EEPROM_MAG_OFF_X + 0x2)
#define EEPROM_MAG_OFF_Z (EEPROM_MAG_OFF_X + 0x4)
#define EEPROM_MAG_SEN_X 0x38
#define EEPROM_MAG_SEN_Y (EEPROM_MAG_SEN_X + 0x2)
#define EEPROM_MAG_SEN_Z (EEPROM_MAG_SEN_X + 0x4)

#define ACC_CALIBRATION  '1'
#define GYRO_CALIBRATION '2'
#define MAG_CALIBRATION  '3'

// To Dps * Deg To Rad * us To s * half
// (1/16.4)*(PI/180)*(1/1000000)*(1/2)
#define GYRO_SCALER 0.00000000053211258f//0.00000000026605629f

#define BOARD_NRF24_CE_PIN PA8

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

#define SYSTEM_NRF_ADDRESS NRF_ADDRESS_02

#if SYSTEM_NRF_ADDRESS == NRF_ADDRESS_01
    #define SYSTEM_NUMBER 1
    #define SYSTEM_NRF_CHANNEL NRF_CHANNEL_01
    #define SYSTEM_FINGER
#elif SYSTEM_NRF_ADDRESS == NRF_ADDRESS_02
    #define SYSTEM_NUMBER 2
    #define SYSTEM_NRF_CHANNEL NRF_CHANNEL_02
#elif SYSTEM_NRF_ADDRESS == NRF_ADDRESS_03
    #define SYSTEM_NUMBER 3
    #define SYSTEM_NRF_CHANNEL NRF_CHANNEL_03
#elif SYSTEM_NRF_ADDRESS == NRF_ADDRESS_04
    #define SYSTEM_NUMBER 4
    #define SYSTEM_NRF_CHANNEL NRF_CHANNEL_04
#elif SYSTEM_NRF_ADDRESS == NRF_ADDRESS_05
    #define SYSTEM_NUMBER 5
    #define SYSTEM_NRF_CHANNEL NRF_CHANNEL_05
#endif

#ifdef SYSTEM_FINGER
    #define FIR_FINGER PB0
    #define SEC_FINGER PA7
    #define THI_FINGER PA6
    #define FOU_FINGER PA5
    #define FIF_FINGER PA4
#endif
#endif // _CONFIG_H_