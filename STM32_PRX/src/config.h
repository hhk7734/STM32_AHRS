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

#define MAX_ERROR 157