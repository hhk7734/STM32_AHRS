/**
 * filename : uart_parse.h
 *
 * created  : 2018/08/21
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : string to int16
 *      str fromat : data1,data2,data3\r\n
 */
#ifndef _UART_PARSE_H_
#define _UART_PARSE_H_

#include <Arduino.h>

#define MAX_PARSING_DATA 20
#define MAX_NUM_DATA 20

class uart_parse
{
public:
    uart_parse();
    ~uart_parse();

    void begin(void);
    void parse(void);
    uint8_t get_data(int16_t *_data, uint8_t size);
protected:

private:
    int16_t data[MAX_NUM_DATA];
    uint8_t data_count = 0;
    char temp[15];
    uint8_t char_count = 0;
    uint8_t last_check = 0;
    uint8_t last_count = 0;

    int16_t str_to_int(char *data);
};

#endif // _UART_PARSE_H_