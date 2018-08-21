/**
 * filename : uart_parse.cpp
 *
 * created  : 2018/08/21
 *
 * Hyeon-ki, Hong
 * hhk7734@gmail.com
 *
 * purpose : string to int
 *      str fromat : data1,data2,data3\r\n
 */

#include "uart_parse.h"

//
// Static Variables initialization
// type class::variables = init;
//

//
// Constructor & Destructor
//

uart_parse::uart_parse(){}
uart_parse::~uart_parse(){}

//
// Public
//
void uart_parse::begin(void)
{
    Serial3.begin(38400);
}

void uart_parse::parse(void)
{
    uint8_t i = Serial3.available();
    if (i > MAX_PARSING_DATA)
    {
        i = MAX_PARSING_DATA;
    }

    for(; i > 0 ;--i)
    {
        char serial_data = Serial3.read();
        switch(serial_data)
        {
        case ',':
            temp[char_count] = '\0';
            data[data_count++] = str_to_int(temp);
            char_count = 0;
            break;
        case '\n':
        case '\r':
            if ( last_check == 0)
            {
                temp[char_count] = '\0';
                data[data_count++] = str_to_int(temp);
                char_count = 0;
                last_count = data_count;
                data_count = 0;
                last_check = 1;
            }
            break;
        default:
            temp[char_count++] = serial_data;
            break;
        }

        if(last_check == 1) break;
    }
    last_check = 0;
}

uint8_t uart_parse::get_data(int16_t *_data, uint8_t size)
{
    if(last_count == size && data_count == 0)
    {
        for (uint8_t i = 0; i < size; ++i)
        {
            _data[i] = data[i];
        }
        return 1;
    }
    return 0;
}

//
// private
//

int16_t uart_parse::str_to_int(char *data)
{
    char* temp = data;
    int16_t nega = 1;
    int16_t result = 0;
    while(*temp != '\0')
    {
        switch(*temp)
        {
            case '-':
                nega = -1;
                ++temp;
                break;
            default:
                result = result * 10 + *temp++ - '0';
                break; 
        }
    }

    return nega * result;
}