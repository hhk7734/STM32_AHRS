#include <Arduino.h>

#include "uart_parse.h"

uart_parse up;

void setup()
{
    Serial.begin(38400);
    up.begin();
    Serial.println("start");
}

int16_t data[12];
int16_t step = 0;

void loop()
{
    // instead of code
    delay(8);

    // parsing
    up.parse();
    if(up.get_data(data, 12))
    {
        // test
        if (step != 11)
        {

            Serial.print(data[step++]);
            Serial.print(',');
        }
        else
        {
            Serial.println(data[step]);
            step = 0;
        }
    }
}