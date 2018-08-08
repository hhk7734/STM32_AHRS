
/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#ifndef __RF24_STM_CONFIG_H__
#define __RF24_STM_CONFIG_H__

#include <Arduino.h>
#include <stddef.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#define PRIPSTR "%S"

#define printf_P printf
#define _BV(bit) (1 << (bit))
#undef PSTR
#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))
#define pgm_read_word(p) (*(p))

#undef SERIAL_DEBUG
// #define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define IF_SERIAL_DEBUG(x) ({x;})
#else
#define IF_SERIAL_DEBUG(x)
#endif

// Avoid spurious warnings
#if 1
    #if ! defined( NATIVE ) && defined( ARDUINO )
    #undef PROGMEM
    #define PROGMEM __attribute__(( section(".progmem.data") ))
    #undef PSTR
    #define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))
    #endif
#endif

#define ALWAYS_POWERUP

#endif // __RF24_STM_CONFIG_H__