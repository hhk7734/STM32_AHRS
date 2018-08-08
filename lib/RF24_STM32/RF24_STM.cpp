/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "RF24_STM_config.h"
#include "RF24_STM.h"

/* Memory Map */
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_CH          0x05
#define NRF_SETUP       0x06
#define NRF_STATUS      0x07
#define NRF_OBSERVE_TX  0x08
#define NRF_CD          0x09
#define NRF_RX_ADDR_P0  0x0A
#define NRF_RX_ADDR_P1  0x0B
#define NRF_RX_ADDR_P2  0x0C
#define NRF_RX_ADDR_P3  0x0D
#define NRF_RX_ADDR_P4  0x0E
#define NRF_RX_ADDR_P5  0x0F
#define NRF_TX_ADDR     0x10
#define NRF_RX_PW_P0    0x11
#define NRF_RX_PW_P1    0x12
#define NRF_RX_PW_P2    0x13
#define NRF_RX_PW_P3    0x14
#define NRF_RX_PW_P4    0x15
#define NRF_RX_PW_P5    0x16
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD	    0x1C
#define NRF_FEATURE	    0x1D

/* Bit Mnemonics */
#define NRF_MASK_RX_DR  6
#define NRF_MASK_TX_DS  5
#define NRF_MASK_MAX_RT 4
#define NRF_EN_CRC      3
#define NRF_CRCO        2
#define NRF_PWR_UP      1
#define NRF_PRIM_RX     0
#define NRF_ENAA_P5     5
#define NRF_ENAA_P4     4
#define NRF_ENAA_P3     3
#define NRF_ENAA_P2     2
#define NRF_ENAA_P1     1
#define NRF_ENAA_P0     0
#define NRF_ERX_P5      5
#define NRF_ERX_P4      4
#define NRF_ERX_P3      3
#define NRF_ERX_P2      2
#define NRF_ERX_P1      1
#define NRF_ERX_P0      0
#define NRF_AW          0
#define NRF_ARD         4
#define NRF_ARC         0
#define NRF_PLL_LOCK    4
#define NRF_DR          3
#define NRF_PWR         6
#define NRF_RX_DR       6
#define NRF_TX_DS       5
#define NRF_MAX_RT      4
#define NRF_RX_P_NO     1
#define NRF_TX_FULL     0
#define NRF_PLOS_CNT    4
#define NRF_ARC_CNT     0
#define NRF_TX_REUSE    6
#define NRF_FIFO_FULL   5
#define NRF_TX_EMPTY    4
#define NRF_RX_FULL     1
#define NRF_RX_EMPTY    0
#define NRF_DPL_P5      5
#define NRF_DPL_P4      4
#define NRF_DPL_P3      3
#define NRF_DPL_P2      2
#define NRF_DPL_P1      1
#define NRF_DPL_P0      0
#define NRF_EN_DPL      2
#define NRF_EN_ACK_PAY  1
#define NRF_EN_DYN_ACK  0

/* Instruction Mnemonics */
#define NRF_R_REGISTER    0x00
#define NRF_W_REGISTER    0x20
#define NRF_REGISTER_MASK 0x1F
#define NRF_ACTIVATE      0x50
#define NRF_R_RX_PL_WID   0x60
#define NRF_R_RX_PAYLOAD  0x61
#define NRF_W_TX_PAYLOAD  0xA0
#define NRF_W_ACK_PAYLOAD 0xA8
#define NRF_FLUSH_TX      0xE1
#define NRF_FLUSH_RX      0xE2
#define NRF_REUSE_TX_PL   0xE3
#define NRF_NOP           0xFF

/* Non-P omissions */
#define NRF_LNA_HCURR   0

/* P model memory Map */
#define NRF_RPD         0x09

/* P model bit Mnemonics */
#define NRF_DR_LOW   5
#define NRF_DR_HIGH  3
#define NRF_PWR_LOW  1
#define NRF_PWR_HIGH 2

SPIClass _SPI2(2);

/****************************************************************************/

void RF24::csn(int16_t mode)
{
    digitalWrite(csn_pin,mode);
}

/****************************************************************************/

void RF24::ce(int16_t level)
{
    digitalWrite(ce_pin,level);
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  csn(LOW);
  status = _SPI2.transfer( NRF_R_REGISTER | ( NRF_REGISTER_MASK & reg ) );
  while ( len-- )
    *buf++ = _SPI2.transfer(0xff);

  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg)
{
  csn(LOW);
  _SPI2.transfer( NRF_R_REGISTER | ( NRF_REGISTER_MASK & reg ) );
  uint8_t result = _SPI2.transfer(0xff);

  csn(HIGH);
  return result;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    uint8_t status;

    IF_SERIAL_DEBUG(Serial1.print("write_register ");
                    Serial1.print(reg, HEX);
                    Serial1.print(',');
                    Serial1.println(len);
    );

    csn(LOW);
    status = _SPI2.transfer( NRF_W_REGISTER | ( NRF_REGISTER_MASK & reg ) );
    while ( len-- )
        _SPI2.transfer(*buf++);
    csn(HIGH);

    return status;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
    uint8_t status;

    IF_SERIAL_DEBUG(Serial1.print("write_register(");
                    Serial1.print(reg, HEX);
                    Serial1.print(',');
                    Serial1.print(value, HEX);
                    Serial1.println(')');
    );

    csn(LOW);
    status = _SPI2.transfer( NRF_W_REGISTER | ( NRF_REGISTER_MASK & reg ) );
    _SPI2.transfer(value);
    csn(HIGH);

    return status;
}

/****************************************************************************/

uint8_t RF24::write_payload(const void* buf, uint8_t len)
{
    uint8_t status;

    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    uint8_t data_len = min(len,payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
    
    
    csn(LOW);
    status = _SPI2.transfer( NRF_W_TX_PAYLOAD );
    while ( data_len-- )
        _SPI2.transfer(*current++);
    while ( blank_len-- )
        _SPI2.transfer(0);
    csn(HIGH);

    return status;
}

/****************************************************************************/

uint8_t RF24::read_payload(void* buf, uint8_t len)
{
  uint8_t status;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);
  
  csn(LOW);
  status = _SPI2.transfer( NRF_R_RX_PAYLOAD );
  while ( data_len-- )
    *current++ = _SPI2.transfer(0xff);
  while ( blank_len-- )
    _SPI2.transfer(0xff);
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::flush_rx(void)
{
  uint8_t status;

  csn(LOW);
  status = _SPI2.transfer( NRF_FLUSH_RX );
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::flush_tx(void)
{
  uint8_t status;

  csn(LOW);
  status = _SPI2.transfer( NRF_FLUSH_TX );
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::get_status(void)
{
  uint8_t status;

  csn(LOW);
  status = _SPI2.transfer( NRF_NOP );
  csn(HIGH);

  return status;
}

/****************************************************************************/

void RF24::print_status(uint8_t status)
{
    Serial1.print("NRF_STATUS\t\t = 0x");
    if( status < 0x10) Serial1.print("0");
    Serial1.print(status, HEX);
    Serial1.print(" NRF_RX_DR=");
    Serial1.print((status & _BV(NRF_RX_DR))?1:0,HEX);
    Serial1.print(" NRF_TX_DS=");
    Serial1.print((status & _BV(NRF_TX_DS))?1:0,HEX);
    Serial1.print(" NRF_MAX_RT=");
    Serial1.print((status & _BV(NRF_MAX_RT))?1:0,HEX);
    Serial1.print(" NRF_RX_P_NO=");
    Serial1.print(((status >> NRF_RX_P_NO) & B111),HEX);
    Serial1.print(" NRF_TX_FULL=");
    Serial1.print((status & _BV(NRF_TX_FULL))?1:0,HEX);
    Serial1.print("\r\n");
}

/****************************************************************************/

void RF24::print_observe_tx(uint8_t value)
{
    Serial1.print("NRF_OBSERVE_TX=");
    Serial1.print(value, HEX);
    Serial1.print(": POLS_CNT=");
    Serial1.print((value >> NRF_PLOS_CNT) & B1111, HEX);
    Serial1.print(" NRF_ARC_CNT=");
    Serial1.println((value >> NRF_ARC_CNT) & B1111, HEX);
    // printf_P(PSTR("NRF_OBSERVE_TX=%02x: POLS_CNT=%x NRF_ARC_CNT=%x\r\n"),
    //         value,
    //         (value >> NRF_PLOS_CNT) & B1111,
    //         (value >> NRF_ARC_CNT) & B1111
    //         );
}

/****************************************************************************/

void RF24::print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
    char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
    Serial1.print(name);
    Serial1.print("\t");
    Serial1.print(extra_tab);
    Serial1.print(" =");
    while (qty--){
        Serial1.print(" 0x");
        int16_t i = read_register(reg++);
        if( i < 0x10) Serial1.print("0");
        Serial1.print(i, HEX);
    }
    Serial1.print("\r\n");
}

/****************************************************************************/

void RF24::print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
    char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
    Serial1.print(name);
    Serial1.print("\t");
    Serial1.print(extra_tab);
    Serial1.print(" =");
    while(qty--)
    {
        uint8_t buffer[5];
        read_register(reg++,buffer,sizeof buffer);

        Serial1.print(" 0x");
        uint8_t* bufptr = buffer + sizeof buffer;
        while( --bufptr >= buffer ){
            if( *bufptr < 0x10) Serial1.print("0");
            Serial1.print(*bufptr, HEX);
        }
    }
    Serial1.print("\r\n");
}

/****************************************************************************/

RF24::RF24(uint8_t _cepin, uint8_t _cspin):
    ce_pin(_cepin), csn_pin(_cspin), wide_band(true), p_variant(false), 
    payload_size(32), ack_payload_available(false), dynamic_payloads_enabled(false),
    pipe0_reading_address(0)
{
}

/****************************************************************************/

void RF24::setChannel(uint8_t channel)
{
    // TODO: This method could take advantage of the 'wide_band' calculation
    // done in setChannel() to require certain channel spacing.

    const uint8_t max_channel = 127;
    write_register(NRF_CH,min(channel,max_channel));
}

/****************************************************************************/

void RF24::setPayloadSize(uint8_t size)
{
    const uint8_t max_payload_size = 32;
    payload_size = min(size,max_payload_size);
}

/****************************************************************************/

uint8_t RF24::getPayloadSize(void)
{
    return payload_size;
}

/****************************************************************************/

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
    rf24_datarate_e_str_0,
    rf24_datarate_e_str_1,
    rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
    rf24_model_e_str_0,
    rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
    rf24_crclength_e_str_0,
    rf24_crclength_e_str_1,
    rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "LA_MED";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_HIGH";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = { 
    rf24_pa_dbm_e_str_0,
    rf24_pa_dbm_e_str_1,
    rf24_pa_dbm_e_str_2,
    rf24_pa_dbm_e_str_3,
};

void RF24::printDetails(void)
{
  print_status(get_status());

  print_address_register(PSTR("NRF_RX_ADDR_P0-1"),NRF_RX_ADDR_P0,2);
  print_byte_register(PSTR("NRF_RX_ADDR_P2-5"),NRF_RX_ADDR_P2,4);
  print_address_register(PSTR("NRF_TX_ADDR"),NRF_TX_ADDR);

  print_byte_register(PSTR("NRF_RX_PW_P0-6"),NRF_RX_PW_P0,6);
  print_byte_register(PSTR("NRF_EN_AA"),NRF_EN_AA);
  print_byte_register(PSTR("NRF_EN_RXADDR"),NRF_EN_RXADDR);
  print_byte_register(PSTR("NRF_CH"),NRF_CH);
  print_byte_register(PSTR("NRF_SETUP"),NRF_SETUP);
  print_byte_register(PSTR("NRF_CONFIG"),NRF_CONFIG);
  print_byte_register(PSTR("NRF_DYNPD/NRF_FEATURE"),NRF_DYNPD,2);

    Serial1.print("Data Rate\t = ");
    Serial1.println(pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
    Serial1.print("Model\t\t = ");
    Serial1.println(pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
    Serial1.print("CRC Length\t = ");
    Serial1.println(pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
    Serial1.print("PA Power\t = ");
    Serial1.println(pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
}

/****************************************************************************/

void RF24::begin(void)
{
// Initialize pins
    pinMode(ce_pin,OUTPUT);
    pinMode(csn_pin,OUTPUT);

    pinMode(BOARD_SPI2_MOSI_PIN, OUTPUT);
    pinMode(BOARD_SPI2_MISO_PIN, INPUT);
    pinMode(BOARD_SPI2_SCK_PIN,  OUTPUT);
// Initialize SPI bus
    _SPI2.begin();
    _SPI2.setBitOrder(MSBFIRST);
    _SPI2.setDataMode(SPI_MODE0);
    _SPI2.setClockDivider(SPI_CLOCK_DIV64); // SPI2 clock == 36MHz / 64

    ce(LOW);
    csn(HIGH);

// Must allow the radio time to settle else configuration bits will not necessarily stick.
// This is actually only required following power up but some settling time also appears to
// be required after resets too. For full coverage, we'll always assume the worst.
// Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
// Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay( 5 ) ;

// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
// sizes must never be used. See documentation for a more complete explanation.
    setRetries(4,15);

// Restore our default PA level
    setPALevel( RF24_PA_MAX ) ;

// Determine if this is a p or non-p RF24 module and then
// reset our data rate back to default value. This works
// because a non-P variant won't allow the data rate to
// be set to 250Kbps.
    // if( setDataRate( RF24_250KBPS ) )
    // {
    //     p_variant = true ;
    // }

// Then set the data rate to the slowest (and most reliable) speed supported by all
// hardware.
    setDataRate( RF24_1MBPS ) ;

// Initialize CRC and request 2-byte (16bit) CRC
    setCRCLength( RF24_CRC_16 ) ;

// Disable dynamic payloads, to match dynamic_payloads_enabled setting
    write_register(NRF_DYNPD,0);

// Reset current status
// Notice reset and flush is the last thing we do
    write_register(NRF_STATUS,_BV(NRF_RX_DR) | _BV(NRF_TX_DS) | _BV(NRF_MAX_RT) );

// Set up default configuration.  Callers can always change it later.
// This channel should be universally safe and not bleed over into adjacent
// spectrum.
    setChannel(70);

// Flush buffers
    flush_rx();
    flush_tx();
}

/****************************************************************************/

void RF24::startListening(void)
{
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) | _BV(NRF_PWR_UP) | _BV(NRF_PRIM_RX));
    write_register(NRF_STATUS, _BV(NRF_RX_DR) | _BV(NRF_TX_DS) | _BV(NRF_MAX_RT) );

    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address)
    write_register(NRF_RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&pipe0_reading_address), 5);

    // Flush buffers
    flush_rx();
    flush_tx();

    // Go!
    ce(HIGH);

    // wait for the radio to come up (130us actually only needed)
    delayMicroseconds(130);
}

/****************************************************************************/

void RF24::stopListening(void)
{
    ce(LOW);
    flush_tx();
    flush_rx();
}

/****************************************************************************/

void RF24::powerDown(void)
{
    write_register(NRF_CONFIG,read_register(NRF_CONFIG) & ~_BV(NRF_PWR_UP));
}

/****************************************************************************/

void RF24::powerUp(void)
{
    write_register(NRF_CONFIG,read_register(NRF_CONFIG) | _BV(NRF_PWR_UP));
}

/******************************************************************/

bool RF24::write( const void* buf, uint8_t len )
{
    bool result = false;

    // Begin the write
    startWrite(buf,len);

    // ------------
    // At this point we could return from a non-blocking write, and then call
    // the rest after an interrupt

    // Instead, we are going to block here until we get NRF_TX_DS (transmission completed and ack'd)
    // or NRF_MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
    // is flaky and we get neither.

    // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
    // if I tighted up the retry logic.  (Default settings will be 1500us.
    // Monitor the send
    uint8_t observe_tx;
    uint8_t status;
    uint32_t sent_at = millis();
    const uint32_t timeout = 500; //ms to wait for timeout
    do
    {
        status = read_register(NRF_OBSERVE_TX,&observe_tx,1);
        IF_SERIAL_DEBUG(Serial1.print(observe_tx,HEX));
    }
    while( ! ( status & ( _BV(NRF_TX_DS) | _BV(NRF_MAX_RT) ) ) && ( millis() - sent_at < timeout ) );

    // The part above is what you could recreate with your own interrupt handler,
    // and then call this when you got an interrupt
    // ------------

    // Call this when you get an interrupt
    // The status tells us three things
    // * The send was successful (NRF_TX_DS)
    // * The send failed, too many retries (NRF_MAX_RT)
    // * There is an ack packet waiting (NRF_RX_DR)
    bool tx_ok, tx_fail;
    whatHappened(tx_ok,tx_fail,ack_payload_available);

    //  Serial1.print(tx_ok);
    //    Serial1.print(tx_fail);
    //    Serial1.println(ack_payload_available);

    result = tx_ok;
    IF_SERIAL_DEBUG(Serial1.print(result?"...OK.":"...Failed"));

    // Handle the ack packet
    if ( ack_payload_available )
    {
    ack_payload_length = getDynamicPayloadSize();
    IF_SERIAL_DEBUG(Serial1.print("[AckPacket]/"));
    IF_SERIAL_DEBUG(Serial1.println(ack_payload_length,DEC));
    }

    // Yay, we are done.

    // Power down
    powerDown();

    // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
    flush_tx();

    return result;
}
/****************************************************************************/

void RF24::startWrite( const void* buf, uint8_t len )
{
    // Transmitter power-up
    write_register(NRF_CONFIG, ( read_register(NRF_CONFIG) | _BV(NRF_PWR_UP) ) & ~_BV(NRF_PRIM_RX) );
    delayMicroseconds(150);

    // Send the payload
    write_payload( buf, len );

    // Allons!
    ce(HIGH);
    delayMicroseconds(15);
    ce(LOW);
}

/****************************************************************************/

uint8_t RF24::getDynamicPayloadSize(void)
{
    uint8_t result = 0;

    csn(LOW);
    _SPI2.transfer( NRF_R_RX_PL_WID );
    result = _SPI2.transfer(0xff);
    csn(HIGH);

    return result;
}

/****************************************************************************/

bool RF24::available(void)
{
    return available(NULL);
}

/****************************************************************************/

bool RF24::available(uint8_t* pipe_num)
{
    uint8_t status = get_status();

    // Too noisy, enable if you really want lots o data!!
    //IF_SERIAL_DEBUG(print_status(status));

    bool result = ( status & _BV(NRF_RX_DR) );

    if (result)
    {
        // If the caller wants the pipe number, include that
        if ( pipe_num )
            *pipe_num = ( status >> NRF_RX_P_NO ) & B111;

        // Clear the status bit

        // ??? Should this REALLY be cleared now?  Or wait until we
        // actually READ the payload?

        write_register(NRF_STATUS,_BV(NRF_RX_DR) );

        // Handle ack payload receipt
        if ( status & _BV(NRF_TX_DS) )
        {
            write_register(NRF_STATUS,_BV(NRF_TX_DS));
        }
    }

    return result;
}

/****************************************************************************/

bool RF24::read( void* buf, uint8_t len )
{
    // Fetch the payload
    read_payload( buf, len );

    // was this the last of the data available?
    return read_register(NRF_FIFO_STATUS) & _BV(NRF_RX_EMPTY);
}

/****************************************************************************/

void RF24::whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    uint8_t status = write_register(NRF_STATUS,_BV(NRF_RX_DR) | _BV(NRF_TX_DS) | _BV(NRF_MAX_RT) );

    // Report to the user what happened
    tx_ok = status & _BV(NRF_TX_DS);
    tx_fail = status & _BV(NRF_MAX_RT);
    rx_ready = status & _BV(NRF_RX_DR);
}

/****************************************************************************/

void RF24::openWritingPipe(uint64_t value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    IF_SERIAL_DEBUG(Serial1.print("openWritingPipe ");
                    Serial1.println(value,HEX););

    write_register(NRF_RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), 5);
    write_register(NRF_TX_ADDR, reinterpret_cast<uint8_t*>(&value), 5);

    const uint8_t max_payload_size = 32;
    write_register(NRF_RX_PW_P0,min(payload_size,max_payload_size));
}

/****************************************************************************/

static const uint8_t child_pipe[] PROGMEM =
{
  NRF_RX_ADDR_P0, NRF_RX_ADDR_P1, NRF_RX_ADDR_P2, NRF_RX_ADDR_P3, NRF_RX_ADDR_P4, NRF_RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  NRF_RX_PW_P0, NRF_RX_PW_P1, NRF_RX_PW_P2, NRF_RX_PW_P3, NRF_RX_PW_P4, NRF_RX_PW_P5
};
static const uint8_t child_pipe_enable[] PROGMEM =
{
  NRF_ERX_P0, NRF_ERX_P1, NRF_ERX_P2, NRF_ERX_P3, NRF_ERX_P4, NRF_ERX_P5
};

void RF24::openReadingPipe(uint8_t child, uint64_t address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0)
        pipe0_reading_address = address;

    if (child <= 6)
    {
        // For pipes 2-5, only write the LSB
        if ( child < 2 )
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 5);
        else
            write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);

        write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(NRF_EN_RXADDR,read_register(NRF_EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
}

/****************************************************************************/

void RF24::toggle_features(void)
{
    csn(LOW);
    _SPI2.transfer( NRF_ACTIVATE );
    _SPI2.transfer( 0x73 );
  csn(HIGH);
}

/****************************************************************************/

void RF24::enableDynamicPayloads(void)
{
    // Enable dynamic payload throughout the system
    write_register(NRF_FEATURE,read_register(NRF_FEATURE) | _BV(NRF_EN_DPL) );

    // If it didn't work, the features are not enabled
    if ( ! read_register(NRF_FEATURE) )
    {
        // So enable them and try again
        toggle_features();
        write_register(NRF_FEATURE,read_register(NRF_FEATURE) | _BV(NRF_EN_DPL) );
    }

    IF_SERIAL_DEBUG(printf("NRF_FEATURE=%i\r\n",read_register(NRF_FEATURE)));

    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register(NRF_DYNPD,read_register(NRF_DYNPD) | _BV(NRF_DPL_P5) | _BV(NRF_DPL_P4) | _BV(NRF_DPL_P3) | _BV(NRF_DPL_P2) | _BV(NRF_DPL_P1) | _BV(NRF_DPL_P0));

    dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24::enableAckPayload(void)
{
    //
    // enable ack payload and dynamic payload features
    //

    write_register(NRF_FEATURE,read_register(NRF_FEATURE) | _BV(NRF_EN_ACK_PAY) | _BV(NRF_EN_DPL) );

    // If it didn't work, the features are not enabled
    if ( ! read_register(NRF_FEATURE) )
    {
        // So enable them and try again
        toggle_features();
        write_register(NRF_FEATURE,read_register(NRF_FEATURE) | _BV(NRF_EN_ACK_PAY) | _BV(NRF_EN_DPL) );
    }

    IF_SERIAL_DEBUG(printf("NRF_FEATURE=%i\r\n",read_register(NRF_FEATURE)));

    //
    // Enable dynamic payload on pipes 0 & 1
    //

    write_register(NRF_DYNPD,read_register(NRF_DYNPD) | _BV(NRF_DPL_P1) | _BV(NRF_DPL_P0));
}

/****************************************************************************/

void RF24::writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  csn(LOW);
  _SPI2.transfer( NRF_W_ACK_PAYLOAD | ( pipe & B111 ) );
  const uint8_t max_payload_size = 32;
  uint8_t data_len = min(len,max_payload_size);
  while ( data_len-- )
    _SPI2.transfer(*current++);

  csn(HIGH);
}

/****************************************************************************/

bool RF24::isAckPayloadAvailable(void)
{
  bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}

/****************************************************************************/

bool RF24::isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void RF24::setAutoAck(bool enable)
{
  if ( enable )
    write_register(NRF_EN_AA, B111111);
  else
    write_register(NRF_EN_AA, 0);
}

/****************************************************************************/

void RF24::setAutoAck( uint8_t pipe, bool enable )
{
    if ( pipe <= 6 )
    {
        uint8_t en_aa = read_register( NRF_EN_AA ) ;
        if( enable )
        {
            en_aa |= _BV(pipe) ;
        }
        else
        {
            en_aa &= ~_BV(pipe) ;
        }
        write_register( NRF_EN_AA, en_aa ) ;
    }
}

/****************************************************************************/

bool RF24::testCarrier(void)
{
  return ( read_register(NRF_CD) & 1 );
}

/****************************************************************************/

bool RF24::testRPD(void)
{
  return ( read_register(NRF_RPD) & 1 ) ;
}

/****************************************************************************/

void RF24::setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = read_register(NRF_SETUP) ;
  setup &= ~(_BV(NRF_PWR_LOW) | _BV(NRF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(NRF_PWR_LOW) | _BV(NRF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(NRF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(NRF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(NRF_PWR_LOW) | _BV(NRF_PWR_HIGH)) ;
  }

  write_register( NRF_SETUP, setup ) ;
}

/****************************************************************************/

rf24_pa_dbm_e RF24::getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = read_register(NRF_SETUP) & (_BV(NRF_PWR_LOW) | _BV(NRF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( power == (_BV(NRF_PWR_LOW) | _BV(NRF_PWR_HIGH)) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == _BV(NRF_PWR_HIGH) )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == _BV(NRF_PWR_LOW) )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

/****************************************************************************/

bool RF24::setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(NRF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(NRF_DR_LOW) | _BV(NRF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the NRF_DR_LOW to 1; NRF_DR_HIGH (used to be NRF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( NRF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, NRF_DR (NRF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(NRF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  write_register(NRF_SETUP,setup);

  // Verify our result
  if ( read_register(NRF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}

/****************************************************************************/

rf24_datarate_e RF24::getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = read_register(NRF_SETUP) & (_BV(NRF_DR_LOW) | _BV(NRF_DR_HIGH));
  
  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(NRF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(NRF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

/****************************************************************************/

void RF24::setCRCLength(rf24_crclength_e length)
{
  uint8_t config = read_register(NRF_CONFIG) & ~( _BV(NRF_CRCO) | _BV(NRF_EN_CRC)) ;
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(NRF_EN_CRC);
  }
  else
  {
    config |= _BV(NRF_EN_CRC);
    config |= _BV( NRF_CRCO );
  }
  write_register( NRF_CONFIG, config ) ;
}

/****************************************************************************/

rf24_crclength_e RF24::getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = read_register(NRF_CONFIG) & ( _BV(NRF_CRCO) | _BV(NRF_EN_CRC)) ;

  if ( config & _BV(NRF_EN_CRC ) )
  {
    if ( config & _BV(NRF_CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/****************************************************************************/

void RF24::disableCRC( void )
{
  uint8_t disable = read_register(NRF_CONFIG) & ~_BV(NRF_EN_CRC) ;
  write_register( NRF_CONFIG, disable ) ;
}

/****************************************************************************/
void RF24::setRetries(uint8_t delay, uint8_t count)
{
 write_register(NRF_SETUP_RETR,(delay&0xf)<<NRF_ARD | (count&0xf)<<NRF_ARC);
}

// vim:ai:cin:sts=2 sw=2 ft=cpp

