#ifndef __U8G2DISPLAY__
#define __U8G2DISPLAY__

#include <stdio.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>

class u8g2Display : public U8G2_SSD1306_128X64_NONAME_F_HW_I2C
{
    public :
    u8g2Display(const u8g2_cb_t *rotation, uint8_t reset = U8X8_PIN_NONE, uint8_t clock = U8X8_PIN_NONE, uint8_t data = U8X8_PIN_NONE) :  U8G2_SSD1306_128X64_NONAME_F_HW_I2C(rotation, reset, clock, data){} 
    char stringArray[4][30]; 

    void u8g2Init(void);
    void u8g2DisplayBottomUp(const String &s);
    String byteToString(unsigned char byteValue);      
};

#endif