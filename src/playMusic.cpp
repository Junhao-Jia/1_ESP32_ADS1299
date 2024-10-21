#include "playMusic.h"

void playMusic::playMusicInit(void)
{
    FPSerial.begin(9600, SERIAL_8N1, /*rx =*/25, /*tx =*/26);
    if (!begin(FPSerial, /*isACK = */true, /*doReset = */true)) 
    {  //Use serial to communicate with mp3.
        Serial.println(F("Unable to begin:"));
        Serial.println(F("1.Please recheck the connection!"));
        Serial.println(F("2.Please insert the SD card!"));
        while(true)
        {
             delay(0); // Code to compatible with ESP8266 watch dog.
        }
    }
    Serial.println(F("DFPlayer Mini online."));

    volume(3);  //Set volume value. From 0 to 30
    play(1);  //Play the first mp3
}
void playMusic::playMusicTest(void)
{
    
}