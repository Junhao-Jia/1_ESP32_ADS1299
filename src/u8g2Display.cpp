#include "u8g2Display.h"

void u8g2Display::u8g2Init(void)
{
   begin();
   enableUTF8Print();
   clearBuffer();
   setFont(u8g2_font_wqy13_t_gb2312);
}

void u8g2Display::u8g2DisplayBottomUp(const String &s) 
 {
   static int i = 0;
   if(i >= 4)
      {
         for(int j = 0; j<3 ; j++) //也可以j < i-1
            strcpy(stringArray[j], stringArray[j + 1]);
         i = 3;
      }
   strcpy(stringArray[i], s.c_str());
   
   switch (i)
   {
      case 0:
         {  
            clearBuffer();
            setCursor(0, 15);
            print(stringArray[0]);
            sendBuffer();
         }
         break;
      case 1:
         {
            clearBuffer();
            setCursor(0, 15);
            print(stringArray[0]);
            setCursor(0, 30);
            print(stringArray[1]);
            sendBuffer();
         }
         break;
      case 2:
         {
            clearBuffer();
            setCursor(0, 15);
            print(stringArray[0]);
            setCursor(0, 30);
            print(stringArray[1]);
            setCursor(0, 45);
            print(stringArray[2]);
            sendBuffer();
         }
         break;
      case 3:
         {
            clearBuffer();
            setCursor(0, 15);
            print(stringArray[0]);
            setCursor(0, 30);
            print(stringArray[1]);
            setCursor(0, 45);
            print(stringArray[2]);
            setCursor(0, 60);
            print(stringArray[3]);
            sendBuffer();
         }
         break;
      default:
         break;
   }
   i++;       
}
String u8g2Display::byteToString(unsigned char byteValue)
{
   String result;
    for (int i = sizeof(byteValue); i >= 0; --i) {
        result += ((byteValue & (1 << i))? '1' : '0');
    }
    return result;
} 