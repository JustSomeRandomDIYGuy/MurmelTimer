#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

float timeCode = 0.0;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 

void setup(void) {

  /* U8g2 Project: SSD1306 Test Board */
  pinMode(0, OUTPUT);
  Serial.begin(115200);
  u8g2.begin();
  u8g2.setFont(u8g2_font_logisoso24_tf);
  // u8g2_font_logisoso26_tr u8g2_font_freedoomr25_mn u8g2_font_inr27_mr 
}

void loop(void) {
  static uint32_t t1= millis();
  String stringBuffer ;

  timeCode = millis() /1000.0  ;
  
  u8g2.clearBuffer();          // clear the internal memory
  //u8g2.drawHLine(0,0,128);
  //u8g2.drawHLine(0,31,128);
  u8g2.setCursor( 0, 29 );
  stringBuffer = String(timeCode,3) + "s";
  int textWidth = u8g2.getStrWidth(stringBuffer.c_str());  
  u8g2.setCursor( 127 - textWidth , 29 );
  //sprintf(stringBuffer,"%3d,%03ds", timeCode/1000, timeCode%1000 );
  u8g2.print(stringBuffer);
  //u8g2.print(String(timeCode,3) + "s");
  u8g2.sendBuffer();         // transfer internal memory to the display
  
  Serial.println(  String(millis() - t1) + "ms | " + String( 1000.0 / ( (float) ( millis() - t1 ) ) ) + "Hz " );
  t1= millis();
}