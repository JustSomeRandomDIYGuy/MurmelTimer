// --- INCLUDES ---
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- CONSTANTS ---
const uint8_t OLED_GND_PIN = MISO;
const uint8_t SENSOR1_GND_PIN = 9;
const uint8_t SENSOR1_DATA_PIN = SCK;
const uint8_t SENSOR2_GND_PIN = 10;
const uint8_t SENSOR2_DATA_PIN = 2;
const uint8_t BUTTON_1_PIN = A0;
const uint8_t BUTTON_2_PIN = A1;
const uint8_t BUTTON_3_PIN = 3;

const uint8_t STATE_NOT_INIT  = 0;
const uint8_t STATE_READY     = 1;
const uint8_t STATE_RUNNING   = 2;
const uint8_t STATE_RESULT    = 3;
const uint8_t STATE_BEST      = 4;

// --- VARIABLES ---
float timeCode = 0.0;
uint8_t myState = STATE_READY;
uint8_t myLastState = STATE_NOT_INIT;

bool but1 = true, but1Prev = false;
bool but2 = true, but2Prev = false;
bool but3 = true, but3Prev = false;
bool sensor1 = true, sensor1Prev = false;
bool sensor2 = true, sensor2Prev = false;
bool  sens1Triggered = false, sens2Triggered = false, 
      but1Changed = false, but2Changed = false, but3Changed = false;

String stringBuffer ;
int textWidth;

uint32_t tStarted, tResult, tBest = 0;

// --- OBJECTS ---
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); 

// --- PROTOTYPES ---
void enablePeriphery();
void disablePeriphery();
void readButtons();
void updateDisplay();

// --------------------------------------------------------------------------
//                               S E T U P 
// --------------------------------------------------------------------------
void setup(void) {

  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nLos gehts...");

  enablePeriphery();
  
  u8g2.begin();

  u8g2.setFont(u8g2_font_logisoso22_tf);
 // u8g2_font_logisoso26_tr u8g2_font_freedoomr25_mn u8g2_font_inr27_mr 
}

// --------------------------------------------------------------------------
//                                L O O P 
// --------------------------------------------------------------------------
void loop(void) {
  static uint32_t t1= millis();
  bool pleaseUpdateDisplay = false;

  readButtons();

  if ( myState == STATE_READY ) {
    if ( myLastState != myState ){
      pleaseUpdateDisplay = true;
    }
    if (sens1Triggered){
      sens1Triggered = false;
      myState = STATE_RUNNING;
      tStarted = millis();
      pleaseUpdateDisplay = true;
    }
    if (but1Changed){
      but1Changed = false;
      myState = STATE_BEST;
      pleaseUpdateDisplay = true;
    } 
    
  }
  else if ( myState == STATE_RUNNING ) {
    pleaseUpdateDisplay = true;
    if (sens1Triggered){
      sens1Triggered = false;
      tStarted = millis();
    }
    if (sens2Triggered){
      sens2Triggered = false;
      myState = STATE_RESULT;
      tResult = millis() - tStarted;
      if ( ( tResult < tBest ) || ( tBest == 0 ) ) {
        tBest = tResult;
        myState = STATE_BEST;
      }
      else {
        myState = STATE_RESULT;
      }
    }
  }
  else if ( myState == STATE_RESULT ) {
    if ( myLastState != myState ){
      pleaseUpdateDisplay = true;
    }
    if (sens1Triggered){
      sens1Triggered = false;
      myState = STATE_RUNNING;
      tStarted = millis();
    }
    if (but1Changed){
      but1Changed = false;
      myState = STATE_BEST;
      pleaseUpdateDisplay = true;
    } 
  }
  else if ( myState == STATE_BEST ) {
    if ( myLastState != myState ){
      pleaseUpdateDisplay = true;
    }
    if (sens1Triggered){
      sens1Triggered = false;
      myState = STATE_RUNNING;
      tStarted = millis();
      pleaseUpdateDisplay = true;
    }
    if (but1Changed){
      but1Changed = false;
      if (tBest > 0 ){
        myState = STATE_RESULT;
      }
      else{
        myState = STATE_READY;
      }
      pleaseUpdateDisplay = true;
    } 
  }
  if (pleaseUpdateDisplay) updateDisplay();
  if (myLastState != myState) Serial.println(myState);
  myLastState = myState;


  //Serial.println( String(millis() - t1) + "ms: b1: " + String(but1) + " | b2: " + String(but2) + " | b3: " + String(but3) + " | s1: " + String(sensor1) + " | s2: " + String(sensor2) );
  
  t1= millis();
  //delay(500);
}

// --------------------------------------------------------------------------
//                        S U B   R O U T I N E S        
// --------------------------------------------------------------------------


// ++++++++++++++++++++++++++++++++++++++
void enablePeriphery(){
  // sensor #1
  pinMode(SENSOR1_GND_PIN, OUTPUT);
  digitalWrite(SENSOR1_GND_PIN, LOW);
  pinMode(SENSOR1_DATA_PIN, INPUT_PULLUP);  

  // sensor #2
  pinMode(SENSOR2_GND_PIN, OUTPUT);
  digitalWrite(SENSOR2_GND_PIN, LOW);
  pinMode(SENSOR2_DATA_PIN, INPUT_PULLUP);  

  // buttons
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);  
}

// ++++++++++++++++++++++++++++++++++++++
void disablePeriphery(){
  pinMode(SENSOR1_GND_PIN, OUTPUT);
  digitalWrite(SENSOR1_GND_PIN, HIGH);

  pinMode(SENSOR2_GND_PIN, INPUT);
  pinMode(SENSOR2_GND_PIN, OUTPUT);
  digitalWrite(SENSOR2_GND_PIN, HIGH);
}

// ++++++++++++++++++++++++++++++++++++++
void readButtons(){
    
  but1      = !digitalRead(BUTTON_1_PIN);
  but2      = !digitalRead(BUTTON_2_PIN);
  but3      = !digitalRead(BUTTON_3_PIN);
  sensor1   = digitalRead(SENSOR1_DATA_PIN);
  sensor2   = digitalRead(SENSOR2_DATA_PIN); 

  if (but1 != but1Prev){
    but1Prev = but1;
    if ( but1){
      but1Changed = true;
      Serial.println("but1");
    }
  } 
  if (but2 != but2Prev){
    but2Prev = but2;
    if ( but2){
      but2Changed = true;
      Serial.println("but2");
    }
  } 
  if (but3 != but3Prev){
    but3Prev = but3;
    if ( but3){
      but3Changed = true;
      Serial.println("but3");
    }
  } 
  if (sensor1 != sensor1Prev){
    sensor1Prev = sensor1;
    if ( sensor1){
      sens1Triggered = true;
      Serial.println("sens1");
    }
  } 
  if (sensor2 != sensor2Prev){
    sensor2Prev = sensor2;
    if ( sensor2){
      sens2Triggered = true;
      Serial.println("sens2");
    }
  } 
}

// ++++++++++++++++++++++++++++++++++++++
void updateDisplay(){
  if (myState == STATE_READY){
    u8g2.firstPage();
    do {
      u8g2.drawHLine(0,31,128);
      u8g2.drawHLine(0,0,128);
      u8g2.setCursor( 0 , 29 );
      u8g2.print("R E A D Y ...");
    } while ( u8g2.nextPage() );
  }
  else if ( myState == STATE_RUNNING ) {

    timeCode = (float)( millis() - tStarted ) / 1000.0  ;

    stringBuffer = String(timeCode,3) + "s";
    textWidth = u8g2.getStrWidth(stringBuffer.c_str());  
    
    u8g2.firstPage();
    do {      
      u8g2.setCursor( 127 - textWidth , 29 );
      u8g2.print(stringBuffer);    
    } while ( u8g2.nextPage() );   
  }
  else if ( myState == STATE_RESULT ){
    timeCode = (float)( tResult ) / 1000.0  ;

    stringBuffer = String(timeCode,3) + "s";
    textWidth = u8g2.getStrWidth(stringBuffer.c_str());  
    
    u8g2.firstPage();
    do {      
      u8g2.setCursor( 127 - textWidth , 29 );
      u8g2.print(stringBuffer);    
    }while ( u8g2.nextPage() );  
  } 
  else if ( myState == STATE_BEST ){
    timeCode = (float)( tBest ) / 1000.0  ;

    stringBuffer = String(timeCode,3) + "s";
    textWidth = u8g2.getStrWidth(stringBuffer.c_str());  
    
    u8g2.firstPage();
    do {      
      u8g2.drawRFrame(0,0,128,32,4);
      u8g2.setCursor( 127 - textWidth , 29 );
      u8g2.print(stringBuffer);    
    }while ( u8g2.nextPage() );  
  } 
}