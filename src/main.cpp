// --- INCLUDES ---
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// --- CONSTANTS ---
const uint8_t OLED_GND_PIN = MISO;
const uint8_t SENSOR1_GND_PIN = 9;
const uint8_t SENSOR1_DATA_PIN = SCK; // PCINT5
const uint8_t SENSOR2_GND_PIN = 10;
const uint8_t SENSOR2_DATA_PIN = 2; // PCINT18
const uint8_t BUTTON_1_PIN = A0;
const uint8_t BUTTON_2_PIN = A1;
const uint8_t BUTTON_3_PIN = 3;

const uint8_t LED_RED_PIN = 7;
const uint8_t LED_GREEN_PIN = 6;
const uint8_t LED_WHITE_PIN = 5;

const uint8_t STATE_NOT_INIT  = 0;
const uint8_t STATE_READY     = 1;
const uint8_t STATE_RUNNING   = 2;
const uint8_t STATE_RESULT    = 3;
const uint8_t STATE_BEST      = 4;

const uint32_t SLEEP_TIMER         = 300000ul;//300000ul; // 5 minutes

// --- VARIABLES ---
float timeCode = 0.0;
uint8_t myState = STATE_READY;
uint8_t myLastState = STATE_NOT_INIT;

bool but1 = true, but1Prev = false;
bool but2 = true, but2Prev = false;
bool but3 = true, but3Prev = false;
bool sensor1 = true, sensor1Prev = false;
bool sensor2 = true, sensor2Prev = false;
volatile bool sens1Triggered = false, sens2Triggered = false; 
bool but1Changed = false, but2Changed = false, but3Changed = false;
bool iAmSleeping = false;
uint32_t lastActionTS = 0;

String stringBuffer ;
int textWidth;

volatile uint32_t tStarted, tResult, tBest = 0;
volatile uint32_t lastInt = 0;

// --- OBJECTS ---
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); 

// --- PROTOTYPES ---
void enablePeriphery();
void disablePeriphery();
void readButtons();
void updateDisplay();
void INT_PINisr(void);
void goToSleep();
// --------------------------------------------------------------------------
//                               S E T U P 
// --------------------------------------------------------------------------
void setup(void) {


  enablePeriphery();
  
 // u8g2_font_logisoso26_tr u8g2_font_freedoomr25_mn u8g2_font_inr27_mr 

  cli();
  PCICR |= 0b00000101; // Enables Ports B and D Pin Change Interrupts
  PCMSK0 |= 0b00100000; // PCINT5
  PCMSK2 |= 0b00000100; // PCINT18
  sei();
}

// --------------------------------------------------------------------------
//                                L O O P 
// --------------------------------------------------------------------------
void loop(void) {
  bool pleaseUpdateDisplay = false;

  readButtons();

  if ( myState == STATE_READY ) {
    if ( myLastState != myState ){
      pleaseUpdateDisplay = true;
    }
    if (sens1Triggered){
      sens1Triggered = false;
      myState = STATE_RUNNING;
      pleaseUpdateDisplay = true;
    }
    if (but1Changed){
      myState = STATE_BEST;
      pleaseUpdateDisplay = true;
    } 
    
  }
  else if ( myState == STATE_RUNNING ) {
    pleaseUpdateDisplay = true;
    if (sens1Triggered){
      sens1Triggered = false;
      lastActionTS = millis();
    }
    if (sens2Triggered){
      sens2Triggered = false;
      myState = STATE_RESULT;
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
      pleaseUpdateDisplay = true;
    }
    if (but1Changed){
      if (tBest > 0 ){
        myState = STATE_RESULT;
      }
      else{
        myState = STATE_READY;
      }
      pleaseUpdateDisplay = true;
    } 
    if (but2Changed){
      tBest = 0;
      pleaseUpdateDisplay = true;
    } 
  }
  if (but3Changed){
    goToSleep();
    pleaseUpdateDisplay = true;
  }

  if (myLastState != myState) {
    Serial.println(myState);
    lastActionTS = millis();
    pleaseUpdateDisplay = true;
  }

  if ( (millis() - lastActionTS ) > SLEEP_TIMER ) {
    goToSleep();
    pleaseUpdateDisplay = true;
  }

  if ( (pleaseUpdateDisplay) )  {
    updateDisplay();
  }
  myLastState = myState;
  but1Changed = false;
  but2Changed = false;
  but3Changed = false;

  //delay(200);
  //Serial.println("Loop()");
}

// --------------------------------------------------------------------------
//                        S U B   R O U T I N E S        
// --------------------------------------------------------------------------


// ++++++++++++++++++++++++++++++++++++++
void enablePeriphery(){
  noInterrupts();

  Serial.begin(115200);
  Serial.println("\n\nLos gehts...");
  // sensor #1
  pinMode(SENSOR1_GND_PIN, OUTPUT);
  digitalWrite(SENSOR1_GND_PIN, LOW);
  pinMode(SENSOR1_DATA_PIN, INPUT_PULLUP);  

  // sensor #2
  pinMode(SENSOR2_GND_PIN, OUTPUT);
  digitalWrite(SENSOR2_GND_PIN, LOW);
  pinMode(SENSOR2_DATA_PIN, INPUT_PULLUP);  

  // buttons
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);  

  pinMode(LED_GREEN_PIN, OUTPUT);
  analogWrite(LED_GREEN_PIN, 30);
  digitalWrite(LED_GREEN_PIN, HIGH);
  
  delay(500);

  interrupts();
  u8g2.begin();
  u8g2.setFont(u8g2_font_logisoso24_tf);

  lastActionTS = millis();
}

// ++++++++++++++++++++++++++++++++++++++
void disablePeriphery(){

  digitalWrite(SENSOR1_GND_PIN, HIGH);
  digitalWrite(SENSOR2_GND_PIN, HIGH);
  pinMode(LED_GREEN_PIN, LOW);
  pinMode(LED_RED_PIN, LOW);
  pinMode(LED_WHITE_PIN, LOW);
  
/* 
pinMode(SENSOR1_GND_PIN, INPUT);
  pinMode(SENSOR2_GND_PIN, INPUT);
  pinMode(LED_GREEN_PIN, INPUT);
  pinMode(LED_RED_PIN, INPUT);
  pinMode(LED_WHITE_PIN, INPUT);
  pinMode(BUTTON_1_PIN, INPUT);
  pinMode(BUTTON_2_PIN, INPUT); 
// bringt nix
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);

  pinMode(MISO, INPUT_PULLUP);
  pinMode(MOSI, INPUT_PULLUP);
  pinMode(SCK, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP); */
}

// ++++++++++++++++++++++++++++++++++++++
void readButtons(){
    
  but1      = !digitalRead(BUTTON_1_PIN);
  but2      = !digitalRead(BUTTON_2_PIN);
  but3      = !digitalRead(BUTTON_3_PIN);
  

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
    if ( !but3){
      but3Changed = true;
      Serial.println("but3");
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
      u8g2.setCursor( 20 , 29 );
      u8g2.print("BEREIT");
    } while ( u8g2.nextPage() );
  }
  else if ( myState == STATE_RUNNING ) {

    timeCode = (float)( millis() - tStarted ) / 1000.0  ;

    stringBuffer = String(timeCode,3) + "s";
    textWidth = u8g2.getStrWidth(stringBuffer.c_str());  
    
    u8g2.firstPage();
    do {      
      u8g2.setCursor( 127 - textWidth + 2 , 28 );
      u8g2.print(stringBuffer);    
    } while ( u8g2.nextPage() );   
  }
  else if ( myState == STATE_RESULT ){
    timeCode = (float)( tResult ) / 1000.0  ;

    stringBuffer = String(timeCode,3) + "s";
    textWidth = u8g2.getStrWidth(stringBuffer.c_str());  
    
    u8g2.firstPage();
    do {      
      u8g2.setCursor( 127 - textWidth +2 , 28 );
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
      u8g2.setCursor( 127 - textWidth , 28 );
      u8g2.print(stringBuffer);    
    }while ( u8g2.nextPage() );  
  } 
}

// ++++++++++++++++++++++++++++++++++++++
void goToSleep(){
  disablePeriphery();
  Serial.println("Gute Nacht");
  Serial.flush();    
  ADCSRA &= ~(1 << ADEN);
  sleep_bod_disable();
  wdt_disable();
  attachInterrupt(1, INT_PINisr, FALLING);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable(); 
  power_all_enable();
  Serial.println("Guten Morgen");
  detachInterrupt(1);
  enablePeriphery();
  myState = STATE_READY;
  sens1Triggered = false;
  sens2Triggered = false;
}

// ++++++++++++++++++++++++++++++++++++++
ISR(PCINT0_vect)
{ 
  uint32_t currTime = millis();
  if ( ( currTime - lastInt ) > 100 ){  
    sens1Triggered = true; 
    sens2Triggered = false; 
    tStarted = currTime;
    lastInt = currTime;
  }
}

// ++++++++++++++++++++++++++++++++++++++
ISR(PCINT2_vect)
{
  uint32_t currTime = millis();
  if ( ( currTime - lastInt ) > 100 ){  
    sens2Triggered = true; 
    tResult = currTime - tStarted;
    lastInt = currTime;
  }
}

// ++++++++++++++++++++++++++++++++++++++
void INT_PINisr(void)
{

}