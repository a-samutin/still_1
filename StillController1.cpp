/*
Workflow:
 - Om power on
    if button pressed upon power off - goto to heater power settings
    if stored Show heater power  for 3 sec
    Show blinking 0 for % power
    change % by encoder rotation
    apply power by button press
    deapply power by button press


Connections (Arduino Nano)

Display
  DIO  - D11
  RCLK - D10
  SCLK - D13
*/

#include <Arduino.h>
/*
#include <stdint.h>
#include <stdlib.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
*/

#include "SPI.h"


#define NDIGITS         8
#define NDIGITS_MASK    7
#define SIMULATE_60HZ
#define encoderPinA 2
#define encoderPinB 3


//Connections
#define ENC_PORT PORTC
#define ENC_A    PINC0       //A0
#define ENC_B    PINC1       //A1
#define ENC_DDR  DDRC


const int latchpin = 10 ;       //PB2       connect to RCLK
const int clockpin = 13 ;       //PB5/SCK   connect to SCLK
const int datapin = 11;         //PB3/MOSI connect to DIO

uint8_t col[NDIGITS] = {
    0b10000000,
    0b01000000,
    0b00100000,
    0b00010000,
    0b00001000,
    0b00000100,
    0b00000010,
    0b00000001
} ;

uint8_t seg[] = {
    0b11000000,  // 0
    0b11111001,  // 1
    0b10100100,  // 2
    0b10110000,  // 3
    0b10011001,  // 4
    0b10010010,  // 5
    0b10000010,  // 6
    0b11111000,  // 7
    0b10000000,  // 8
    0b10010000,  // 9
} ;

uint8_t segbuf[NDIGITS] ;
int segcnt = 0;
//ClickEncoder *encoder;

volatile uint16_t encoderPos = 0;  // a counter for the dial
volatile uint16_t lastReportedPos = 1;   // change management
volatile uint8_t rotating = false;    // debounce management

// interrupt service routine vars
volatile uint8_t A_set = false;
volatile uint8_t B_set = false;
volatile uint8_t blinking=1;



volatile uint16_t tik=1;



//Takes around 10uSec when divider set to SPI_CLOCK_DIV8
//Shoud run 8 times between 60Hz int
//ie interval should be close but less than 1.85 mSec  -  1/60/9 Sec
void LED_irq(void)
{
  static uint8_t cnt=0;
  static uint8_t every300ms=0;
  digitalWrite(latchpin, LOW);
  if (blinking && every300ms)
  {
    SPI_MasterTransmit(0xFF); // select the segment
    SPI_MasterTransmit(0) ;   // select the digit...
  }
  else
  {
    SPI_MasterTransmit(segbuf[segcnt]); // select the segment
    SPI_MasterTransmit(col[segcnt]) ;   // select the digit...
  }
  digitalWrite(latchpin, HIGH) ;
  segcnt ++ ;
  segcnt &= NDIGITS_MASK;
  ++cnt;
  if (cnt>125)
    every300ms =1;
  else
    every300ms =0;
}

void ZeroCrossing_irq(void)
{


	//RODO a brezingham here
}

#ifdef SIMULATE_60HZ
ISR(TIMER2_COMPA_vect)
{
  ZeroCrossing_irq();

}
#endif //SIMULATE_60HZ


// 1 mSec interrupts
ISR(TIMER0_COMPA_vect)
{
  static uint8_t cnt;
   cnt++;
   if(cnt & 1) LED_irq();  //launch a display refresh every 2mS
}

// Interrupt on A changing state
void doEncoderA() {
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change?
  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos += 1;

    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      encoderPos -= 1;

    rotating = false;
  }
}


void Init_1mS_Timer(void)
{

    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function below
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
}

void InitDisplay(void)
{
  pinMode(latchpin, OUTPUT) ;
  SPI_MasterInit();
  for (int i=0; i<8; i++)
      segbuf[i] = 0xFF ;
  Init_1mS_Timer();  //Will use a Timero0 for refresh
}

void InitZeroCrossing(void)
{
  // initialize LED digital pin as an output.


#ifdef SIMULATE_60HZ
  //Используем Таймер 2 для 61HZ прерываний для эмуляции сети
  //set timer2 interrupt at 61HZ
    TCCR2A = 0;// set entire TCCR2A register to 0
    TCCR2B = 0;// same for TCCR2B
    TCNT2  = 0;//initialize count_bv macroer value to 0
    // set compare match register for 8khz increments
    OCR2A = 0xFF;
    // turn on CTC mode (for 255 couneter we can use normal mode)
    TCCR2A |= (1 << WGM21);
    // Set CS21 bit for 1024 prescaler
    TCCR2B =7;//|= (1 << CS20 || 1 << CS21 || 1 << CS22);
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);
#else
#endif


}


void InitEncoder(void)
{
pinMode(encoderPinA, INPUT);
pinMode(encoderPinB, INPUT);
//pinMode(clearButton, INPUT);
// turn on pullup resistors
digitalWrite(encoderPinA, HIGH);
digitalWrite(encoderPinB, HIGH);
//digitalWrite(clearButton, HIGH);

// encoder pin on interrupt 0 (pin 2)
attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)
attachInterrupt(1, doEncoderB, CHANGE);
//  encoder = new ClickEncoder(A1, A0, A2);
//  Init_1mS_Timer();

}

void setup()
{
  cli();
/* unsigned long t1,t2;
t1 = micros();
t2 = micros();
Serial.println(t2-t1) ; */

  InitDisplay();
  InitZeroCrossing();
  InitEncoder();

  sei();
  Serial.begin(38400) ;
  Serial.println("Start");



    // Should auto-run at this point...
}

#define regMax (100)   //How many power levels we want
signed short regError=regMax/2;
unsigned short regValue;             //Current power level

void  SetNewLevel(unsigned short Level)
{
   regError=regMax/2;
   regValue = Level;
}

void ZeroCrossingInterrupt()
{
    regError=regError-regValue;
    if (regError<=0)
    {
      regError=regError+regMax;
  /*    SwitchOn(); */
    }
    else
    {
  /*    SwitchOff(); */
    }
}


void DisplayClear(/* arguments */)
{
  for (byte i=0;i<NDIGITS; i++)
    segbuf[i]=0xFF;
}
void DislayOver(byte pos)
{
  if (pos<=4)
  {
    segbuf[pos++] = 0b11000000;  //0
    segbuf[pos++] = 0b11000001;  //U
    segbuf[pos++] = 0b10000110;  //E
    segbuf[pos++] = 0b10101111;  //r
  }
}
void DisplayWord(word num, byte pos)
{
  if (pos >4)
    return;
  if (num>9999)
  {
     DislayOver( pos);
     return;
  }
  for (int i=3;i>=0;i--)
  {
    if (!num && i != 3)
      segbuf[pos+i]=0xFF; //blank
    else
      segbuf[pos+i]=seg[num % 10];
    num = num / 10;
  }
}
void DisplayLongUint(unsigned long num)
{
  segbuf[NDIGITS-1]=seg[num % 10];

  for(int i=NDIGITS-2;i>=0;i--)
  {
    num = num / 10;
    if (num)
      segbuf[i] = seg[num % 10];
    else
       segbuf[i]=0xFF;
  }

}
unsigned long time;
void loop()
{
  int16_t val;
    //Serial.println("Looping...") ;
  //  delay(150) ;
//    val+=encoder->getValue();
//   DisplayWord(val,0);
//   DisplayWord(tik, 4);
//Serial.print("Time: ");
//time = millis();

//Serial.println(time);
      DisplayLongUint(encoderPos);
 //     Serial.println(tik);
//      delay(1000);

//    DisplayWord(9999+2,4);

}
/*


https://playground.arduino.cc/Main/RotaryEncoders

ClickEncoder - https://github.com/0xPIT/encoder/tree/arduino


*/
