/*
Work flow:
 - O power on
    if button pressed upon power off - goto to heater power settings
    if PowerStored Show heater power  for 3 sec
    	Show Power and %-age
    else
        Show only %-age

    Show blinking 0 for % power
    change % by encoder rotation
    apply power by button press
    de-apply power by button press


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



#define SIMULATE_60HZ
#define encoderPinA 2
#define encoderPinB 3

#define EEPROM_ADDR 22       //address of heater element power value
//======================================================
// Encoder Connections
#define ENC_PORT PORTC
#define ENC_PIN  PINC
#define ENC_A    PINC0       //A0
#define ENC_B    PINC1       //A1
#define ENC_BUT  PINC2       //A2
#define ENC_DDR  DDRC
#define ENC_MAX  100
#define ENC_MIN  0
#define ENC_NORM 25000
#define ENC_FAST 12000
#define MAX_DEB 20


volatile int16_t  gEncoderPos = 0;  // a counter for the dial
volatile uint8_t  rotating = false;    // debounce management
volatile uint8_t  OldEncPinState=0;
// interrupt service routine vars
volatile uint8_t  A_set = false;
volatile uint8_t  B_set = false;
uint8_t gEnc_max = ENC_MAX;
uint8_t b_pressed =0;
uint8_t b_not_released = 0;
uint8_t gStartKeyScan = 0;

//=======================================================

// Display defines
#define NDIGITS         8
#define NDIGITS_MASK    7
const int latchpin = 10 ;       //PB2       connect to RCLK
const int clockpin = 13 ;       //PB5/SCK   connect to SCLK
const int datapin = 11;         //PB3/MOSI connect to DIO
#define SET_LATCH (PORTB |= _BV(PB2))
#define CLR_LATCH (PORTB &=~_BV(PB2))

//              GFEDCBA
#define _O_ (0b11000000)
#define _U_ (0b11000001)
#define _E_ (0b10000110)
#define _r_ (0b10101111)
#define _H_ (0b10001001)
#define _P_ (0b10001100)

#define _NO_ (0xFF)    //blank

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
volatile uint8_t gBlinking=0;



#define MAX_POWER 9000
#define PWR_STEP  50
uint16_t gHeaterPower=0;
uint16_t gRatio;



inline void SwitchOn()
{
	segbuf[4] &= 0b01111111;
}

inline void SwitchOff()
{
	segbuf[4] |=0b10000000;
}


#define regMax (ENC_MAX)   //How many power levels we want
signed short gRegError=regMax/2;
unsigned short gPowerPercent;             //Current power level

void  SetNewPLevel(uint8_t Level)
{
   gRegError=regMax/2;
   gPowerPercent = Level;
}

void ZeroCrossing_irq()
{
    gRegError=gRegError-gPowerPercent;
    if (gRegError<=0)
    {
      gRegError=gRegError+regMax;
      SwitchOn();
    }
    else
    {
      SwitchOff();
    }
}




//Takes around 10uSec when divider set to SPI_CLOCK_DIV8
//Shoud run 8 times between 60Hz int
//ie interval should be close but less than 1.85 mSec  -  1/60/9 Sec
void LED_irq(void)
{
  static uint8_t cnt=0;
  static uint8_t off=0;

  CLR_LATCH;
  if (gBlinking && off)        // do not display
  {
    SPI_MasterTransmit(0xFF); // select the segment
    SPI_MasterTransmit(0) ;   // select the digit...
  }
  else
  {
    SPI_MasterTransmit(segbuf[segcnt]); // select the segment
    SPI_MasterTransmit(col[segcnt]) ;   // select the digit...
  }
  SET_LATCH;
  sei();
  segcnt ++ ;
  segcnt &= NDIGITS_MASK;
  ++cnt;
  if (cnt<100)
    off =1;
  else
    off =0;
}


//uint32_t tick=0;

#ifdef SIMULATE_60HZ
ISR(TIMER2_COMPA_vect)
{
//	++tick;
  ZeroCrossing_irq();

}
#endif //SIMULATE_60HZ



uint8_t button_pressed()
{
    if (b_pressed)
    {
        b_pressed=0;
        return 1;
    }
    return 0;
}

//Read button state
inline uint8_t getbutton()
{
	return !(ENC_PIN & _BV(ENC_BUT));
}

inline void button_irq()
{

  if (getbutton())  //when contacts closed
  {
      if (!b_not_released)  //check if not de-bouncing
      {
    	  b_pressed = 1;
      }
      b_not_released = MAX_DEB; //start debounsing
  }
  else if (b_not_released)
  {
      --b_not_released;        //do debounsing MAX_DEB times
  }
}

inline uint8_t readEncPin(uint8_t pin)
{
	if (ENC_PIN & (_BV(pin))) return 1;
	return 0;
}

// Do encoder acceleration
uint8_t enc_step()
{
  static uint32_t old_time=0;
  uint8_t ret;
  uint32_t time = micros();
  uint32_t dt   = time-old_time;
  old_time=time;
  if (dt> ENC_NORM) ret = 1;
  else if (dt<ENC_FAST) ret =  3;
  else ret = 2;
//Serial.println(ret);
  return ret;
}

// Interrupt on A changing state
void doEncoderA() {
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change?
  if (readEncPin(ENC_A) != A_set ) { // debounce once more)
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
    {

      gEncoderPos += enc_step();
      if (gEncoderPos > gEnc_max) gEncoderPos = gEnc_max;
    }

    rotating = false;  // no more debouncing until loop() hits again
  }
}


// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delay (1);
  if ( readEncPin(ENC_B) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
    {
      gEncoderPos -= enc_step();
      if (gEncoderPos<ENC_MIN) gEncoderPos = ENC_MIN;

    }
    rotating = false;
  }
}

// 1 mSec interrupts
ISR(TIMER0_COMPA_vect)
{
   static uint8_t cnt;
   cnt++;
   if(cnt & 1) LED_irq();  //launch a display refresh every 2mS
   if (gStartKeyScan) button_irq();           //check and debunce button
}

//Pin change 1 interrupt de-multiplexer
ISR(PCINT1_vect)
{
  uint8_t pins  = (ENC_PIN ^ OldEncPinState) & (_BV(ENC_A) | _BV(ENC_B)/* | _BV(ENC_BUT) */);
  OldEncPinState= ENC_PIN;
  sei();
  if (pins & _BV(ENC_A))
  {
	  doEncoderA();
  }
  if (pins & _BV(ENC_B))
  {
	  doEncoderB();
  }
  /*  don't using button interrupts for now. Just polling
  if (pins & _BV(ENC_BUT))
  {
	  if (ENC_PIN & _BV(ENC_BUT))
	  {
		  Serial.print('d');
	  }
	  else
	  {
		  Serial.print('p');
	  }
  }
  */
}

void InitEncoder(void)
{

  ENC_DDR &= ~(_BV(ENC_A) | _BV(ENC_B) | _BV(ENC_BUT) ); // Set pins for input
  ENC_PORT |= (_BV(ENC_A) | _BV(ENC_B) | _BV(ENC_BUT) ); //Set pull ups
  PCICR  |= _BV(PCIE1);  //Enable Pin Change int from PCINT[15:8]
  PCMSK1 |= (_BV(ENC_A) | _BV(ENC_B)/* | _BV(ENC_BUT)*/ ); //enable int from encoder pins
  OldEncPinState = ENC_PIN;
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
    TCCR2B =7;//|= (1 << CS20 | 1 << CS21 | 1 << CS22);
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);
#else
#endif

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
    segbuf[pos++] = _O_;  //0
    segbuf[pos++] = _U_;  //U
    segbuf[pos++] = _E_;  //E
    segbuf[pos++] = _r_;  //r
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

void DisplayHP(uint16_t power)
{
	uint8_t pos=0;
    segbuf[pos++] = _NO_;  //0
    segbuf[pos++] = _H_;
    segbuf[pos++] = _P_;
    segbuf[pos++] = _NO_;
    DisplayWord(power, 4);
}

void DisplayPower(uint8_t pwr_p, uint16_t ratio)
{
	uint16_t watts = pwr_p * ratio;
	if (watts) DisplayWord(watts,0);
	else DisplayClear();
	DisplayWord(pwr_p,4);
}


//Setup and save to EEPROM heater nominal power

uint16_t do_gHeaterPower_setup(uint16_t power)
{
   uint8_t old_encmax= gEnc_max;
   uint8_t cnt=0xFF;
   uint8_t btn;
   Serial.println("do_gHeaterPower_setup");
   gEnc_max= MAX_POWER/PWR_STEP;
   gBlinking = 1;
   DisplayHP(power);
   while (cnt)   //wait till button released;
   {
	   cnt--;
	   btn = getbutton();
	   if (btn) cnt=200;
   }
   gEncoderPos = power/PWR_STEP;     //set current encoder position according to stored power
   while (! getbutton())
   {
	   power = gEncoderPos*PWR_STEP;
	   DisplayHP(power);
   }
   Serial.print("upd EEPROM ");
   Serial.println(power);
   eeprom_update_word(( uint16_t *) EEPROM_ADDR, power);
   gBlinking = 0;
   gEnc_max=old_encmax;
   return power;
}

void setup()
{
  cli();

  InitDisplay();
  InitZeroCrossing();
  InitEncoder();
  sei();

  Serial.begin(38400) ;
  Serial.println("Start");
  gHeaterPower = eeprom_read_word(( uint16_t *) EEPROM_ADDR);
  Serial.print("HP EEPROM = ");
  Serial.println(gHeaterPower);
  if (gHeaterPower>MAX_POWER) gHeaterPower =0;
  if (getbutton())
  {
	gHeaterPower=do_gHeaterPower_setup(gHeaterPower);
  }
  gRatio= gHeaterPower/100;
  Serial.print("HP ratio=");
  Serial.println(gRatio);

  gEncoderPos=0;
  SetNewPLevel(0);
  if (gHeaterPower)
  {
      DisplayHP(gHeaterPower);
      delay(3000);
  }
  gStartKeyScan =1;
  Serial.println("end of setup");

}

uint8_t gPwrOld=0;

unsigned long time;
void loop()
{
    uint8_t pwr = gEncoderPos;
    rotating = true;
    if (gPwrOld != pwr)
    {
        SetNewPLevel(pwr);
        gPwrOld=pwr;
        DisplayPower(pwr,gRatio);
    }
    if (button_pressed())    //disable power temporary
    {
    	Serial.println(" bt1");
    	gBlinking=1;
    	SetNewPLevel(0);
    	DisplayPower(pwr,0);
    	while(!button_pressed())  // Wait for next button click
    	{
    		delay(50);
    	}
    	gEncoderPos=pwr;
    	SetNewPLevel(pwr);
    	gBlinking=0;

    }
    delay(5);
//    Serial.println(tick);

}

