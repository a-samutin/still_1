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
#include "encoder.h"
/*
#include <stdint.h>
#include <stdlib.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
*/

#include "SPI.h"


#define EEPROM_ADDR 22       //address of heater element power value
//======================================================


// AC interrupt
//#define SIMULATE_60HZ
#define ACINT (INT0) // (D2)
#define AC_DELAY 13
uint8_t gACdelay_cnt;

//Triac control
#define SSR_PORT     PORTD
#define SSR_PIN_REG  PIND
#define SSR_PIN      PIND6
#define SSR_DDR		 DDRD
//==================

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
	SSR_PORT |= _BV(SSR_PIN);
	segbuf[4] &= 0b01111111;  //TODO remove it later

}

inline void SwitchOff()
{
	SSR_PORT &= ~_BV(SSR_PIN);
	segbuf[4] |=0b10000000;   //TODO remove it later
}


#define regMax (ENC_MAX)   //How many power levels we want
signed short gRegError=regMax/2;
unsigned short gPowerPercent;             //Current power level

void inline DisableAC_Int()
{
	EIMSK &= ~_BV(ACINT);

}
void inline EnableAC_Int()
{
	EIMSK |= _BV(ACINT);
}

uint8_t inline readACpin()
{
	return PIND & _BV(PD2);  //TODO use define
}

void  SetNewPLevel(uint8_t Level)
{
	cli();
    gRegError=regMax/2;
    gPowerPercent = Level;
    sei();
}

void do_bresenham()
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
//Should run 8 times between 60Hz int
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


#ifdef SIMULATE_60HZ
ISR(TIMER2_COMPA_vect)
{
//	++tick;
	do_bresenham();

}
#else
ISR(INT0_vect)
{
	for (uint8_t cnt=0;cnt<5;++cnt)
	{
		if (readACpin()) return;
	}

	gACdelay_cnt=0;   //disable AC interrupts for AC_DELAY ms
	DisableAC_Int();
	do_bresenham();

}
#endif //SIMULATE_60HZ


// 1 mSec interrupts
ISR(TIMER0_COMPA_vect)
{
   static uint8_t cnt;
   cnt++;
   gACdelay_cnt++;
   if (gACdelay_cnt>=AC_DELAY)
   {
	   EnableAC_Int();
//	   SwitchOff();
   }
   do_encoder();
   if(cnt & 1) LED_irq();  //launch a display refresh every 2mS
   if (gStartKeyScan) do_button();           //check and de-bounce button
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
    //TODO use defines
    DDRD &= ~_BV(PIND2); // Set pins for input
    PORTD &= ~_BV(PIND2); //Unset pullup
 //   EICRA = 0;  //low level interrupt
       EICRA = 2;  //Falling endge interrupt
    EnableAC_Int();
#endif
}

void InitSSR()
{

   SSR_DDR |= _BV(SSR_PIN);    //set as output pin
   SSR_PORT &= ~_BV(SSR_PIN);  //set to 0;
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
	uint16_t watts = pwr_p * ratio / 10;
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
  InitSSR();
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
  gRatio= gHeaterPower/10;
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
  DisplayPower(0,gRatio);
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

