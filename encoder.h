/*
 * encoder.h
 *
 *  Created on: Mar 15, 2019
 *      Author: alex
 */

#ifndef ENCODER_H_
#define ENCODER_H_

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

#define EncActive 0   //Active low
volatile int16_t  gEncoderPos = 0;  // a counter for the dial
volatile uint8_t  rotating = false;    // debounce management
volatile uint8_t  OldEncPinState=0;

uint8_t gEnc_max = ENC_MAX;
uint8_t b_pressed =0;
uint8_t b_not_released = 0;
uint8_t gStartKeyScan = 0;

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

inline void do_button()
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

void do_encoder()
{
  enum {
    WaitingForTransition,
    WaitingForBothActive,
    WaitingForBothInactive,
  };

  static uint8_t state = WaitingForTransition;

  switch (state)
  {
    case WaitingForTransition:
   //   cli();
      if (readEncPin(ENC_A) == EncActive)
      {
// Serial.println("#1");
        state = WaitingForBothActive;
        if (readEncPin(ENC_B) == EncActive)
        	gEncoderPos += enc_step();
        else
        	gEncoderPos -= enc_step();
      }
 //     sei();
      if (gEncoderPos<ENC_MIN) gEncoderPos = ENC_MIN;
      if (gEncoderPos > gEnc_max) gEncoderPos = gEnc_max;

      break;
    case WaitingForBothActive:
 //     cli();
      if ((readEncPin(ENC_A) == EncActive) && (readEncPin(ENC_B) == EncActive))
      {
        state = WaitingForBothInactive;
//Serial.println("#2");
      }
 //     sei();
      break;
    case  WaitingForBothInactive:
//      cli();
      if ((readEncPin(ENC_A) != EncActive) && (readEncPin(ENC_B) != EncActive))
      {
        state = WaitingForTransition;
//Serial.println("#3");
      }
//      sei();
      break;

  }

}
void InitEncoder(void)
{

  ENC_DDR &= ~(_BV(ENC_A) | _BV(ENC_B) | _BV(ENC_BUT) ); // Set pins for input
  ENC_PORT |= (_BV(ENC_A) | _BV(ENC_B) | _BV(ENC_BUT) ); //Set pull ups
 // PCICR  |= _BV(PCIE1);  //Enable Pin Change int from PCINT[15:8]
 // PCMSK1 |= (_BV(ENC_A) | _BV(ENC_B)/* | _BV(ENC_BUT)*/ ); //enable int from encoder pins
  OldEncPinState = ENC_PIN;
}



#endif /* ENCODER_H_ */
