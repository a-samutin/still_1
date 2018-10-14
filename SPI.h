
//Pin settings for ATMEGA328P
#define DDR_SPI DDRB
#define DD_MOSI PB3
#define DD_SCK  PB5


#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01 // SPI2X = bit 0 on SPSR


inline static void SPI_setDataMode(uint8_t dataMode)
{
  SPCR = (SPCR & ~SPI_MODE_MASK) | dataMode;
}

inline static void SPI_setBitOrder(uint8_t bitOrder)
{
  if (bitOrder == LSBFIRST) SPCR |= _BV(DORD);
  else SPCR &= ~(_BV(DORD));
}

inline static void SPI_setClockDivider(uint8_t clockDiv) {
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (clockDiv & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((clockDiv >> 2) & SPI_2XCLOCK_MASK);
}
/*
inline void SPI_begin(void)
{

	   // Set MOSI and SCK output, all others input
	   DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);

}
*/
inline void SPI_MasterInit(void)
{
   /* Set MOSI and SCK output, all others input */
   DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);
   /* Enable SPI, Master, set clock rate fck/16 */
   SPSR=_BV( SPI2X);
   SPCR=_BV(SPE) | _BV(MSTR) | _BV(SPR0);

 }

void SPI_MasterTransmit(char cData)
{

   /* Start transmission */
   SPDR = cData;
   asm volatile("nop");
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)))
      ;
}

