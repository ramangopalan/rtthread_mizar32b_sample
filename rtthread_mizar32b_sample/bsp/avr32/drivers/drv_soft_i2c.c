/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author              Notes
 * 2023-10-25     Raman Gopalan       Initial version
 */

// This file implements I2C protocol on AVR32 devices in eLua.
//
// For reasons outlined below, it does not use the Atmel TWI hardware
// but implements it by bit-banging two GPIO lines.
//
//      Martin Guy <martinwguy@gmail.com>, June 2011

// AVR32 has two families of "two wire" interfaces, both of them inadequate:
//
// AT32UC3A[01]* and AT32UC3B* have a single "TWI" "I2C-compatibile" interface
//   that can be switched between master and slave modes;
// AT32UC3A[34]* and AT32UC3C* devices have separate "TWIM" and "TWIS"
//   interfaces, three of each in the C series.
// Their registers and functionality are completely different,
//
// All currently supported eLua AVR32 targets
// (EVK1100 Mizar32 == AT32UC3A0* and EVK1101 == AT32UC3B*)
// have one "TWI" interface.
//
// 1) "TWI" hardware limitations
// 
// The only repeated start sequence that TWI can generate as master is
// START/ADDRESS(W)/WRITE 1, 2 or 3 bytes/START/ADDRESS(R)/READ n BYTES/STOP.
//
// It cannot emit START/ADDRESS(R/W)/STOP, which is necessary to probe
// for the presence of a device on the bus. Their SDK writes a single 0 bytes
// to do this, which has different effects on different hardware.
// This makes I2C unimplementable using this variant of the hardware.
//
// Worse, you have to have all the data in hand before you start any transfer
// because if you fail to rewrite the "Transmit Holding Register" with the next
// byte while the current byte is being sent, the hardware automatically
// generates a STOP without asking you.
// This makes eLua's current Lua and C I2C interfaces unimplementable.
//
// 2) "TWIM" and "TWIS" hardware limitations
//
// This seems to be able to generate a wider range of repeated start signals
// but the amount of data you can send or receive in one packet is limited to
// 255 bytes.
//
// 3) Bit-banging them as GPIO pins
//
// Given the two sets of incompatible hardware and the fact that neither is
// capable either of speaking I2C or of implementing the current eLua I2C
// interface, we just bit-bang the IO pins.
//
// Some of the chips (e.g. AVR32UC3[01]*) have a GPIO open-collector mode,
// which sounds promising, but others (AVR32UC3A[34], AVR32UC3[BC]) do not
// so we obtain a pseudo-open-collector mode by switching the GPIO pins
// between output (always low) and input (of high impedance).
//
// The disadvantage of bit-banging GPIO pins instead of using the TWI hardware
// is that in TWI mode, the pins "are open-drain outputs with slew-rate
// limitation and inputs with spike-filtering" (AT32UC3A Datasheet, para 8.3).
// In GPIO mode, a "glitch filter" is available to reject pulses of 1 CPU cycle
// but this only affects the interrupt function, not the value read from the
// PVR register. (para 22.4.8)

// This first hack only support a single I2C channel and is only tested on the
// currently supported hardware (AVR32UC3A0*), all of which has a single TWI
// interface.
// To extend it to have multiple I2C channels you need to turn the variables
//   started, delay, sda_regs, scl_regs
// and the constants
//   {SDA,SCL}_{PIN,PORT,PINMASK}
// into arrays[NUM_I2C] and index them with id.

#include "compiler.h"
#include "gpio.h"
#include "drv_soft_i2c.h"

#ifdef RT_USING_I2C

// Which port/pins are used for I2C?

#ifdef BSP_BOARD_MIZAR32B
// One master-slave TWI interface
#define SDA_PIN     AVR32_PIN_PA29
#define SCL_PIN     AVR32_PIN_PA30
#else
// One master-slave TWI interface
#define SDA_PIN     AVR32_PIN_PA10
#define SCL_PIN     AVR32_PIN_PA09
#endif

// Split these into port and pinmask
#define SDA_PORT    ( SDA_PIN >> 5 )
#define SCL_PORT    ( SCL_PIN >> 5 )
#define SDA_PINMASK ( 1 << ( SDA_PIN & 31 ) )
#define SCL_PINMASK ( 1 << ( SCL_PIN & 31 ) )

// The set of GPIO registers we will be using for each bus line.
// In practice, these two will always have the same value. Ho hum.
static volatile avr32_gpio_port_t *sda_regs =
        &AVR32_GPIO.port[ SDA_PORT ];
static volatile avr32_gpio_port_t *scl_regs =
        &AVR32_GPIO.port[ SCL_PORT ];

// Half an I2C bus clock cycle, as a number of HSB(==CPU) clock cycles;
// Be default, use the slow mode setting.
// This is exported to the LCD display driver ("disp") so that it can
// change the bus speed as required by the LCD, then restore it.
unsigned int i2c_delay = REQ_CPU_FREQ / 100000 / 2;

// Local functions used by the bit-banger
static void I2CDELAY(void); // Pause for half an I2C bus clock cycle
static int READSCL(void);   // Set SCL as input and return current level of line
static int READSDA(void);   // Set SDA as input and return current level of line
static void CLRSCL(void);   // Actively drive SCL signal low
static void CLRSDA(void);   // Actively drive SDA signal low
static void ARBITRATION_LOST(void); // Bus control was lost

// ************************
// The bitbanger itself, taken from http://en.wikipedia.org/wiki/I2C

// One-time initialization of the I2C hardware.
// Should be called at all valid initial i2c entry points.
static void i2c_init()
{
  static bool i2c_is_initialized = 0;

  if( i2c_is_initialized ) return;

  // First, set both pins as high-impedance inputs to avoid startup glitches
  sda_regs->oderc = SDA_PINMASK;
  scl_regs->oderc = SCL_PINMASK;

  // When they are outputs, they will always output 0.
  sda_regs->ovrc = SDA_PINMASK;
  scl_regs->ovrc = SCL_PINMASK;

  // Let the GPIO hardware control these pins
  sda_regs->gpers = SDA_PINMASK;
  scl_regs->gpers = SCL_PINMASK;

  i2c_is_initialized++;    // set to true
}

// We don't use GPIO open-drain mode, which is not available on all hardware
// models. Instead, we use two modes to simulate open-drain:
// output of 0 and input.
unsigned int i2c_setup( unsigned int speed )
{
  i2c_init();

  // Limit range to possible values, to avoid divisions by zero below.
  if (speed == 0) speed = 1;
  if (speed > REQ_CPU_FREQ / 2) speed = REQ_CPU_FREQ / 2;

  // Figure out how many clock cycles correspond to half a clock waveform.
  // "+(speed-1)" ensures we never set a faster speed than what they asked for.
  i2c_delay = ( REQ_CPU_FREQ / 2 + (speed - 1) ) / speed;

  // Return the closest integer to the actual speed we set
  return ( REQ_CPU_FREQ / 2 + i2c_delay / 2 ) / i2c_delay;
}

// Are we between a start bit and a stop bit?
static int started = 0;

void i2c_start_cond(void)
{
  i2c_init();

  if (started) {
    // if started, do a restart cond
    // set SDA to 1
    READSDA();
    I2CDELAY();
    // Clock stretching
    while (READSCL() == 0)
      ;  // You can add a timeout to this loop to
         // recover from SCL being stuck low.
    // Repeated start setup time, minimum 4.7us
    I2CDELAY();
  }
  if (READSDA() == 0)
    ARBITRATION_LOST();
  // SCL is high, set SDA from 1 to 0
  CLRSDA();
  I2CDELAY();
  CLRSCL();
  started = true;
}
 
void i2c_stop_cond(void)
{
  /* set SDA to 0 */
  CLRSDA();
  I2CDELAY();
  /* Clock stretching */
  while (READSCL() == 0)
    ;  /* You should add timeout to this loop */
  /* SCL is high. Respect I2C spec's minimum stop setup time of 4ms. */
  I2CDELAY();
  /* set SDA from 0 to 1 */
  if (READSDA() == 0)
    ARBITRATION_LOST();
  I2CDELAY();
  started = false;
}
 
/* Write a bit to I2C bus */
static void i2c_write_bit(int bit)
{
  if (bit) 
    READSDA();
  else 
    CLRSDA();
  I2CDELAY();
  /* Clock stretching */
  while (READSCL() == 0)
    ;  /* You should add timeout to this loop */
  /* SCL is high, now data is valid */
  /* If SDA is high, check that nobody else is driving SDA */
  if (bit && READSDA() == 0) 
    ARBITRATION_LOST();
  I2CDELAY();
  CLRSCL();
}
 
/* Read a bit from I2C bus */
static int i2c_read_bit(void)
{
  int bit;

  /* Let the slave drive data */
  READSDA();
  I2CDELAY();
  /* Clock stretching */
  while (READSCL() == 0)
    ;  /* You should add timeout to this loop */
  /* SCL is high, now data is valid */
  bit = READSDA();
  I2CDELAY();
  CLRSCL();
  return bit;
}
 
/* Write a byte to I2C bus.
 * Returns the nack bit received from the slave: 0 if acknowledged.
 */
int i2c_write_byte(unsigned char byte)
{
  unsigned bit;
  int nack;

  for (bit = 0; bit < 8; bit++) {
    i2c_write_bit((byte & 0x80) != 0);
    byte <<= 1;
  }
  nack = i2c_read_bit();

  return nack;
}
 
/* Read a byte from I2C bus.
 * "nack" is the acknowledge bit to send: 0 to acknowledge, 1 to reject/stop.
 */
unsigned char i2c_read_byte(int nack)
{
  unsigned char byte = 0;
  unsigned bit;

  for (bit = 0; bit < 8; bit++)
    byte = (byte << 1) | i2c_read_bit();             
  i2c_write_bit(nack);

  return byte;
}

//*******************
// Low-level functions used by the bit-banger

// Pause for half an I2C bus clock cycle
static void I2CDELAY()
{
  // Use the CPU cycle counter (CPU and HSB clocks are the same).
  unsigned int delay_start_cycle = Get_system_register(AVR32_COUNT);

  // at 60MHz the count register wraps every 71.68 secs, at 66MHz every 65s.
  // The following unsigned arithmetic handles the wraparound condition.
  while( (unsigned)Get_system_register(AVR32_COUNT) - delay_start_cycle < i2c_delay )
    /* wait */;
}

// Set SCL as input and return current level of line
static int READSCL()
{
  scl_regs->oderc = SCL_PINMASK;
  return ( scl_regs->pvr & SCL_PINMASK ) ? 1 : 0;
}

// Set SDA as input and return current level of line
static int READSDA()
{
  sda_regs->oderc = SDA_PINMASK;
  return ( sda_regs->pvr & SDA_PINMASK ) ? 1 : 0;
}

// Actively drive SCL signal low
static void CLRSCL(void)
{
  scl_regs->oders = SCL_PINMASK;
}

// Actively drive SDA signal low
static void CLRSDA(void)
{
  sda_regs->oders = SDA_PINMASK;
}

// Bus control was lost.
// Not currently used. With a higher-level I2C interface, this can do a
// longjmp back to the eLua C interface routine which can retry the transfer
// when the bus is free.
static void ARBITRATION_LOST(void)
{
}

/*
 * A higher-level interface that deals in whole I2C messages.
 * "slave_address" is the 8-bit slave address (so bit 0 should be 0)
 * "stop" is true if we should end with a stop cindition.  This allows
 *     repeated starts by setting "stop" false in the previous message.
 * This interface will also allow us to implement collision detection and retry.
 */

// i2c_send() sends data to a slave.
// "address" is a 7-bit slave address.
// "stop" says whether we should send a final stop bit or not.
// It returns the number of bytes that were sent and acknowledged
// or -1 if the slave did not acknowledge its address.

int i2c_send(unsigned char address, const unsigned char *data, unsigned len, bool stop)
{
  int retval;

  i2c_start_cond();
  // Convert 7-bit address to 8-bit address with bottom bit clear for "write"
  if( i2c_write_byte( address << 1 ) ) {
    // There was no acknowledgement from the slave
    retval = -1;
  } else {
    retval = 0;  // Number of bytes transmitted
    while( len > 0 ) {
      if( i2c_write_byte( *data++ ) ) {
        // Not acknowledged: stop transmitting
        break;
      }
      len--; retval++;
    }
  }
  if( stop ) i2c_stop_cond();
  return( retval );
}

// i2c_recv() reads "count" bytes from a 7-bit slave device.
// It returns the number of bytes sent or -1 if the slave did not acknowledge its address.
int i2c_recv(unsigned char address, unsigned char *data, unsigned len, bool stop)
{
  int retval;

  i2c_start_cond();
  // Convert 7-bit address to 8-bit address with bottom bit set for "read"
  if( i2c_write_byte( ( address << 1 ) | 1 ) ) {
    // There was no acknowledgement from the slave
    retval = -1;
  } else {
    retval = len;  // It always reads all requested bytes unless there is a
                   // collision.
    while( len > 0 ) {
      // ACK all bytes except the last one.
      *data++ = i2c_read_byte(--len == 0);
    }
  }
  if( stop ) i2c_stop_cond();
  return( retval );
}

// See whether a certain slave is present by writing 0 bytes from it.
// Returns 1 if it is present or 0 if it didn't acknowledge its address.
bool i2c_probe(unsigned char slave_address)
{
  return( i2c_send( slave_address, NULL, 0, true ) == 0 );
}

unsigned int platform_i2c_setup( unsigned id, unsigned int speed )
{
	return i2c_setup(speed);
}

void platform_i2c_send_start( unsigned id )
{
	i2c_start_cond();
}

void platform_i2c_send_stop( unsigned id )
{
	i2c_stop_cond();
}

int platform_i2c_send_address( unsigned id, uint16_t address, int direction )
{
	// Convert enum codes to R/w bit value.
	// If TX == 0 and RX == 1, this test will be removed by the compiler
	if ( ! ( PLATFORM_I2C_DIRECTION_TRANSMITTER == 0 &&
	PLATFORM_I2C_DIRECTION_RECEIVER == 1 ) ) {
		direction = ( direction == PLATFORM_I2C_DIRECTION_TRANSMITTER ) ? 0 : 1;
	}

	// Low-level returns nack (0=acked); we return ack (1=acked).
	return ! i2c_write_byte( (address << 1) | direction );
}

int platform_i2c_send_byte( unsigned id, unsigned char data )
{
	// Low-level returns nack (0=acked); we return ack (1=acked).
	return ! i2c_write_byte( data );
}

int platform_i2c_recv_byte( unsigned id, int ack )
{
	return i2c_read_byte( !ack );
}

static int lcd (int args, char *argv[])
{
	char lbuf[16] = {'H', 'e', 'l', 'l', 'o', ' ', 'S', 'e', 'r', 'g', 'i', 'o', '!', ' ', ':', ')'};
    platform_i2c_setup(0, 50000);
	platform_i2c_send_start(0);
	int ack = platform_i2c_send_address( 0, 63, PLATFORM_I2C_DIRECTION_TRANSMITTER ); 
	if (ack)
	{
		for (int i = 0; i < 16; i++)
			platform_i2c_send_byte(0, lbuf[i]);
	}
	platform_i2c_send_stop(0);
}
MSH_CMD_EXPORT(lcd, LCD Test Function);

#endif /* #ifdef RT_USING_I2C */