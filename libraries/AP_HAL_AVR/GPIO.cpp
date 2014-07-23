#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/interrupt.h>
#include <avr/io.h>
#include "Scheduler.h"
#include "utility/pins_arduino_mega.h"

#include "GPIO.h"
using namespace AP_HAL_AVR;


AP_HAL::Proc AVRGPIO::_interrupt_6 = NULL;

SIGNAL(INT6_vect) {
    if (AVRGPIO::_interrupt_6) {
        AVRGPIO::_interrupt_6();
    }
}   

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)


// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )

void AVRGPIO::pinMode(uint8_t pin, uint8_t mode) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *reg;

    if (port == NOT_A_PIN) return;

    // JWS: can I let the optimizer do this?
    reg = portModeRegister(port);

    if (mode == HAL_GPIO_INPUT) {
        uint8_t oldSREG = SREG;
                cli();
        *reg &= ~bit;
        SREG = oldSREG;
    } else {
        uint8_t oldSREG = SREG;
                cli();
        *reg |= bit;
        SREG = oldSREG;
    }
}

int8_t AVRGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return analogInputToDigitalPin(pin);
}

uint8_t AVRGPIO::read(uint8_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PIN) return 0;

    if (*portInputRegister(port) & bit) return 1;
    return 0;
}

void AVRGPIO::write(uint8_t pin, uint8_t value) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (value == 0) {
        *out &= ~bit;
    } else {
        *out |= bit;
    }

    SREG = oldSREG;
}

void AVRGPIO::toggle(uint8_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    *out ^= bit;

    SREG = oldSREG;
}

/* Implement GPIO Interrupt 6, used for MPU6000 data ready on APM2. */
bool AVRGPIO::attach_interrupt(
        uint8_t interrupt_num, AP_HAL::Proc proc, uint8_t mode) {
    /* Mode is to set the ISCn0 and ISCn1 bits.
     * These correspond to the GPIO_INTERRUPT_ defs in AP_HAL.h */
    if (!((mode == HAL_GPIO_INTERRUPT_LOW)||
          (mode == HAL_GPIO_INTERRUPT_HIGH)||
          (mode == HAL_GPIO_INTERRUPT_FALLING)||
          (mode == HAL_GPIO_INTERRUPT_RISING))) return false;
    if (interrupt_num == 6) {
	uint8_t oldSREG = SREG;
	cli();	
        _interrupt_6 = proc;
        /* Set the ISC60 and ICS61 bits in EICRB according to the value
         * of mode. */
        EICRB = (EICRB & ~((1 << ISC60) | (1 << ISC61))) | (mode << ISC60);
        EIMSK |= (1 << INT6);
	SREG = oldSREG;
        return true;
    } else {
        return false;
    }
}


AP_HAL::DigitalSource* AVRGPIO::channel(uint16_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PIN) return NULL;
    return new AVRDigitalSource(bit, port);
}

void AVRDigitalSource::mode(uint8_t output) {
    const uint8_t bit = _bit;
    const uint8_t port = _port;

    volatile uint8_t* reg;
    reg = portModeRegister(port);

    if (output == HAL_GPIO_INPUT) {
        uint8_t oldSREG = SREG;
                cli();
        *reg &= ~bit;
        SREG = oldSREG;
    } else {
        uint8_t oldSREG = SREG;
                cli();
        *reg |= bit;
        SREG = oldSREG;
    }
}

uint8_t AVRDigitalSource::read() {
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    if (*portInputRegister(port) & bit) return 1;
    return 0;
}

void AVRDigitalSource::write(uint8_t value) {
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    volatile uint8_t* out;
    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (value == 0) {
        *out &= ~bit;
    } else {
        *out |= bit;
    }

    SREG = oldSREG;
}

void AVRDigitalSource::pulse() {
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    volatile uint8_t* out;
    out = portOutputRegister(_port);
	//*out = 1;
	//*out = 0;
	uint8_t oldSREG = SREG;
	*out |= bit;
	*out &= ~bit;

    
   // cli();

   /* if (value == 0) {
        *out &= ~bit;
    } else {
        *out |= bit;
    }*/

    SREG = oldSREG;
}

void AVRDigitalSource::toggle() {
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    volatile uint8_t* out;
    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    *out ^= bit;

    SREG = oldSREG;
}

unsigned long AVRDigitalSource::pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);
	unsigned long width = 0; // keep initialization out of time critical area
	
	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration.
	unsigned long numloops = 0;
	//unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
	unsigned long maxloops = timeout*clockCyclesPerMicrosecond() / 16;
	// wait for any previous pulse to end
	while ((*portInputRegister(port) & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to start
	while ((*portInputRegister(port) & bit) != stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to stop
	while ((*portInputRegister(port) & bit) == stateMask) {
		if (numloops++ == maxloops)
			return 0;
		width++;
	}

	// convert the reading to microseconds. The loop has been determined
	// to be 20 clock cycles long and have about 16 clocks between the edge
	// and the start of the loop. There will be some error introduced by
	// the interrupt handlers.
	return clockCyclesToMicroseconds(width * 21 + 16); 
}
/*
  return true when USB is connected
 */
bool AVRGPIO::usb_connected(void)
{
#if HAL_GPIO_USB_MUX_PIN != -1
    pinMode(HAL_GPIO_USB_MUX_PIN, HAL_GPIO_INPUT);
    return !read(HAL_GPIO_USB_MUX_PIN);
#else
    return false;
#endif
}


#endif
