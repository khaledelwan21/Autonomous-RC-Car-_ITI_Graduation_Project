#ifndef MAIN_H
#define MAIN_H

#define F_CPU 8000000UL   // change according to your actual clock (1MHz, 8MHz, 16MHz...)
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

// --- UART ---
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

void uart_init(void) {
    UBRRH = (unsigned char)(UBRR_VALUE >> 8);
    UBRRL = (unsigned char)UBRR_VALUE;

    UCSRB = (1 << TXEN) | (1 << RXEN);

    // 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void uart_transmit(unsigned char data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

void uart_print_string(const char *str) {
    while (*str) {
        uart_transmit(*str);
        str++;
    }
}

// --- ADC (3 analog IR sensors on ADC0, ADC1, ADC2 / PA0, PA1, PA2) ---
void adc_init(void) {
    // AVCC as reference voltage, right-adjusted result
    ADMUX = (1 << REFS0);

    // Enable ADC, prescaler = 64 -> ADC clock = 8MHz / 64 = 125kHz (within the
    // recommended 50-200kHz range for full 10-bit accuracy)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

// Returns 1 on success (result written to *result), 0 if the conversion
// never finished within the timeout. A bounded wait means a hardware fault
// (e.g. AVCC/AREF not wired) shows up as a printed error instead of the
// whole board silently hanging forever with no UART output at all.
unsigned char adc_read(unsigned char channel, unsigned int *result) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    ADCSRA |= (1 << ADSC);

    unsigned int guard = 0;
    while (ADCSRA & (1 << ADSC)) {
        guard++;
        if (guard > 60000) {
            return 0;  // conversion never completed - likely AVCC/AREF wiring issue
        }
    }

    *result = ADC;
    return 1;
}

// --- Ultrasonic (HC-SR04) obstacle detection ---
// TRIG on PD6, ECHO on PB2 - matches actual wiring.
#define TRIG_PIN PD6
#define ECHO_PIN PB2

const int STOP_DISTANCE_CM = 10;

void ultrasonic_init(void) {
    DDRD |= (1 << TRIG_PIN);    // TRIG (PD6) as output
    DDRB &= ~(1 << ECHO_PIN);   // ECHO (PB2) as input
    PORTD &= ~(1 << TRIG_PIN);

    // Timer1, prescaler = 8 -> at F_CPU = 8MHz, each tick = 1us.
    // (If you change F_CPU, adjust the prescaler bits so 1 tick stays ~1us,
    // or adjust the /58 division in ultrasonic_read_cm accordingly.)
    TCCR1B = (1 << CS11);
}

// Returns distance in cm, or -1 if no valid echo was received (timeout,
// nothing in range, or sensor not wired/responding).
int ultrasonic_read_cm(void) {
    // Send a 10us trigger pulse
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);

    // Wait for ECHO to go high (start of pulse), bounded so a disconnected
    // sensor can't hang the whole robot.
    unsigned int guard = 0;
    while (!(PINB & (1 << ECHO_PIN))) {
        guard++;
        if (guard > 30000) return -1;
    }

    TCNT1 = 0;  // start timing right as the echo pulse begins

    // Wait for ECHO to go low again (end of pulse). Timeout is checked
    // against the timer's own tick count (not a loop-iteration counter) so
    // it can't drift with compiler/optimization changes, and it stops well
    // before TCNT1 (16-bit, max 65535 ticks ~ 65.5ms at this prescaler)
    // could silently overflow and wrap back to a small, wrong-looking value.
    const unsigned int MAX_ECHO_TICKS = 40000;  // ~40ms of travel time - well beyond the sensor's real max range, safely under the 65535 overflow point
    while (PINB & (1 << ECHO_PIN)) {
        if (TCNT1 > MAX_ECHO_TICKS) return -1;  // out of range / stuck high
    }

    unsigned int pulse_us = TCNT1;          // 1 tick = 1us at this prescaler
    int distance_cm = (int)(pulse_us / 58); // standard HC-SR04 conversion
    return distance_cm;
}

// --- Line position calculation ---
const int WEIGHT_LEFT = -1;
const int WEIGHT_MID = 0;
const int WEIGHT_RIGHT = 1;
const int ERROR_SCALE = 100;
const int MIN_TOTAL_SIGNAL = 30;
// When the line is completely lost (e.g. mid-turn), steer hard toward the
// side it was last seen on instead of drifting with a weak stale reading.
const int LOST_LINE_ERROR = 100;

#endif // MAIN_H