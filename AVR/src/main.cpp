#include "main.h"

int main(void) {
    uart_init();
    adc_init();
    ultrasonic_init();

    char buffer[16];
    int lastError = 0;

    // Let things settle, then announce we're alive even before any sensor
    // reading happens - if this line never shows up on the ESP32 monitor,
    // the problem is UART wiring/baud, not the ADC or the sensors.
    _delay_ms(200);
    uart_print_string("E:0\n");

    while (1) {
        int distance_cm = ultrasonic_read_cm();
        if (distance_cm != -1 && distance_cm < STOP_DISTANCE_CM) {
            // Obstacle too close - tell the ESP32 to stop instead of sending
            // a line-error reading this cycle.
            uart_print_string("S:1\n");
            _delay_ms(20);
            continue;
        }

        unsigned int left, mid, right;
        unsigned char okLeft  = adc_read(0, &left);
        unsigned char okMid   = adc_read(1, &mid);
        unsigned char okRight = adc_read(2, &right);

        if (!okLeft || !okMid || !okRight) {
            // ADC didn't respond - report it so it's visible on the ESP32
            // side instead of just going quiet.
            uart_print_string("E:ADC_TIMEOUT\n");
            _delay_ms(200);
            continue;
        }

        long total = (long)left + mid + right;
        int error;

        if (total < MIN_TOTAL_SIGNAL) {
            // No sensor sees the line strongly enough to trust a reading.
            // Don't just repeat the last (often weak) error - that lets the
            // robot keep drifting the wrong way through a sharp turn. Instead
            // steer at full strength toward whichever side the line was last
            // seen on, so it actively searches back toward it.
            if (lastError > 0)      error = LOST_LINE_ERROR;
            else if (lastError < 0) error = -LOST_LINE_ERROR;
            else                    error = 0;
        } else {
            long weightedSum = ((long)left * WEIGHT_LEFT) +
                                ((long)mid * WEIGHT_MID) +
                                ((long)right * WEIGHT_RIGHT);
            // weightedSum / total is roughly in -1..1, scaled by ERROR_SCALE
            error = (int)((weightedSum * ERROR_SCALE) / total);
            lastError = error;
        }

        uart_transmit('E');
        uart_transmit(':');
        itoa(error, buffer, 10);
        uart_print_string(buffer);
        uart_transmit('\n');

        _delay_ms(20);  // ~50 updates per second
    }

    return 0;
}