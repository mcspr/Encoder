/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "utility/direct_pin_read.h"

#if defined(ENCODER_USE_INTERRUPTS) || !defined(ENCODER_DO_NOT_USE_INTERRUPTS)
#define ENCODER_USE_INTERRUPTS
#define ENCODER_ARGLIST_SIZE CORE_NUM_INTERRUPT
#include "utility/interrupt_pins.h"
#include "utility/interrupt_definition.h"
#ifdef ENCODER_OPTIMIZE_INTERRUPTS
#include "utility/interrupt_config.h"
#endif
#else
#define ENCODER_ARGLIST_SIZE 0
#endif

// All the data needed by interrupts is consolidated into this ugly struct
// to facilitate assembly language optimizing of the speed critical update.
// The assembly code uses auto-incrementing addressing modes, so the struct
// must remain in exactly this order.
typedef struct {
	volatile IO_REG_TYPE * pin1_register;
	volatile IO_REG_TYPE * pin2_register;
	IO_REG_TYPE            pin1_bitmask;
	IO_REG_TYPE            pin2_bitmask;
	uint8_t                state;
	int32_t                position;
} Encoder_internal_state_t;

class Encoder {

private:

    Encoder_internal_state_t encoder;
#ifdef ENCODER_USE_INTERRUPTS
    uint8_t interrupts_in_use;
#endif

public:

    Encoder(uint8_t, uint8_t);
    ~Encoder();

    static void IRAM_ATTR update(Encoder_internal_state_t *);

    // For "inline" to work, implementation must be here

    #ifdef ENCODER_USE_INTERRUPTS
    inline int32_t read() {
        if (interrupts_in_use < 2) {
            noInterrupts();
            update(&encoder);
        } else {
            noInterrupts();
        }
        int32_t ret = encoder.position;
        interrupts();
        return ret;
    }

    inline int32_t readAndReset() {
        if (interrupts_in_use < 2) {
            noInterrupts();
            update(&encoder);
        } else {
            noInterrupts();
        }
        int32_t ret = encoder.position;
        encoder.position = 0;
        interrupts();
        return ret;
    }

    inline void write(int32_t p) {
        noInterrupts();
        encoder.position = p;
        interrupts();
    }

    #else // !defined(ENCODER_USE_INTERRUPTS)

    inline int32_t read() {
        update(encoder);
        return encoder.position;
    }

    inline int32_t readAndReset() {
        update(encoder);
        int32_t ret = encoder.position;
        encoder.position = 0;
        return ret;
    }

    inline void write(int32_t p) {
        encoder.position = p;
    }

    #endif

}; // class Encoder {}
