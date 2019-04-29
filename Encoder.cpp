#include "Encoder.h"

#if defined(ENCODER_USE_INTERRUPTS)
static uint8_t attach_interrupt(uint8_t, Encoder_internal_state_t *);
static Encoder_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE] = { nullptr };
#endif

Encoder::Encoder(uint8_t pin1, uint8_t pin2) {
    #ifdef INPUT_PULLUP
        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);
    #else
        pinMode(pin1, INPUT);
        digitalWrite(pin1, HIGH);
        pinMode(pin2, INPUT);
        digitalWrite(pin2, HIGH);
    #endif

    encoder.pin1_register = PIN_TO_BASEREG(pin1);
    encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
    encoder.pin2_register = PIN_TO_BASEREG(pin2);
    encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
    encoder.position = 0;
    // allow time for a passive R-C filter to charge
    // through the pullup resistors, before reading
    // the initial state
    delayMicroseconds(2000);
    uint8_t s = 0;
    if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
    if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
    encoder.state = s;
    #ifdef ENCODER_USE_INTERRUPTS
        interrupts_in_use = attach_interrupt(pin1, &encoder);
        interrupts_in_use += attach_interrupt(pin2, &encoder);
    #endif
    //update_finishup();  // to force linker to include the code (does not work)
}

Encoder::~Encoder() {
    #ifdef ENCODER_USE_INTERRUPTS
        noInterrupts();
        for (unsigned char n=0; n < ENCODER_ARGLIST_SIZE; n++) {
            Encoder_internal_state_t * arg = interruptArgs[n];
            if (arg && (arg == &encoder)) {
                detachInterrupt(n);
                interruptArgs[n] = nullptr;
            }
        }
        interrupts();
    #endif
}

// update() is not meant to be called from outside Encoder,
// but it is public to allow static interrupt routines.
// DO NOT call update() directly from sketches.

// Note: IRAM_ATTR is required for ESP8266 / ESP32
// It is defined as "nothing" on other platforms

//                           _______         _______
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

//	new	new	old	old
//	pin2	pin1	pin2	pin1	Result
//	----	----	----	----	------
//	0	0	0	0	no movement
//	0	0	0	1	+1
//	0	0	1	0	-1
//	0	0	1	1	+2  (assume pin1 edges only)
//	0	1	0	0	-1
//	0	1	0	1	no movement
//	0	1	1	0	-2  (assume pin1 edges only)
//	0	1	1	1	+1
//	1	0	0	0	+1
//	1	0	0	1	-2  (assume pin1 edges only)
//	1	0	1	0	no movement
//	1	0	1	1	-1
//	1	1	0	0	+2  (assume pin1 edges only)
//	1	1	0	1	-1
//	1	1	1	0	+1
//	1	1	1	1	no movement

void IRAM_ATTR Encoder::update(Encoder_internal_state_t *arg) {
    #if defined(__AVR__)
    // The compiler believes this is just 1 line of code, so
    // it will inline this function into each interrupt
    // handler.  That's a tiny bit faster, but grows the code.
    // Especially when used with ENCODER_OPTIMIZE_INTERRUPTS,
    // the inline nature allows the ISR prologue and epilogue
    // to only save/restore necessary registers, for very nice
    // speed increase.
    asm volatile (
            "ld	r30, X+"		"\n\t"
            "ld	r31, X+"		"\n\t"
            "ld	r24, Z"			"\n\t"	// r24 = pin1 input
            "ld	r30, X+"		"\n\t"
            "ld	r31, X+"		"\n\t"
            "ld	r25, Z"			"\n\t"  // r25 = pin2 input
            "ld	r30, X+"		"\n\t"  // r30 = pin1 mask
            "ld	r31, X+"		"\n\t"	// r31 = pin2 mask
            "ld	r22, X"			"\n\t"	// r22 = state
            "andi	r22, 3"			"\n\t"
            "and	r24, r30"		"\n\t"
            "breq	L%=1"			"\n\t"	// if (pin1)
            "ori	r22, 4"			"\n\t"	//	state |= 4
            "L%=1:"	"and	r25, r31"		"\n\t"
            "breq	L%=2"			"\n\t"	// if (pin2)
            "ori	r22, 8"			"\n\t"	//	state |= 8
            "L%=2:" "ldi	r30, lo8(pm(L%=table))"	"\n\t"
            "ldi	r31, hi8(pm(L%=table))"	"\n\t"
            "add	r30, r22"		"\n\t"
            "adc	r31, __zero_reg__"	"\n\t"
            "asr	r22"			"\n\t"
            "asr	r22"			"\n\t"
            "st	X+, r22"		"\n\t"  // store new state
            "ld	r22, X+"		"\n\t"
            "ld	r23, X+"		"\n\t"
            "ld	r24, X+"		"\n\t"
            "ld	r25, X+"		"\n\t"
            "ijmp"				"\n\t"	// jumps to update_finishup()
            // TODO move this table to another static function,
            // so it doesn't get needlessly duplicated.  Easier
            // said than done, due to linker issues and inlining
            "L%=table:"				"\n\t"
            "rjmp	L%=end"			"\n\t"	// 0
            "rjmp	L%=plus1"		"\n\t"	// 1
            "rjmp	L%=minus1"		"\n\t"	// 2
            "rjmp	L%=plus2"		"\n\t"	// 3
            "rjmp	L%=minus1"		"\n\t"	// 4
            "rjmp	L%=end"			"\n\t"	// 5
            "rjmp	L%=minus2"		"\n\t"	// 6
            "rjmp	L%=plus1"		"\n\t"	// 7
            "rjmp	L%=plus1"		"\n\t"	// 8
            "rjmp	L%=minus2"		"\n\t"	// 9
            "rjmp	L%=end"			"\n\t"	// 10
            "rjmp	L%=minus1"		"\n\t"	// 11
            "rjmp	L%=plus2"		"\n\t"	// 12
            "rjmp	L%=minus1"		"\n\t"	// 13
            "rjmp	L%=plus1"		"\n\t"	// 14
            "rjmp	L%=end"			"\n\t"	// 15
            "L%=minus2:"				"\n\t"
            "subi	r22, 2"			"\n\t"
            "sbci	r23, 0"			"\n\t"
            "sbci	r24, 0"			"\n\t"
            "sbci	r25, 0"			"\n\t"
            "rjmp	L%=store"		"\n\t"
            "L%=minus1:"				"\n\t"
            "subi	r22, 1"			"\n\t"
            "sbci	r23, 0"			"\n\t"
            "sbci	r24, 0"			"\n\t"
            "sbci	r25, 0"			"\n\t"
            "rjmp	L%=store"		"\n\t"
            "L%=plus2:"				"\n\t"
            "subi	r22, 254"		"\n\t"
            "rjmp	L%=z"			"\n\t"
            "L%=plus1:"				"\n\t"
            "subi	r22, 255"		"\n\t"
            "L%=z:"	"sbci	r23, 255"		"\n\t"
            "sbci	r24, 255"		"\n\t"
            "sbci	r25, 255"		"\n\t"
            "L%=store:"				"\n\t"
            "st	-X, r25"		"\n\t"
            "st	-X, r24"		"\n\t"
            "st	-X, r23"		"\n\t"
            "st	-X, r22"		"\n\t"
            "L%=end:"				"\n"
            : : "x" (arg) : "r22", "r23", "r24", "r25", "r30", "r31");
    #else // !defined(__AVR__)
        uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
        uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
        uint8_t state = arg->state & 3;
        if (p1val) state |= 4;
        if (p2val) state |= 8;
        arg->state = (state >> 2);
        switch (state) {
            case 1: case 7: case 8: case 14:
                arg->position++;
                return;
            case 2: case 4: case 11: case 13:
                arg->position--;
                return;
            case 3: case 12:
                arg->position += 2;
                return;
            case 6: case 9:
                arg->position -= 2;
                return;
        }
    #endif // defined(__AVR__)
}

/*

// TODO: this must be a no inline function
// even noinline does not seem to solve difficult
// problems with this.  Oh well, it was only meant
// to shrink code size - there's no performance
// improvement in this, only code size reduction.

void __attribute__((noinline)) Encoder::update_finishup(void) {
    #if defined(__AVR__)
		asm volatile (
			"ldi	r30, lo8(pm(Ltable))"	"\n\t"
			"ldi	r31, hi8(pm(Ltable))"	"\n\t"
		"Ltable:"				"\n\t"
			"rjmp	L%=end"			"\n\t"	// 0
			"rjmp	L%=plus1"		"\n\t"	// 1
			"rjmp	L%=minus1"		"\n\t"	// 2
			"rjmp	L%=plus2"		"\n\t"	// 3
			"rjmp	L%=minus1"		"\n\t"	// 4
			"rjmp	L%=end"			"\n\t"	// 5
			"rjmp	L%=minus2"		"\n\t"	// 6
			"rjmp	L%=plus1"		"\n\t"	// 7
			"rjmp	L%=plus1"		"\n\t"	// 8
			"rjmp	L%=minus2"		"\n\t"	// 9
			"rjmp	L%=end"			"\n\t"	// 10
			"rjmp	L%=minus1"		"\n\t"	// 11
			"rjmp	L%=plus2"		"\n\t"	// 12
			"rjmp	L%=minus1"		"\n\t"	// 13
			"rjmp	L%=plus1"		"\n\t"	// 14
			"rjmp	L%=end"			"\n\t"	// 15
		"L%=minus2:"				"\n\t"
			"subi	r22, 2"			"\n\t"
			"sbci	r23, 0"			"\n\t"
			"sbci	r24, 0"			"\n\t"
			"sbci	r25, 0"			"\n\t"
			"rjmp	L%=store"		"\n\t"
		"L%=minus1:"				"\n\t"
			"subi	r22, 1"			"\n\t"
			"sbci	r23, 0"			"\n\t"
			"sbci	r24, 0"			"\n\t"
			"sbci	r25, 0"			"\n\t"
			"rjmp	L%=store"		"\n\t"
		"L%=plus2:"				"\n\t"
			"subi	r22, 254"		"\n\t"
			"rjmp	L%=z"			"\n\t"
		"L%=plus1:"				"\n\t"
			"subi	r22, 255"		"\n\t"
		"L%=z:"	"sbci	r23, 255"		"\n\t"
			"sbci	r24, 255"		"\n\t"
			"sbci	r25, 255"		"\n\t"
		"L%=store:"				"\n\t"
			"st	-X, r25"		"\n\t"
			"st	-X, r24"		"\n\t"
			"st	-X, r23"		"\n\t"
			"st	-X, r22"		"\n\t"
		"L%=end:"				"\n"
		: : : "r22", "r23", "r24", "r25", "r30", "r31");
    #endif
}
*/

#if defined(ENCODER_USE_INTERRUPTS) && !defined(ENCODER_OPTIMIZE_INTERRUPTS)

#ifdef CORE_INT0_PIN
ENCODER_DEFINE_ISR(0)
#endif
#ifdef CORE_INT1_PIN
ENCODER_DEFINE_ISR(1)
#endif
#ifdef CORE_INT2_PIN
ENCODER_DEFINE_ISR(2)
#endif
#ifdef CORE_INT3_PIN
ENCODER_DEFINE_ISR(3)
#endif
#ifdef CORE_INT4_PIN
ENCODER_DEFINE_ISR(4)
#endif
#ifdef CORE_INT5_PIN
ENCODER_DEFINE_ISR(5)
#endif
#ifdef CORE_INT6_PIN
ENCODER_DEFINE_ISR(6)
#endif
#ifdef CORE_INT7_PIN
ENCODER_DEFINE_ISR(7)
#endif
#ifdef CORE_INT8_PIN
ENCODER_DEFINE_ISR(8)
#endif
#ifdef CORE_INT9_PIN
ENCODER_DEFINE_ISR(9)
#endif
#ifdef CORE_INT10_PIN
ENCODER_DEFINE_ISR(10)
#endif
#ifdef CORE_INT11_PIN
ENCODER_DEFINE_ISR(11)
#endif
#ifdef CORE_INT12_PIN
ENCODER_DEFINE_ISR(12)
#endif
#ifdef CORE_INT13_PIN
ENCODER_DEFINE_ISR(13)
#endif
#ifdef CORE_INT14_PIN
ENCODER_DEFINE_ISR(14)
#endif
#ifdef CORE_INT15_PIN
ENCODER_DEFINE_ISR(15)
#endif
#ifdef CORE_INT16_PIN
ENCODER_DEFINE_ISR(16)
#endif
#ifdef CORE_INT17_PIN
ENCODER_DEFINE_ISR(17)
#endif
#ifdef CORE_INT18_PIN
ENCODER_DEFINE_ISR(18)
#endif
#ifdef CORE_INT19_PIN
ENCODER_DEFINE_ISR(19)
#endif
#ifdef CORE_INT20_PIN
ENCODER_DEFINE_ISR(20)
#endif
#ifdef CORE_INT21_PIN
ENCODER_DEFINE_ISR(21)
#endif
#ifdef CORE_INT22_PIN
ENCODER_DEFINE_ISR(22)
#endif
#ifdef CORE_INT23_PIN
ENCODER_DEFINE_ISR(23)
#endif
#ifdef CORE_INT24_PIN
ENCODER_DEFINE_ISR(24)
#endif
#ifdef CORE_INT25_PIN
ENCODER_DEFINE_ISR(25)
#endif
#ifdef CORE_INT26_PIN
ENCODER_DEFINE_ISR(26)
#endif
#ifdef CORE_INT27_PIN
ENCODER_DEFINE_ISR(27)
#endif
#ifdef CORE_INT28_PIN
ENCODER_DEFINE_ISR(28)
#endif
#ifdef CORE_INT29_PIN
ENCODER_DEFINE_ISR(29)
#endif
#ifdef CORE_INT30_PIN
ENCODER_DEFINE_ISR(30)
#endif
#ifdef CORE_INT31_PIN
ENCODER_DEFINE_ISR(31)
#endif
#ifdef CORE_INT32_PIN
ENCODER_DEFINE_ISR(32)
#endif
#ifdef CORE_INT33_PIN
ENCODER_DEFINE_ISR(33)
#endif
#ifdef CORE_INT34_PIN
ENCODER_DEFINE_ISR(34)
#endif
#ifdef CORE_INT35_PIN
ENCODER_DEFINE_ISR(35)
#endif
#ifdef CORE_INT36_PIN
ENCODER_DEFINE_ISR(36)
#endif
#ifdef CORE_INT37_PIN
ENCODER_DEFINE_ISR(37)
#endif
#ifdef CORE_INT38_PIN
ENCODER_DEFINE_ISR(38)
#endif
#ifdef CORE_INT39_PIN
ENCODER_DEFINE_ISR(39)
#endif
#ifdef CORE_INT40_PIN
ENCODER_DEFINE_ISR(40)
#endif
#ifdef CORE_INT41_PIN
ENCODER_DEFINE_ISR(41)
#endif
#ifdef CORE_INT42_PIN
ENCODER_DEFINE_ISR(42)
#endif
#ifdef CORE_INT43_PIN
ENCODER_DEFINE_ISR(43)
#endif
#ifdef CORE_INT44_PIN
ENCODER_DEFINE_ISR(44)
#endif
#ifdef CORE_INT45_PIN
ENCODER_DEFINE_ISR(45)
#endif
#ifdef CORE_INT46_PIN
ENCODER_DEFINE_ISR(46)
#endif
#ifdef CORE_INT47_PIN
ENCODER_DEFINE_ISR(47)
#endif
#ifdef CORE_INT48_PIN
ENCODER_DEFINE_ISR(48)
#endif
#ifdef CORE_INT49_PIN
ENCODER_DEFINE_ISR(49)
#endif
#ifdef CORE_INT50_PIN
ENCODER_DEFINE_ISR(50)
#endif
#ifdef CORE_INT51_PIN
ENCODER_DEFINE_ISR(51)
#endif
#ifdef CORE_INT52_PIN
ENCODER_DEFINE_ISR(52)
#endif
#ifdef CORE_INT53_PIN
ENCODER_DEFINE_ISR(53)
#endif
#ifdef CORE_INT54_PIN
ENCODER_DEFINE_ISR(54)
#endif
#ifdef CORE_INT55_PIN
ENCODER_DEFINE_ISR(55)
#endif
#ifdef CORE_INT56_PIN
ENCODER_DEFINE_ISR(56)
#endif
#ifdef CORE_INT57_PIN
ENCODER_DEFINE_ISR(57)
#endif
#ifdef CORE_INT58_PIN
ENCODER_DEFINE_ISR(58)
#endif
#ifdef CORE_INT59_PIN
ENCODER_DEFINE_ISR(59)
#endif

// this giant function is an unfortunate consequence of Arduino's
// attachInterrupt function not supporting any way to pass a pointer
// or other context to the attached function.
// TODO: FunctionalInterrupt with recent ESP8266/ESP32 Cores allows to pass "state" directly
static uint8_t attach_interrupt(uint8_t pin, Encoder_internal_state_t *state) {
    switch (pin) {
    #ifdef CORE_INT0_PIN
        case CORE_INT0_PIN:
            interruptArgs[0] = state;
            attachInterrupt(0, isr0, CHANGE);
            break;
    #endif
    #ifdef CORE_INT1_PIN
        case CORE_INT1_PIN:
            interruptArgs[1] = state;
            attachInterrupt(1, isr1, CHANGE);
            break;
    #endif
    #ifdef CORE_INT2_PIN
        case CORE_INT2_PIN:
            interruptArgs[2] = state;
            attachInterrupt(2, isr2, CHANGE);
            break;
    #endif
    #ifdef CORE_INT3_PIN
        case CORE_INT3_PIN:
            interruptArgs[3] = state;
            attachInterrupt(3, isr3, CHANGE);
            break;
    #endif
    #ifdef CORE_INT4_PIN
        case CORE_INT4_PIN:
            interruptArgs[4] = state;
            attachInterrupt(4, isr4, CHANGE);
            break;
    #endif
    #ifdef CORE_INT5_PIN
        case CORE_INT5_PIN:
            interruptArgs[5] = state;
            attachInterrupt(5, isr5, CHANGE);
            break;
    #endif
    #ifdef CORE_INT6_PIN
        case CORE_INT6_PIN:
            interruptArgs[6] = state;
            attachInterrupt(6, isr6, CHANGE);
            break;
    #endif
    #ifdef CORE_INT7_PIN
        case CORE_INT7_PIN:
            interruptArgs[7] = state;
            attachInterrupt(7, isr7, CHANGE);
            break;
    #endif
    #ifdef CORE_INT8_PIN
        case CORE_INT8_PIN:
            interruptArgs[8] = state;
            attachInterrupt(8, isr8, CHANGE);
            break;
    #endif
    #ifdef CORE_INT9_PIN
        case CORE_INT9_PIN:
            interruptArgs[9] = state;
            attachInterrupt(9, isr9, CHANGE);
            break;
    #endif
    #ifdef CORE_INT10_PIN
        case CORE_INT10_PIN:
            interruptArgs[10] = state;
            attachInterrupt(10, isr10, CHANGE);
            break;
    #endif
    #ifdef CORE_INT11_PIN
        case CORE_INT11_PIN:
            interruptArgs[11] = state;
            attachInterrupt(11, isr11, CHANGE);
            break;
    #endif
    #ifdef CORE_INT12_PIN
        case CORE_INT12_PIN:
            interruptArgs[12] = state;
            attachInterrupt(12, isr12, CHANGE);
            break;
    #endif
    #ifdef CORE_INT13_PIN
        case CORE_INT13_PIN:
            interruptArgs[13] = state;
            attachInterrupt(13, isr13, CHANGE);
            break;
    #endif
    #ifdef CORE_INT14_PIN
        case CORE_INT14_PIN:
            interruptArgs[14] = state;
            attachInterrupt(14, isr14, CHANGE);
            break;
    #endif
    #ifdef CORE_INT15_PIN
        case CORE_INT15_PIN:
            interruptArgs[15] = state;
            attachInterrupt(15, isr15, CHANGE);
            break;
    #endif
    #ifdef CORE_INT16_PIN
        case CORE_INT16_PIN:
            interruptArgs[16] = state;
            attachInterrupt(16, isr16, CHANGE);
            break;
    #endif
    #ifdef CORE_INT17_PIN
        case CORE_INT17_PIN:
            interruptArgs[17] = state;
            attachInterrupt(17, isr17, CHANGE);
            break;
    #endif
    #ifdef CORE_INT18_PIN
        case CORE_INT18_PIN:
            interruptArgs[18] = state;
            attachInterrupt(18, isr18, CHANGE);
            break;
    #endif
    #ifdef CORE_INT19_PIN
        case CORE_INT19_PIN:
            interruptArgs[19] = state;
            attachInterrupt(19, isr19, CHANGE);
            break;
    #endif
    #ifdef CORE_INT20_PIN
        case CORE_INT20_PIN:
            interruptArgs[20] = state;
            attachInterrupt(20, isr20, CHANGE);
            break;
    #endif
    #ifdef CORE_INT21_PIN
        case CORE_INT21_PIN:
            interruptArgs[21] = state;
            attachInterrupt(21, isr21, CHANGE);
            break;
    #endif
    #ifdef CORE_INT22_PIN
        case CORE_INT22_PIN:
            interruptArgs[22] = state;
            attachInterrupt(22, isr22, CHANGE);
            break;
    #endif
    #ifdef CORE_INT23_PIN
        case CORE_INT23_PIN:
            interruptArgs[23] = state;
            attachInterrupt(23, isr23, CHANGE);
            break;
    #endif
    #ifdef CORE_INT24_PIN
        case CORE_INT24_PIN:
            interruptArgs[24] = state;
            attachInterrupt(24, isr24, CHANGE);
            break;
    #endif
    #ifdef CORE_INT25_PIN
        case CORE_INT25_PIN:
            interruptArgs[25] = state;
            attachInterrupt(25, isr25, CHANGE);
            break;
    #endif
    #ifdef CORE_INT26_PIN
        case CORE_INT26_PIN:
            interruptArgs[26] = state;
            attachInterrupt(26, isr26, CHANGE);
            break;
    #endif
    #ifdef CORE_INT27_PIN
        case CORE_INT27_PIN:
            interruptArgs[27] = state;
            attachInterrupt(27, isr27, CHANGE);
            break;
    #endif
    #ifdef CORE_INT28_PIN
        case CORE_INT28_PIN:
            interruptArgs[28] = state;
            attachInterrupt(28, isr28, CHANGE);
            break;
    #endif
    #ifdef CORE_INT29_PIN
        case CORE_INT29_PIN:
            interruptArgs[29] = state;
            attachInterrupt(29, isr29, CHANGE);
            break;
    #endif

    #ifdef CORE_INT30_PIN
        case CORE_INT30_PIN:
            interruptArgs[30] = state;
            attachInterrupt(30, isr30, CHANGE);
            break;
    #endif
    #ifdef CORE_INT31_PIN
        case CORE_INT31_PIN:
            interruptArgs[31] = state;
            attachInterrupt(31, isr31, CHANGE);
            break;
    #endif
    #ifdef CORE_INT32_PIN
        case CORE_INT32_PIN:
            interruptArgs[32] = state;
            attachInterrupt(32, isr32, CHANGE);
            break;
    #endif
    #ifdef CORE_INT33_PIN
        case CORE_INT33_PIN:
            interruptArgs[33] = state;
            attachInterrupt(33, isr33, CHANGE);
            break;
    #endif
    #ifdef CORE_INT34_PIN
        case CORE_INT34_PIN:
            interruptArgs[34] = state;
            attachInterrupt(34, isr34, CHANGE);
            break;
    #endif
    #ifdef CORE_INT35_PIN
        case CORE_INT35_PIN:
            interruptArgs[35] = state;
            attachInterrupt(35, isr35, CHANGE);
            break;
    #endif
    #ifdef CORE_INT36_PIN
        case CORE_INT36_PIN:
            interruptArgs[36] = state;
            attachInterrupt(36, isr36, CHANGE);
            break;
    #endif
    #ifdef CORE_INT37_PIN
        case CORE_INT37_PIN:
            interruptArgs[37] = state;
            attachInterrupt(37, isr37, CHANGE);
            break;
    #endif
    #ifdef CORE_INT38_PIN
        case CORE_INT38_PIN:
            interruptArgs[38] = state;
            attachInterrupt(38, isr38, CHANGE);
            break;
    #endif
    #ifdef CORE_INT39_PIN
        case CORE_INT39_PIN:
            interruptArgs[39] = state;
            attachInterrupt(39, isr39, CHANGE);
            break;
    #endif
    #ifdef CORE_INT40_PIN
        case CORE_INT40_PIN:
            interruptArgs[40] = state;
            attachInterrupt(40, isr40, CHANGE);
            break;
    #endif
    #ifdef CORE_INT41_PIN
        case CORE_INT41_PIN:
            interruptArgs[41] = state;
            attachInterrupt(41, isr41, CHANGE);
            break;
    #endif
    #ifdef CORE_INT42_PIN
        case CORE_INT42_PIN:
            interruptArgs[42] = state;
            attachInterrupt(42, isr42, CHANGE);
            break;
    #endif
    #ifdef CORE_INT43_PIN
        case CORE_INT43_PIN:
            interruptArgs[43] = state;
            attachInterrupt(43, isr43, CHANGE);
            break;
    #endif
    #ifdef CORE_INT44_PIN
        case CORE_INT44_PIN:
            interruptArgs[44] = state;
            attachInterrupt(44, isr44, CHANGE);
            break;
    #endif
    #ifdef CORE_INT45_PIN
        case CORE_INT45_PIN:
            interruptArgs[45] = state;
            attachInterrupt(45, isr45, CHANGE);
            break;
    #endif
    #ifdef CORE_INT46_PIN
        case CORE_INT46_PIN:
            interruptArgs[46] = state;
            attachInterrupt(46, isr46, CHANGE);
            break;
    #endif
    #ifdef CORE_INT47_PIN
        case CORE_INT47_PIN:
            interruptArgs[47] = state;
            attachInterrupt(47, isr47, CHANGE);
            break;
    #endif
    #ifdef CORE_INT48_PIN
        case CORE_INT48_PIN:
            interruptArgs[48] = state;
            attachInterrupt(48, isr48, CHANGE);
            break;
    #endif
    #ifdef CORE_INT49_PIN
        case CORE_INT49_PIN:
            interruptArgs[49] = state;
            attachInterrupt(49, isr49, CHANGE);
            break;
    #endif
    #ifdef CORE_INT50_PIN
        case CORE_INT50_PIN:
            interruptArgs[50] = state;
            attachInterrupt(50, isr50, CHANGE);
            break;
    #endif
    #ifdef CORE_INT51_PIN
        case CORE_INT51_PIN:
            interruptArgs[51] = state;
            attachInterrupt(51, isr51, CHANGE);
            break;
    #endif
    #ifdef CORE_INT52_PIN
        case CORE_INT52_PIN:
            interruptArgs[52] = state;
            attachInterrupt(52, isr52, CHANGE);
            break;
    #endif
    #ifdef CORE_INT53_PIN
        case CORE_INT53_PIN:
            interruptArgs[53] = state;
            attachInterrupt(53, isr53, CHANGE);
            break;
    #endif
    #ifdef CORE_INT54_PIN
        case CORE_INT54_PIN:
            interruptArgs[54] = state;
            attachInterrupt(54, isr54, CHANGE);
            break;
    #endif
    #ifdef CORE_INT55_PIN
        case CORE_INT55_PIN:
            interruptArgs[55] = state;
            attachInterrupt(55, isr55, CHANGE);
            break;
    #endif
    #ifdef CORE_INT56_PIN
        case CORE_INT56_PIN:
            interruptArgs[56] = state;
            attachInterrupt(56, isr56, CHANGE);
            break;
    #endif
    #ifdef CORE_INT57_PIN
        case CORE_INT57_PIN:
            interruptArgs[57] = state;
            attachInterrupt(57, isr57, CHANGE);
            break;
    #endif
    #ifdef CORE_INT58_PIN
        case CORE_INT58_PIN:
            interruptArgs[58] = state;
            attachInterrupt(58, isr58, CHANGE);
            break;
    #endif
    #ifdef CORE_INT59_PIN
        case CORE_INT59_PIN:
            interruptArgs[59] = state;
            attachInterrupt(59, isr59, CHANGE);
            break;
    #endif
        default:
            return 0;
    }

    return 1;
}
#endif // ENCODER_USE_INTERRUPTS


#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(__AVR__)
#if defined(INT0_vect) && CORE_NUM_INTERRUPT > 0
ISR(INT0_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(0)]); }
#endif
#if defined(INT1_vect) && CORE_NUM_INTERRUPT > 1
ISR(INT1_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(1)]); }
#endif
#if defined(INT2_vect) && CORE_NUM_INTERRUPT > 2
ISR(INT2_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(2)]); }
#endif
#if defined(INT3_vect) && CORE_NUM_INTERRUPT > 3
ISR(INT3_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(3)]); }
#endif
#if defined(INT4_vect) && CORE_NUM_INTERRUPT > 4
ISR(INT4_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(4)]); }
#endif
#if defined(INT5_vect) && CORE_NUM_INTERRUPT > 5
ISR(INT5_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(5)]); }
#endif
#if defined(INT6_vect) && CORE_NUM_INTERRUPT > 6
ISR(INT6_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(6)]); }
#endif
#if defined(INT7_vect) && CORE_NUM_INTERRUPT > 7
ISR(INT7_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(7)]); }
#endif
#endif // AVR
#if defined(attachInterrupt)
// Don't intefere with other libraries or sketch use of attachInterrupt()
// https://github.com/PaulStoffregen/Encoder/issues/8
#undef attachInterrupt
#endif
#endif // ENCODER_OPTIMIZE_INTERRUPTS
