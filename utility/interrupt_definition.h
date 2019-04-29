#pragma once

// ESP8266 & ESP32 require to manually specify that we want ISR handler in RAM
// Allow to use the same ATTR token for both platforms
#if defined(ESP8266) || defined(ESP32)

#ifndef IRAM_ATTR
#define IRAM_ATTR ICACHE_RAM_ATTR
#endif

#define ENCODER_DEFINE_ISR(pin) static void IRAM_ATTR isr##pin (void) { Encoder::update(interruptArgs[pin]); }

#else

#define IRAM_ATTR
#define ENCODER_DEFINE_ISR(pin) static void isr##pin (void) { Encoder::update(interruptArgs[pin]); }

#endif
