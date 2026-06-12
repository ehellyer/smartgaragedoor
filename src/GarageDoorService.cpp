// =============================================================================
//  GarageDoorService.cpp  —  Out-of-line definitions
//
//  The obstruction-sensor ISR must be DEFINED here, not inline in the header:
//  IRAM_ATTR places the function in .iram1, and when such a function is
//  emitted inline from a header the Xtensa linker can end up with its literal
//  pool after the code, failing with
//      "dangerous relocation: l32r: literal placed after use".
// =============================================================================

#include "GarageDoorService.h"

volatile uint32_t GarageDoorService::obstLowCount = 0;

// Falling-edge ISR on the obstruction sensor pin — counts the ~7 ms LOW
// pulses the safety-beam line emits while the beam is clear.
void IRAM_ATTR GarageDoorService::obstISR() {
    obstLowCount = obstLowCount + 1;
}
