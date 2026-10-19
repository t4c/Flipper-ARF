#ifndef HOST_INPUT_MOCK_H
#define HOST_INPUT_MOCK_H
#include "../furi.h"

typedef enum {
    InputKeyUp, InputKeyDown, InputKeyRight, InputKeyLeft, InputKeyOk, InputKeyBack, InputKeyMAX
} InputKey;

typedef struct {
    const GpioPin* gpio;
    InputKey key;
    bool inverted;
    const char* name;
} InputPin;

/* Empty input map on host (no physical keys). */
static const InputPin input_pins[] = { { 0, InputKeyMAX, false, "none" } };
static const size_t input_pins_count = 0;

#endif
