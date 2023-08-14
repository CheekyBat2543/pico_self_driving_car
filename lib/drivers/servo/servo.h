#ifndef servo_h
#define servo_h

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

void setMillis(uint servoPin, float millis);
void setServo(uint servoPin, float startMillis);
void setMillisRound(uint servoPin, float startMillis, int roundInterval);
#endif