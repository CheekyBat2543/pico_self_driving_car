// Control a servo by degrees or millis

#include "servo.h"

float clockDiv = 64;
float wrap = 39062;

void setMillis(uint servoPin, float millis)
{
    pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

void setServo(uint servoPin, float startMillis)
{
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(5);
    clockDiv = 64;
    wrap = 39062;

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}

void setMillisRound(uint servoPin, float startMillis, int roundInterval){
    int millis = (int)startMillis;
    millis = (int)millis / roundInterval;
    millis = millis * roundInterval;
    setMillis(servoPin, (float)millis);
}