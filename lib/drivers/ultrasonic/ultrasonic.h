#ifndef ultrasonic_h
#define ultrasonic_h

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

void ultrasonic_setup_pins(uint trigPin, uint echoPin);
uint16_t ultrasonic_get_distance_cm(uint trigPin, uint echoPin);
uint16_t ultrasonic_get_distance_temperature_compansated_cm(uint trigPin, uint echoPin, float temperature);
uint16_t ultrasonic_get_inch(uint trigPin, uint echoPin);
int ultrasonic_lpf(int sensor_reading, float filterValue, float smoothedValue);
#endif