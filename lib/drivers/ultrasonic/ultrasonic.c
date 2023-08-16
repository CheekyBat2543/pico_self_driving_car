//Get readings from ultrasonic sensor
#include "ultrasonic.h"

const uint trigger_timeout = 12000;
const uint timeout = 38000;

void ultrasonic_setup_pins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_put(trigPin, 0);
}

uint16_t ultrasonic_get_pulse(uint trigPin, uint echoPin)
{
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint16_t width = 0;

    while (gpio_get(echoPin) == 0) {
        if(width++ > trigger_timeout) return 0;
        sleep_us(1);
    }
    width = 0;
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1) 
    {
        width++;
        sleep_us(1);
        if (width > timeout) return 0U;
    }
    absolute_time_t endTime = get_absolute_time();
    
    return absolute_time_diff_us(startTime, endTime);
}

uint16_t ultrasonic_get_distance_cm(uint trigPin, uint echoPin)
{
    uint16_t pulseLength = ultrasonic_get_pulse(trigPin, echoPin);
    return pulseLength * 0.017f;
}

uint16_t ultrasonic_get_distance_temperature_compansated_cm(uint trigPin, uint echoPin, float temperature)
{
    uint16_t pulseLength = ultrasonic_get_pulse(trigPin, echoPin);
    return pulseLength * (331.0f + 0.6f * temperature) / 10000 / 2;
}

uint16_t ultrasonic_get_inch(uint trigPin, uint echoPin)
{
    uint16_t pulseLength = ultrasonic_get_pulse(trigPin, echoPin);
    return (long)pulseLength / 74.f / 2.f;
}

int ultrasonic_lpf(int sensor_reading, float filterValue, float smoothedValue){
  
  // Checking validity of filterValue; if beyond range, set to max/min value if out of range.
  if (filterValue > 1){      
    filterValue = .99;
  }
  else if (filterValue <= 0){
    filterValue = 0;
  }
  
  smoothedValue = (sensor_reading * (1 - filterValue)) + (smoothedValue  *  filterValue);
  return (int)smoothedValue;
}