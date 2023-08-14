#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pico/stdlib.h"

#include <stdio.h>

#define TASK_ON_CORE_ZERO       0x01
#define TASK_ON_CORE_ONE        0x02
#define TASK_ON_BOTH_CORES      (TASK_ON_CORE_ZERO | TASK_ON_CORE_ONE)

#ifdef PICO_DEFAULT_LED_PIN
TaskHandle_t xLed_Task_Handle = NULL;
#endif

/* Toggle the onboard led pin (if it exists) on and off for 1 seconds as a means to observe if the program did not freeze. */
#ifdef PICO_DEFAULT_LED_PIN
void led_task() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    
    while(true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        vTaskDelay((TickType_t)1000 / portTICK_PERIOD_MS);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        vTaskDelay((TickType_t)1000 / portTICK_PERIOD_MS);
    }
}
#endif

int main() {

    #ifdef PICO_DEFAULT_LED_PIN
    xTaskCreate(led_task, "Led_Task", configMINIMAL_STACK_SIZE, 
                NULL, tskIDLE_PRIORITY, &xLed_Task_Handle);
    
    vTaskCoreAffinitySet(xLed_Task_Handle, TASK_ON_BOTH_CORES);
    #endif

    vTaskStartScheduler();
    /* The code should never reach here. */
    while(true) {
        return 0;
    }
}