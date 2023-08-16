/* FreeRTOS Includes: */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Pico SDK Includes: */
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/float.h"
#include "hardware/i2c.h"

/* C Standard Library Includes: */
#include <stdio.h>

/* Hardware Driver Includes: */
#include "dht.h"
#include "servo.h"
#include "ultrasonic.h"
#include "inv_mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ssd1306.h"
#include "BMSPA_font.h"

#define TASK_ON_CORE_ZERO       (UBaseType_t)(0x1)
#define TASK_ON_CORE_ONE        (UBaseType_t)(0x2)
#define TASK_ON_BOTH_CORES      (UBaseType_t)(0x3)

#define PRINT_GYROSCOPE
#define KALMAN_FILTER_SENSOR
#define MPU_INTERRUPT_MODE

                /* Pin Configurations: */
/*------------------------------------------------------------*/

#define I2C0_SDA_PIN         12 
#define I2C0_SCL_PIN         13

#define MPU6050_INT_PIN      15

#define LEFT_IR_SENSOR_PIN   10
#define RIGHT_IR_SENSOR_PIN  11

#define DHT_PIN              4

#define RIGHT_TRIG_PIN       16
#define RIGHT_ECHO_PIN       17
#define LEFT_TRIG_PIN        19
#define LEFT_ECHO_PIN        18
#define FRONT_TRIG_PIN       6 
#define FRONT_ECHO_PIN       7

#define MOTOR_ESC_PIN        5
#define SERVO_PIN            2

/*------------------------------------------------------------*/

                    /* Config Values: */
/*------------------------------------------------------------*/

#define FRONT_MAX_DISTANCE_TO_READ          450     // CM
#define FRONT_MIN_DISTANCE_TO_READ          10      // CM
#define SIDE_MAX_DISTANCE_TO_READ           200     // CM
#define SIDE_MIN_DISTANCE_TO_READ           5       // CM

#define FRONT_MIN_DISTANCE_TO_TURN          80      // CM    
#define SIDE_MAX_DISTANCE_TO_TURN           60      // CM
#define SIDE_MIN_DISTANCE_TO_TURN           5       // CM

#define MOTOR_FORWARD_DIRECTION             1       // Forward
#define MOTOR_BACKWARD_DIRECTION            0       // Backward
            
#define MOTOR_MIN_FORWARD_MICROS            1570    // Microseconds
#define MOTOR_MAX_FORWARD_MICROS            1585    // Microseconds
#define MOTOR_BRAKE_MICROS                  1500    // Microseconds
#define MOTOR_BACKWARD_MICROS               1375    // Microseconds
#define MOTOR_MAX_MICROS                    2000    // Microseconds
#define MOTOR_MIN_MICROS                    1000    // Microseconds

#define MOTOR_MIN_DISTANCE_TO_BRAKE         50      // CM
#define MOTOR_BRAKE_DURATION                500     // Millisecond

#define SERVO_MIN_MICROS                    1000    // Right
#define SERVO_MIDDLE_MICROS                 1000    // Middle
#define SERVO_MAX_MICROS                    2000    // Left
#define SERVO_ROUND_INTERVAL                10 

#define MPU_SAMPLE_RATE                     1000    // HZ
#define MPU_FIFO_RATE                       200     // HZ
#define MPU_LOW_PASS_FILTER                 42      // HZ
#define MPU_ACCEL_FSR                       2       // G
#define MPU_GYRO_FSR                        2000    // DPS

#define KALMAN_ULTRASONIC_Q                 0.03f
#define KALMAN_ULTRASONIC_R                 1.0f

#define OLED_ADRESS                         0x3C
#define MPU6050_ADRESS                      0x68    

//Time in milliseconds
#define MOTOR_STATE_CHANGE_DURATION         800     // Milliseconds
#define FRONT_SENSOR_READ_PERIOD            60      // Milliseconds
#define SIDE_SENSOR_READ_PERIOD             60      // Milliseconds
#define SERVO_UPDATE_PERIOD                 60      // Milliseconds
#define MOTOR_UPDATE_PERIOD                 5       // Milliseconds
#define MPU6050_READ_PERIOD                 5       // Milliseconds
#define LED_BLINK_PERIOD                    1000    // Milliseconds        
#define OLED_REFRESH_PERIOD                 10      // Milliseconds
#define DHT_SENSOR_READ_PERIOD              2000    // Milliseconds

/*------------------------------------------------------------*/


/* FreeRTOS Queues: */
static QueueHandle_t xDhtQueue          = NULL;
static QueueHandle_t xLeftQueue         = NULL;
static QueueHandle_t xRightQueue        = NULL;
static QueueHandle_t xFrontQueue        = NULL;
static QueueHandle_t xServoQueue        = NULL;
static QueueHandle_t xMotorQueue        = NULL;
static QueueHandle_t xMpuQueue          = NULL;

/* Component mask represents which components are currently connected to the pico. 
 *\n First Bit : OLED Screen,
 *\n Second Bit: MPU6050,
 *\n Third Bit : DHT22
*/
static QueueHandle_t xComponentMask     = NULL;

/* FreeRTOS Semaphores: */
static SemaphoreHandle_t xMpu_Semaphore = NULL;

/* FreeRTOS Task Handles: */
#ifdef PICO_DEFAULT_LED_PIN
TaskHandle_t xLed_Task_Handle           = NULL;
#endif
TaskHandle_t xOled_Screen_Task_Handle   = NULL;
TaskHandle_t xDht_Sensor_Handle         = NULL;
TaskHandle_t xFront_Sensor_Handle       = NULL;
TaskHandle_t xLeft_Sensor_Handle        = NULL;
TaskHandle_t xRight_Sensor_Handle       = NULL;
TaskHandle_t xMpu_Sensor_Handle         = NULL;
TaskHandle_t xServo_Task_Handle         = NULL;
TaskHandle_t xMotor_Task_Handle         = NULL;

#ifdef MPU_INTERRUPT_MODE
/* IRQ callback function that updates a binary semaphore when a MPU6050 data-ready event occurs. */
void mpu_irq_callback(uint gpio, uint32_t event) {
  // Signal the alert clearance task
  static BaseType_t higher_priority_task_woken = pdFALSE;
  xSemaphoreGiveFromISR(xMpu_Semaphore, &higher_priority_task_woken);
  // Exit to context switch if necessary
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
#endif

/* Toggle the onboard led pin (if it exists) on and off for 1 seconds as a means to observe if the program did not freeze. */
void led_task() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    
    while(true) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        vTaskDelay((TickType_t)LED_BLINK_PERIOD / portTICK_PERIOD_MS);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        vTaskDelay((TickType_t)LED_BLINK_PERIOD / portTICK_PERIOD_MS);
        printf("BRUH!\n");
    }
}

void front_sensor_task(void *pvParameters) {

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Front Sensor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    float temperature = 27.0f;

    #if defined KALMAN_FILTER_SENSOR
    //initial values for the kalman filter
    float x_est_last = 0;
    float P_last = 0;
    
    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    #endif

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {

        xQueuePeek(xDhtQueue, &temperature, 0);
        uint16_t front_distance = ultrasonic_get_distance_temperature_compansated_cm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, temperature);
        if(front_distance > FRONT_MAX_DISTANCE_TO_READ) front_distance = FRONT_MAX_DISTANCE_TO_READ;

        #if defined KALMAN_FILTER_SENSOR
        x_temp_est = x_est_last;
        P_temp = P_last + KALMAN_ULTRASONIC_Q;
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp + KALMAN_ULTRASONIC_R));
        //correct
        x_est = x_temp_est + K * ((float)front_distance - x_temp_est); 
        P = (1- K) * P_temp;
        //we have our new system        
        uint16_t distance_to_send = (uint16_t)x_est;
        //update our last's
        P_last = P;
        x_est_last = x_est;
        // printf("%3u, %3u\n", distance_to_send, front_distance);
        #else
        uint16_t distance_to_send = front_distance;
        #endif
        UBaseType_t core_number = vTaskCoreAffinityGet(NULL);
        // printf("Front Distance = %u, On Core = %u\n", distance_to_send, core_number);
        xQueueOverwrite(xFrontQueue, &distance_to_send);

        xTaskDelayUntil(&xNextWaitTime, (TickType_t)FRONT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void left_sensor_task(void *pvParameters){

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Left Sensor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    float temperature = 27.0f;
 
    #if defined KALMAN_FILTER_SENSOR
    //initial values for the kalman filter
    float x_est_last = 0;
    float P_last = 0;
    
    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    #endif
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        uint16_t left_distance = ultrasonic_get_distance_temperature_compansated_cm(LEFT_TRIG_PIN, LEFT_ECHO_PIN, temperature);
        if(left_distance > SIDE_MAX_DISTANCE_TO_READ) left_distance = SIDE_MAX_DISTANCE_TO_READ;

        #if defined KALMAN_FILTER_SENSOR
        x_temp_est = x_est_last;
        P_temp = P_last + KALMAN_ULTRASONIC_Q;
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp + KALMAN_ULTRASONIC_R));
        //correct
        x_est = x_temp_est + K * ((float)left_distance - x_temp_est); 
        P = (1- K) * P_temp;
        //we have our new system        
        uint16_t distance_to_send = (uint16_t)x_est;
        //update our last's
        P_last = P;
        x_est_last = x_est;
        // printf("%3u, %3u\n", distance_to_send, front_distance);
        #else
        uint16_t distance_to_send = left_distance;
        #endif

        // printf("Left Distance = %u\n", distance_to_send);
        xQueueOverwrite(xLeftQueue, &distance_to_send);
        
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void right_sensor_task(void *pvParameters){

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Right Sensor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    float temperature = 27.0f;

    #if defined KALMAN_FILTER_SENSOR
    //initial values for the kalman filter
    float x_est_last = 0;
    float P_last = 0;
    
    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    #endif
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        uint16_t right_distance = ultrasonic_get_distance_temperature_compansated_cm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, temperature);
        if(right_distance > SIDE_MAX_DISTANCE_TO_READ) right_distance = SIDE_MAX_DISTANCE_TO_READ;

        #if defined KALMAN_FILTER_SENSOR
        x_temp_est = x_est_last;
        P_temp = P_last + KALMAN_ULTRASONIC_Q;
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp + KALMAN_ULTRASONIC_R));
        //measure
        //correct
        x_est = x_temp_est + K * ((float)right_distance - x_temp_est); 
        P = (1- K) * P_temp;
        //we have our new system        
        uint16_t distance_to_send = (uint16_t)x_est;
        //update our last's
        P_last = P;
        x_est_last = x_est;
        // printf("%3u, %3u\n", distance_to_send, front_distance);
        #else
        uint16_t distance_to_send = right_distance;
        #endif

        // printf("Right Distance = %u\n", distance_to_send);
        xQueueOverwrite(xRightQueue, &distance_to_send);
        
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void mpu_task(void * pvParameters) {

    struct int_param_s mpu6050_int_param = {
        #ifndef MPU6050_INT_PIN
        #warning "No MPU6005 interrupt pin is defined."
        #else
        .int_pin = MPU6050_INT_PIN,
        #endif
        .int_event = GPIO_IRQ_LEVEL_LOW,
        .cb = NULL
    };

    mpu_data_f mpu_data= {
        .accel_x_f = 0,
        .accel_y_f = 0,
        .accel_z_f = 0,
        .gyro_x_f = 0,
        .gyro_y_f = 0,
        .gyro_z_f = 0,
        .pitch = 0,
        .roll = 0,
        .yaw = 0
    };
    xQueueOverwrite(xMpuQueue, &mpu_data);
    sleep_ms(200);
    /* Always use vTaskSuspendAll before doing I2C operations to prevent communication errors. */
    vTaskSuspendAll();
    /* Initiliaze MPU6050. */
    if(mpu_init(&mpu6050_int_param)) {
        printf("Could not initiliaze MPU6050\n");
        xTaskResumeAll();
        vTaskDelete(NULL);
    }
    printf("Mpu is initiliazed.\n");
    sleep_ms(10);
    /* To get the best performance from dmp quaternions, Accel = -+2G and Gyro = -+2000DPS is recommended */
    mpu_set_accel_fsr(MPU_ACCEL_FSR);
    sleep_ms(10);
    mpu_set_gyro_fsr(MPU_GYRO_FSR);
    sleep_ms(10);
    /* Initiliaze low pass filter and high pass filter */
    mpu_set_lpf(MPU_LOW_PASS_FILTER);
    sleep_ms(10);
    mpu_set_hpf(MPU6050_DHPF_1_25HZ);
    sleep_ms(10);
    /* RP2020 can easily handle 1khz reading speed from MPU6050*/
    mpu_set_sample_rate(MPU_SAMPLE_RATE);
    sleep_ms(10);
    /* Configure which sensors are pushed to the FIFO */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    sleep_ms(10);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    sleep_ms(10);
    /* Get the accelerometer and gyroscope conversion factor to convert hardware units to G or DPS. 
    Formula is: (Hardware_Units / Sensor_Sensitivity) */
    unsigned short accel_sens = 0.0f;
    float gyro_sens = 0.0f;
    mpu_get_accel_sens(&accel_sens);
    mpu_get_gyro_sens(&gyro_sens);

    /* Load the firmware of DMP.*/
    if(dmp_load_motion_driver_firmware()) {
        printf("DMP could not be initiliazed.\n");
        xTaskResumeAll();
        vTaskDelete(NULL);
    } else {
        printf("DMP is initiliazed.\n");
        sleep_ms(100);
        /* Set FIFO rate of DMP to 200 to get the best performance for quaternion calculations */
        dmp_set_fifo_rate(MPU_FIFO_RATE);
        sleep_ms(10);
        /* Enable DMP */
        mpu_set_dmp_state(1);
        sleep_ms(100);
        #if defined(MPU_INTERRUPT_MODE)
        gpio_set_irq_enabled_with_callback(mpu6050_int_param.int_pin, GPIO_IRQ_EDGE_FALL, true, mpu_irq_callback);
        #endif
        /* Enable which features are pushed to the fifo. 
        If DMP_FEATURE_GYRO_CAL is active, the sensor automatically calibrates the gyro if there is no motion for 8 seconds */
        dmp_enable_feature(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_6X_LP_QUAT);
        sleep_ms(100);
        /* Calculate the accelerometer and gyroscope by pushing bias values to the registers */
        long gyro_bias[] = {0, 0, 0};
        long accel_bias[] = {0, 0, 0};
        mpu_find_gyro_calibration_biases(gyro_bias);
        mpu_find_accel_calibration_biases_pid(accel_bias);
        mpu_set_gyro_bias_reg(gyro_bias);
        mpu_set_accel_bias_6050_reg(accel_bias);
        dmp_set_gyro_bias(gyro_bias);
        dmp_set_accel_bias(accel_bias);
        mpu_set_accel_fsr(MPU_ACCEL_FSR);
        mpu_set_gyro_fsr(MPU_GYRO_FSR);
        printf("MPU6050 Calibration is complete.\n");
        xTaskResumeAll();
    }
    mpu_reset_fifo();
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {
        int16_t gyro[3] = {0, 0, 0};
        int16_t accel[3] = {0, 0, 0};
        long quat[4]   = {0, 0, 0, 0};
        unsigned long timestamp = 0;
        /* Sensor mask to choose which values are read from the FIFO */
        short sensors = INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_WXYZ_QUAT;
        unsigned char more = 0;
        #ifdef MPU_INTERRUPT_MODE
        /* Wait for the semaphore */
        if(xSemaphoreTake(xMpu_Semaphore, (TickType_t)(MPU_FIFO_RATE / portTICK_PERIOD_MS))) {
            vTaskSuspendAll();
            dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
            xTaskResumeAll();
            if(sensors & (INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_WXYZ_QUAT)) {
                dmp_convert_sensor_data_real_units(&mpu_data, gyro, accel, quat, sensors);
                // printf("Correct!\n");
                /*
                printf("\nGyro          ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.gyro_x_f, mpu_data.gyro_y_f, mpu_data.gyro_z_f);
                printf("Accelerometer ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.accel_x_f, mpu_data.accel_y_f, mpu_data.accel_z_f);
                printf("Angles        ==> Roll: %5.1f, Pitch: %5.1f, Yaw: %5.1f\n", mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
                */
            } else {
                vTaskSuspendAll();
                mpu_reset_fifo();
                xTaskResumeAll();
                // printf("False!\n");
    
            }
        }
        #else
        vTaskSuspendAll();
        dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
        if(sensors & (INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_WXYZ_QUAT)) {
            xTaskResumeAll();
            dmp_convert_sensor_data_real_units(&mpu_data, gyro, accel, quat, sensors);
            /*
            printf("\nGyro          ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.gyro_x_f, mpu_data.gyro_y_f, mpu_data.gyro_z_f);
            printf("Accelerometer   ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.accel_x_f, mpu_data.accel_y_f, mpu_data.accel_z_f);
            printf("Angles        ==> Roll: %5.1f, Pitch: %5.1f, Yaw: %5.1f\n", mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
            printf("Remaining Packages ==> %u\n", more);
            */
        } else {
            xTaskResumeAll();
        }       
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)(MPU6050_READ_PERIOD / portTICK_PERIOD_MS));        
        #endif
        xQueueOverwrite(xMpuQueue, &mpu_data);
    }
}

void dht_sensor_task(void *pvParameters) {

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("DHT22 task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    static const dht_model_t DHT_MODEL = DHT22;

    /* Take the default temperature value as 27 Celcius degrees. Humidity is ignored in the program. */
    float humidity = 0.0f;
    float temperature_c = 27.0f;
    float previous_temperature_c = 27.0f;
    /* Write the first temperature data as 27 Celcius degrees in case DHT is not connected. */
    xQueueOverwrite(xDhtQueue, &temperature_c);
    /* Create DHT struct and initiliaze DHT22. */
    dht_t dht;
    dht_init(&dht, DHT_MODEL, pio0, DHT_PIN, true);

    while (true) {
        /* Start the DHT22 measurement. */
        vTaskSuspendAll();
        dht_start_measurement(&dht);
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
        xTaskResumeAll();
        /* If there were no problems in the */
        if (result == DHT_RESULT_OK) {
            UBaseType_t core_number = vTaskCoreAffinityGet(NULL);
            float temperature_to_send = 0.6f * previous_temperature_c + 0.4f * temperature_c;
            printf("\nTemperature = %2.1f, On Core = %u\n\n", temperature_to_send, core_number);
            xQueueOverwrite(xDhtQueue, &temperature_to_send);
        } 
        if (result == DHT_RESULT_TIMEOUT) {
            temperature_c = previous_temperature_c;
            dht_deinit(&dht);
            gpio_deinit(DHT_PIN);
            printf("\n-------------------------------------------------\n");
            printf("DHT SENSOR TASK IS DELETED\n");
            printf("---------------------------------------------------\n\n");            
            vTaskDelete(NULL);
        }
        previous_temperature_c = temperature_c;
        vTaskDelay((TickType_t)DHT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void oled_screen_task(void *pvParameters) {

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("OLED Screen task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    vTaskDelay((TickType_t)(250 / portTICK_PERIOD_MS));
    uint16_t front_sensor_distance = 0;
    uint16_t left_sensor_distance = 0;
    uint16_t right_sensor_distance = 0;
    #ifdef PRINT_GYROSCOPE
    mpu_data_f mpu_data= {
        .accel_x_f = 0,
        .accel_y_f = 0,
        .accel_z_f = 0,
        .gyro_x_f = 0,
        .gyro_y_f = 0,
        .gyro_z_f = 0,
        .pitch = 0,
        .roll = 0,
        .yaw = 0
    };
    #else
    float servo_micros = 0.0f;
    float motor_micros = 0.0f;
    #endif
    float temperature = 27.0f;

    // Left: 400    Front 400   Right 400
    char front_sensor_text[10];
    char left_sensor_text[9];
    char right_sensor_text[10];
    #ifdef PRINT_GYROSCOPE
    char accel_text_x[8];
    char gyro_text_x[8];
    char accel_text_y[8];
    char gyro_text_y[8];
    char accel_text_z[8];
    char gyro_text_z[8];
    char angle_text[24];
    #else
    char servo_text[15];
    char motor_text[15];
    #endif
    char temperature_text[8];
    uint8_t bitmask = 0;
    xQueuePeek(xComponentMask, &bitmask, portMAX_DELAY);

    ssd1306_t disp;
    disp.external_vcc = false;
    vTaskDelay(250 / portTICK_PERIOD_MS);
    vTaskSuspendAll();
    bool sensor_is_connected = ssd1306_init(&disp, 128, 64, 0x3C, i2c_default);
    // Delete the task if an OLED screen is not connected 
    if(sensor_is_connected != true) {
        vTaskResume(xMpu_Sensor_Handle);
        xTaskResumeAll();
        vTaskDelete(NULL);
    }
    ssd1306_clear(&disp);
    for(int y=0; y<31; ++y) {
        ssd1306_draw_line(&disp, 0, y, 127, y);
        ssd1306_show(&disp);
        sleep_ms(10);
        ssd1306_clear(&disp);
    }
    for(int y=0, i=1; y>=0; y+=i) {
            ssd1306_draw_line(&disp, 0, 31-y, 127, 31+y);
            ssd1306_draw_line(&disp, 0, 31+y, 127, 31-y);
            ssd1306_show(&disp);
            sleep_ms(10);
            ssd1306_clear(&disp);
            if(y==32) break;
    }
    sleep_ms(100);
    if(bitmask & (0x02)) {
        ssd1306_draw_string_with_font(&disp, 8, 24, 1, BMSPA_font,"Starting");;
        ssd1306_draw_string_with_font(&disp, 8, 37, 1, BMSPA_font,"Calibration");
        ssd1306_show(&disp);
        sleep_ms(25);
        vTaskResume(xMpu_Sensor_Handle);
    } else {
        ssd1306_draw_string_with_font(&disp, 8, 31, 1, BMSPA_font, "MPU6050 ERROR!");
        ssd1306_show(&disp);
        vTaskDelay((TickType_t)(2000 / portTICK_PERIOD_MS));
    }
    xTaskResumeAll();
    if(disp.status == false) {
        printf("\n-------------------------------------------------\n");
        printf("OLED TASK IS DELETED\n");
        printf("---------------------------------------------------\n\n");
        vTaskDelete(NULL);
    }    
    /*ssd1306_bmp_show_image(&disp, image_data, image_size);
    ssd1306_show(&disp);*/


    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {

        xQueuePeek(xFrontQueue, &front_sensor_distance, portMAX_DELAY);
        xQueuePeek(xRightQueue, &right_sensor_distance, portMAX_DELAY);
        xQueuePeek(xLeftQueue, &left_sensor_distance, portMAX_DELAY);
        #ifdef PRINT_GYROSCOPE
        if(bitmask & (0x02)) {
            xQueuePeek(xMpuQueue, &mpu_data, portMAX_DELAY);
        }
        #else
        xQueuePeek(xMotorQueue, &motor_micros, portMAX_DELAY);
        xQueuePeek(xServoQueue, &servo_micros, portMAX_DELAY);
        #endif
        xQueuePeek(xDhtQueue, &temperature, portMAX_DELAY);

        vTaskSuspendAll();
        ssd1306_clear(&disp);

        snprintf(front_sensor_text, 10, "Front:%3u", front_sensor_distance);
        ssd1306_draw_string(&disp, 28, 10, 1, front_sensor_text);

        snprintf(left_sensor_text, 9, "Left:%3u", left_sensor_distance);
        ssd1306_draw_string(&disp, 10, 24, 1, left_sensor_text);

        snprintf(right_sensor_text, 10, "Right:%3u", right_sensor_distance);
        ssd1306_draw_string(&disp, 70, 24, 1, right_sensor_text);  

        #ifdef PRINT_GYROSCOPE
        snprintf(accel_text_x, 8,"x:%4.1f", mpu_data.accel_x_f);
        ssd1306_draw_string(&disp, 1, 38, 1, accel_text_x);
        snprintf(accel_text_y, 8,"y:%4.1f", mpu_data.accel_y_f);
        ssd1306_draw_string(&disp, 45, 38, 1, accel_text_y);
        snprintf(accel_text_z, 8,"z:%4.1f", mpu_data.accel_z_f);
        ssd1306_draw_string(&disp, 93, 38, 1, accel_text_z);

        /*snprintf(gyro_text_z, 15,"z:%4.1f", mpu_data.gyro_z_f);
        snprintf(gyro_text_x, 15,"x:%4.1f\0", mpu_data.gyro_x_f);
        ssd1306_draw_string(&disp, 1, 52, 1, gyro_text_x);
        snprintf(gyro_text_y, 15,"y:%4.1f", mpu_data.gyro_y_f);
        ssd1306_draw_string(&disp, 45, 52, 1, gyro_text_y);
        ssd1306_draw_string(&disp, 93, 52, 1, gyro_text_z);*/
        snprintf(angle_text, 24, "x:%4.1f y:%4.1f z:%4.1f", mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
        ssd1306_draw_string(&disp, 2, 52, 1, angle_text);
        #else
        snprintf(motor_text, 14, "Motor:%7.1f\0", motor_micros);
        ssd1306_draw_string(&disp, 30, 38, 1, motor_text);

        snprintf(servo_text, 14, "Servo:%7.1f\0", servo_micros);
        ssd1306_draw_string(&disp, 2, 52, 1, servo_text);
        #endif


        snprintf(temperature_text, 8, "%3.1f*C", temperature);
        ssd1306_draw_string(&disp, 95, 10, 1, temperature_text);

        ssd1306_show(&disp);
        xTaskResumeAll();
        if(disp.status == false) {
            printf("\n-------------------------------------------------\n");
            printf("OLED TASK IS DELETED\n");
            printf("---------------------------------------------------\n\n");
            vTaskDelete(NULL);
        }
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)OLED_REFRESH_PERIOD / portTICK_PERIOD_MS);
    }
}

void vStartTasks(void) {
    /* Initiliaze input pins of IR sensor pins. Do this first as IR sensor output
    works independently of the microcontroller and it might cause damage if pins are not initiliazed. */
    #if defined(LEFT_IR_SENSOR_PIN) && defined(RIGHT_IR_SENSOR_PIN)
    gpio_init(LEFT_IR_SENSOR_PIN);
    gpio_init(RIGHT_IR_SENSOR_PIN);
    gpio_set_dir(LEFT_IR_SENSOR_PIN, GPIO_IN);
    gpio_set_dir(RIGHT_IR_SENSOR_PIN, GPIO_IN);
    gpio_pull_down(LEFT_IR_SENSOR_PIN);
    gpio_pull_down(RIGHT_IR_SENSOR_PIN);
    #endif

    #if defined MPU6050_INT_PIN
    gpio_init(MPU6050_INT_PIN);
    gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU6050_INT_PIN);
    #endif

    /* A bitmask that represents which I2C devices are connected. */
    uint8_t i2c_bitmask = 0x03;

    /* Initiliaze I2C line and set the baudrate to 400kHZ since every I2C component in this project is compatible with it. */
    #if defined(I2C0_SDA_PIN) && defined(I2C0_SCL_PIN)
    i2c_init(i2c_default, 400 * 1000);
    gpio_init(I2C0_SDA_PIN);
    gpio_init(I2C0_SCL_PIN);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);
    sleep_ms(200);

    /* Test if OLED and MPU6050 are connected to the microcontroller with a dummy write. */
    uint8_t data = 0;
    if (i2c_read_blocking(i2c_default, MPU6050_ADRESS, &data, 1, false) == PICO_ERROR_GENERIC) 
        i2c_bitmask &= ~(0x01 << 1);
    if(i2c_read_blocking(i2c_default, OLED_ADRESS, &data, 1, false) == PICO_ERROR_GENERIC)
        i2c_bitmask &= ~(0x01);
    /* If OLED is connected, create its task. */
    if(i2c_bitmask & (0x01)) {
    xTaskCreate(oled_screen_task, "OLED_Task", configMINIMAL_STACK_SIZE * 4, 
                NULL, configMAX_PRIORITIES - 3, &xOled_Screen_Task_Handle);
    vTaskCoreAffinitySet(xOled_Screen_Task_Handle, TASK_ON_CORE_ZERO);
    }
    /* If MPU6050 is connected, create its task. */
    if(i2c_bitmask & (0x02)) {
        xMpuQueue      = xQueueCreate(1, sizeof(mpu_data_f));
        xMpu_Semaphore = xSemaphoreCreateCounting(50, 0);
        xTaskCreate(mpu_task, "MPU6050_Task", configMINIMAL_STACK_SIZE * 16,
                    NULL, configMAX_PRIORITIES - 2, &xMpu_Sensor_Handle);
        vTaskCoreAffinitySet(xMpu_Sensor_Handle, TASK_ON_CORE_ZERO);
        vTaskSuspend(xMpu_Sensor_Handle);
    }
    /* If there are no I2C devices connected, deinitiliaze I2C. */
    if(!(i2c_bitmask & (0x03))) {
        i2c_deinit(i2c_default);
        gpio_set_pulls(I2C0_SDA_PIN, 0, 0);
        gpio_set_pulls(I2C0_SCL_PIN, 0, 0);
        gpio_deinit(I2C0_SDA_PIN);
        gpio_deinit(I2C0_SCL_PIN);
    }

    xComponentMask = xQueueCreate(1, sizeof(uint8_t));
    xQueueOverwrite(xComponentMask, &i2c_bitmask);

    #else
    #warning "NO I2C PINS ARE DEFINED!"
    #endif

    /* Initiliaze ultrasonic sensor pins and tasks. All ultrasonic sensors run on core zero since 
    reading from an ultrasonic sensor can take as long as 30ms which would slow down motor control task. */
    #if defined(FRONT_ECHO_PIN) && defined(FRONT_TRIG_PIN)
    ultrasonic_setup_pins(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    xFrontQueue = xQueueCreate(1, sizeof(uint16_t));
    xTaskCreate(front_sensor_task, "Front_Sensor", configMINIMAL_STACK_SIZE,
                NULL, configMAX_PRIORITIES, &xFront_Sensor_Handle);
    vTaskCoreAffinitySet(xFront_Sensor_Handle, TASK_ON_CORE_ONE);
    #endif

    #if defined(LEFT_ECHO_PIN) && defined(LEFT_TRIG_PIN)
    ultrasonic_setup_pins(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    xLeftQueue = xQueueCreate(1, sizeof(uint16_t));
    xTaskCreate(left_sensor_task, "Left_Sensor_Task", configMINIMAL_STACK_SIZE,
                NULL, configMAX_PRIORITIES - 1, &xLeft_Sensor_Handle);
    vTaskCoreAffinitySet(xLeft_Sensor_Handle, TASK_ON_CORE_ONE);
    #endif

    #if defined(RIGHT_ECHO_PIN) && defined(RIGHT_TRIG_PIN)
    ultrasonic_setup_pins(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    xRightQueue = xQueueCreate(1, sizeof(uint16_t));
    xTaskCreate(right_sensor_task, "Right_Sensor_Task", configMINIMAL_STACK_SIZE,
                NULL, configMAX_PRIORITIES - 1, &xRight_Sensor_Handle);
    vTaskCoreAffinitySet(xRight_Sensor_Handle, TASK_ON_CORE_ONE);
    #endif

    /* Initiliaze PWM pin that will connect to the ESC. */
    #if defined(MOTOR_ESC_PIN)
    setServo(MOTOR_ESC_PIN, MOTOR_BRAKE_MICROS);
    #endif

    /* Initiliaze PWM pin that will connect to the servo motor. */
    #if defined(SERVO_PIN)
    setServo(SERVO_PIN, SERVO_MIDDLE_MICROS);
    #endif

    /* Initiliaze DHT22 task. */
    #if defined(DHT_PIN)
    xDhtQueue = xQueueCreate(1, sizeof(float));
    xTaskCreate(dht_sensor_task, "DHT_Task", configMINIMAL_STACK_SIZE, 
                NULL, tskIDLE_PRIORITY + 1, &xDht_Sensor_Handle);
    vTaskCoreAffinitySet(xDht_Sensor_Handle, TASK_ON_CORE_ZERO);
    #endif

    /* Initiliaze on-board led pin if the current RP2040 varient has a default led pin. */
    #if defined PICO_DEFAULT_LED_PIN
    xTaskCreate(led_task, "Led_Task", configMINIMAL_STACK_SIZE, 
                NULL, tskIDLE_PRIORITY, &xLed_Task_Handle);
    vTaskCoreAffinitySet(xLed_Task_Handle, TASK_ON_BOTH_CORES);
    #endif    

    vTaskStartScheduler();
}

int main() {
    /* Initiliaze USB serial communication line. */
    stdio_init_all();
    set_sys_clock_khz(250 * 1000, true);
    sleep_ms(1000);
    vStartTasks();

    /* The code should never reach here. */
    while(true) {
        return 0;
    }
}