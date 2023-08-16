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

#define TASK_ON_CORE_ZERO       0x01
#define TASK_ON_CORE_ONE        0x02
#define TASK_ON_BOTH_CORES      (TASK_ON_CORE_ZERO | TASK_ON_CORE_ONE)

#define KALMAN_FILTER_SENSOR
#define MPU_INTERRUPT_MODE

                /* Pin Configurations: */
/*------------------------------------------------------------*/

#define I2C0_SDA_PIN         12 
#define I2C0_SCL_PIN         13

#define MPU6050_INT_PIN      0

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
#define OLED_REFRESH_PERIOD                 50      // Milliseconds
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

/* IRQ callback function that updates a binary semaphore when a MPU6050 data-ready event occurs. */
void mpu_irq_callback(uint gpio, uint32_t event) {
  // Signal the alert clearance task
  static BaseType_t higher_priority_task_woken = pdFALSE;
  xSemaphoreGiveFromISR(xMpu_Semaphore, &higher_priority_task_woken);
  // Exit to context switch if necessary
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

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
    //the noise in the system
    float Q = 0.025; // Response time decreases as Q increases
    float R = 1.0;
    
    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    float z_measured; //the 'noisy' value we measured
    #endif

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        int front_distance = ultrasonic_get_distance_temperature_compansated_cm(FRONT_TRIG_PIN, FRONT_ECHO_PIN, temperature);
        if(front_distance > FRONT_MAX_DISTANCE_TO_READ) front_distance = FRONT_MAX_DISTANCE_TO_READ;

        #if defined KALMAN_FILTER_SENSOR
        x_temp_est = x_est_last;
        P_temp = P_last + Q;
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp + R));
        //measure
        z_measured = front_distance; 
        //correct
        x_est = x_temp_est + K * (z_measured - x_temp_est); 
        P = (1- K) * P_temp;
        //we have our new system        
        int distance_to_send = (int)x_est;
        //update our last's
        P_last = P;
        x_est_last = x_est;
        printf("%3d, %3d\n", distance_to_send, front_distance);
        #else
        int distance_to_send = front_distance;
        #endif
        
        /*printf("\nFront Distance = %d\n", distance_to_send);*/
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
    //the noise in the system
    float Q = 0.025; // Response time decreases as Q increases
    float R = 1.0;
    
    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    float z_measured; //the 'noisy' value we measured
    #endif
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        int left_distance = ultrasonic_get_distance_temperature_compansated_cm(LEFT_TRIG_PIN, LEFT_ECHO_PIN, temperature);
        if(left_distance > SIDE_MAX_DISTANCE_TO_READ) left_distance = SIDE_MAX_DISTANCE_TO_READ;

        #if defined KALMAN_FILTER_SENSOR
        x_temp_est = x_est_last;
        P_temp = P_last + Q;
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp + R));
        //measure
        z_measured = left_distance; 
        //correct
        x_est = x_temp_est + K * (z_measured - x_temp_est); 
        P = (1- K) * P_temp;
        //we have our new system        
        int distance_to_send = (int)x_est;
        //update our last's
        P_last = P;
        x_est_last = x_est;
        // printf("%3d, %3d\n", distance_to_send, front_distance);
        #else
        int distance_to_send = left_distance;
        #endif

        /*printf("left Distance = %d\n", distance_to_send);*/
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
    //the noise in the system
    float Q = 0.025; // Response time decreases as Q increases
    float R = 1.0;
    
    float K;
    float P;
    float P_temp;
    float x_temp_est;
    float x_est;
    float z_measured; //the 'noisy' value we measured
    #endif
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        int right_distance = ultrasonic_get_distance_temperature_compansated_cm(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, temperature);
        if(right_distance > SIDE_MAX_DISTANCE_TO_READ) right_distance = SIDE_MAX_DISTANCE_TO_READ;

        #if defined KALMAN_FILTER_SENSOR
        x_temp_est = x_est_last;
        P_temp = P_last + Q;
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp + R));
        //measure
        z_measured = right_distance; 
        //correct
        x_est = x_temp_est + K * (z_measured - x_temp_est); 
        P = (1- K) * P_temp;
        //we have our new system        
        int distance_to_send = (int)x_est;
        //update our last's
        P_last = P;
        x_est_last = x_est;
        // printf("%3d, %3d\n", distance_to_send, front_distance);
        #else
        int distance_to_send = right_distance;
        #endif

        /*printf("Right Distance = %d\n", distance_to_send);*/
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
    /* Initiliaze I2C. To use i2c1, "#define USE_I2C1" must be added to inv_mpu6050.c */
    short data[3] = {0, 0, 0};
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
        dmp_set_fifo_rate(MPU_SAMPLE_RATE);
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
        sleep_ms(100);
    }
    xTaskResumeAll();
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
                /*
                printf("\nGyro          ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.gyro_x_f, mpu_data.gyro_y_f, mpu_data.gyro_z_f);
                printf("Accelerometer ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.accel_x_f, mpu_data.accel_y_f, mpu_data.accel_z_f);
                printf("Angles        ==> Roll: %5.1f, Pitch: %5.1f, Yaw: %5.1f\n", mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
                */
            } else {
                vTaskSuspendAll();
                mpu_reset_fifo();
                xTaskResumeAll();
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
            printf("Accelerometer ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.accel_x_f, mpu_data.accel_y_f, mpu_data.accel_z_f);
            printf("Angles        ==> Roll: %5.1f, Pitch: %5.1f, Yaw: %5.1f\n", mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
            */
        } else {
            xTaskResumeAll();
        }
        #endif
        xQueueOverwrite(xMpuQueue, &mpu_data);
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)(MPU6050_READ_PERIOD / portTICK_PERIOD_MS));        
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
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (true) {
        /* Start the DHT22 measurement. */
        dht_start_measurement(&dht);
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
        /* If there were no problems in the */
        if (result == DHT_RESULT_OK) {
            float temperature_to_send = 0.6f * previous_temperature_c + 0.4f * temperature_c;
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
    
    }
    /* If MPU6050 is connected, create its task. */
    if(i2c_bitmask & (0x01)) {
        xMpu_Semaphore =  xSemaphoreCreateBinary();
        xTaskCreate(mpu_task, "MPU6050_Task", configMINIMAL_STACK_SIZE * 4,
                    NULL, configMAX_PRIORITIES - 2, &xMpu_Sensor_Handle);
        vTaskCoreAffinitySet(xMpu_Sensor_Handle, TASK_ON_BOTH_CORES);
    }
    /* If there are no I2C devices connected, deinitiliaze I2C. */
    if(!(i2c_bitmask & (0x03))) {
        i2c_deinit(i2c_default);
        gpio_set_pulls(I2C0_SDA_PIN, 0, 0);
        gpio_set_pulls(I2C0_SCL_PIN, 0, 0);
        gpio_deinit(I2C0_SDA_PIN);
        gpio_deinit(I2C0_SCL_PIN);
    }
    #else
    #warning "NO I2C PINS ARE DEFINED!"
    #endif

    /* Initiliaze ultrasonic sensor pins and tasks. All ultrasonic sensors run on core zero since 
    reading from an ultrasonic sensor can take as long as 30ms which would slow down motor control task. */
    #if defined(FRONT_ECHO_PIN) && defined(FRONT_TRIG_PIN)
    ultrasonic_setup_pins(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    xTaskCreate(front_sensor_task, "Front_Sensor_Task", configMINIMAL_STACK_SIZE * 2,
                NULL, configMAX_PRIORITIES, &xFront_Sensor_Handle);
    vTaskCoreAffinitySet(xFront_Sensor_Handle, TASK_ON_CORE_ZERO);
    #endif

    #if defined(LEFT_ECHO_PIN) && defined(LEFT_TRIG_PIN)
    ultrasonic_setup_pins(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    xTaskCreate(left_sensor_task, "Left_Sensor_Task", configMINIMAL_STACK_SIZE * 2,
                NULL, configMAX_PRIORITIES - 1, &xLeft_Sensor_Handle);
    vTaskCoreAffinitySet(xLeft_Sensor_Handle, TASK_ON_CORE_ZERO);
    #endif

    #if defined(RIGHT_ECHO_PIN) && defined(RIGHT_TRIG_PIN)
    ultrasonic_setup_pins(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    xTaskCreate(right_sensor_task, "Right_Sensor_Task", configMINIMAL_STACK_SIZE * 2,
                NULL, configMAX_PRIORITIES - 1, &xRight_Sensor_Handle);
    vTaskCoreAffinitySet(xRight_Sensor_Handle, TASK_ON_CORE_ZERO);
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

    vStartTasks();

    /* The code should never reach here. */
    while(true) {
        return 0;
    }
}