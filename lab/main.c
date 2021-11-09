/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>


/* Example/Board Header files */
#include "Board.h"
#include "SensorOpt3001.h"
#include "SensorHdc1000.h"
#include "SensorUtil.h"
#include "SensorI2C.h"
#include "SensorMpu9250.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <ti/drivers/PWM.h>

#define TASKSTACKSIZE       2048
#define TMP007_OBJ_TEMP     0x0003  /* Object Temp Result Register */

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                             Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW
                                     | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                             PIN_TERMINATE };

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
#define HDC1000_REG_TEMP           0x00 // Temperature
#define HDC1000_REG_HUM            0x01 // Humidity
#define HDC1000_REG_CONFIG         0x02 // Configuration
#define HDC1000_REG_SERID_H        0xFB // Serial ID high
#define HDC1000_REG_SERID_M        0xFC // Serial ID middle
#define HDC1000_REG_SERID_L        0xFD // Serial ID low
#define HDC1000_REG_MANF_ID        0xFE // Manufacturer ID
#define HDC1000_REG_DEV_ID         0xFF // Device ID

// Fixed values
#define HDC1000_VAL_MANF_ID        0x5449
#define HDC1000_VAL_DEV_ID         0x1000
#define HDC1000_VAL_CONFIG         0x1000 // 14 bit, acquired in sequence

#define SENSOR_DESELECT()   SensorI2C_deselect()

#define MPU_TYPE_ACC 0
#define MPU_TYPE_MAG 1
#define MPU_TYPE_GYRO 2

#define NUM_MPU_TYPE 3
#define NUM_XYZ 3

#define X 0
#define Y 1
#define Z 2

typedef bool (*FUNC_PTR_MPU_READ)(uint16_t*);
typedef float (*FUNC_PTR_MPU_CONV)(int16_t);

Task_Struct task1Struct, task2Struct, task3Struct;
Char task1Stack[TASKSTACKSIZE], task2Stack[TASKSTACKSIZE], task3Stack[TASKSTACKSIZE];
Semaphore_Struct semStruct;
Semaphore_Handle semHandle;

PWM_Handle pwm1;

float _mpu_reading_curr[NUM_MPU_TYPE][NUM_XYZ] = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
};

float mpu_reading_curr[] = {
    {0.0, 0.0, 0.0},
};

UInt32 sleepTickCount;
float result_mpu;
float result_opt;

float flt_abs(float inp) {
    return inp < 0 ? -inp : inp;
}

float process_results_from_sensors(float result, int num_iterations, float low, float high) {
    // Gets magnitude of reading.
    result = result < 0 ? -result : result;
    // Finds average.
    result = result / num_iterations;
    // Standardises result.
    result = (result - low) / (high - low);
    // Ensures result is between 0 and 1. If out of bounds, assign the closer boundary.
    result = (result > 1 ? 1 : result < 0 ? 0 : result);
    return result;
}

void setUpOpt3001() {
    // config sensor
    SensorOpt3001_init();
    SensorOpt3001_enable(true);
    if (!SensorOpt3001_test())
    {
        System_printf("SensorOpt3001 did not pass test!\n");
    }
}

bool readOPT3001(uint16_t* rawdata, unsigned int i)
{


    // read data
    
    // CS3237 TODO: add code to read optical sensor data. The API is provided in OPT3001 library code

    if (SensorOpt3001_read(rawdata))
    {

        *rawdata = SensorOpt3001_convert(*rawdata);
        //System_printf("SensorOpt3001 Sample  %u: %d \n", i, *rawdata);
        //CS3237 TODO: please add code to convert data. API provided in OPT3001 library code.
        // ...
        System_flush();
        return true;
    }
    else
    {
        System_printf("SensorOpt3001 I2C fault!\n");
        System_flush();
        return false;
    }
    
}

void setUpMpu9250()
{
    // config MPU
    SensorMpu9250_powerOn();
    if (!SensorMpu9250_init())
    {
        System_printf("SensorMPU9250_ cannot init!\n");
        return;
    }
    // SensorMpu9250_accSetRange(ACC_RANGE_2G);

    SensorMpu9250_enable(63);
    // SensorMpu9250_enable(9);

    // SensorMpu9250_enableWom(1);
    if (!SensorMpu9250_test())
    {
        System_printf("SensorMPU9250_ did not pass test!\n");
    }
}


bool readMpu9250(float* rawdata, unsigned int i)
{
    char** str = (char**)calloc(3, sizeof(char*));
    int j;
    float tmp;
    // read MPU data
    uint16_t* rawdata_raw = (uint16_t*)calloc(3, sizeof(uint16_t));
    //CS3237 TODO: add code to read MPU accelermoter data. The API is provided in MPU library code.
    if (SensorMpu9250_accRead(rawdata_raw))
    {
        //System_printf("SensorMPU9250_ no.Sample %u: %d (C)\n", i, rawdata_raw);
        // No need to convert
        // System_printf("A");
        // System_printf("%d %d %d\n", *rawdata_raw, *(rawdata_raw + 1), *(rawdata_raw + 2));

        for (j = 0; j < 3; j++) {
            tmp = SensorMpu9250_accConvert(rawdata_raw[j]);
            rawdata[j] = tmp - mpu_reading_curr[j];
            mpu_reading_curr[j] = tmp;

            // str[j] = (char*)calloc(80, sizeof(char));
            // sprintf(str[j], "%f", rawdata[j]);
        }

        //System_printf("SensorMPU9250_ no.Sample %u: %d (C)\n", i, *rawdata);
        // System_printf("SensorMPU9250_ no.Sample %u: %s %s %s (C)\n", i, str[0], str[1], str[2]);

        // Clean-up allocated memory
        free(rawdata_raw);
        
        for (j = 0; j < 3; j++) {
            free(str[j]);
        }

        free(str);
        // See if conversion is necessary
        //....
        System_flush();
        
        return true;
    }
    else
    {
        System_printf("SensorMPU9250_ I2C fault!\n");
        free(rawdata_raw);
        System_flush();
        return false;
    }
        
}

bool readMpu9250ByMputypeMag(float* rawdata, int i) {
    char** str = (char**)calloc(3, sizeof(char*));
    int j;
    float tmp;
    uint8_t status;
    // read MPU data
    int16_t* rawdata_raw = (int16_t*)calloc(3, sizeof(int16_t));
    //CS3237 TODO: add code to read MPU accelermoter data. The API is provided in MPU library code.
    if (status = SensorMpu9250_magRead(rawdata_raw) == MAG_STATUS_OK)
    {
        //System_printf("SensorMPU9250_ no.Sample %u: %d (C)\n", i, rawdata_raw);
        // No need to convert
        // System_printf("A");
        System_printf("%d %d %d\n", *rawdata_raw, *(rawdata_raw + 1), *(rawdata_raw + 2));

        for (j = 0; j < 3; j++) {
            rawdata[j] = (float) rawdata_raw[j];
            // rawdata[j] = tmp - _mpu_reading_curr[MPU_TYPE_MAG][j];
            // _mpu_reading_curr[MPU_TYPE_MAG][j] = tmp;

            str[j] = (char*)calloc(80, sizeof(char));
            sprintf(str[j], "%f", rawdata[j]);
        }

        //System_printf("SensorMPU9250_ no.Sample %u: %d (C)\n", i, *rawdata);
        System_printf("MODE %d - SensorMPU9250_ no.Sample %u: %s %s %s (C)\n", MPU_TYPE_MAG, i, str[0], str[1], str[2]);

        // Clean-up allocated memory
        free(rawdata_raw);
        
        for (j = 0; j < 3; j++) {
            free(str[j]);
        }

        free(str);
        // See if conversion is necessary
        //....
        System_flush();
        
        return true;
    }
    else
    {
        System_printf("Magnometer status error: %d\n", status);
        System_printf("SensorMPU9250_ I2C fault!\n");
        free(rawdata_raw);
        System_flush();
        return false;
    }
}


bool readMpu9250ByMputype(int mpu_type, float* rawdata, int i)
{
    char** str = (char**)calloc(3, sizeof(char*));
    int j;
    float tmp;
    // read MPU data
    uint16_t* rawdata_raw; 
    
    FUNC_PTR_MPU_READ mpu_read;
    FUNC_PTR_MPU_CONV mpu_conv;
    System_printf("\n\n---------------------MODE %d----------------------\n", mpu_type);
    switch(mpu_type) {
        case MPU_TYPE_ACC:
            mpu_read = SensorMpu9250_accRead;
            mpu_conv = SensorMpu9250_accConvert;
            break;
        case MPU_TYPE_GYRO:
            mpu_read = SensorMpu9250_gyroRead;
            mpu_conv = SensorMpu9250_gyroConvert;
            break;
        case MPU_TYPE_MAG:
            return readMpu9250ByMputypeMag(rawdata, i);
        default:
            System_printf("Weird mode\n");
    }

    rawdata_raw = (uint16_t*)calloc(3, sizeof(uint16_t));
    //CS3237 TODO: add code to read MPU accelermoter data. The API is provided in MPU library code.
    if (mpu_read(rawdata_raw))
    {
        //System_printf("SensorMPU9250_ no.Sample %u: %d (C)\n", i, rawdata_raw);
        // No need to convert
        // System_printf("A");
        System_printf("%d %d %d\n", *rawdata_raw, *(rawdata_raw + 1), *(rawdata_raw + 2));

        for (j = 0; j < 3; j++) {
            rawdata[j] = mpu_conv(rawdata_raw[j]);
            // tmp = mpu_conv(rawdata_raw[j]);
            // rawdata[j] = tmp - _mpu_reading_curr[mpu_type][j];
            // _mpu_reading_curr[mpu_type][j] = tmp;

            str[j] = (char*)calloc(80, sizeof(char));
            sprintf(str[j], "%f", rawdata[j]);
        }

        //System_printf("SensorMPU9250_ no.Sample %u: %d (C)\n", i, *rawdata);
        System_printf("MODE %d - SensorMPU9250_ no.Sample %u: %s %s %s (C)\n", mpu_type, i, str[0], str[1], str[2]);


        // Clean-up allocated memory
        free(rawdata_raw);
        
        for (j = 0; j < 3; j++) {
            free(str[j]);
        }

        free(str);
        // See if conversion is necessary
        //....
        System_flush();
        
        return true;
    }
    else
    {
        System_printf("SensorMPU9250_ I2C fault!\n");
        free(rawdata_raw);
        System_flush();
        return false;
    }
        
}

void setUpPwm() 
{
    PWM_Params params;
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);
    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }
    PWM_start(pwm1);
}

void setPwmLed(float pwmFactor)
{
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds
    uint16_t   duty;

    if (pwmFactor < 0 || pwmFactor > 1) {
        System_printf("Invalid paramaters given!");
        pwmFactor = 0.5;
    }

    duty = (uint16_t)(pwmFactor * pwmPeriod);

    PWM_setDuty(pwm1, duty);
}

// You need to write the task functions here.

void read_opt_task() {
    char str[80];

    uint16_t rawdata;
    float result = 0;
    int i = 1;
    int num_iterations = 5;

    float low = 0;
    float high = 2000;

    setUpOpt3001();
    for(;;) {

        // if (Semaphore_getCount(semHandle) == 0) {
        //     System_printf("Sem blocked in opt\n\n");
        // } else {
        //     System_printf("Sem taken in opt\n\n");
        // }

        Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

        if (readOPT3001(&rawdata, i)) {
            i++;
            result += (float) rawdata;
            
            if (i > num_iterations) {

                i = 1;
                
                result_opt = process_results_from_sensors(result, num_iterations, low, high);

                sprintf(str, "%f", result);
                System_printf("OPT: %s\n\n", str);

                // Compares which value to take - MPU or OPT. Go with the larger value.
                setPwmLed(result_opt > result_mpu ? result_opt : result_mpu);
                result = 0;
            }
        }

        Semaphore_post(semHandle);

        Task_sleep(sleepTickCount);
    }
}

// void read_mpu_task() {
//     char str[80];

//     float *rawdata = calloc(3, sizeof(float));
//     float result = 0;
//     int i = 1;

//     int num_iterations = 5;

//     // Hardcoded values to standardise results
//     float low = 0;
//     float high = 0.1;

//     // Set up MPU
//     setUpMpu9250();
//     for(;;) {
//         // Prints debug messages.

//         // if (Semaphore_getCount(semHandle) == 0) {
//         //     System_printf("Sem blocked in mpu\n\n");
//         // } else {
//         //     System_printf("Sem taken in mpu\n\n");
//         // }

//         // Acquire resources.
//         Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

//         // If a value is successfully read, increment the count.
//         if (
//             // readMpu9250(rawdata, i)
//             readMpu9250ByMputype(MPU_TYPE_ACC, rawdata)
//             ) {      

//             i++;

//             // Find average change across axes x, y and z

//             result += (flt_abs(rawdata[0]) + flt_abs(rawdata[1]) + flt_abs(rawdata[2])) / 3;
            
//             sprintf(str, "%f", (flt_abs(rawdata[0]) + flt_abs(rawdata[1]) + flt_abs(rawdata[2])) / 3);
//             System_printf("unit mpu: %s\n\n", str);

//             if (i > num_iterations) {

//                 i = 1;
//                 result_mpu = process_results_from_sensors(result, num_iterations, low, high);

//                 // Compares which value to take - MPU or OPT. Go with the larger value.
//                 setPwmLed(result_opt > result_mpu ? result_opt : result_mpu);
//                 result = 0;
//             }
//         }
//         // release resources
//         Semaphore_post(semHandle);
//         // delay for a set amount of time.
//         Task_sleep(sleepTickCount);
//     }
// }

void read_mpu_task() {
    char str[80];

    float *rawdata = calloc(3, sizeof(float));
    float result = 0;
    int i = 1;

    int num_iterations = 5;

    // Hardcoded values to standardise results
    float low = 0;
    float high = 0.1;

    int mode;

    // float** payload = (float**)calloc(3, sizeof(float*));
    // for (i = 0; i < 3; i++) {
    //     payload[i] = (float*)calloc(3, sizeof(float));
    // }
    float payload[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

    // Set up MPU
    setUpMpu9250();
    for(;;) {

        // Acquire resources.
        Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
        for (mode = MPU_TYPE_ACC; mode < MPU_TYPE_GYRO + 1; mode++) {
            
            // If a value is successfully read, increment the count.
            if (readMpu9250ByMputype(mode, rawdata, i)) {   
                for (i = 0; i < 3; i++) { 
                    payload[mode][i] = rawdata[i];
                }
            }
            // delay for a set amount of time.
            Task_sleep(sleepTickCount);
        }
        System_flush();


        for (mode = MPU_TYPE_ACC; mode < MPU_TYPE_GYRO + 1; mode++) {
            System_printf("PAYLOAD[%d]: ", mode);
            for (i = 0; i < 3; i++) {
                sprintf(str, "%f", payload[mode][i]); 
                System_printf("%s_", str);
            }
            System_printf("\n");
        }

        

        System_flush();

        // release resources
        Semaphore_post(semHandle);

        // delay for a set amount of time.
        Task_sleep(sleepTickCount);
    }
}





/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initGPIO();
    Board_initPWM();

    GPIO_write(Board_LED0, Board_LED_ON);

    // Set up PWM controller.
    setUpPwm();

    // CS3237 TODO: Create task structures for reading sensor data and performing PWM. Do not forgot to open I2C, which is available in the library file SensorI2C.c.
    if (!SensorI2C_open())
    {
        System_abort("I2C did not open");
    }

    /* Construct writer/reader Task threads */
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task1Stack;
    taskParams.priority = 2;
    Task_construct(&task1Struct, (Task_FuncPtr)read_mpu_task, &taskParams, NULL);

    taskParams.stack = &task2Stack;
    taskParams.priority = 1;
    // Task_construct(&task2Struct, (Task_FuncPtr)read_opt_task, &taskParams, NULL);

    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 1, &semParams);
    
    /* Obtain instance handle */
    semHandle = Semaphore_handle(&semStruct);

    /* We want to sleep for 10000 microseconds */
    sleepTickCount = 50000 / Clock_tickPeriod;

    // CS3237 new tip: It might happen that you get I2C fault or some weird data in the first read. You can get right data if you read sensor again.

    System_printf("Starting the I2C example\nSystem provider is set to SysMin."
                  " Halt the target to view any SysMin contents in ROV.\n");


    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
