/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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

/** ============================================================================
 *  @file       SensorMpu9250.c
 *
 *  @brief      Driver for the InvenSense MPU9250 Motion Processing Unit.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "Board.h"
#include "SensorMpu9250_Q.h"
#include "SensorOpt3001.h" // For reset of I2C bus
#include "SensorUtil.h"
#include "SensorI2C.h"

/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/
// Registers
#define SELF_TEST_X_GYRO              0x00 // R/W
#define SELF_TEST_Y_GYRO              0x01 // R/W
#define SELF_TEST_Z_GYRO              0x02 // R/W
#define SELF_TEST_X_ACCEL             0x0D // R/W
#define SELF_TEST_Z_ACCEL             0x0E // R/W
#define SELF_TEST_Y_ACCEL             0x0F // R/W

#define XG_OFFSET_H                   0x13 // R/W
#define XG_OFFSET_L                   0x14 // R/W
#define YG_OFFSET_H                   0x15 // R/W
#define YG_OFFSET_L                   0x16 // R/W
#define ZG_OFFSET_H                   0x17 // R/W
#define ZG_OFFSET_L                   0x18 // R/W

#define SMPLRT_DIV                    0x19 // R/W
#define CONFIG                        0x1A // R/W
#define GYRO_CONFIG                   0x1B // R/W
#define ACCEL_CONFIG                  0x1C // R/W
#define ACCEL_CONFIG_2                0x1D // R/W
#define LP_ACCEL_ODR                  0x1E // R/W
#define WOM_THR                       0x1F // R/W
#define FIFO_EN                       0x23 // R/W

// .. registers 0x24 - 0x36 are not applicable to the SensorTag HW configuration

#define INT_PIN_CFG                   0x37 // R/W
#define INT_ENABLE                    0x38 // R/W
#define INT_STATUS                    0x3A // R
#define ACCEL_XOUT_H                  0x3B // R
#define ACCEL_XOUT_L                  0x3C // R
#define ACCEL_YOUT_H                  0x3D // R
#define ACCEL_YOUT_L                  0x3E // R
#define ACCEL_ZOUT_H                  0x3F // R
#define ACCEL_ZOUT_L                  0x40 // R
#define TEMP_OUT_H                    0x41 // R
#define TEMP_OUT_L                    0x42 // R
#define GYRO_XOUT_H                   0x43 // R
#define GYRO_XOUT_L                   0x44 // R
#define GYRO_YOUT_H                   0x45 // R
#define GYRO_YOUT_L                   0x46 // R
#define GYRO_ZOUT_H                   0x47 // R
#define GYRO_ZOUT_L                   0x48 // R

// .. registers 0x49 - 0x60 are not applicable to the SensorTag HW configuration
// .. registers 0x63 - 0x67 are not applicable to the SensorTag HW configuration

#define SIGNAL_PATH_RESET             0x68 // R/W
#define ACCEL_INTEL_CTRL              0x69 // R/W
#define USER_CTRL                     0x6A // R/W
#define PWR_MGMT_1                    0x6B // R/W
#define PWR_MGMT_2                    0x6C // R/W
#define FIFO_COUNT_H                  0x72 // R/W
#define FIFO_COUNT_L                  0x73 // R/W
#define FIFO_R_W                      0x74 // R/W
#define WHO_AM_I                      0x75 // R/W

// Masks is mpuConfig valiable
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07

// Values PWR_MGMT_1
#define MPU_SLEEP                     0x4F  // Sleep + stop all clocks
#define MPU_WAKE_UP                   0x09  // Disable temp. + intern osc

// Values PWR_MGMT_2
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38

// Data sizes
#define DATA_SIZE                     6

// Output data rates
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255

// Bit values
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

// User control register
#define BIT_LATCH_EN                  0x20
#define BIT_ACTL                      0x80

// INT Pin / Bypass Enable Configuration
#define BIT_BYPASS_EN                 0x02
#define BIT_AUX_IF_EN                 0x20

// Magnetometer registers
#define MAG_WHO_AM_I                  0x00  // Should return 0x48
#define MAG_INFO                      0x01
#define MAG_ST1                       0x02  // Data ready status: bit 0
#define MAG_XOUT_L                    0x03  // Data array
#define MAG_XOUT_H                    0x04
#define MAG_YOUT_L                    0x05
#define MAG_YOUT_H                    0x06
#define MAG_ZOUT_L                    0x07
#define MAG_ZOUT_H                    0x08
#define MAG_ST2                       0x09  // Overflow(bit 3), read err(bit 2)
#define MAG_CNTL1                     0x0A  // Mode bits 3:0, resolution bit 4
#define MAG_CNTL2                     0x0B  // System reset, bit 0
#define MAG_ASTC                      0x0C  // Self test control
#define MAG_I2CDIS                    0x0F  // I2C disable
#define MAG_ASAX                      0x10  // x-axis sensitivity adjustment
#define MAG_ASAY                      0x11  // y-axis sensitivity adjustment
#define MAG_ASAZ                      0x12  // z-axis sensitivity adjustment

#define MAG_DEVICE_ID                 0x48

// Mode
#define MAG_MODE_OFF                  0x00
#define MAG_MODE_SINGLE               0x01
#define MAG_MODE_CONT1                0x02
#define MAG_MODE_CONT2                0x06
#define MAG_MODE_FUSE                 0x0F

// Resolution
#define MFS_14BITS                    0     // 0.6 mG per LSB
#define MFS_16BITS                    1     // 0.15 mG per LSB

#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
#define FIFO_COUNTH      0x72
#define I2C_MST_CTRL     0x24

// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1,Board_MPU9250_ADDR)
#define SENSOR_SELECT_MAG()           SensorI2C_select(SENSOR_I2C_1,Board_MPU9250_MAG_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()

/* -----------------------------------------------------------------------------
*  Local Functions
* ------------------------------------------------------------------------------
*/
static void sensorMpuSleep(void);
static void sensorMpu9250WakeUp(void);
static void sensorMpu9250SelectAxes(void);
static void SensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId);
static void sensorMagInit(void);
static void sensorMagEnable(bool);
static bool sensorMpu9250SetBypass(void);
//static void SensorMPU9250_calibrate_Mag(float * dest1, float * dest2);
static bool _SensorI2C_writeReg(uint8_t addr, uint8_t data);
//static void SensorMPU9250_calibrate_GyroAcc(float *dest1, float *dest2);

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/

static uint8_t mpuConfig;
static uint8_t magStatus;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t val;

//// Magnetometer calibration
//static uint16_t calX;
//static uint16_t calY;
//static uint16_t calZ;
static float magScale[3] = {0, 0, 0};  // Factory mag calibration and mag bias

// Magnetometer control
static uint8_t scale = MFS_16BITS;      // 16 bit resolution
static uint8_t mode = MAG_MODE_CONT1;  // Operating mode

// Pins that are used by the MPU9250
static PIN_Config MpuPinTable[] =
{
    Board_MPU_INT    | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

    PIN_TERMINATE
};
static PIN_State pinGpioState;
static PIN_Handle hMpuPin;

// The application may register a callback to handle interrupts
static SensorMpu9250CallbackFn_t isrCallbackFn = NULL;

/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/

void getMagScale(uint8_t* input) {
   memcpy(input, &magScale, sizeof(float) * 3);
}

/*******************************************************************************
* @fn          SensorMpu9250_powerOn
*
* @brief       This function turns on the power supply to MPU9250
*
* @return      none
*/
void SensorMpu9250_powerOn(void)
{
    // Turn on power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    DELAY_MS(100);
    SensorMpu9250_reset();
}

/*******************************************************************************
* @fn          SensorMpu9250_powerOff
*
* @brief       This function turns off the power supply to MPU9250
*
* @return      none
*/
void SensorMpu9250_powerOff(void)
{
    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    // Turn off power supply
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

    // Force an access on I2C bus #0 (sets the I2C lines to a defined state)
    SensorOpt3001_test();
}

/*******************************************************************************
* @fn          SensorMpu9250_powerIsOn
*
* @brief       Return 'true' if MPU power is on
*
* @return      state of MPU power
*/
bool SensorMpu9250_powerIsOn(void)
{
    return PIN_getOutputValue(Board_MPU_POWER) == Board_MPU_POWER_ON;
}

/*******************************************************************************
* @fn          SensorMpu9250_registerCallback
*
* @brief       Register a call-back for interrupt processing
*
* @return      none
*/
void SensorMpu9250_registerCallback(SensorMpu9250CallbackFn_t pfn)
{
    isrCallbackFn = pfn;
}

/*******************************************************************************
* @fn          SensorMpu9250_init
*
* @brief       This function initializes the MPU abstraction layer.
*
* @return      True if success
*/
bool SensorMpu9250_init(void)
{
    // Pins used by MPU
    hMpuPin = PIN_open(&pinGpioState, MpuPinTable);

    // Register MPU interrupt
    PIN_registerIntCb(hMpuPin, SensorMpu9250_Callback);

    // Application callback initially NULL
    isrCallbackFn = NULL;

    return SensorMpu9250_reset();
}


/*******************************************************************************
* @fn          SensorMpu9250_reset
*
* @brief       This function resets the MPU
*
* @return      True if success
*/
bool SensorMpu9250_reset(void)
{
    bool ret;

    // Make sure pin interrupt is disabled
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_DIS);

    accRange = ACC_RANGE_INVALID;
    mpuConfig = 0;   // All axes off
    magStatus = 0;

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Device reset
    val = 0x80;
    SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    SENSOR_DESELECT();

    DELAY_MS(100);

    ret = SensorMpu9250_test();
    if (ret)
    {
        // Initial configuration
        SensorMpu9250_accSetRange(ACC_RANGE_8G);
        sensorMagInit();

        // Power save
        sensorMpuSleep();
    }

    return ret;
}


/*******************************************************************************
* @fn          SensorMpu9250_enableWom
*
* @brief       Enable Wake On Motion functionality
*
* @param       threshold - wake-up trigger threshold (unit: 4 mg, max 1020mg)
*
* @return      True if success
*/
bool SensorMpu9250_enableWom(uint8_t threshold)
{
    ST_ASSERT(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure accelerometer is running
    val = 0x09;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Enable accelerometer, disable gyro
    val = 0x07;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_2, &val, 1));

    // Set Accel LPF setting to 184 Hz Bandwidth
    val = 0x01;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG_2, &val, 1));

    // Enable Motion Interrupt
    val = BIT_WOM_EN;
    ST_ASSERT(SensorI2C_writeReg(INT_ENABLE, &val, 1));

    // Enable Accel Hardware Intelligence
    val = 0xC0;
    ST_ASSERT(SensorI2C_writeReg(ACCEL_INTEL_CTRL, &val, 1));

    // Set Motion Threshold
    val = threshold;
    ST_ASSERT(SensorI2C_writeReg(WOM_THR, &val, 1));

    // Set Frequency of Wake-up
    val = INV_LPA_20HZ;
    ST_ASSERT(SensorI2C_writeReg(LP_ACCEL_ODR, &val, 1));

    // Enable Cycle Mode (Accel Low Power Mode)
    val = 0x29;
    ST_ASSERT(SensorI2C_writeReg(PWR_MGMT_1, &val, 1));

    // Select the current range
    ST_ASSERT(SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1));

    // Clear interrupt
    SensorI2C_readReg(INT_STATUS,&val,1);

    SENSOR_DESELECT();

    mpuConfig = 0;

    // Enable pin for wake-on-motion interrupt
    PIN_setInterrupt(hMpuPin, PIN_ID(Board_MPU_INT)|PIN_IRQ_POSEDGE);

    return true;
}

/*******************************************************************************
* @fn          SensorMpu9250_irqStatus
*
* @brief       Check whether a data or wake on motion interrupt has occurred
*
* @return      Return interrupt status
*/
uint8_t SensorMpu9250_irqStatus(void)
{
    uint8_t intStatus;

    intStatus = 0;
    ST_ASSERT(SensorMpu9250_powerIsOn());

    if (SENSOR_SELECT())
    {
        if (!SensorI2C_readReg(INT_STATUS,&intStatus,1))
        {
            intStatus = 0;
        }
        SENSOR_DESELECT();
    }

    return intStatus;
}

/*******************************************************************************
* @fn          SensorMpu9250_enable
*
* @brief       Enable accelerometer readout
*
* @param       Axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
* @                  Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
*                    MPU  bit [6], all axes
*
* @return      None
*/
void SensorMpu9250_enable(uint16_t axes)
{
    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (mpuConfig == 0 && axes != 0)
    {
        // Wake up the sensor if it was off
        sensorMpu9250WakeUp();
    }

    mpuConfig = axes;

    if (mpuConfig != 0)
    {
        // Enable gyro + accelerometer + magnetometer readout
        sensorMpu9250SelectAxes();
    }
    else if (mpuConfig == 0)
    {
        sensorMpuSleep();
    }
}


/*******************************************************************************
* @fn          SensorMpu9250_accSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       newRange: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*
* @return      true if write succeeded
*/
bool SensorMpu9250_accSetRange(uint8_t newRange)
{
    bool success;

    if (newRange == accRange)
    {
        return true;
    }

    ST_ASSERT(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    accRangeReg = (newRange << 3);

    // Apply the range
    success = SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    if (success)
    {
        accRange = newRange;
    }

    return success;
}

/*******************************************************************************
* @fn          SensorMpu9250_accReadRange
*
* @brief       Apply the selected accelerometer range
*
* @param       none
*
* @return      range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
*/
uint8_t SensorMpu9250_accReadRange(void)
{
    ST_ASSERT(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Apply the range
    SensorI2C_readReg(ACCEL_CONFIG, &accRangeReg, 1);
    SENSOR_DESELECT();

    accRange = (accRangeReg>>3) & 3;

    return accRange;
}


/*******************************************************************************
* @fn          SensorMpu9250_accRead
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 words
*
* @return      True if data is valid
*/
bool SensorMpu9250_accRead(uint16_t *data )
{
    bool success;

    ST_ASSERT(SensorMpu9250_powerIsOn());

    // Burst read of all accelerometer values
    if (!SENSOR_SELECT())
    {
        return false;
    }

    success = SensorI2C_readReg(ACCEL_XOUT_H, (uint8_t*)data, DATA_SIZE);
    SENSOR_DESELECT();

    if (success)
    {
        SensorUtil_convertToLe((uint8_t*)data,DATA_SIZE);
    }

    return success;
}

/*******************************************************************************
* @fn          SensorMpu9250_gyroRead
*
* @brief       Read data from the gyroscope - X, Y, Z - 3 words
*
* @return      TRUE if valid data, FALSE if not
*/
bool SensorMpu9250_gyroRead(uint16_t *data )
{
    bool success;

    ST_ASSERT(SensorMpu9250_powerIsOn());

    // Select this sensor
    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Burst read of all gyroscope values
    success = SensorI2C_readReg(GYRO_XOUT_H, (uint8_t*)data, DATA_SIZE);

    SENSOR_DESELECT();

    if (success)
    {
        SensorUtil_convertToLe((uint8_t*)data,DATA_SIZE);
    }

    return success;
}

/*******************************************************************************
 * @fn          SensorMpu9250_test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorMpu9250_test(void)
{
    static bool first = true;

    ST_ASSERT(SensorMpu9250_powerIsOn());

    // Select Gyro/Accelerometer
    if (!SENSOR_SELECT())
    {
        return false;
    }

    // Make sure power is ramped up
    if (first)
    {
        DELAY_MS(100);
        first = false;
    }

    // Check the WHO AM I register
    ST_ASSERT(SensorI2C_readReg(WHO_AM_I, &val, 1));
    ST_ASSERT(val == 0x71);

    SENSOR_DESELECT();

    return true;
}


/*******************************************************************************
 * @fn          SensorMpu9250_accConvert
 *
 * @brief       Convert raw data to G units
 *
 * @param       rawData - raw data from sensor
 *
 * @return      Converted value
 ******************************************************************************/
float SensorMpu9250_accConvert(int16_t rawData)
{
    float v;

    switch (accRange)
    {
    case ACC_RANGE_2G:
        //-- calculate acceleration, unit G, range -2, +2
        v = (rawData * 1.0) / (32768/2);
        break;

    case ACC_RANGE_4G:
        //-- calculate acceleration, unit G, range -4, +4
        v = (rawData * 1.0) / (32768/4);
        break;

    case ACC_RANGE_8G:
        //-- calculate acceleration, unit G, range -8, +8
        v = (rawData * 1.0) / (32768/8);
        break;

    case ACC_RANGE_16G:
        //-- calculate acceleration, unit G, range -16, +16
        v = (rawData * 1.0) / (32768/16);
        break;
    }

    return v;
}

/*******************************************************************************
 * @fn          SensorMpu9250_gyroConvert
 *
 * @brief       Convert raw data to deg/sec units
 *
 * @param       data - raw data from sensor
 *
 * @return      none
 ******************************************************************************/
float SensorMpu9250_gyroConvert(int16_t data)
{
    //-- calculate rotation, unit deg/s, range -250, +250
    return (data * 1.0) / (65536 / 500);
}

/*******************************************************************************
* @fn          sensorMpuSleep
*
* @brief       Place the MPU in low power mode
*
* @return
*/
static void sensorMpuSleep(void)
{
    bool success;

    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    val = ALL_AXES;
    success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
    if (success)
    {
        val = MPU_SLEEP;
        success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);
    }

    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorMpu9250WakeUp
*
* @brief       Exit low power mode
*
* @return      none
*/
static void sensorMpu9250WakeUp(void)
{
    bool success;

    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (!SENSOR_SELECT())
    {
        return;
    }

    val = MPU_WAKE_UP;
    success = SensorI2C_writeReg(PWR_MGMT_1, &val, 1);

    if (success)
    {
        // All axis initially disabled
        val = ALL_AXES;
        success = SensorI2C_writeReg(PWR_MGMT_2, &val, 1);
        mpuConfig = 0;
    }

    if (success)
    {
        // Restore the range
        success = SensorI2C_writeReg(ACCEL_CONFIG, &accRangeReg, 1);

        if (success)
        {
            // Clear interrupts
            success = SensorI2C_readReg(INT_STATUS,&val,1);
        }
    }

    SENSOR_DESELECT();
}

/*******************************************************************************
* @fn          sensorMpu9250SelectAxes
*
* @brief       Select gyro, accelerometer, magnetometer
*
* @return      none
*/
static void sensorMpu9250SelectAxes(void)
{
    if (!SENSOR_SELECT())
    {
        return;
    }

    // Select gyro and accelerometer (3+3 axes, one bit each)
    val = ~mpuConfig;
    SensorI2C_writeReg(PWR_MGMT_2, &val, 1);

    SENSOR_DESELECT();

    // Select magnetometer (all axes at once)
    sensorMagEnable(!!(mpuConfig & MPU_AX_MAG));
}

/*******************************************************************************
* @fn          sensorMpu9250SetBypass
*
* @brief       Allow the I2C bus to control the magnetomoter
*
* @return      true if success
*/
static bool sensorMpu9250SetBypass(void)
{
    bool success;

    if (SENSOR_SELECT())
    {
        val = BIT_BYPASS_EN | BIT_LATCH_EN;
        success = SensorI2C_writeReg(INT_PIN_CFG, &val, 1);
        DELAY_MS(10);

        SENSOR_DESELECT();
    }
    else
    {
        success = false;
    }

    return success;
}

/*******************************************************************************
* @fn          sensorMagInit
*
* @brief       Initialize the compass
*
* @return      none
*/
static void sensorMagInit(void)
{
    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (!sensorMpu9250SetBypass())
    {
        return;
    }

    if (SENSOR_SELECT_MAG())
    {
        static uint8_t rawData[3];

        // Enter Fuse ROM access mode
        val = MAG_MODE_FUSE;
        SensorI2C_writeReg(MAG_CNTL1, &val, 1);
        DELAY_MS(10);

        // Get calibration data
        if (SensorI2C_readReg(MAG_ASAX, &rawData[0], 3))
        {
            // Return x-axis sensitivity adjustment values, etc.
            magScale[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;
            magScale[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
            magScale[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
        }

        // Turn off the sensor by doing a reset
        val = 0x01;
        SensorI2C_writeReg(MAG_CNTL2, &val, 1);

        SENSOR_DESELECT();
    }
}

/*******************************************************************************
 * @fn          SensorMpu9250_magReset
 *
 * @brief       Reset the magnetometer
 *
 * @return      none
 */
void SensorMpu9250_magReset(void)
{
    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (sensorMpu9250SetBypass())
    {
        if (SENSOR_SELECT_MAG())
        {
            // Turn off the sensor by doing a reset
            val = 0x01;
            SensorI2C_writeReg(MAG_CNTL2, &val, 1);
            DELAY_MS(10);

            // Re-enable if already active
            if (mpuConfig & MPU_AX_MAG)
            {
                // Set magnetometer data resolution and sample ODR
                val = (scale << 4) | mode;
                SensorI2C_writeReg(MAG_CNTL1, &val, 1);
            }
            SENSOR_DESELECT();
        }
    }
}

/*******************************************************************************
 * @fn          sensorMagEnable
 *
 * @brief       Enable or disable the compass part of the MPU9250
 *
 * @return      none
 */
static void sensorMagEnable(bool enable)
{
    uint8_t val;

    ST_ASSERT_V(SensorMpu9250_powerIsOn());

    if (!sensorMpu9250SetBypass())
    {
        return;
    }

    if (SENSOR_SELECT_MAG())
    {
        if (enable)
        {
            // Set magnetometer data resolution and sample ODR
            val = (scale << 4) | mode;
        }
        else
        {
            // Power down magnetometer
            val = 0x00;
        }
        SensorI2C_writeReg(MAG_CNTL1, &val, 1);

        SENSOR_DESELECT();
    }
}

/*******************************************************************************
 * @fn          SensorMpu9250_magTest
 *
 * @brief       Run a magnetometer self test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool SensorMpu9250_magTest(void)
{
    ST_ASSERT(SensorMpu9250_powerIsOn());

    // Connect magnetometer internally in MPU9250
    sensorMpu9250SetBypass();

    // Select magnetometer
    SENSOR_SELECT_MAG();

    // Check the WHO AM I register
    val = 0xFF;
    ST_ASSERT(SensorI2C_readReg(MAG_WHO_AM_I, &val, 1));
    ST_ASSERT(val == MAG_DEVICE_ID);

    SENSOR_DESELECT();

    return true;
}

/*******************************************************************************
* @fn          SensorMpu9250_magRead
*
* @brief       Read data from the compass - X, Y, Z - 3 words
*
* @return      Magnetometer status
*/
uint8_t SensorMpu9250_magRead(int16_t *data)
{
    uint8_t val;
    uint8_t rawData[7];  // x/y/z compass register data, ST2 register stored here,
    // must read ST2 at end of data acquisition
    magStatus = MAG_NO_POWER;
    ST_ASSERT(SensorMpu9250_powerIsOn());

    magStatus = MAG_STATUS_OK;

    // Connect magnetometer internally in MPU9250
    SENSOR_SELECT();
    val = BIT_BYPASS_EN | BIT_LATCH_EN;
    if (!SensorI2C_writeReg(INT_PIN_CFG, &val, 1))
    {
        magStatus = MAG_BYPASS_FAIL;
    }
    SENSOR_DESELECT();

    if (magStatus != MAG_STATUS_OK)
    {
        return false;
    }

    // Select this sensor
    SENSOR_SELECT_MAG();

    if (SensorI2C_readReg(MAG_ST1,&val,1))
    {
        // Check magnetometer data ready bit
        if (val & 0x01)
        {
            // Burst read of all compass values + ST2 register
            if (SensorI2C_readReg(MAG_XOUT_L, &rawData[0],7))
            {
                val = rawData[6]; // ST2 register

                // Check if magnetic sensor overflow set, if not report data
                if(!(val & 0x08))
                {
                    // Turn the MSB and LSB into a signed 16-bit value,
                    // data stored as little Endian
                    data[0] = ((int16_t)rawData[1] << 8) | rawData[0];
                    data[1] = ((int16_t)rawData[3] << 8) | rawData[2];
                    data[2] = ((int16_t)rawData[5] << 8) | rawData[4];

                    // Sensitivity adjustment
                    // data[0] = data[0] * (uint8_t)magScale[0] >> 8 + (int16_t)1.0;
                    // data[1] = data[1] * (uint8_t)magScale[1] >> 8 + (int16_t)1.0;
                    // data[2] = data[2] * (uint8_t)magScale[2] >> 8 + (int16_t)1.0;
                }
                else
                {
                    magStatus = MAG_OVERFLOW;
                }
            }
            else
            {
                magStatus = MAG_READ_DATA_ERR;
            }
        }
        else
        {
            magStatus = MAG_DATA_NOT_RDY;
        }
    }
    else
    {
        magStatus = MAG_READ_ST_ERR;
    }

    // Set magnetometer data resolution and sample ODR
    // Start new conversion
    val = (scale << 4) | mode;
    SensorI2C_writeReg(MAG_CNTL1, &val, 1);

    SENSOR_DESELECT();

    return magStatus;
}

/*******************************************************************************
* @fn          SensorMpu9250_magStatus
*
* @brief       Return the magnetometer status
*
* @return      mag status
*/
uint8_t SensorMpu9250_magStatus(void)
{
    return magStatus;
}

/*******************************************************************************
 *  @fn         SensorMpu9250_Callback
 *
 *  Interrupt service routine for the MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void SensorMpu9250_Callback(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == Board_MPU_INT)
    {
        if (isrCallbackFn != NULL)
        {
            isrCallbackFn();
        }
    }
}

static bool _SensorI2C_writeReg(uint8_t addr, uint8_t data) {
    return SensorI2C_writeReg(addr, &data, 1);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void SensorMPU9250_calibrate_GyroAcc(float *dest1, float *dest2) {

    //void SensorMPU9250_calibrate_all(float *dest1, float *dest2) {
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers

    _SensorI2C_writeReg(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    DELAY_MS(0.1 * 1000);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    _SensorI2C_writeReg(PWR_MGMT_1, 0x01);
    _SensorI2C_writeReg(PWR_MGMT_2, 0x00);
    DELAY_MS(0.2 * 1000);

    // Configure device for bias calculation
    _SensorI2C_writeReg(INT_ENABLE, 0x00);   // Disable all interrupts
    _SensorI2C_writeReg(FIFO_EN, 0x00);      // Disable FIFO
    _SensorI2C_writeReg(PWR_MGMT_1, 0x00);   // Turn on internal clock source
    _SensorI2C_writeReg(I2C_MST_CTRL, 0x00); // Disable I2C master
    _SensorI2C_writeReg(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    _SensorI2C_writeReg(USER_CTRL, 0x0C);    // Reset FIFO and DMP
    DELAY_MS(0.015 * 1000);

    // Configure MPU9250 gyro and accelerometer for bias calculation
    _SensorI2C_writeReg(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    _SensorI2C_writeReg(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    _SensorI2C_writeReg(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    _SensorI2C_writeReg(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    _SensorI2C_writeReg(USER_CTRL, 0x40);   // Enable FIFO
    _SensorI2C_writeReg(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
    DELAY_MS(0.04 * 1000); // accumulate 40 samples in 80 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    _SensorI2C_writeReg(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    SensorI2C_readReg(FIFO_COUNTH, &data[0], 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    SensorI2C_readReg(FIFO_R_W, &data[0], 12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    /// Push gyro biases to hardware registers
    _SensorI2C_writeReg(XG_OFFSET_H, data[0]);
    _SensorI2C_writeReg(XG_OFFSET_L, data[1]);
    _SensorI2C_writeReg(YG_OFFSET_H, data[2]);
    _SensorI2C_writeReg(YG_OFFSET_L, data[3]);
    _SensorI2C_writeReg(ZG_OFFSET_H, data[4]);
    _SensorI2C_writeReg(ZG_OFFSET_L, data[5]);

    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    SensorI2C_readReg(XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    SensorI2C_readReg(YA_OFFSET_H, &data[0], 2);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    SensorI2C_readReg(ZA_OFFSET_H, &data[0], 2);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    _SensorI2C_writeReg(XA_OFFSET_H, data[0]);
    _SensorI2C_writeReg(XA_OFFSET_L, data[1]);
    _SensorI2C_writeReg(YA_OFFSET_H, data[2]);
    _SensorI2C_writeReg(YA_OFFSET_L, data[3]);
    _SensorI2C_writeReg(ZA_OFFSET_H, data[4]);
    _SensorI2C_writeReg(ZA_OFFSET_L, data[5]);
    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

//void SensorMPU9250_calibrate_all(float *dest1, float *dest2) {
//    SensorMPU9250_calibrate_GyroAcc(dest1, dest2);
////    SensorMPU9250_calibrate_Mag(dest3, magScale);
//    return;
//}
//
//static void SensorMPU9250_calibrate_Mag(float * dest1, float* dest2)
// {
//    uint16_t ii = 0, sample_count = 0;
//    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
//    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
//
//    DELAY_MS(4000);
//
//    // shoot for ~fifteen seconds of mag data
//    if(mode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
//    if(mode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
//    for(ii = 0; ii < sample_count; ii++) {
//
//        SensorMpu9250_magRead(mag_temp);  // Read the mag data
//        for (int jj = 0; jj < 3; jj++) {
//            mag_temp[jj] = ((mag_temp[jj] - (int16_t)1.0) << 8)/magScale[jj];
//            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
//            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
//        }
//        if(mode == 0x02) DELAY_MS(135);  // at 8 Hz ODR, new mag data is available every 125 ms
//        if(mode == 0x06) DELAY_MS(12);  // at 100 Hz ODR, new mag data is available every 10 ms
//    }
//
//
//    // Get hard iron correction
//    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
//    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
//    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
//
//    dest1[0] = (float) mag_bias[0]*16*mag_scale[0] ;  // save mag biases in G for main program
//    dest1[1] = (float) mag_bias[1]*16*mag_scale[1];
//    dest1[2] = (float) mag_bias[2]*16*mag_scale[2];
//
//    // Get soft iron correction estimate
//    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
//    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
//    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
//
//    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
//    avg_rad /= 3.0;
//
//    dest2[0] = avg_rad/((float)mag_scale[0]);
//    dest2[1] = avg_rad/((float)mag_scale[1]);
//    dest2[2] = avg_rad/((float)mag_scale[2]);
// }

// Gyro, Acc, Magbias, magscale

