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
#include "SensorMpu9250.h"
#include "SensorMpu9250Dmp.h"
#include "SensorOpt3001.h" // For reset of I2C bus
#include "SensorUtil.h"
#include "SensorI2C.h"

/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/
// Registers
#define TAP_X               0x01
#define TAP_Y               0x02
#define TAP_Z               0x04
#define TAP_XYZ             0x07

#define TAP_X_UP            0x01
#define TAP_X_DOWN          0x02
#define TAP_Y_UP            0x03
#define TAP_Y_DOWN          0x04
#define TAP_Z_UP            0x05
#define TAP_Z_DOWN          0x06

#define ANDROID_ORIENT_PORTRAIT             0x00
#define ANDROID_ORIENT_LANDSCAPE            0x01
#define ANDROID_ORIENT_REVERSE_PORTRAIT     0x02
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    0x03

#define DMP_INT_GESTURE     0x01
#define DMP_INT_CONTINUOUS  0x02

#define DMP_FEATURE_TAP             0x001
#define DMP_FEATURE_ANDROID_ORIENT  0x002
#define DMP_FEATURE_LP_QUAT         0x004
#define DMP_FEATURE_PEDOMETER       0x008
#define DMP_FEATURE_6X_LP_QUAT      0x010
#define DMP_FEATURE_GYRO_CAL        0x020
#define DMP_FEATURE_SEND_RAW_ACCEL  0x040
#define DMP_FEATURE_SEND_RAW_GYRO   0x080
#define DMP_FEATURE_SEND_CAL_GYRO   0x100

#define INV_WXYZ_QUAT       0x100

// Sensor selection/de-selection
#define SENSOR_SELECT()               SensorI2C_select(SENSOR_I2C_1,Board_MPU9250_ADDR)
#define SENSOR_SELECT_MAG()           SensorI2C_select(SENSOR_I2C_1,Board_MPU9250_MAG_ADDR)
#define SENSOR_DESELECT()             SensorI2C_deselect()

int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;
long qw, qx, qy, qz;
long temperature;
unsigned long time;
float pitch, roll, yaw;
float heading;

static unsigned char mpu9250_orientation;
static unsigned char tap_count;
static unsigned char tap_direction;
static bool _tap_available;
static void orient_cb(unsigned char orient);
static void tap_cb(unsigned char direction, unsigned char count);

float PI = 3.14159265358979323846f;
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

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/
static uint8_t mpuConfig;
static uint8_t magStatus;
static uint8_t accRange;
static uint8_t accRangeReg;
static uint8_t val;

// Magnetometer calibration
static int16_t calX;
static int16_t calY;
static int16_t calZ;

// Magnetometer control
static uint8_t scale = MFS_16BITS;      // 16 bit resolution
static uint8_t mode = MAG_MODE_SINGLE;  // Operating mode

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
            calX =  (int16_t)rawData[0] + 128;
            calY =  (int16_t)rawData[1] + 128;
            calZ =  (int16_t)rawData[2] + 128;
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
                    data[0] = data[0] * calX >> 8;
                    data[1] = data[1] * calY >> 8;
                    data[2] = data[2] * calZ >> 8;
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


inv_error_t dmp_begin(unsigned short features, unsigned short fifoRate) {
    unsigned short feat = features;
	unsigned short rate = fifoRate;

	if (dmpLoad() != INV_SUCCESS)
		return INV_ERROR;
	
	// 3-axis and 6-axis LP quat are mutually exclusive.
	// If both are selected, default to 3-axis
	if (feat & DMP_FEATURE_LP_QUAT)
	{
		feat &= ~(DMP_FEATURE_6X_LP_QUAT);
		dmp_enable_lp_quat(1);
	}
	else if (feat & DMP_FEATURE_6X_LP_QUAT)
		dmp_enable_6x_lp_quat(1);
	
	if (feat & DMP_FEATURE_GYRO_CAL)
		dmp_enable_gyro_cal(1);
	
	if (dmpEnableFeatures(feat) != INV_SUCCESS)
		return INV_ERROR;
	
	rate = constrain(rate, 1, 200);
	if (dmpSetFifoRate(rate) != INV_SUCCESS)
		return INV_ERROR;
	
	return mpu_set_dmp_state(1);
}

inv_error_t updateTemperature(void)
{
	return mpu_get_temperature(&temperature, &time);
}

int selfTest(unsigned char debug)
{
	long gyro[3], accel[3];
	return mpu_run_self_test(gyro, accel);
}

inv_error_t dmpLoad(void)
{
	return dmp_load_motion_driver_firmware();
}

unsigned short dmpGetFifoRate(void)
{
	unsigned short rate;
	if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
		return rate;
	
	return 0;
}

inv_error_t dmpSetFifoRate(unsigned short rate)
{
	if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
	return dmp_set_fifo_rate(rate);
}

inv_error_t dmpUpdateFifo(void)
{
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;
	
	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }
	
	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		qw = quat[0];
		qx = quat[1];
		qy = quat[2];
		qz = quat[3];
	}
	
	time = timestamp;
	
	return INV_SUCCESS;
}

inv_error_t dmpEnableFeatures(unsigned short mask)
{
	unsigned short enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP; 
	return dmp_enable_feature(enMask);
}

unsigned short dmpGetEnabledFeatures(void)
{
	unsigned short mask;
	if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
		return mask;
	return 0;
}

inv_error_t dmpSetOrientation(const signed char * orientationMatrix)
{
	unsigned short scalar;
	scalar = orientation_row_2_scale(orientationMatrix);
	scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
	scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;
	
    dmp_register_android_orient_cb(orient_cb);
	
	return dmp_set_orientation(scalar);
}

unsigned char dmpGetOrientation(void)
{
	return mpu9250_orientation;
}

inv_error_t dmpEnable3Quat(void)
{
	unsigned short dmpFeatures;
	
	// 3-axis and 6-axis quat are mutually exclusive
	dmpFeatures = dmpGetEnabledFeatures();
	dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
	dmpFeatures |= DMP_FEATURE_LP_QUAT;
	
	if (dmpEnableFeatures(dmpFeatures) != INV_SUCCESS)
		return INV_ERROR;
	
	return dmp_enable_lp_quat(1);
}

float calcAccel(int axis)
{
	return (float) axis / (float) _aSense;
}

float calcGyro(int axis)
{
	return (float) axis / (float) _gSense;
}

float calcMag(int axis)
{
	return (float) axis / (float) _mSense;
}

float calcQuat(long axis)
{
	return qToFloat(axis, 30);
}
	
float qToFloat(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void computeEulerAngles(bool degrees)
{
    float dqw = qToFloat(qw, 30);
    float dqx = qToFloat(qx, 30);
    float dqy = qToFloat(qy, 30);
    float dqz = qToFloat(qz, 30);
    
    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
  
	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
  
    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);
	
	if (degrees)
	{
		pitch *= (180.0 / PI);
		roll *= (180.0 / PI);
		yaw *= (180.0 / PI);
		if (pitch < 0) pitch = 360.0 + pitch;
		if (roll < 0) roll = 360.0 + roll;
		if (yaw < 0) yaw = 360.0 + yaw;	
	}
}

unsigned short orientation_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}
		
static void tap_cb(unsigned char direction, unsigned char count)
{
	_tap_available = true;
	tap_count = count;
	tap_direction = direction;
}

static void orient_cb(unsigned char orient)
{
	mpu9250_orientation = orient;
}