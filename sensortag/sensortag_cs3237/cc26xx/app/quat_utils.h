#ifndef QUATUTILS_H
#define QUATUTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "st_util.h"
#define sampleFreq	100.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern float magCalibration[3], magbias[3];  // Factory mag calibration and mag bias
extern float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
//---------------------------------------------------------------------------------------------------
// Function declarations
extern void readQuatData( uint8* data );
extern void updateQuatData( void );



#ifdef __cplusplus
}
#endif

#endif /* QUATUTILS_H */
