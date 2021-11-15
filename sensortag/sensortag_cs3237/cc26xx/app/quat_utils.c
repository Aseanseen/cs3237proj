#include "quat_utils.h"
#include "quatservice.h"
#include "hal_types.h"
#include "SensorMpu9250_Q.h"

#include <math.h>

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float PI = 3.14159265358979323846f;

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method
// extern float gyroBiasX=0, gyroBiasY=0,gyroBiasZ=0, accelBiasX=0, accelBiasY=0,accelBiasZ=0;

float invSqrt(float x);

void MahonyQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void updateQuatData( void );
void readQuatData( uint8 data[9] );

static float calcXValueG( uint8 data[] )
{
    //Orientation of sensor on board means we need to swap X (multiplying with -1)
    // int16 rawX = (data[0] & 0xff) | ((data[1] << 8) & 0xff00);
	// GFS 250dps
	int16 rawX = (data[0] & 0xff) | ((data[1] << 8) & 0xff00);
    float lastX = (((float)rawX * 1.0) / ( 65536 / 500.0 )) * -1;
    return lastX;
}

static float calcYValueG( uint8 data[] )
{
    //Orientation of sensor on board means we need to swap X (multiplying with -1)
    int16 rawY = (data[2] & 0xff) | ((data[3] << 8) & 0xff00);
    float lastY = (((float)rawY * 1.0) / ( 65536 / 500.0 )) * -1;
    return lastY;
}

static float calcZValueG( uint8 data[] )
{
    //Orientation of sensor on board means we need to swap X (multiplying with -1)
    int16 rawZ = (data[4] & 0xff) | ((data[5] << 8) & 0xff00);
    float lastZ = (((float)rawZ * 1.0) / ( 65536 / 500.0 ));
    return lastZ;
}

static float calcXValueA( uint8 data[] )
{
	int16 rawA = (data[0] & 0xff) | ((data[1] << 8) & 0xff00);
    return (((float)rawA * 1.0) / (32768 / 4.0));
}
static float calcYValueA( uint8 data[] )
{
    //Orientation of sensor on board means we need to swap Y (multiplying with -1)
	int16 rawA = (data[2] & 0xff) | ((data[3] << 8) & 0xff00) * -1;
    return (((float)rawA * 1.0) / (32768 / 4.0));
}
static float calcZValueA( uint8 data[] )
{
	int16 rawA = (data[4] & 0xff) | ((data[5] << 8) & 0xff00);
    return (((float)rawA * 1.0) / (32768 / 4.0));
}


float calcXValueM( uint8 data[] )
{
	int16 rawM = (data[0] & 0xff) | ((data[1] << 8) & 0xff00) * -1;
	float lastX = ((float)rawM * 1.0) * 10.0*4912.0/32760.0 * magCalX;
	// return ((float)rawM) * 10.0*4912.0/8190.0;
    // //Orientation of sensor on board means we need to swap X (multiplying with -1)
    // int16 rawX = (data[0] & 0xff) | ((data[1] << 8) & 0xff00);
    // float lastX = (((float)rawM * 1.0) / ( 65536 / 2000.0 ));
    // return lastX;
	return lastX;
}
float calcYValueM( uint8 data[] )
{
    // //Orientation of sensor on board means we need to swap Y (multiplying with -1)
    int16 rawM = ((data[2] & 0xff) | ((data[3] << 8) & 0xff00)) * -1;
	float lastY = ((float)rawM * 1.0) * 10.0*4912.0/32760.0 * magCalY;
    // float lastY = (((float)rawM * 1.0) / ( 65536 / 2000.0 )) ;
	// int16 rawM = (data[2] & 0xff) | ((data[3] << 8) & 0xff00) * -1;
    // return ((float)rawM) * 10.0*4912.0/8190.0;
	return lastY;
}
float calcZValueM( uint8 data[] )
{
    int16 rawM = (data[4] & 0xff) | ((data[5] << 8) & 0xff00);
	float lastZ =  ((float)rawM * 1.0) * 10.0*4912.0/32760.0 * magCalZ;
    // float lastZ =  ((float)rawM * 1.0) / ( 65536 / 2000.0 );
    // return lastZ;
	// int16 rawM = (data[4] & 0xff) | ((data[5] << 8) & 0xff00);
	// return ((float)rawM) * 10.0*4912.0/8190.0;
	return lastZ;
}

union Data
{
   float f;
   uint8 u[4];
};

void updateQuatData( void )
{
  	union Data qq0,qq1,qq2,qq3;
	qq0.f=q0;
	qq1.f=q1;
	qq2.f=q2;
	qq3.f=q3;
	// qq0.f = magScaleX;
	// qq1.f = magScaleY;
	// qq2.f = magScaleZ;
	// qq3.f = magBiasX;
	// qq0.f = 1.0;
	// qq1.f = 2.0;
	// qq2.f = 3.0;
	// qq3.f = 4.0;
	  
	uint8 qData[QUAT_DATA_LEN];

	qData[0]=qq0.u[0];
	qData[1]=qq0.u[1];
	qData[2]=qq0.u[2];
	qData[3]=qq0.u[3]; 

	qData[4]=qq1.u[0];
	qData[5]=qq1.u[1];
	qData[6]=qq1.u[2];
	qData[7]=qq1.u[3];

	qData[8]=qq2.u[0];
	qData[9]=qq2.u[1];
	qData[10]=qq2.u[2];
	qData[11]=qq2.u[3]; 

	qData[12]=qq3.u[0];
	qData[13]=qq3.u[1];
	qData[14]=qq3.u[2];
	qData[15]=qq3.u[3];
	qData[16]=0;
	qData[17]=0;

	Quat_setParameter( SENSOR_DATA, QUAT_DATA_LEN, qData);
	// //repeat
	// qq0.f = magBiasY;
	// qq1.f = magBiasZ;
	// qq2.f = accelBiasY;
	// qq3.f = accelBiasZ;
	// // qq0.f = 5.0;
	// // qq1.f = 6.0;
	// // qq2.f = 7.0;
	// // qq3.f = 8.0;
	  
	// qData[0]=qq0.u[0];
	// qData[1]=qq0.u[1];
	// qData[2]=qq0.u[2];
	// qData[3]=qq0.u[3]; 

	// qData[4]=qq1.u[0];
	// qData[5]=qq1.u[1];
	// qData[6]=qq1.u[2];
	// qData[7]=qq1.u[3];

	// qData[8]=qq2.u[0];
	// qData[9]=qq2.u[1];
	// qData[10]=qq2.u[2];
	// qData[11]=qq2.u[3]; 

	// qData[12]=qq3.u[0];
	// qData[13]=qq3.u[1];
	// qData[14]=qq3.u[2];
	// qData[15]=qq3.u[3];
	// qData[16]=0;
	// qData[17]=0;


    // Quat_setParameter( SENSOR_DATA, QUAT_DATA_LEN, qData);
}


void readQuatData( uint8 data[18] )
{
	uint8 aData[6];
	uint8 mData[6];
	uint8 gData[6];

	for (int i = 0; i < 6; i++) {
		gData[i] = data[0 + i];
		aData[i] = data[6 + i];
		mData[i] = data[12 + i];
	}

	// getMagScale(&magScale);
	float gx=calcXValueG(gData) - gyroBiasX;
	float gy=calcYValueG(gData) - gyroBiasY;
	float gz=calcZValueG(gData) - gyroBiasZ;
	float ax=calcXValueA(aData) - accelBiasX;
	float ay=calcYValueA(aData) - accelBiasY;
	float az=calcZValueA(aData) - accelBiasZ;
	float mx=(calcXValueM(mData) - magBiasX) * magScaleX;
	float my=(calcYValueM(mData) - magBiasY) * magScaleY;
	float mz=(calcZValueM(mData) - magBiasZ) * magScaleZ;
	// MahonyQuaternionUpdate(gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, ax, ay, az, mx, my, mz);
	MadgwickQuaternionUpdate(gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, ax, ay, az, 1,1,1);
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q0mx;
	float _2q0my;
	float _2q0mz;
	float _2q1mx;
	float _4bx;
	float _4bz;
	float _2q0 = 2.0f * q0;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q0q2 = 2.0f * q0 * q2;
	float _2q2q3 = 2.0f * q2 * q3;
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	// float beta = sqrt(3.0f / 4.0f) * 3.14159265358979323846f * (60.0f / 180.0f);
	float beta = 0.1f;
	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s2 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s3 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s4 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s1;
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s2;
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s3;
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s4;

	// Integrate to yield quaternion
	q0 += qDot1 / sampleFreq;
	q1 += qDot2 / sampleFreq;
	q2 += qDot3 / sampleFreq;
	q3 += qDot4 / sampleFreq;
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);    // normalise quaternion
	norm = 1.0f/norm;
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

}
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
void MahonyQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;   

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q1;
	pb = q2;
	pc = q3;
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (0.5f / sampleFreq);
	q1 = pa + (q0 * gx + pb * gz - pc * gy) * (0.5f / sampleFreq);
	q2 = pb + (q0 * gy - pa * gz + pc * gx) * (0.5f / sampleFreq);
	q3 = pc + (q0 * gz + pa * gy - pb * gx) * (0.5f / sampleFreq);

	// Normalise quaternion
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	norm = 1.0f / norm;
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}




