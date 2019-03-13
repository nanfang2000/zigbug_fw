/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Quaternion.h"

#include <math.h>
#include "FastMath.h"

inline void Quaternion_Add(float *r, float *a, float *b)
{
    r[0] = a[0] + b[0];
    r[1] = a[1] + b[1];
    r[2] = a[2] + b[2];
    r[3] = a[3] + b[3];
}

inline void Quaternion_Sub(float *r, float *a, float *b)
{
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
    r[3] = a[3] - b[3];
}

inline void Quaternion_Multiply(float *r, float *a, float *b)
{
    r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

inline void Quaternion_Conjugate(float *r, float *a)
{
    r[0] = a[0];
    r[1] = -a[1];
    r[2] = -a[2];
    r[3] = -a[3];
}

inline void Quaternion_Scalar(float *r, float *q, float scalar)
{
    r[0] = q[0] * scalar;
    r[1] = q[1] * scalar;
    r[2] = q[2] * scalar;
    r[3] = q[3] * scalar;
}

void Quaternion_Normalize(float *q)
{
	float norm = FastSqrtI(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= norm;
	q[1] *= norm;
	q[2] *= norm;
	q[3] *= norm;
}

void Quaternion_FromEuler(float *q, float *rpy)
{
	float sPhi2, cPhi2; // sin(phi/2) and cos(phi/2)
	float sThe2, cThe2; // sin(theta/2) and cos(theta/2)
	float sPsi2, cPsi2; // sin(psi/2) and cos(psi/2)
	// calculate sines and cosines
	
	FastSinCos(0.5f * rpy[0], &sPhi2, &cPhi2);
	FastSinCos(0.5f * rpy[1], &sThe2, &cThe2);
	FastSinCos(0.5f * rpy[2], &sPsi2, &cPsi2);
	
	// compute the quaternion elements
	q[0] = cPsi2 * cThe2 * cPhi2 + sPsi2 * sThe2 * sPhi2;
	q[1] = cPsi2 * cThe2 * sPhi2 - sPsi2 * sThe2 * cPhi2;
	q[2] = cPsi2 * sThe2 * cPhi2 + sPsi2 * cThe2 * sPhi2;
	q[3] = sPsi2 * cThe2 * cPhi2 - cPsi2 * sThe2 * sPhi2;
}

void Quaternion_FromRotationMatrix(float *R, float *Q)
{
#if 0
	// calculate the trace of the matrix
	float trace = R[0] + R[4] + R[8];
	float s;
	if(trace > 0){
		s = 0.5f * FastSqrt(trace + 1.0f);
		Q[0] = 0.25f / s;
		Q[1] = (R[7] - R[5]) * s;
		Q[2] = (R[2] - R[6]) * s;
		Q[3] = (R[3] - R[1]) * s;
	}
	else{
		if(R[0] > R[4] && R[0] > R[8] ){
			s = 0.5f * FastSqrtI(1.0f + R[0] - R[4] - R[8]);
			Q[0] = (R[7] - R[5]) * s;
			Q[1] = 0.25f / s;
			Q[2] = (R[1] + R[3]) * s;
			Q[3] = (R[2] + R[6]) * s;
		}
		else if(R[4] > R[8]) {
			s = 0.5f * FastSqrtI(1.0f + R[4] - R[0] - R[8]);
			Q[0] = (R[2] - R[6]) * s;
			Q[1] = (R[1] + R[3]) * s;
			Q[2] = 0.25f / s;
			Q[3] = (R[5] + R[7]) * s;
		}
		else{
			s = 0.5f * FastSqrtI(1.0f + R[8] - R[0] - R[4]);
			Q[0] = (R[3] - R[1]) * s;
			Q[1] = (R[2] + R[6]) * s;
			Q[2] = (R[5] + R[7]) * s;
			Q[3] = 0.25f / s;
		}
	}
#else
	// get the instantaneous orientation quaternion
	float fq0sq; // q0^2
	float recip4q0; // 1/4q0
	float fmag; // quaternion magnitude
#define SMALLQ0 0.01F // limit where rounding errors may appear
	// get q0^2 and q0
	fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
	Q[0] = (float)FastSqrt(FastAbs(fq0sq));
	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (Q[0] > SMALLQ0){
		// calculate q1 to q3
		recip4q0 = 0.25F / Q[0];
		Q[1] = recip4q0 * (R[5] - R[7]);
		Q[2] = recip4q0 * (R[6] - R[2]);
		Q[3] = recip4q0 * (R[1] - R[3]);
	}
	else{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		Q[1] = FastSqrt(FastAbs(0.5f * (1.0f + R[0]) - fq0sq));
		Q[2] = FastSqrt(FastAbs(0.5f * (1.0f + R[4]) - fq0sq));
		Q[3] = FastSqrt(FastAbs(0.5f * (1.0f + R[8]) - fq0sq));
		// first assume q1 is positive and ensure q2 and q3 are consistent with q1
		if ((R[1] + R[3]) < 0.0f){
			// q1*q2 < 0 so q2 is negative
			Q[2] = -Q[2];
			if ((R[5] + R[7]) > 0.0f){
				// q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
				Q[3] = -Q[3];
			}
		}
		else if ((R[1] + R[3]) > 0.0f){
			if ((R[5] + R[7]) < 0.0f){
				// q1*q2 > 0 and q2*q3 < 0 so q3 is negative
				Q[3] = -Q[3];
			}
		}
		// negate the vector components if q1 should be negative
		if ((R[5] - R[7]) < 0.0f){
			Q[1] = -Q[1];
			Q[2] = -Q[2];
			Q[3] = -Q[3];
		}
	}
	// finally re-normalize
	fmag = FastSqrtI(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= fmag;
	Q[1] *= fmag;
	Q[2] *= fmag;
	Q[3] *= fmag;
#endif
}

void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize)
{
	float half = 0.5f;
	float two = 2.0f;
	float qw[4], k2[4], k3[4], k4[4];
	float tmpq[4], tmpk[4];

	//qw = q * w * half;
	Quaternion_Multiply(qw, q, w);
	Quaternion_Scalar(qw, qw, half);
	//k2 = (q + qw * dt * half) * w * half;
	Quaternion_Scalar(tmpk, qw, dt * half);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k2, tmpk, w);
	Quaternion_Scalar(k2, k2, half);
	//k3 = (q + k2 * dt * half) * w * half;
	Quaternion_Scalar(tmpk, k2, dt * half);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k3, tmpk, w);
	Quaternion_Scalar(k3, k3, half);
	//k4 = (q + k3 * dt) * w * half;
	Quaternion_Scalar(tmpk, k3, dt);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k4, tmpk, w);
	Quaternion_Scalar(k4, k4, half);
	//q += (qw + k2 * two + k3 * two + k4) * (dt / 6);
	Quaternion_Scalar(tmpk, k2, two);
	Quaternion_Add(tmpq, qw, tmpk);
	Quaternion_Scalar(tmpk, k3, two);
	Quaternion_Add(tmpq, tmpq, tmpk);
	Quaternion_Add(tmpq, tmpq, k4);
	Quaternion_Scalar(tmpq, tmpq, dt / 6.0f);
	Quaternion_Add(q, q, tmpq);

	if (normalize){
		Quaternion_Normalize(q);
	}
}
#define EKF_HALFPI 1.5707963267948966192313216916398f
#define EKF_PI 3.1415926535897932384626433832795f
#define EKF_TWOPI 6.283185307179586476925286766559f
#define EKF_TODEG(x) ((x) * 57.2957796f)
void Quaternion_GetAngle(float *q, float* rpy)
{
    float CBn[9];
    float q0q0 = q[0] * q[0];

    //x-y-z
    CBn[0] = 2.0f * (q0q0 + q[1] * q[1]) - 1.0f;
    CBn[1] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    CBn[2] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    //CBn[3] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
    //CBn[4] = 2.0f * (q0q0 + q[2] * q[2]) - 1.0f;
    CBn[5] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    //CBn[6] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
    //CBn[7] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
    CBn[8] = 2.0f * (q0q0 + q[3] * q[3]) - 1.0f;

    //roll
    rpy[0] = FastAtan2(CBn[5], CBn[8]);
    if (rpy[0] == EKF_PI)
        rpy[0] = -EKF_PI;
    //pitch
    if (CBn[2] >= 1.0f)
        rpy[1] = -EKF_HALFPI;
    else if (CBn[2] <= -1.0f)
        rpy[1] = EKF_HALFPI;
    else
        rpy[1] = FastAsin(-CBn[2]);
    //yaw
    rpy[2] = FastAtan2(CBn[1], CBn[0]);
    if (rpy[2] < 0.0f){
        rpy[2] += EKF_TWOPI;
    }
    if (rpy[2] >= EKF_TWOPI){
        rpy[2] = 0.0f;
    }

    rpy[0] = EKF_TODEG(rpy[0]);
    rpy[1] = EKF_TODEG(rpy[1]);
    rpy[2] = EKF_TODEG(rpy[2]);
}

inline float Quaternion_radiansToDegrees (float radians)
{
    return 57.2957795130823f * radians;
}

#if 1
void Quaternion_getEulerAngles(float *q, float* rpy)
{
    rpy[0] = Quaternion_radiansToDegrees(FastAtan2(2.0f * (q[2] * q[3] - q[0] * q[1]), 2.0f * q[0] * q[0] - 1.0f + 2.0f * q[3] * q[3]));
    //rpy[1] = Quaternion_radiansToDegrees(-atan((2.0f * (q[1] * q[3] + q[0] * q[2])) / FastSqrt(1.0f - pow((2.0f * q[1] * q[3] + 2.0f * q[0] * q[2]), 2.0f))));
    rpy[1] = Quaternion_radiansToDegrees(FastAsin(-2.0f * (q[1] * q[3] - q[0] * q[2])));
    rpy[2] = Quaternion_radiansToDegrees(FastAtan2(2.0f * (q[1] * q[2] - q[0] * q[3]), 2.0f * q[0] * q[0] - 1.0f + 2.0f * q[1] * q[1]));
	rpy[2] *= 10;//??
}
#else
void Quaternion_getEulerAngles(float *q, float* rpy)
{
    double test = q[0]*q[1] + q[2]*q[3];
    if (test > 0.4959) { // singularity at north pole
        rpy[1] = 2 * atan2(q[0],q[3]);
        rpy[0] = EKF_HALFPI;
        rpy[2] = 0;
        rpy[0] = EKF_TODEG(rpy[0]);
        rpy[1] = EKF_TODEG(rpy[1]);
        rpy[2] = EKF_TODEG(rpy[2]);        return;
    }
    if (test < -0.459) { // singularity at south pole
        rpy[1] = -2 * atan2(q[0],q[3]);
        rpy[0] = - EKF_HALFPI;
        rpy[2] = 0;
        rpy[0] = EKF_TODEG(rpy[0]);
        rpy[1] = EKF_TODEG(rpy[1]);
        rpy[2] = EKF_TODEG(rpy[2]);        return;
    }
    double sqx = q[0]*q[0];
    double sqy = q[1]*q[1];
    double sqz = q[2]*q[2];
    rpy[1] = atan2(2*q[2]*q[3]-2*q[0]*q[2] , 1 - 2*sqy - 2*sqz);
    rpy[0] = asin(2*test);
    rpy[2] = atan2(2*q[0]*q[3]-2*q[1]*q[2] , 1 - 2*sqx - 2*sqz);

    rpy[0] = EKF_TODEG(rpy[0]);
    rpy[1] = EKF_TODEG(rpy[1]);
    rpy[2] = EKF_TODEG(rpy[2]);

    float CBn[9];
    float q0q0 = q[0] * q[0];

//    //x-y-z
//    CBn[0] = 2.0f * (q0q0 + q[1] * q[1]) - 1.0f;
//    CBn[1] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
//    CBn[2] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
//    CBn[5] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
//    CBn[8] = 2.0f * (q0q0 + q[3] * q[3]) - 1.0f;

//    //pitch
//    if (CBn[2] >= 1.0f)
//    {
//        rpy[0] = 0;
//        rpy[1] = -EKF_HALFPI;
//        rpy[2] = -2 * atan2(q[0],q[3]);
//        return;
//    }
//    else if (CBn[2] <= -1.0f)
//    {
//        rpy[0] = 0;
//        rpy[1] = EKF_HALFPI;
//        rpy[2] = -2 * atan2(q[0],q[3]);
//        return;
//    }
//    else
//        rpy[1] = FastAsin(-CBn[2]);

//    rpy[0] = Quaternion_radiansToDegrees(FastAtan2(2.0f * (q[2] * q[3] - q[0] * q[1]), 2.0f * q[0] * q[0] - 1.0f + 2.0f * q[3] * q[3]));
//    rpy[1] = Quaternion_radiansToDegrees(-atan((2.0f * (q[1] * q[3] + q[0] * q[2])) / sqrt(1.0f - pow((2.0f * q[1] * q[3] + 2.0f * q[0] * q[2]), 2.0f))));
//    rpy[2] = Quaternion_radiansToDegrees(FastAtan2(2.0f * (q[1] * q[2] - q[0] * q[3]), 2.0f * q[0] * q[0] - 1.0f + 2.0f * q[1] * q[1]));
}
#endif
