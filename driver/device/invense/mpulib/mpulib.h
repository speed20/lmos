////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpulib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef MPULIB_H
#define MPULIB_H

#include "quaternion.h"

#define MAG_SENSOR_RANGE 	4096
#define ACCEL_SENSOR_RANGE 	32000

typedef struct {
	short offset[3];
	short range[3];
} caldata_t;

typedef struct {
	short rawGyro[3];
	short rawAccel[3];
	long rawQuat[4];
	unsigned long dmpTimestamp;

	short rawMag[3];
	unsigned long magTimestamp;

	short calibratedAccel[3];
	short calibratedMag[3];

	quaternion_t fusedQuat;
	vector3d_t fusedEuler;

	float lastDMPYaw;
	float lastYaw;
} mpudata_t;


void mpulib_set_debug(int on);
int mpulib_init(int sample_rate, int yaw_mixing_factor);
int mpulib_exit();
int mpulib_read(mpudata_t *mpu);
int mpulib_read_dmp(mpudata_t *mpu);
int mpulib_read_mag(mpudata_t *mpu);
void mpulib_set_accel_cal(caldata_t *cal);
void mpulib_set_mag_cal(caldata_t *cal);
void set_calibration(int mag);

#endif /* MPULIB_H */

