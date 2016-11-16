#pragma once

#define PI 3.14159

#define SIGNAL_PATH_REGISTER 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT 0x6B

#define CALIBRATION_RATE 10
#define GYRO_DEG_PER_SEC 131
#define FILTER_GAIN 0.93

enum XYZ { X, Y, Z };
enum ACC_GYR { ACCEL, GYRO };

struct GyroInfo
{
	float standard[2][3];
	float raw[2][3];
	float value[2][3];
	float degree[3];
};

class Gyro
{
public:
	GyroInfo data;
	long gyr_dt, pre_gyr_dt;

	void initGyro();
	void CalibrationGyro();
	void GetDegree();
	void ReadAccel();
	void ReadGyro();
	void Filtering();
}; 
