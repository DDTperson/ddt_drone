#include<Wire.h>
#include<math.h>
#include"Gyro.h"

void Gyro::initGyro()
{
	Wire.begin();
	Wire.beginTransmission(SIGNAL_PATH_REGISTER);
	Wire.write(PWR_MGMT);
	Wire.write(0);
	Wire.endTransmission(true);

	for (int species = ACCEL; species < GYRO; species++)
	{
		for (int shaft = X; shaft < Z; shaft++)
		{
			data.standard[species][shaft] = 0;
			data.raw[species][shaft] = 0;
			data.degree[shaft] = 0;
			data.value[species][shaft] = 0;
		}
	}
}

void Gyro::CalibrationGyro()
{
	double cali_sum[2][3] = { 0 };

	for (int i = 0; i < CALIBRATION_RATE; i++)
	{
		ReadAccel();
		ReadGyro();
		for (int species = ACCEL; species < GYRO; species++)
		{
			for (int shaft = X; shaft < Z; shaft++)
			{
				cali_sum[species][shaft] += data.raw[species][shaft];
			}
		}
	}

	for (int species = ACCEL; species < GYRO; species++)
	{
		for (int shaft = X; shaft < Z; shaft++)
		{
			data.standard[species][shaft] = cali_sum[species][shaft] / CALIBRATION_RATE;
		}
	}
}

void Gyro::ReadAccel()
{
	Wire.begin();
	Wire.beginTransmission(SIGNAL_PATH_REGISTER);
	Wire.write(ACCEL_XOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom(SIGNAL_PATH_REGISTER, 6, true);
	for(int shaft = X; shaft < Z; shaft ++)
		data.raw[ACCEL][shaft] = Wire.read() << 8 | Wire.read();
}

void Gyro::ReadGyro()
{
	Wire.begin();
	Wire.beginTransmission(SIGNAL_PATH_REGISTER);
	Wire.write(GYRO_XOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom(SIGNAL_PATH_REGISTER, 6, true);
	for (int shaft = X; shaft < Z; shaft++)
		data.raw[GYRO][shaft] = Wire.read() << 8 | Wire.read();
}

void Gyro::GetDegree()
{
	ReadAccel();

	data.value[ACCEL][X] = data.raw[ACCEL][X] -data.standard[ACCEL][X];
	data.value[ACCEL][Y] = data.raw[ACCEL][Y] - data.standard[ACCEL][Y];
	data.value[ACCEL][Z] = data.raw[ACCEL][Z] + (16384 - data.standard[ACCEL][Z]);

	data.value[ACCEL][X] = atan2(data.value[ACCEL][Z], data.value[ACCEL][X]) * 180 / PI;
	data.value[ACCEL][Y] = atan2(data.value[ACCEL][Z], data.value[ACCEL][Y]) * 180 / PI;

	//gyr_dt = (millis() - pre_gyr_dt) / 1000;
	ReadGyro();
	//pre_gyr_dt = millis();

	data.value[GYRO][X] += (data.raw[GYRO][X] - data.standard[GYRO][X]) / GYRO_DEG_PER_SEC * gyr_dt;
	data.value[GYRO][Y] += (data.raw[GYRO][Y] - data.standard[GYRO][Y]) / GYRO_DEG_PER_SEC * gyr_dt;
	data.value[GYRO][Z] += (data.raw[GYRO][Z] - data.standard[GYRO][Z]) / GYRO_DEG_PER_SEC * gyr_dt;

	Filtering();
}

void Gyro::Filtering()
{
	double tempX, tempY, tempZ;

	tempX = data.degree[X] + (data.raw[GYRO][X] - data.standard[GYRO][X]) / GYRO_DEG_PER_SEC * gyr_dt;
	tempY = data.degree[Y] + (data.raw[GYRO][Y] - data.standard[GYRO][Y]) / GYRO_DEG_PER_SEC * gyr_dt;
	tempZ = data.degree[Z] + (data.raw[GYRO][Z] - data.standard[GYRO][Z]) / GYRO_DEG_PER_SEC * gyr_dt;

	data.degree[X] = FILTER_GAIN*tempX + (1 - FILTER_GAIN)*data.value[ACCEL][X];
	data.degree[Y] = FILTER_GAIN*tempY + (1 - FILTER_GAIN)*data.value[ACCEL][Y];
	data.degree[Z] = tempZ;
}
