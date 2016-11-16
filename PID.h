#pragma once

#define MAX_OUTPUT 90
#define MIN_OUTPUT -90

#define P_GAIN 0.35
#define I_GAIN 0.1
#define D_GAIN 0.15

enum {
	aP, aI, aD, aPI, aPD,
	rI, rD, rPI, rPD
};

class P
{
public:
	float error;

	float OutPut();
};

class I
{
public:
	float error;
	float sum_error;
	float sum_rate;
	float dt, start_dt;

	float OutPut(float rate);
};

class D
{
public:
	float error;
	float pre_error;
	float dt, start_dt;

	float OutPut(float rate);
};

class PID
{
public:
	float std_angle;
	float angle, rate;
	float error;

	P p_controll;
	I i_controll;
	D d_controll;

	void initPID(float);
	void upDate(float, float);
	float OutPut(int);
};
