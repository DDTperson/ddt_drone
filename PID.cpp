#include"PID.h"

void PID::initPID(float std)
{
	std_angle = std;
	angle = 0;
	rate = 0;
	error = 0;
	i_controll.sum_error = 0;
	i_controll.sum_rate = 0;
	d_controll.pre_error = 0;
	i_controll.dt = 0;
	i_controll.start_dt = 0;
	d_controll.dt = 0;
	d_controll.start_dt = 0;
}

void PID::upDate(float new_angle, float new_rate)
{
	angle = new_angle;
	rate = new_rate;
	error = std_angle - angle;
  if(error > -0.5 && error < 0.5)
    error = 0;
	p_controll.error = error;
	i_controll.error = error;
	d_controll.error = error;
}

float PID::OutPut(int controll_case)
{
	switch (controll_case)
	{
	case aP:
		return p_controll.OutPut();
	case aI:
		return i_controll.OutPut(0);
	case aD:
		return d_controll.OutPut(0);
	case aPI:
		return p_controll.OutPut() + i_controll.OutPut(0);
	case aPD:
		return p_controll.OutPut() + d_controll.OutPut(0);
	case rI:
		return i_controll.OutPut(rate);
	case rD:
		return d_controll.OutPut(rate);
	case rPI:
		return p_controll.OutPut() + i_controll.OutPut(rate);
	case rPD:
		return p_controll.OutPut() + d_controll.OutPut(rate);
	}
}

float P::OutPut()
{
	return error*P_GAIN;
}

float I::OutPut(float n_rate)
{
	sum_error += error*dt;
	sum_rate += n_rate*dt;

	if (!n_rate)
		return sum_error*I_GAIN;
	else
		return sum_rate*I_GAIN;
}

float D::OutPut(float n_rate)
{
	if (!n_rate)
		return ((error - pre_error) / dt)*D_GAIN;
	else
		return n_rate*D_GAIN;

	pre_error = error;
}
