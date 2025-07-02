/*
 * signal.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include <cmath>
#include "../include/signal.hpp"

void
wrap180_ptr (volatile double *signal)
{
	if (*signal > 180)
		*signal -= 360;
	if (*signal < -180)
		*signal += 360;
}

void
wrapPi_ptr (volatile double *signal)
{
	if (*signal > M_PI)
		*signal -= M_PI;
	if (*signal < -M_PI)
		*signal += M_PI;
}

double
wrap180 (double signal)
{
	if (signal > 180)
		return signal - 360;
	if (signal < -180)
		return signal + 360;
	return signal;
}

short
get_sign (double num)
{
	if (num > 0)
		return 1;
	if (num < 0)
		return -1;
	return 0;
}

void
saturation (volatile double *signal, double max, double min)
{
	if (*signal > max)
		*signal = max;
	else if (*signal < min)
		*signal = min;
}

void
scale_vel_ref (volatile double *ref_1, volatile double *ref_2, double limit)
{
	double abs_max_var = abs_max (*ref_1, *ref_2);
	if (abs_max_var > limit)
		{
			double factor = limit / abs_max_var;
			*ref_1 *= factor;
			*ref_2 *= factor;
		}
}

double
abs_max (double a, double b)
{
	if (fabs (a) > fabs (b))
		return fabs (a);
	return fabs (b);
}

double
abs_min (double a, double b)
{
	if (fabs (a) < fabs (b))
		return fabs (a);
	return fabs (b);
}

unsigned long
unsigned_min (unsigned long a, unsigned long b)
{
	if (a < b)
		return a;
	return b;
}

void
vel_ramp_up_ptr (double *signal, double reference, double acc)
{
	if (fabs (reference) - fabs (*signal) > acc)
		*signal += get_sign (reference) * acc;
	else
		*signal = reference;
}

double
vel_ramp_up (double signal, double reference, double acc)
{
	double edited_ref = signal;
	if (fabs (reference) - fabs (signal) > acc)
		edited_ref += get_sign (reference) * acc;
	else
		edited_ref = reference;
	return edited_ref;
}

double
vel_s_curve_up_webots (double *vel, double prev_vel, double vel_ref, double jerk)
{
	double acc_approx = *vel - prev_vel;
	double acc_calc = vel_ref - *vel;
	double out = *vel;

	if (fabs (vel_ref) > fabs (*vel))
		{
			if (fabs (acc_calc) - fabs (acc_approx) > jerk)
				out = *vel + acc_approx + get_sign (vel_ref) * jerk;
			else
				out = vel_ref;
		}
	return out;
}

double
vel_s_curve_up (double vel, double accel, double vel_ref, double jerk)
{
	double accel_des = vel_ref - vel;
	double edited_ref = vel_ref;

	if (fabs (vel_ref) > fabs (vel))	// ako ubrzava
		{
			if (fabs (accel_des) - fabs (accel) > jerk)
				{
					edited_ref = vel + accel + get_sign (vel_ref) * jerk;
				}
		}
	return edited_ref;
}

double
min3 (double a, double b, double c)
{
	double min = a;
	if (b < min)
		min = b;
	if (c < min)
		min = c;
	return min;
}

double
snap_phi (double phi)
{
	return roundf ((phi + 180) / 90.0f) * 90.0f - 180;
}