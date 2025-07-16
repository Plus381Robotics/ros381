/*
 * signal.c
 *
 *  Created on: Nov 9, 2024
 *      Author: lazar
 */

#include "../include/signal.hpp"
#include <algorithm>
#include <cmath>

unsigned char stacked(double time_limit, double v, double v_min, double freq, unsigned *cnt)
{
    if (fabs(v) < v_min * 0.5)
        (*cnt)++;
    else
        *cnt = 0;
    if (*cnt / freq >= time_limit)
        return 1;
    return 0;
}

double synthesis_7(double distance, double velocity, double acceleration, double J_MAX, double stopping_distance,
                   double v_max, double v_min, double dt)
{
    double abs_distance = fabs(distance);
    double abs_velocity = fabs(velocity);
    double abs_acceleration = fabs(acceleration);
    double v_ref = 0;
    if (dt <= 0 || std::isnan(dt))
        return 0.0;

    if (abs_distance <= stopping_distance)
    {
        double x = abs_distance / stopping_distance;
        v_ref = v_max * (35.0f * pow(x, 4) - 84.0f * pow(x, 5) + 70.0f * pow(x, 6) - 20.0f * pow(x, 7));
    }
    else
    {
        double j_step = J_MAX * dt;
        if (abs_velocity < v_max * 0.5f)
            v_ref = abs_velocity + (abs_acceleration + j_step) * dt;
        else if (j_step < abs_acceleration * 1.05)
            v_ref = abs_velocity + (abs_acceleration - j_step) * dt;
        else
            v_ref = v_max;
    }
    v_ref = std::clamp(v_ref, v_min, v_max);

    return std::clamp(get_sign(distance) * v_ref, -v_max, v_max);
}

double wrap(double signal, double min, double max)
{
    double temp = signal;
    wrap_ptr(&temp, min, max);
    return temp;
}

void wrap_ptr(double *signal, double min, double max)
{
    double diff = max - min;
    while (*signal > max)
        *signal -= diff;
    while (*signal < min)
        *signal += diff;
}

short get_sign(double num)
{
    if (num > 0)
        return 1;
    if (num < 0)
        return -1;
    return 0;
}

void scale_vel_ref(volatile double *ref_1, volatile double *ref_2, double limit)
{
    double abs_max_var = abs_max(*ref_1, *ref_2);
    if (abs_max_var > limit)
    {
        double factor = limit / abs_max_var;
        *ref_1 *= factor;
        *ref_2 *= factor;
    }
}

double abs_max(double a, double b)
{
    if (fabs(a) > fabs(b))
        return fabs(a);
    return fabs(b);
}

double abs_min(double a, double b)
{
    if (fabs(a) < fabs(b))
        return fabs(a);
    return fabs(b);
}

unsigned long unsigned_min(unsigned long a, unsigned long b)
{
    if (a < b)
        return a;
    return b;
}

void vel_ramp_up_ptr(double *signal, double reference, double acc)
{
    if (fabs(reference) - fabs(*signal) > acc)
        *signal += get_sign(reference) * acc;
    else
        *signal = reference;
}

double vel_ramp_up(double signal, double reference, double acc)
{
    double edited_ref = signal;
    if (fabs(reference) - fabs(signal) > acc)
        edited_ref += get_sign(reference) * acc;
    else
        edited_ref = reference;
    return edited_ref;
}

double vel_s_curve_up_webots(double *vel, double prev_vel, double vel_ref, double jerk)
{
    double acc_approx = *vel - prev_vel;
    double acc_calc = vel_ref - *vel;
    double out = *vel;

    if (fabs(vel_ref) > fabs(*vel))
    {
        if (fabs(acc_calc) - fabs(acc_approx) > jerk)
            out = *vel + acc_approx + get_sign(vel_ref) * jerk;
        else
            out = vel_ref;
    }
    return out;
}

double vel_s_curve_up(double vel, double accel, double vel_ref, double jerk)
{
    double accel_des = vel_ref - vel;
    double edited_ref = vel_ref;

    if (fabs(vel_ref) > fabs(vel)) // ako ubrzava
    {
        if (fabs(accel_des) - fabs(accel) > jerk)
        {
            edited_ref = vel + accel + get_sign(vel_ref) * jerk;
        }
    }
    return edited_ref;
}

double min3(double a, double b, double c)
{
    double min = a;
    if (b < min)
        min = b;
    if (c < min)
        min = c;
    return min;
}

double snap_angle(double angle, double step)
{
    return roundf((angle + 2 * step) / step) * step - 2 * step;
}

double snap_ortho_deg(double phi)
{
    return roundf((phi + 180) / 90.0f) * 90.0f - 180;
}