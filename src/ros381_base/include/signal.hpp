double correct_param(double param, double error, double eta, double min, double max);
unsigned char stacked(double time_limit, double v, double v_min, double freq, unsigned *cnt);
double speedup_synthesis_7(double distance, double velocity, double acceleration, double J_MAX, double v_max,
                           double v_min, double dt, double percentage);
double slowing_synthesis_7(double distance, double velocity, double acceleration, double J_MAX, double v_max,
                           double v_min, double dt, double percentage);
double synthesis_7(double distance, double velocity, double acceleration, double J_MAX, double stopping_distance,
                   double v_max, double v_min, double dt);
double wrap(double signal, double min, double max);
void wrap_ptr(double *signal, double max, double min);
short get_sign(double num);
double scale_vel_ref(volatile double *ref_1, volatile double *ref_2, double limit);
double abs_max(double a, double b);
double abs_min(double a, double b);
unsigned long unsigned_min(unsigned long a, unsigned long b);
void vel_ramp_up_ptr(double *signal, double reference, double acc);
double vel_ramp_up(double signal, double reference, double acc_max);
double vel_s_curve_up_webots(double *vel, double prev_vel, double vel_ref, double jerk_slope);
double vel_s_curve_up(double vel, double accel, double vel_ref, double jerk);
double min3(double a, double b, double c);
double snap_angle(double angle, double step);
double snap_ortho_deg(double phi);