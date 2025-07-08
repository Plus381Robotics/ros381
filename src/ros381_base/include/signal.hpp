double
wrap(double signal, double max, double min);
void
wrap_ptr (double *signal, double max, double min);
short
get_sign(double num);
void
scale_vel_ref(volatile double *ref_1, volatile double *ref_2, double limit);
double
abs_max(double a, double b);
double
abs_min(double a, double b);
unsigned long
unsigned_min (unsigned long a, unsigned long b);
void
vel_ramp_up_ptr(double *signal, double reference, double acc);
double
vel_ramp_up(double signal, double reference, double acc_max);
double
vel_s_curve_up_webots(double *vel, double prev_vel, double vel_ref,
		double jerk_slope);
double
vel_s_curve_up(double vel, double accel, double vel_ref, double jerk);
double
min3(double a, double b, double c);
double
snap_phi(double phi);