m1 = 0.2;
m2 = 0.2;
mM = 0.215;
mB = 0.1;

I1 = (1/3) * m1 * (l1/2)^2;
I2 = (1/3) * m2 * (l2/2)^2;

l1 = 0.2;
l2 = 0.2;

k_rubber = 0.3;

y0 = 0.5;
g = 9.81;

% using the suvat equations to determine heights, velocities and dt
y_dot_impact = sqrt(2*9.81*y0);
y_dot_rebound = y_dot_impact * k_rubber;
time_to_peak = y_dot_rebound/g;
omega = 2*pi / (0.5*time_to_peak);
accel = omega^2 * l1/2;
% conservation of angular momentum
torque = I1*accel
