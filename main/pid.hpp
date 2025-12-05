#ifndef __PID__H
#define __PID__H

Vector3f get_angular_rate_command(Euler att, Euler att_desired);
Vector3f get_torque(Vector3f omega_set, Vector3f omega_gyro, float dt);
#endif