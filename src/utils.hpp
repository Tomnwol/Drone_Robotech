#ifndef __UTILS__H
#define __UTILS__H
#include "types.hpp"

float passe_bas(float value, float last_filtered_value, float dt, float w0);
void fill_tab(float * tab, int size_tab);
float mean_tab(float * tab, int size_tab);
void fill_queue(float * tab, int size_tab, float valeur);
Vector3f up_vector(float q0, float q1, float q2, float q3);

//Quaternions
Quaternion quat_conjugate(Quaternion q);
Quaternion quat_multiply(Quaternion q1, Quaternion q2);
Vector3f quat_to_attitude_error(Quaternion q_err);
Vector3f vec_scale(Vector3f v, float s);
Vector3f vec_sub(Vector3f v1, Vector3f v2);
int int_clamp(int value, int min, int max);
#endif