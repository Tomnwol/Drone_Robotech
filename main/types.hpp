#ifndef __TYPES_H
#define __TYPES_H

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

struct Euler {
  float roll;
  float pitch;
  float yaw;
};

#endif