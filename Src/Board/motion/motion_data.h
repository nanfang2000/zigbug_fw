#ifndef __MOTION_DATA_H__
#define __MOTION_DATA_H__
#include <stdint.h>
#include "vector3.h"
#ifdef  __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif

    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

typedef PACKED( struct
{
    vector3_t   accel;
    vector3_t    gyro;
    vector3_t compass;
    float temp;
    float timestamp;
}) motion_raw_t;

typedef PACKED( struct
{
    uint8_t dir;
    uint8_t cnt;
}) motion_tap_t;

typedef uint8_t motion_orientation_t;

typedef PACKED( struct
{
    float w;
    float x;
    float y;
    float z;
}) motion_quat_t;

typedef PACKED( struct
{
    float roll;
    float pitch;
    float yaw;
}) motion_euler_t;

typedef PACKED( struct
{
    int16_t matrix[9];
}) motion_rot_mat_t;

typedef int32_t motion_heading_t;

typedef PACKED( struct
{
    float x;
    float y;
    float z;
}) motion_gravity_t;

typedef PACKED( struct
{
    uint32_t steps;
    uint32_t time_ms;
}) motion_pedo_t;

typedef PACKED( struct
{
    //Raw
    vector3_t   accel;
    vector3_t    gyro;
    vector3_t compass;
    float temp;
    float timestamp;
    //Derived virtual sensor data
    vector3_t normAcc;
    vector3_t velocity;
    motion_euler_t euler;
    vector3_t coordinate;
    motion_quat_t quat;
    motion_rot_mat_t rot_matrix;
    int gesture;
}) motion_data_t;

#ifdef  __cplusplus
}  
#endif

#endif // __MOTION_DATA_H__
