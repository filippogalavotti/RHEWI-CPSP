#pragma once

#include <stdbool.h>
#include "ms8607.h"
#include "bno055.h"

/* --- HIL and Sensors sampling frequencies ------------------- */
#define HIL_FREQ_HZ 50.0f
#define SENSORS_SAMPLE_FREQ_HZ 50.0f

/* --- HIL Commands ------------------- */
#define HIL_CMD_START 0x01
#define HIL_CMD_STOP 0x02
#define HIL_CMD_NEW_MEAS 0x03

/* --- Application Modes ------------------- */
#define APP_MODE_HIL 1
#define APP_MODE_REAL 2
#define APP_MODE APP_MODE_REAL

/* --- Sensors data structure ------------------------------------------------- */
typedef struct {
    ms8607_sensor_t barometer;  // Pressure and temperature sensor
    bno055_sensor_t imu;    // IMU sensor (accelerometer, gyroscope, magnetometer)
    float time_s;   // Timestamp of the latest sensor readings in seconds
    bool valid; // Flag indicating whether the latest sensor readings are valid (true if all sensors were read successfully, false if any sensor read failed)
} sensors_t;

extern sensors_t g_sensors;

/* --- Application context --------------------------------------------------- */
typedef struct {
    float ekf_x[16];    // EKF state vector
    float ekf_P[16][16];    // EKF covariance matrix
    float target_enu[3];    // Target position in ENU coordinates (x, y, z)
    float total_distance;   // Total distance to travel to reach the target in meters
    float time_s;   // Timestamp of the latest EKF update in seconds
    struct {
        float cm;   // Pitch moment coefficient command
        float cn;   // Yaw moment coefficient command
        float cant; // Roll moment coefficient command
    } actuators;
    bool running;   // Flag indicating whether the control loop is running (true if the system is actively controlling towards the target, false if it is idle)
} app_context_t;

extern app_context_t g_app;