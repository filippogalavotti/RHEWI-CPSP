#include "app_sensors.h"
#include "app_context.h"
#include "bsp.h"
#include "ekf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include "math.h"

static const char *TAG = "app_sensors";

/** 
 * @brief Unwrap an angle to the range [-π, π] 
 * @param prev Previous angle value
 * @param curr Current angle value
 * @return Unwrapped angle value
 */
static float unwrap(float prev, float curr) {
    float diff = curr - prev;
    if (diff >  M_PI) curr -= 2.0f * M_PI;
    if (diff < -M_PI) curr += 2.0f * M_PI;
    return curr;
}

/**
 * @brief Helper function to convert a quaternion to roll, pitch, and yaw angles. It takes the components of the quaternion as input and computes the corresponding roll, pitch, and yaw angles in radians using standard mathematical formulas for quaternion to Euler angle conversion.
 * @param q0, q1, q2, q3  Quaternion components.
 * @param roll  Pointer to store the computed roll angle [rad].
 * @param pitch Pointer to store the computed pitch angle [rad].
 * @param yaw   Pointer to store the computed yaw angle [rad].
 */
static inline void quat_to_rpy(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw)
{
    static float last_roll = 0, last_pitch = 0, last_yaw = 0;

    float r = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    float p = asinf (2.0f*(q0*q2 - q3*q1));
    float y = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));

    *roll  = unwrap(last_roll,  r);
    *pitch = unwrap(last_pitch, p);
    *yaw   = unwrap(last_yaw,   y);

    last_roll = *roll; last_pitch = *pitch; last_yaw = *yaw;
}

/**
 * @brief Print RAW sensor data and EKF state in Teleplot format
 *
 * Visualization is focused on physically meaningful quantities:
 * - altitude
 * - roll, pitch, yaw
 *
 * Quaternions are used internally but not plotted.
 */
static inline void print_Teleplot_format(app_context_t ctx, sensors_t sensors)
{
    /* --- RAW SENSORS ----------------------------------------------------- */
    /* Barometer altitude */
    printf(">BARO_z:%.2f|m|\n", sensors.barometer.altitude_m);

    /* RAW attitude (from IMU quaternion) */
    float raw_roll, raw_pitch, raw_yaw;
    quat_to_rpy(sensors.imu.latest_sample.quat[0], sensors.imu.latest_sample.quat[1], sensors.imu.latest_sample.quat[2], sensors.imu.latest_sample.quat[3], &raw_roll, &raw_pitch, &raw_yaw);
    printf(">RAW_roll:%.4f|rad|\n",  raw_roll);
    printf(">RAW_pitch:%.4f|rad|\n", raw_pitch);
    printf(">RAW_yaw:%.4f|rad|\n",   raw_yaw);

    /* --- EKF STATE ----------------------------------------------------- */
    /* EKF altitude */
    printf(">EKF_z:%.2f|m|\n", ctx.ekf_x[2]);

    /* EKF attitude (from EKF quaternion) */
    float ekf_roll, ekf_pitch, ekf_yaw;
    quat_to_rpy(ctx.ekf_x[9], ctx.ekf_x[10], ctx.ekf_x[11], ctx.ekf_x[12], &ekf_roll, &ekf_pitch, &ekf_yaw);
    printf(">EKF_roll:%.4f|rad|\n",  ekf_roll);
    printf(">EKF_pitch:%.4f|rad|\n", ekf_pitch);
    printf(">EKF_yaw:%.4f|rad|\n",   ekf_yaw);
}

/**
 * @brief Helper function to read sensor values and fill the measurement array for EKF processing. It reads the latest values from the barometer and IMU sensors, computes the altitude from the pressure reading, and fills the measurement array with the relevant sensor data. The function returns true if all sensor readings were successful and valid, or false if any sensor read failed.
 * @param sensors The sensors_t structure containing the latest sensor readings to be read and processed
 * @param meas The measurement array to be filled with sensor data
 * @return true if all sensor readings were successful and valid, false otherwise
 */
static inline bool get_measure(sensors_t *sensors, float *meas)
{   
    bool ok = true;

    /* Barometer */
    if (ms8607_get_pt_values(&sensors->barometer) == ESP_OK) {
        sensors->barometer.altitude_m = ms8607_compute_altitude(sensors->barometer);
    } else {
        ok = false;
        return ok;
    }

    /* IMU */
    if (bno055_get_agq_values(sensors->imu.imu_handle,&sensors->imu.latest_sample,&sensors->imu.status) != ESP_OK) {
        ok = false;
        return ok;
    }

    meas[0] = sensors->barometer.altitude_m;
    meas[1] = sensors->imu.latest_sample.acc_mps2[0];
    meas[2] = sensors->imu.latest_sample.acc_mps2[1];
    meas[3] = sensors->imu.latest_sample.acc_mps2[2];
    meas[4] = sensors->imu.latest_sample.quat[0];
    meas[5] = sensors->imu.latest_sample.quat[1];
    meas[6] = sensors->imu.latest_sample.quat[2];
    meas[7] = sensors->imu.latest_sample.quat[3];
    meas[8] = sensors->imu.latest_sample.gyro_radps[0];
    meas[9] = sensors->imu.latest_sample.gyro_radps[1];
    meas[10] = sensors->imu.latest_sample.gyro_radps[2];

    sensors->time_s = esp_timer_get_time() * 1e-6f;

    return ok;
}

/* --- Public API ---------------------------------------------------------- */

esp_err_t app_sensors_init(void)
{
    esp_err_t ret;
    i2c_master_bus_handle_t pt_bus, imu_bus;

    ret = bsp_i2c_init(&pt_bus, &imu_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c init failed");
        return ret;
    }

    ret = ms8607_init(&g_sensors.barometer, pt_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ms8607_init failed");
        return ret;
    }

    ret = bno055_init(&g_sensors.imu, imu_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bno055_init failed");
        return ret;
    }

    ekf_init(g_app.ekf_x, g_app.ekf_P);

    ESP_LOGI(TAG, "Sensors initialized");
    return ESP_OK;
}

void app_sensors_loop(app_context_t *ctx, sensors_t *sensors) 
{  
    int dt = (int)(1000.0f / SENSORS_SAMPLE_FREQ_HZ);
    float dt_s = 1.0f / SENSORS_SAMPLE_FREQ_HZ;
    
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(dt); // 50 Hz

    for (;;) {
        xTaskDelayUntil(&last, period);

        float meas[11];
        
        sensors->valid = get_measure(sensors, meas);
        
        ekf_processing(ctx, meas, dt_s);

        print_Teleplot_format(*ctx, *sensors);
    }
}