#pragma once

#include "app_context.h"

/* --- SENSOR NOISE STANDARD DEVIATIONS (σ) ---------------------------------*/
#define NOISE_BARO_M        33.2f     // [m]
#define NOISE_ACC_MPS2      0.01f     // [m/s^2]
#define NOISE_QUAT          0.02f     // [-]
#define NOISE_RATE_RADPS    0.005f    // [rad/s]

/* --- EKF PROCESS NOISE PARAMETERS (Q diagonal elements) --------------------*/ 
#define EKF_Q_POS_M2       1.0f      // [m^2]
#define EKF_Q_VEL_M2PS2    0.5f      // [(m/s)^2]
#define EKF_Q_ACC_M2PS4    1.0f      // [(m/s^2)^2]
#define EKF_Q_QUAT         1e-6f     // [-]
#define EKF_Q_RATE_RAD2    1e-4f     // [(rad/s)^2]

#define NX 16
#define NZ 11
extern const float EKF_Q[NX][NX];
extern const float EKF_R[NZ][NZ];

/**
 * @brief EKF state initialization. Must be called before the first ekf_step.
 * @param x   Initial state estimate (16).
 * @param P   Initial covariance (16×16).
 */
void ekf_init(float x[NX], float P[NX][NX]);

/**
 * @brief Perform one EKF step: state prediction + measurement update.
 * @param ctx   Application context.
 * @param meas  Measurement vector (11).
 * @param dt    Time step [s].
 */
void ekf_processing(app_context_t *ctx, const float meas[NZ], float dt);