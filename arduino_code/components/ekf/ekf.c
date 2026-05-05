#include "ekf.h"
#include "math.h"
#include <stdio.h>
#include <stdint.h>

/* --- EKF Process Noise Covariance Q --------------------------------- */
// State: [x y z vx vy vz ax ay az e0 e1 e2 e3 wx wy wz]
const float EKF_Q[NX][NX] = {
    // x   y   z
    {EKF_Q_POS_M2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, EKF_Q_POS_M2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, EKF_Q_POS_M2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    // vx vy vz
    {0, 0, 0, EKF_Q_VEL_M2PS2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, EKF_Q_VEL_M2PS2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, EKF_Q_VEL_M2PS2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    // ax ay az
    {0, 0, 0, 0, 0, 0, EKF_Q_ACC_M2PS4, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, EKF_Q_ACC_M2PS4, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_ACC_M2PS4, 0, 0, 0, 0, 0, 0, 0},

    // e0 e1 e2 e3
    {0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_QUAT, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_QUAT, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_QUAT, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_QUAT, 0, 0, 0},

    // wx wy wz
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_RATE_RAD2, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_RATE_RAD2, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, EKF_Q_RATE_RAD2}
};

/* --- EKF Measurement Noise Covariance R ------------------------------*/
// Measurement: [z ax ay az e0 e1 e2 e3 wx wy wz]
#define VAR_BARO   (NOISE_BARO_M      * NOISE_BARO_M)
#define VAR_ACC    (NOISE_ACC_MPS2    * NOISE_ACC_MPS2)
#define VAR_QUAT   (NOISE_QUAT        * NOISE_QUAT)
#define VAR_RATE   (NOISE_RATE_RADPS  * NOISE_RATE_RADPS)

const float EKF_R[NZ][NZ] = {
    {VAR_BARO, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    {0, VAR_ACC, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, VAR_ACC, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, VAR_ACC, 0, 0, 0, 0, 0, 0, 0},

    {0, 0, 0, 0, VAR_QUAT, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, VAR_QUAT, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, VAR_QUAT, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, VAR_QUAT, 0, 0, 0},

    {0, 0, 0, 0, 0, 0, 0, 0, VAR_RATE, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, VAR_RATE, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, VAR_RATE}
};

/* --- Physical / guidance constants ---------------------------------------- */
#define ROCKET_MASS      22.740f          // kg
#define PITCH_INERTIA    14.304f          // kg·m²
#define ROCKET_RADIUS    0.075f           // m
#define REF_AREA         (3.14159265f * ROCKET_RADIUS * ROCKET_RADIUS)   // m²
#define REF_LENGTH       1.785f           // m

#define N_PN             4.0f             // Proportional Navigation gain
#define CM_CLIP          12.0f            // [-] hard moment coefficient limit
#define INTERCEPT_RADIUS 20.0f            // m

/* --- Roll control ---------------------------------------------------------- */
#define ROLL_SETPOINT    0.0f             // deg
#define ROLL_KP_ANGLE    0.01f            // cant_deg / deg
#define ROLL_KP_RATE     0.01f            // cant_deg / (rad/s)
#define ROLL_KD          0.01f            // cant_deg / (rad/s²)
#define MAX_CANT         5.0f             // deg


/* --- Roll controller state ------------------------------------------------ */
static float prev_spin = 0.0f;
static float prev_time = -1.0f;   // negative sentinel = "not yet initialized"


/**
 * @brief Helper function to normalize a quaternion.
 * @param q0, q1, q2, q3  Pointer to the quaternion components.
 */
static inline void quat_normalize(float *q0, float *q1, float *q2, float *q3)
{
    float n = sqrtf((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));
    if (n > 1e-12f) {
        *q0 /= n; *q1 /= n; *q2 /= n; *q3 /= n;
    }
}

/**
 * @brief Helper function to perform Cholesky decomposition of a 11x11 symmetric positive definite matrix.
 * @param A  Input matrix (11×11).
 * @param L  Output lower triangular matrix (11×11).
 * @return   1 if successful, 0 otherwise.
 */
static inline int cholesky11(const float A[11][11], float L[11][11])
{
    for (int i = 0; i < 11; i++) {
        for (int j = 0; j <= i; j++) {

            float sum = A[i][j];
            for (int k = 0; k < j; k++)
                sum -= L[i][k] * L[j][k];

            if (i == j) {
                if (sum <= 0.0f)
                    return 0; // non SPD → error
                L[i][j] = sqrtf(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }

        for (int j = i+1; j < 11; j++)
            L[i][j] = 0.0f;
    }
    return 1;
}

/**
 * @brief Perform forward substitution: solve L*y = b.
 * @param L  Lower triangular matrix (11×11).
 * @param b  Right-hand side vector (11).
 * @param y  Solution vector (11).
 */
static inline void forward_sub_11(const float L[11][11], const float b[11], float y[11])
{
    for (int i = 0; i < 11; i++) {
        float sum = b[i];
        for (int j = 0; j < i; j++)
            sum -= L[i][j] * y[j];
        y[i] = sum / L[i][i];
    }
}

/**
 * @brief Perform backward substitution: solve Lᵀ*x = y.
 * @param L  Lower triangular matrix (11×11).
 * @param y  Right-hand side vector (11).
 * @param x  Solution vector (11).
 */
static inline void backward_sub_11(const float L[11][11], const float y[11], float x[11])
{
    for (int i = 10; i >= 0; i--) {
        float sum = y[i];
        for (int j = i+1; j < 11; j++)
            sum -= L[j][i] * x[j];
        x[i] = sum / L[i][i];
    }
}

/**
 * @brief Perform one EKF step: state prediction + measurement update.
 *
 * State layout (NX=16):
 *   [x, y, z, vx, vy, vz, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
 *
 * Measurement layout (NZ=11):
 *   [z_baro, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
 *
 * @param x   Prior state estimate (16), updated in-place.
 * @param P   Prior covariance (16×16), updated in-place.
 * @param z   Noisy measurement (11).
 * @param dt  Step duration [s].
 * @param Q   Process noise covariance (16×16).
 * @param R   Measurement noise covariance (11×11).
 */
static void ekf_step(float x[NX], float P[NX][NX], const float z[NZ], float dt, const float Q[NX][NX], const float R[NZ][NZ])
{
    /* --- Unpack prior state ----------------------------------------------- */
    float x0 = x[0],  y0 = x[1],  z0_ = x[2];
    float vx = x[3],  vy = x[4],  vz  = x[5];
    float ax = x[6],  ay = x[7],  az  = x[8];
    float e0 = x[9],  e1 = x[10], e2  = x[11], e3 = x[12];
    float wx = x[13], wy = x[14], wz  = x[15];

    /* --- Rotation matrix R(q) -------------------------------------------- */
    float R00 = 1.0f - 2.0f*(e2*e2 + e3*e3);
    float R01 = 2.0f*(e1*e2 - e0*e3);
    float R02 = 2.0f*(e1*e3 + e0*e2);
    float R10 = 2.0f*(e1*e2 + e0*e3);
    float R11 = 1.0f - 2.0f*(e1*e1 + e3*e3);
    float R12 = 2.0f*(e2*e3 - e0*e1);
    float R20 = 2.0f*(e1*e3 - e0*e2);
    float R21 = 2.0f*(e2*e3 + e0*e1);
    float R22 = 1.0f - 2.0f*(e1*e1 + e2*e2);

    /* --- STEP 1: State prediction (Euler integration) --------------------- */
    float xp[NX];

    xp[0]  = x0  + vx * dt;
    xp[1]  = y0  + vy * dt;
    xp[2]  = z0_ + vz * dt;

    xp[3]  = vx + (R00*ax + R01*ay + R02*az) * dt;
    xp[4]  = vy + (R10*ax + R11*ay + R12*az) * dt;
    xp[5]  = vz + (R20*ax + R21*ay + R22*az) * dt;

    xp[6]  = ax;  // constant acceleration model
    xp[7]  = ay;
    xp[8]  = az;

    xp[9]  = e0 + 0.5f*(-e1*wx - e2*wy - e3*wz) * dt;
    xp[10] = e1 + 0.5f*( e0*wx + e2*wz - e3*wy) * dt;
    xp[11] = e2 + 0.5f*( e3*wx + e0*wy - e1*wz) * dt;
    xp[12] = e3 + 0.5f*(-e2*wx + e1*wy + e0*wz) * dt;

    xp[13] = wx;  // constant rate model
    xp[14] = wy;
    xp[15] = wz;

    quat_normalize(&xp[9], &xp[10], &xp[11], &xp[12]);

    /* --- STEP 2: Discrete Jacobian Fd = I + Fc*dt ------------------------ */
    float Fd[NX][NX] = {0};
    for (int i = 0; i < NX; i++) Fd[i][i] = 1.0f;

    // ∂ṗ/∂v
    Fd[0][3] += dt;
    Fd[1][4] += dt;
    Fd[2][5] += dt;

    // ∂v̇x/∂(ax,ay,az)
    Fd[3][6] += R00*dt;  Fd[3][7] += R01*dt;  Fd[3][8] += R02*dt;
    // ∂v̇x/∂q
    Fd[3][9]  += (-2.0f*e3*ay + 2.0f*e2*az)*dt;
    Fd[3][10] += ( 2.0f*e2*ay + 2.0f*e3*az)*dt;
    Fd[3][11] += (-4.0f*e2*ax + 2.0f*e1*ay + 2.0f*e0*az)*dt;
    Fd[3][12] += (-4.0f*e3*ax - 2.0f*e0*ay + 2.0f*e1*az)*dt;

    // ∂v̇y/∂(ax,ay,az)
    Fd[4][6] += R10*dt;  Fd[4][7] += R11*dt;  Fd[4][8] += R12*dt;
    // ∂v̇y/∂q
    Fd[4][9]  += ( 2.0f*e3*ax - 2.0f*e1*az)*dt;
    Fd[4][10] += ( 2.0f*e2*ax - 4.0f*e1*ay - 2.0f*e0*az)*dt;
    Fd[4][11] += ( 2.0f*e1*ax + 2.0f*e3*az)*dt;
    Fd[4][12] += ( 2.0f*e0*ax - 4.0f*e3*ay + 2.0f*e2*az)*dt;

    // ∂v̇z/∂(ax,ay,az)
    Fd[5][6] += R20*dt;  Fd[5][7] += R21*dt;  Fd[5][8] += R22*dt;
    // ∂v̇z/∂q
    Fd[5][9]  += (-2.0f*e2*ax + 2.0f*e1*ay)*dt;
    Fd[5][10] += ( 2.0f*e3*ax + 2.0f*e0*ay - 4.0f*e1*az)*dt;
    Fd[5][11] += (-2.0f*e0*ax + 2.0f*e3*ay - 4.0f*e2*az)*dt;
    Fd[5][12] += ( 2.0f*e1*ax + 2.0f*e2*ay)*dt;

    // ∂ė0/∂(e1,e2,e3,wx,wy,wz)
    Fd[9][10]  += -0.5f*wx*dt;
    Fd[9][11]  += -0.5f*wy*dt;
    Fd[9][12]  += -0.5f*wz*dt;
    Fd[9][13]  += -0.5f*e1*dt;
    Fd[9][14]  += -0.5f*e2*dt;
    Fd[9][15]  += -0.5f*e3*dt;

    // ∂ė1/∂(e0,e2,e3,wx,wy,wz)
    Fd[10][9]  +=  0.5f*wx*dt;
    Fd[10][11] +=  0.5f*wz*dt;
    Fd[10][12] += -0.5f*wy*dt;
    Fd[10][13] +=  0.5f*e0*dt;
    Fd[10][14] += -0.5f*e3*dt;
    Fd[10][15] +=  0.5f*e2*dt;

    // ∂ė2/∂(e0,e1,e3,wx,wy,wz)
    Fd[11][9]  +=  0.5f*wy*dt;
    Fd[11][10] += -0.5f*wz*dt;
    Fd[11][12] +=  0.5f*wx*dt;
    Fd[11][13] +=  0.5f*e3*dt;
    Fd[11][14] +=  0.5f*e0*dt;
    Fd[11][15] += -0.5f*e1*dt;

    // ∂ė3/∂(e0,e1,e2,wx,wy,wz)
    Fd[12][9]  +=  0.5f*wz*dt;
    Fd[12][10] +=  0.5f*wy*dt;
    Fd[12][11] += -0.5f*wx*dt;
    Fd[12][13] += -0.5f*e2*dt;
    Fd[12][14] +=  0.5f*e1*dt;
    Fd[12][15] +=  0.5f*e0*dt;

    /* --- STEP 3: Covariance prediction Pp = Fd*P*Fdᵀ + Q ----------------- */
    float Pp[NX][NX];
    float temp[NX][NX];

    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++) {
            float s = 0;
            for (int k = 0; k < NX; k++)
                s += Fd[i][k] * P[k][j];
            temp[i][j] = s;
        }

    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++) {
            float s = 0;
            for (int k = 0; k < NX; k++)
                s += temp[i][k] * Fd[j][k];   // Fd[j][k] = Fdᵀ[k][j]
            Pp[i][j] = s + Q[i][j];
        }

    /* --- STEP 4: Measurement mapping H (sparse, via index table) ----------------- */
    // H is identity on columns {2,6,7,8,9,10,11,12,13,14,15}
    static const uint8_t Hrow[NZ] = {2, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    /* --- STEP 5: Innovation ν = z – H*xp ----------------------------------- */
    float innov[NZ];
    for (int i = 0; i < NZ; i++)
        innov[i] = z[i] - xp[Hrow[i]];

    /* --- STEP 6: Innovation covariance S = H*Pp*Hᵀ + R (11×11) ------------ */
    // Because H selects rows/cols, S[i][j] = Pp[Hrow[i]][Hrow[j]] + R[i][j]
    float S[NZ][NZ];
    for (int i = 0; i < NZ; i++)
        for (int j = 0; j < NZ; j++)
            S[i][j] = Pp[Hrow[i]][Hrow[j]] + R[i][j];

    /* --- STEP 7: Cholesky decomposition S = L*Lᵀ ------------------------- */
    float Lchol[11][11];
    if (!cholesky11(S, Lchol)) {
        // S is not SPD (numerical error) → skip update, keep predicted state
        for (int i = 0; i < NX; i++) x[i] = xp[i];
        // P stays as Pp
        for (int i = 0; i < NX; i++)
            for (int j = 0; j < NX; j++)
                P[i][j] = Pp[i][j];
        return;
    }

    /* --- STEP 8: Kalman gain K = Pp*Hᵀ*S⁻¹  (16×11) --------------------- */
    //
    //   PpHt[i][j] = Pp[i][Hrow[j]]   for all i in 0..15, j in 0..10
    //
    // Then for each column j of S⁻¹ (solved via Cholesky):
    //   K[:,j] = PpHt * Sinv_col
    //
    float K[NX][NZ];

    // Pre-compute PpHt (16×11)
    float PpHt[NX][NZ];
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NZ; j++)
            PpHt[i][j] = Pp[i][Hrow[j]];

    // For each column j of S⁻¹, solve S*Sinv_col = e_j, then K[:,j] = PpHt * Sinv_col
    for (int j = 0; j < NZ; j++) {
        float ej[NZ]      = {0};  ej[j] = 1.0f;
        float ytmp[NZ],  Sinv_col[NZ];
        forward_sub_11(Lchol, ej,    ytmp);
        backward_sub_11(Lchol, ytmp, Sinv_col);

        for (int i = 0; i < NX; i++) {
            float s = 0;
            for (int r = 0; r < NZ; r++)
                s += PpHt[i][r] * Sinv_col[r];
            K[i][j] = s;
        }
    }

    /* --- STEP 9: State update x = xp + K*ν -------------------------------- */
    for (int i = 0; i < NX; i++) {
        float s = 0;
        for (int j = 0; j < NZ; j++)
            s += K[i][j] * innov[j];
        x[i] = xp[i] + s;
    }
    quat_normalize(&x[9], &x[10], &x[11], &x[12]);

    /* --- STEP 10: Covariance update (Joseph form) -------------------------- */
    // I_KH = I - K*H
    float I_KH[NX][NX];
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++)
            I_KH[i][j] = (i == j) ? 1.0f : 0.0f;

    for (int i = 0; i < NZ; i++)
        for (int r = 0; r < NX; r++)
            I_KH[r][Hrow[i]] -= K[r][i];

    // temp1 = (I-KH)*Pp
    float temp1[NX][NX];
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++) {
            float s = 0;
            for (int k = 0; k < NX; k++)
                s += I_KH[i][k] * Pp[k][j];
            temp1[i][j] = s;
        }

    // P = temp1*(I-KH)ᵀ + K*R*Kᵀ
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++) {
            float s = 0;
            for (int k = 0; k < NX; k++)
                s += temp1[i][k] * I_KH[j][k];

            float rterm = 0;
            for (int k = 0; k < NZ; k++)
                for (int p = 0; p < NZ; p++)
                    rterm += K[i][k] * R[k][p] * K[j][p];

            P[i][j] = s + rterm;
        }
}


/**
 * @brief Helper function to convert a quaternion to a rotation matrix.
 * @param e0, e1, e2, e3  Quaternion components.
 * @param Rmat              Output rotation matrix (3×3).
 */
static inline void quat_to_R(float e0, float e1, float e2, float e3, float Rmat[3][3])
{
    Rmat[0][0] = 1.0f - 2.0f*(e2*e2 + e3*e3);
    Rmat[0][1] = 2.0f*(e1*e2 - e0*e3);
    Rmat[0][2] = 2.0f*(e1*e3 + e0*e2);

    Rmat[1][0] = 2.0f*(e1*e2 + e0*e3);
    Rmat[1][1] = 1.0f - 2.0f*(e1*e1 + e3*e3);
    Rmat[1][2] = 2.0f*(e2*e3 - e0*e1);

    Rmat[2][0] = 2.0f*(e1*e3 - e0*e2);
    Rmat[2][1] = 2.0f*(e2*e3 + e0*e1);
    Rmat[2][2] = 1.0f - 2.0f*(e1*e1 + e2*e2);
}

/**
 * @brief Proportional Navigation guidance law.
 *
 * Matches the Python compute_guidance() exactly:
 *   a_cmd  = N * V_norm * cross(los_rate, los)
 *   a_body = Rᵀ * a_cmd
 *   cm     =  a_body[1] * I / (q_dyn * S * l)
 *   cn     = -a_body[0] * I / (q_dyn * S * l)
 *
 * @param state      EKF state [x,y,z, vx,vy,vz, ax,ay,az, e0,e1,e2,e3, wx,wy,wz]
 * @param target_enu Target position in ENU frame [m]
 * @param cm_cmd     Output pitch moment coefficient
 * @param cn_cmd     Output yaw moment coefficient
 */
static void compute_guidance(const float state[16], const float target_enu[3], const float total_distance, float sim_time, float *cm_cmd, float *cn_cmd)
{
    /* --- Unpack state ------------------------------------ */
    float x  = state[0];
    float y  = state[1];
    float z  = state[2];
    float vx = state[3];
    float vy = state[4];
    float vz = state[5];

    float e0 = state[9];
    float e1 = state[10];
    float e2 = state[11];
    float e3 = state[12];

    /* --- Lead prediction ------------------------------------- */ 
    const float tau = 0.1f; // 100 ms
    x += vx * tau;
    y += vy * tau;
    z += vz * tau;

    float V_norm = sqrtf(vx*vx + vy*vy + vz*vz);

    // Suppress guidance at low speed (rail, liftoff)
    if (V_norm < 20.0f) {
        *cm_cmd = 0.0f;
        *cn_cmd = 0.0f;
        return;
    }

    /* --- Position error --------------------------------------*/
    float rx = target_enu[0] - x;
    float ry = target_enu[1] - y;
    float rz = target_enu[2] - z;

    float R_norm = sqrtf(rx*rx + ry*ry + rz*rz);
    if (R_norm < 1e-6f) {
        *cm_cmd = 0.0f;
        *cn_cmd = 0.0f;
        return;
    }

    float inv_R = 1.0f / R_norm;

    // LOS unit vector
    float los[3] = {
        rx * inv_R,
        ry * inv_R,
        rz * inv_R
    };

    /*  --- LOS rate  r × v / |r|² ---------------------------- */
    float cross_rv[3] = {
        ry*vz - rz*vy,
        rz*vx - rx*vz,
        rx*vy - ry*vx
    };

    float inv_R2 = inv_R * inv_R;
    float los_rate[3] = {
        cross_rv[0] * inv_R2,
        cross_rv[1] * inv_R2,
        cross_rv[2] * inv_R2
    };

    /* --- LOS rate clamp ------------------------------------- */
    const float LOS_RATE_MAX = 0.5f; // rad/s
    float lr_mag = sqrtf(
        los_rate[0]*los_rate[0] +
        los_rate[1]*los_rate[1] +
        los_rate[2]*los_rate[2]
    );

    if (lr_mag > LOS_RATE_MAX) {
        float scale = LOS_RATE_MAX / lr_mag;
        los_rate[0] *= scale;
        los_rate[1] *= scale;
        los_rate[2] *= scale;
    }

    float progress = 1.0f - (R_norm / total_distance);
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    float N = 3.0f * (1.0f - progress);

    /* --- PN commanded acceleration ------------------------------ */
    float cross_cmd[3] = {
        los_rate[1]*los[2] - los_rate[2]*los[1],
        los_rate[2]*los[0] - los_rate[0]*los[2],
        los_rate[0]*los[1] - los_rate[1]*los[0]
    };

    float a_cmd[3] = {
        N * V_norm * cross_cmd[0],
        N * V_norm * cross_cmd[1],
        N * V_norm * cross_cmd[2]
    };

    /* --- Gravity feed-forward -------------------------------- */
    a_cmd[2] -= 9.81f;

    /* --- Rotate inertial -> body ------------------------------ */
    float Rmat[3][3];
    quat_to_R(e0, e1, e2, e3, Rmat);

    float a_body[3] = {
        Rmat[0][0]*a_cmd[0] + Rmat[1][0]*a_cmd[1] + Rmat[2][0]*a_cmd[2],
        Rmat[0][1]*a_cmd[0] + Rmat[1][1]*a_cmd[1] + Rmat[2][1]*a_cmd[2],
        Rmat[0][2]*a_cmd[0] + Rmat[1][2]*a_cmd[1] + Rmat[2][2]*a_cmd[2]
    };

    /* --- Dynamic pressure ------------------------------------ */
    float rho = 1.225f * expf(-z / 8500.0f);
    float q_dyn = 0.5f * rho * V_norm * V_norm;

    float moment_scale = q_dyn * REF_AREA * REF_LENGTH;
    if (moment_scale < 1e-3f) {
        *cm_cmd = 0.0f;
        *cn_cmd = 0.0f;
        return;
    }

    /* --- Convert acceleration to moment coefficients ------------------------- */
    float cm =  a_body[1] * PITCH_INERTIA / moment_scale;
    float cn = -a_body[0] * PITCH_INERTIA / moment_scale;

    /* --- Clip ---------------------------------------------- */
    if (cm >  CM_CLIP) cm =  CM_CLIP;
    if (cm < -CM_CLIP) cm = -CM_CLIP;
    if (cn >  CM_CLIP) cn =  CM_CLIP;
    if (cn < -CM_CLIP) cn = -CM_CLIP;

    *cm_cmd = cm;
    *cn_cmd = cn;
}

/**
 * @brief PD roll controller.
 *
 * Matches the Python compute_roll() exactly:
 *   roll_angle = atan2(R[2][1], R[2][2])  [deg]
 *   roll_error = (roll_angle - setpoint + 180) % 360 - 180
 *   cant = -(Kp_angle*roll_error + Kp_rate*w1 + Kd*w1dot)
 *
 * @param state    EKF state vector
 * @param sim_time Current time [s]  — use esp_timer_get_time()*1e-6f in SIM mode
 * @return cant angle [deg], clamped to ±MAX_CANT
 */
static float compute_roll(const float state[16], float sim_time)
{
    float e0 = state[9],  e1 = state[10];
    float e2 = state[11], e3 = state[12];
    float w1 = state[13];   // body roll rate [rad/s]

    float Rmat[3][3];
    quat_to_R(e0, e1, e2, e3, Rmat);

    // Roll angle from rotation matrix (same convention as Python)
    float roll_angle = atan2f(Rmat[2][1], Rmat[2][2]) * 57.29578f;  // rad → deg

    // Shortest-path error, wrapped to [-180, 180] deg
    float roll_error = fmodf(roll_angle - ROLL_SETPOINT + 180.0f, 360.0f) - 180.0f;

    // FIX: use prev_time < 0 as the "first step" sentinel (not > 0),
    // matching the Python "if _prev_roll_t is not None" pattern.
    float w1dot = 0.0f;
    if (prev_time >= 0.0f) {
        float dt = fmaxf(sim_time - prev_time, 1e-9f);   // min 1 ns guard
        w1dot = (w1 - prev_spin) / dt;
    }

    prev_spin = w1;
    prev_time = sim_time;

    float cant = -(ROLL_KP_ANGLE * roll_error +
                   ROLL_KP_RATE  * w1         +
                   ROLL_KD       * w1dot);

    if (cant >  MAX_CANT) cant =  MAX_CANT;
    if (cant < -MAX_CANT) cant = -MAX_CANT;

    return cant;
}

/* --- User API ------------------------------------------------------------- */
void ekf_init(float x[NX], float P[NX][NX])
{
    /* --- State initialization ---------- */ 
    for (int i = 0; i < NX; i++) {
        x[i] = 0.0f;
    }
    

    // Quaternion = identity
    x[9] = 1.0f;   // e0 = 1
    x[10] = 0.0f;
    x[11] = 0.0f;
    x[12] = 0.0f;

    /* --- Covariance initialization ---------- */
    for (int i = 0; i < NX; i++)
        for (int j = 0; j < NX; j++)
            P[i][j] = 0.0f;

    // Position [m^2]
    P[0][0] = 1.0f;
    P[1][1] = 1.0f;
    P[2][2] = 1.0f;

    // Velocity [(m/s)^2]
    P[3][3] = 1.0f;
    P[4][4] = 1.0f;
    P[5][5] = 1.0f;

    // Acceleration [(m/s^2)^2]
    P[6][6] = 100.0f;
    P[7][7] = 100.0f;
    P[8][8] = 100.0f;

    // Quaternion
    P[9][9]  = 1e-3f;
    P[10][10] = 1e-3f;
    P[11][11] = 1e-3f;
    P[12][12] = 1e-3f;

    // Angular rates [(rad/s)^2]
    P[13][13] = 0.1f;
    P[14][14] = 0.1f;
    P[15][15] = 0.1f;
}

void ekf_processing(app_context_t *ctx, const float meas[NZ], float dt)
{
    ekf_step(ctx->ekf_x, ctx->ekf_P, meas, dt, EKF_Q, EKF_R);

#if APP_MODE == APP_MODE_HIL
    float cm, cn;
    compute_guidance(ctx->ekf_x, ctx->target_enu, ctx->total_distance, ctx->time_s, &cm, &cn);
    float cant = compute_roll(ctx->ekf_x, ctx->time_s);
    ctx->actuators.cm = cm;
    ctx->actuators.cn = cn;
    ctx->actuators.cant = cant;
#endif
}