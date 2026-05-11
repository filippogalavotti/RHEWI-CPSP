import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as mplcm
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from pathlib import Path
import csv
import struct
import time
import serial

from rocketpy import Environment, Flight, Rocket, SolidMotor, GenericSurface
from rocketpy.tools import (
    quaternions_to_spin,
    quaternions_to_nutation,
    quaternions_to_precession,
)

try:
    BASE_DIR = Path(__file__).resolve().parent
except NameError:
    BASE_DIR = Path(".").resolve()

# ── MISSION ────────────────────────────────────────────────────────────────
LAUNCH_LAT, LAUNCH_LON, LAUNCH_ALT = 44.48800684283911, 11.32889678578902, 0.0

TARGET_LAT, TARGET_LON, TARGET_ALT = 44.48256575070729, 11.354638821647374, 1000.0

# ── PLOTTING CONSTANTS ────────────────────────────────────────────────────────────────

SAMPLING_RATE = 50 # [Hz]
PLOT_HZ = SAMPLING_RATE

# ── ROCKET CONSTANTS ───────────────────────────────────────────────────────
ROCKET_RADIUS   = 0.075          # m
ROCKET_MASS     = 22.740         # kg
PITCH_INERTIA   = 14.304         # kg·m²
MOTOR_BURN_TIME = 5.3            # s
REF_AREA        = np.pi * ROCKET_RADIUS**2   # 0.01767 m²
REF_LENGTH      = 1.785          # m  full rocket length (maximises moment)

# ── Body-frame geometry helpers ─────────────────────────────────────────────
# Body frame: +Z = nose, X/Y = lateral.
# All helpers return lists of (N,3) float arrays for Poly3DCollection.
# apply_rotation() maps them body -> ENU inertial.

_FIN_ROOT  = 0.30
_FIN_TIP   = 0.093
_FIN_SPAN  = 0.16
_BODY_R    = 0.075
_BODY_HALF = 0.20
_NOSE_H    = 0.10

# ── GUIDANCE PARAMETERS ────────────────────────────────────────────────────
GUIDANCE_KP_BOOST = 0.006   # [cm per m error] during boost
GUIDANCE_KP_COAST = 0.004   # coast (lower q, need slightly more cm)
CM_CLIP           = 12.0    # hard limit — large enough to dominate fin restoring
INTERCEPT_RADIUS  = 10.0     # meters

# ── ROLL CONTROL ───────────────────────────────────────────────────────────
ROLL_SETPOINT  = 0     # deg — desired roll angle
ROLL_KP_ANGLE  = 0.01  # cant deg / deg  roll-angle error (proportional)
ROLL_KP_RATE   = 0.01  # cant deg / (rad/s)  roll-rate damping
ROLL_KD        = 0.01  # cant deg / (rad/s²) angular-accel damping
MAX_CANT       = 5.0   # deg
MAX_CANT_PLOT  = 30    # deg

PLOTSCALE_FINS = MAX_CANT_PLOT / MAX_CANT

# ── SENSOR NOISE STANDARD DEVIATIONS ──────────────────────────────────────

# ── NOISE CONSTANTS ────────────────────────────────────────────────────────
# Sensor: MS8607
NOISE_BARO = 33.2    # [m]

# Sensor: BNO055
NOISE_ACC  = 0.01    # [m/s²]
NOISE_RATE = 0.005   # [rad/s]
NOISE_QUAT = 0.02    # [–]

# ── FIXED BIASES (drawn once per simulated flight) ─────────────────────────
BIAS_BARO = np.random.uniform(-33.2, +33.2)   # [m]     
BIAS_ACC  = np.random.uniform(-0.785, +0.785, size=3)   # [x, y, z] [m/s²]
BIAS_RATE = np.random.uniform(-0.016, +0.016, size=3)  # [x, y, z] [rad/s]

# ── EXTENDED KALMAN FILTER PARAMETERS ─────────────────────────────────────
# EKF state  (16): [x, y, z, vx, vy, vz, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
# Measurement(11): [z, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
#
# Process noise Q — how much we distrust the process model per step.
# ax,ay,az get a large value because thrust and drag change rapidly and
# the constant-acceleration assumption breaks down quickly.
EKF_Q_POS   = 1.0    # [m²]
EKF_Q_VEL   = 0.5    # [(m/s)²]
EKF_Q_ACC   = 1.0  # 10.0   # [(m/s²)²]  large — aero forces change fast
EKF_Q_QUAT  = 1e-6 # 1e-4   # [-]
EKF_Q_RATE  = 1e-4 # 0.01   # [(rad/s)²]

# Measurement noise R — matched to the noise sigmas above.
EKF_R_BARO  = NOISE_BARO**2   # [m²]
EKF_R_ACC   = NOISE_ACC**2    # [(m/s²)²]
EKF_R_QUAT  = NOISE_QUAT**2   # [-]
EKF_R_RATE  = NOISE_RATE**2   # [(rad/s)²]

# Q (16×16) — indices: 0-2 pos, 3-5 vel, 6-8 acc, 9-12 quat, 13-15 rate
EKF_Q = np.diag([
    EKF_Q_POS,  EKF_Q_POS,  EKF_Q_POS,              # x,  y,  z
    EKF_Q_VEL,  EKF_Q_VEL,  EKF_Q_VEL,              # vx, vy, vz
    EKF_Q_ACC,  EKF_Q_ACC,  EKF_Q_ACC,              # ax, ay, az
    EKF_Q_QUAT, EKF_Q_QUAT, EKF_Q_QUAT, EKF_Q_QUAT, # e0, e1, e2, e3
    EKF_Q_RATE, EKF_Q_RATE, EKF_Q_RATE,             # wx, wy, wz
])

# R (11×11) — indices: 0 z_baro, 1-3 acc, 4-7 quat, 8-10 rate
EKF_R = np.diag([
    EKF_R_BARO,                                      # z
    EKF_R_ACC,  EKF_R_ACC,  EKF_R_ACC,              # ax, ay, az
    EKF_R_QUAT, EKF_R_QUAT, EKF_R_QUAT, EKF_R_QUAT, # e0, e1, e2, e3
    EKF_R_RATE, EKF_R_RATE, EKF_R_RATE,             # wx, wy, wz
])

# ── HARDWARE-IN-THE-LOOP (HIL) CONFIGURATION ──────────────────────────────
HIL_ENABLED = False            # flip to True to use the Arduino
HIL_PORT    = "/dev/ttyUSB0"   # e.g. "COM3" on Windows
HIL_BAUD    = 115200
HIL_TIMEOUT = 0.018            # [s] — keep below 1/SAMPLING_RATE

# ── SHARED GUIDANCE STATE (read by GenericSurface callables) ───────────────
_gs = {"cm": 0.0, "cn": 0.0}


def get_fin_polygons(cant_deg):
    """
    Cant rotation: pivot = midpoint of fin along span (normal to body).
    Order: translate to pivot -> cant -> translate back -> offset -> angular placement.
    """
    cant_rad = np.radians(cant_deg)
    c, s = np.cos(cant_rad), np.sin(cant_rad)
    # Rotation about X
    R_cant = np.array([[1, 0,  0],
                       [0, c, -s],
                       [0, s,  c]], dtype=float)
    polys = []

    # Pivot point (mid-span of fin)
    pivot = np.array([_FIN_SPAN / 2, 0, 0])

    for i in range(3):
        phi = np.radians(i * 120)
        cp, sp = np.cos(phi), np.sin(phi)

        v = np.array([
            [0,          0, -_FIN_ROOT/2],
            [0,          0,  _FIN_ROOT/2],
            [_FIN_SPAN,  0,  _FIN_TIP/2 ],
            [_FIN_SPAN,  0, -_FIN_TIP/2 ],
        ], dtype=float)

        # --- Move to pivot frame ---
        v -= pivot

        # --- Apply cant ---
        v = v @ R_cant.T

        # --- Move back ---
        v += pivot

        # Offset to body surface
        v[:, 0] += _BODY_R

        # Angular placement around rocket body
        R_pos = np.array([[cp, -sp, 0],
                          [sp,  cp, 0],
                          [0,   0, 1]], dtype=float)

        v = v @ R_pos.T

        polys.append(v)

    return polys


def get_cylinder_polygons(n=18):
    th = np.linspace(0, 2*np.pi, n, endpoint=False)
    quads = []
    for t0, t1 in zip(th, np.roll(th,-1)):
        x0,y0 = _BODY_R*np.cos(t0), _BODY_R*np.sin(t0)
        x1,y1 = _BODY_R*np.cos(t1), _BODY_R*np.sin(t1)
        quads.append(np.array([
            [x0,y0,-_BODY_HALF],[x1,y1,-_BODY_HALF],
            [x1,y1, _BODY_HALF],[x0,y0, _BODY_HALF],
        ], dtype=float))
    return quads


def get_nose_polygons(n=18):
    th   = np.linspace(0, 2*np.pi, n, endpoint=False)
    apex = np.array([0.,0., _BODY_HALF+_NOSE_H])
    tris = []
    for t0,t1 in zip(th, np.roll(th,-1)):
        b0 = np.array([_BODY_R*np.cos(t0), _BODY_R*np.sin(t0), _BODY_HALF])
        b1 = np.array([_BODY_R*np.cos(t1), _BODY_R*np.sin(t1), _BODY_HALF])
        tris.append(np.array([apex,b0,b1], dtype=float))
    return tris


def get_tail_polygons(n=18):
    th  = np.linspace(0, 2*np.pi, n, endpoint=False)
    ctr = np.array([0.,0.,-_BODY_HALF])
    tris = []
    for t0,t1 in zip(th, np.roll(th,-1)):
        b0 = np.array([_BODY_R*np.cos(t0), _BODY_R*np.sin(t0), -_BODY_HALF])
        b1 = np.array([_BODY_R*np.cos(t1), _BODY_R*np.sin(t1), -_BODY_HALF])
        tris.append(np.array([ctr,b0,b1], dtype=float))
    return tris


def apply_rotation(polys, R):
    return [v @ R.T for v in polys]


def _cm_func(alpha, beta, mach, re, q_rate, r_rate, p_rate):
    return _gs["cm"]

def _cn_func(alpha, beta, mach, re, q_rate, r_rate, p_rate):
    return _gs["cn"]

# ── COORDINATE HELPERS ─────────────────────────────────────────────────────
def geodetic_to_enu(lat, lon, alt, rlat, rlon, ralt):
    mpd_lat = 111320.0
    mpd_lon = 111320.0 * np.cos(np.radians(rlat))
    return np.array([(lon-rlon)*mpd_lon, (lat-rlat)*mpd_lat, alt-ralt])

TARGET_ENU = geodetic_to_enu(TARGET_LAT, TARGET_LON, TARGET_ALT,
                              LAUNCH_LAT, LAUNCH_LON, LAUNCH_ALT)
print(f"Target ENU: E={TARGET_ENU[0]:.1f}  N={TARGET_ENU[1]:.1f}  Up={TARGET_ENU[2]:.1f} m")

TOTAL_DISTANCE = np.linalg.norm(TARGET_ENU)

def quat_to_R(e0, e1, e2, e3):
    n = np.sqrt(e0**2+e1**2+e2**2+e3**2)
    if n < 1e-12: return np.eye(3)
    e0,e1,e2,e3 = e0/n,e1/n,e2/n,e3/n
    return np.array([
        [1-2*(e2**2+e3**2),   2*(e1*e2-e0*e3),   2*(e1*e3+e0*e2)],
        [  2*(e1*e2+e0*e3), 1-2*(e1**2+e3**2),   2*(e2*e3-e0*e1)],
        [  2*(e1*e3-e0*e2),   2*(e2*e3+e0*e1), 1-2*(e1**2+e2**2)],
    ])

def non_rolling_R(R):
    bz = R[:,2]
    e  = np.array([1.,0.,0.])
    if abs(np.dot(bz,e)) > 0.9: e = np.array([0.,1.,0.])
    pa = e - np.dot(e,bz)*bz
    n  = np.linalg.norm(pa)
    if n < 1e-9: return R
    pa /= n
    ya  = np.cross(bz, pa)
    return np.column_stack([pa, ya, bz])


# ── NOISE INJECTION ────────────────────────────────────────────────────────
def build_noisy_measurement(state, state_prev, dt):
    """
    Build the 11-element noisy measurement vector from the RocketPy ODE state.

    Simulation state layout (RocketPy):
      [x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]

    Measurement vector returned (11,):
      [0]  z_baro      [m]       barometer altitude
      [1]  ax_body     [m/s²]    body-frame linear acceleration X
      [2]  ay_body     [m/s²]    body-frame linear acceleration Y
      [3]  az_body     [m/s²]    body-frame linear acceleration Z
      [4]  e0          [-]       BNO055 quaternion scalar
      [5]  e1          [-]       BNO055 quaternion i
      [6]  e2          [-]       BNO055 quaternion j
      [7]  e3          [-]       BNO055 quaternion k
      [8]  wx          [rad/s]   BNO055 gyro X
      [9]  wy          [rad/s]   BNO055 gyro Y
      [10] wz          [rad/s]   BNO055 gyro Z

    Body-frame accelerations are derived by finite-differencing the inertial
    velocity between consecutive states, then rotating into the body frame
    with R^T.  On hardware, replace this block with a direct BNO055 read of
    the linear-acceleration registers (gravity already subtracted by the chip).

    state_prev=None on the first call → acceleration returned as zero.
    """

    # ── Quaternion — add noise then renormalise ───────────────────────────
    e0_n = state[6] + np.random.normal(0.0, NOISE_QUAT)
    e1_n = state[7] + np.random.normal(0.0, NOISE_QUAT)
    e2_n = state[8] + np.random.normal(0.0, NOISE_QUAT)
    e3_n = state[9] + np.random.normal(0.0, NOISE_QUAT)
    q_norm = np.sqrt(e0_n**2 + e1_n**2 + e2_n**2 + e3_n**2)
    if q_norm > 1e-12:
        e0_n /= q_norm;  e1_n /= q_norm
        e2_n /= q_norm;  e3_n /= q_norm

    R_bn = quat_to_R(e0_n, e1_n, e2_n, e3_n)   # body-to-inertial rotation

    if state_prev is not None and dt > 1e-9:
        # Inertial acceleration [m/s²]
        ax_i = (state[3] - state_prev[3]) / dt
        ay_i = (state[4] - state_prev[4]) / dt
        az_i = (state[5] - state_prev[5]) / dt
        # Rotate inertial → body:  a_body = R^T · a_inertial
        ax_b = R_bn[0,0]*ax_i + R_bn[1,0]*ay_i + R_bn[2,0]*az_i
        ay_b = R_bn[0,1]*ax_i + R_bn[1,1]*ay_i + R_bn[2,1]*az_i
        az_b = R_bn[0,2]*ax_i + R_bn[1,2]*ay_i + R_bn[2,2]*az_i
    else:
        ax_b = 0.0;  ay_b = 0.0;  az_b = 0.0

    # ── Barometer altitude ────────────────────────────────────────────────
    z_baro = state[2] + BIAS_BARO + np.random.normal(0.0, NOISE_BARO)

    # ── Accelerometer (body frame) ────────────────────────────────────────
    ax_noisy = ax_b + BIAS_ACC[0] + np.random.normal(0.0, NOISE_ACC)
    ay_noisy = ay_b + BIAS_ACC[1] + np.random.normal(0.0, NOISE_ACC)
    az_noisy = az_b + BIAS_ACC[2] + np.random.normal(0.0, NOISE_ACC)

    # ── Angular rates ─────────────────────────────────────────────────────
    wx_n = state[10] + BIAS_RATE[0] + np.random.normal(0.0, NOISE_RATE)
    wy_n = state[11] + BIAS_RATE[1] + np.random.normal(0.0, NOISE_RATE)
    wz_n = state[12] + BIAS_RATE[2] + np.random.normal(0.0, NOISE_RATE)

    # ── Assemble 11-element measurement vector ────────────────────────────
    meas = np.zeros(11)
    meas[0]  = z_baro    # barometer altitude   [m]
    meas[1]  = ax_noisy  # body accel X         [m/s²]
    meas[2]  = ay_noisy  # body accel Y         [m/s²]
    meas[3]  = az_noisy  # body accel Z         [m/s²]
    meas[4]  = e0_n      # quaternion scalar    [-]
    meas[5]  = e1_n      # quaternion i         [-]
    meas[6]  = e2_n      # quaternion j         [-]
    meas[7]  = e3_n      # quaternion k         [-]
    meas[8]  = wx_n      # gyro X               [rad/s]
    meas[9]  = wy_n      # gyro Y               [rad/s]
    meas[10] = wz_n      # gyro Z               [rad/s]

    return meas


# ── EXTENDED KALMAN FILTER ─────────────────────────────────────────────────
def ekf_step(x_ekf, P_ekf, measurement, dt, Q, R):
    """
    One predict-update cycle of the Extended Kalman Filter.

    EKF State layout (indices 0-15)
    ─────────────────────────────────────────────────────────
      0  x    inertial East  position   [m]
      1  y    inertial North position   [m]
      2  z    inertial Up   position    [m]
      3  vx   inertial East  velocity   [m/s]
      4  vy   inertial North velocity   [m/s]
      5  vz   inertial Up   velocity    [m/s]
      6  ax   body-frame linear accel X [m/s²]
      7  ay   body-frame linear accel Y [m/s²]
      8  az   body-frame linear accel Z [m/s²]
      9  e0   quaternion scalar  (Hamilton)
     10  e1   quaternion i
     11  e2   quaternion j
     12  e3   quaternion k
     13  wx   body angular rate X       [rad/s]
     14  wy   body angular rate Y       [rad/s]
     15  wz   body angular rate Z       [rad/s]

    Measurement layout (indices 0-10) — output of build_noisy_measurement():
    ─────────────────────────────────────────────────────────
      0  z_baro    [m]      barometer altitude
      1  ax        [m/s²]   body-frame accel X
      2  ay        [m/s²]   body-frame accel Y
      3  az        [m/s²]   body-frame accel Z
      4  e0        [-]      BNO055 quaternion scalar
      5  e1        [-]      BNO055 quaternion i
      6  e2        [-]      BNO055 quaternion j
      7  e3        [-]      BNO055 quaternion k
      8  wx        [rad/s]  BNO055 gyro X
      9  wy        [rad/s]  BNO055 gyro Y
     10  wz        [rad/s]  BNO055 gyro Z

    Process model (continuous, Euler-discretised)
    ─────────────────────────────────────────────────────────
      ẋ  = vx
      ẏ  = vy
      ż  = vz
      v̇x = R[0,0]*ax + R[0,1]*ay + R[0,2]*az   ← body accel rotated to inertial
      v̇y = R[1,0]*ax + R[1,1]*ay + R[1,2]*az
      v̇z = R[2,0]*ax + R[2,1]*ay + R[2,2]*az
      ȧx = 0   (constant-acceleration over one step; Q absorbs rapid changes)
      ȧy = 0
      ȧz = 0
      ė0 = 0.5*(-e1*wx - e2*wy - e3*wz)
      ė1 = 0.5*( e0*wx + e2*wz - e3*wy)
      ė2 = 0.5*( e3*wx + e0*wy - e1*wz)
      ė3 = 0.5*(-e2*wx + e1*wy + e0*wz)
      ẇx = 0
      ẇy = 0
      ẇz = 0

    Arguments
    ─────────────────────────────────────────────────────────
      x_ekf       : (16,)    prior state estimate
      P_ekf       : (16,16)  prior covariance
      measurement : (11,)    output of build_noisy_measurement()
      dt          : float    step duration [s]
      Q           : (16,16)  process noise covariance
      R           : (11,11)  measurement noise covariance

    Returns
    ─────────────────────────────────────────────────────────
      x_upd : (16,) updated state estimate
      P_upd : (16,16) updated covariance (Joseph form)
    """

    # ── Unpack prior state ────────────────────────────────────────────────
    x_  = x_ekf[0];  y_  = x_ekf[1];  z_  = x_ekf[2]
    vx_ = x_ekf[3];  vy_ = x_ekf[4];  vz_ = x_ekf[5]
    ax_ = x_ekf[6];  ay_ = x_ekf[7];  az_ = x_ekf[8]
    e0_ = x_ekf[9];  e1_ = x_ekf[10]; e2_ = x_ekf[11]; e3_ = x_ekf[12]
    wx_ = x_ekf[13]; wy_ = x_ekf[14]; wz_ = x_ekf[15]

    # ── Rotation matrix R(q): body → inertial ────────────────────────────
    # Used in the process model to rotate body-frame acceleration to inertial.
    # Computed from the prior quaternion (evaluated at x_ekf, not x_pred).
    R00 = 1.0 - 2.0*(e2_**2 + e3_**2)
    R01 = 2.0*(e1_*e2_ - e0_*e3_)
    R02 = 2.0*(e1_*e3_ + e0_*e2_)
    R10 = 2.0*(e1_*e2_ + e0_*e3_)
    R11 = 1.0 - 2.0*(e1_**2 + e3_**2)
    R12 = 2.0*(e2_*e3_ - e0_*e1_)
    R20 = 2.0*(e1_*e3_ - e0_*e2_)
    R21 = 2.0*(e2_*e3_ + e0_*e1_)
    R22 = 1.0 - 2.0*(e1_**2 + e2_**2)

    # ── STEP 1 — State prediction (Euler integration) ─────────────────────
    x_pred = np.zeros(16)

    x_pred[0]  = x_  + vx_ * dt                                              # x
    x_pred[1]  = y_  + vy_ * dt                                              # y
    x_pred[2]  = z_  + vz_ * dt                                              # z
    x_pred[3]  = vx_ + (R00*ax_ + R01*ay_ + R02*az_) * dt                   # vx
    x_pred[4]  = vy_ + (R10*ax_ + R11*ay_ + R12*az_) * dt                   # vy
    x_pred[5]  = vz_ + (R20*ax_ + R21*ay_ + R22*az_) * dt                   # vz
    x_pred[6]  = ax_                                                          # ax (const)
    x_pred[7]  = ay_                                                          # ay (const)
    x_pred[8]  = az_                                                          # az (const)
    x_pred[9]  = e0_ + 0.5*(-e1_*wx_ - e2_*wy_ - e3_*wz_) * dt             # e0
    x_pred[10] = e1_ + 0.5*( e0_*wx_ + e2_*wz_ - e3_*wy_) * dt             # e1
    x_pred[11] = e2_ + 0.5*( e3_*wx_ + e0_*wy_ - e1_*wz_) * dt             # e2
    x_pred[12] = e3_ + 0.5*(-e2_*wx_ + e1_*wy_ + e0_*wz_) * dt             # e3
    x_pred[13] = wx_                                                          # wx (const)
    x_pred[14] = wy_                                                          # wy (const)
    x_pred[15] = wz_                                                          # wz (const)

    # Renormalise predicted quaternion
    q_n = np.sqrt(x_pred[9]**2 + x_pred[10]**2 + x_pred[11]**2 + x_pred[12]**2)
    if q_n > 1e-12:
        x_pred[9]  /= q_n;  x_pred[10] /= q_n
        x_pred[11] /= q_n;  x_pred[12] /= q_n

    # ── STEP 2 — Continuous Jacobian  F_c = ∂f/∂x  (16×16) ──────────────
    # Evaluated at x_ekf (prior state).
    # Each non-zero entry is annotated with the partial derivative it encodes.
    F_c = np.zeros((16, 16))

    # ∂ẋ/∂vx,  ∂ẏ/∂vy,  ∂ż/∂vz
    F_c[0, 3] = 1.0
    F_c[1, 4] = 1.0
    F_c[2, 5] = 1.0

    # ── ∂v̇x/∂(ax,ay,az) = R row-0 ───────────────────────────────────────
    F_c[3, 6] = R00   # ∂v̇x/∂ax
    F_c[3, 7] = R01   # ∂v̇x/∂ay
    F_c[3, 8] = R02   # ∂v̇x/∂az

    # ── ∂v̇x/∂(e0,e1,e2,e3)  — from ∂(R·a)/∂q  ──────────────────────────
    # v̇x = (1-2e2²-2e3²)·ax + 2(e1e2-e0e3)·ay + 2(e1e3+e0e2)·az
    F_c[3,  9] = -2.0*e3_*ay_ + 2.0*e2_*az_                  # ∂v̇x/∂e0
    F_c[3, 10] =  2.0*e2_*ay_ + 2.0*e3_*az_                  # ∂v̇x/∂e1
    F_c[3, 11] = -4.0*e2_*ax_ + 2.0*e1_*ay_ + 2.0*e0_*az_   # ∂v̇x/∂e2
    F_c[3, 12] = -4.0*e3_*ax_ - 2.0*e0_*ay_ + 2.0*e1_*az_   # ∂v̇x/∂e3

    # ── ∂v̇y/∂(ax,ay,az) = R row-1 ───────────────────────────────────────
    F_c[4, 6] = R10   # ∂v̇y/∂ax
    F_c[4, 7] = R11   # ∂v̇y/∂ay
    F_c[4, 8] = R12   # ∂v̇y/∂az

    # ── ∂v̇y/∂(e0,e1,e2,e3) ──────────────────────────────────────────────
    # v̇y = 2(e1e2+e0e3)·ax + (1-2e1²-2e3²)·ay + 2(e2e3-e0e1)·az
    F_c[4,  9] =  2.0*e3_*ax_ - 2.0*e1_*az_                  # ∂v̇y/∂e0
    F_c[4, 10] =  2.0*e2_*ax_ - 4.0*e1_*ay_ - 2.0*e0_*az_   # ∂v̇y/∂e1
    F_c[4, 11] =  2.0*e1_*ax_ + 2.0*e3_*az_                  # ∂v̇y/∂e2
    F_c[4, 12] =  2.0*e0_*ax_ - 4.0*e3_*ay_ + 2.0*e2_*az_   # ∂v̇y/∂e3

    # ── ∂v̇z/∂(ax,ay,az) = R row-2 ───────────────────────────────────────
    F_c[5, 6] = R20   # ∂v̇z/∂ax
    F_c[5, 7] = R21   # ∂v̇z/∂ay
    F_c[5, 8] = R22   # ∂v̇z/∂az

    # ── ∂v̇z/∂(e0,e1,e2,e3) ──────────────────────────────────────────────
    # v̇z = 2(e1e3-e0e2)·ax + 2(e2e3+e0e1)·ay + (1-2e1²-2e2²)·az
    F_c[5,  9] = -2.0*e2_*ax_ + 2.0*e1_*ay_                  # ∂v̇z/∂e0
    F_c[5, 10] =  2.0*e3_*ax_ + 2.0*e0_*ay_ - 4.0*e1_*az_   # ∂v̇z/∂e1
    F_c[5, 11] = -2.0*e0_*ax_ + 2.0*e3_*ay_ - 4.0*e2_*az_   # ∂v̇z/∂e2
    F_c[5, 12] =  2.0*e1_*ax_ + 2.0*e2_*ay_                  # ∂v̇z/∂e3

    # Rows 6-8: ȧ = 0 — all zeros (already initialised)

    # ── ∂ė0/∂(e1,e2,e3, wx,wy,wz) ───────────────────────────────────────
    # ė0 = 0.5*(-e1*wx - e2*wy - e3*wz)
    F_c[9, 10] = -0.5*wx_   # ∂ė0/∂e1
    F_c[9, 11] = -0.5*wy_   # ∂ė0/∂e2
    F_c[9, 12] = -0.5*wz_   # ∂ė0/∂e3
    F_c[9, 13] = -0.5*e1_   # ∂ė0/∂wx
    F_c[9, 14] = -0.5*e2_   # ∂ė0/∂wy
    F_c[9, 15] = -0.5*e3_   # ∂ė0/∂wz

    # ── ∂ė1/∂(e0,e2,e3, wx,wy,wz) ───────────────────────────────────────
    # ė1 = 0.5*(e0*wx + e2*wz - e3*wy)
    F_c[10,  9] =  0.5*wx_   # ∂ė1/∂e0
    F_c[10, 11] =  0.5*wz_   # ∂ė1/∂e2
    F_c[10, 12] = -0.5*wy_   # ∂ė1/∂e3
    F_c[10, 13] =  0.5*e0_   # ∂ė1/∂wx
    F_c[10, 14] = -0.5*e3_   # ∂ė1/∂wy
    F_c[10, 15] =  0.5*e2_   # ∂ė1/∂wz

    # ── ∂ė2/∂(e0,e1,e3, wx,wy,wz) ───────────────────────────────────────
    # ė2 = 0.5*(e3*wx + e0*wy - e1*wz)
    F_c[11,  9] =  0.5*wy_   # ∂ė2/∂e0
    F_c[11, 10] = -0.5*wz_   # ∂ė2/∂e1
    F_c[11, 12] =  0.5*wx_   # ∂ė2/∂e3
    F_c[11, 13] =  0.5*e3_   # ∂ė2/∂wx
    F_c[11, 14] =  0.5*e0_   # ∂ė2/∂wy
    F_c[11, 15] = -0.5*e1_   # ∂ė2/∂wz

    # ── ∂ė3/∂(e0,e1,e2, wx,wy,wz) ───────────────────────────────────────
    # ė3 = 0.5*(-e2*wx + e1*wy + e0*wz)
    F_c[12,  9] =  0.5*wz_   # ∂ė3/∂e0
    F_c[12, 10] =  0.5*wy_   # ∂ė3/∂e1
    F_c[12, 11] = -0.5*wx_   # ∂ė3/∂e2
    F_c[12, 13] = -0.5*e2_   # ∂ė3/∂wx
    F_c[12, 14] =  0.5*e1_   # ∂ė3/∂wy
    F_c[12, 15] =  0.5*e0_   # ∂ė3/∂wz

    # Rows 13-15: ẇ = 0 — all zeros (already initialised)

    # ── STEP 3 — Discrete Jacobian  F_d = I + F_c·dt  (first-order ZOH) ─
    F_d = np.eye(16) + F_c * dt

    # ── STEP 4 — Covariance prediction ────────────────────────────────────
    # P_pred = F_d · P · F_dᵀ + Q
    P_pred = F_d @ P_ekf @ F_d.T + Q

    # ── STEP 5 — Measurement matrix  H  (11×16)  — constant, linear ──────
    # h(x) = [x[2], x[6], x[7], x[8], x[9], x[10], x[11], x[12], x[13], x[14], x[15]]
    #
    #   meas 0  ↔ state 2   (z      — barometer)
    #   meas 1  ↔ state 6   (ax     — body accel X)
    #   meas 2  ↔ state 7   (ay     — body accel Y)
    #   meas 3  ↔ state 8   (az     — body accel Z)
    #   meas 4  ↔ state 9   (e0)
    #   meas 5  ↔ state 10  (e1)
    #   meas 6  ↔ state 11  (e2)
    #   meas 7  ↔ state 12  (e3)
    #   meas 8  ↔ state 13  (wx)
    #   meas 9  ↔ state 14  (wy)
    #   meas 10 ↔ state 15  (wz)
    H = np.zeros((11, 16))
    H[0,   2] = 1.0   # z_baro
    H[1,   6] = 1.0   # ax
    H[2,   7] = 1.0   # ay
    H[3,   8] = 1.0   # az
    H[4,   9] = 1.0   # e0
    H[5,  10] = 1.0   # e1
    H[6,  11] = 1.0   # e2
    H[7,  12] = 1.0   # e3
    H[8,  13] = 1.0   # wx
    H[9,  14] = 1.0   # wy
    H[10, 15] = 1.0   # wz

    # ── STEP 6 — Innovation  ν = meas - H·x_pred  (11,) ──────────────────
    z_pred = np.zeros(11)
    z_pred[0]  = x_pred[2]    # z
    z_pred[1]  = x_pred[6]    # ax
    z_pred[2]  = x_pred[7]    # ay
    z_pred[3]  = x_pred[8]    # az
    z_pred[4]  = x_pred[9]    # e0
    z_pred[5]  = x_pred[10]   # e1
    z_pred[6]  = x_pred[11]   # e2
    z_pred[7]  = x_pred[12]   # e3
    z_pred[8]  = x_pred[13]   # wx
    z_pred[9]  = x_pred[14]   # wy
    z_pred[10] = x_pred[15]   # wz

    innov = measurement - z_pred   # (11,)

    # ── STEP 7 — Innovation covariance  S = H·P_pred·Hᵀ + R  (11×11) ────
    S = H @ P_pred @ H.T + R

    # ── STEP 8 — Kalman gain  K = P_pred·Hᵀ·S⁻¹  (16×11) ───────────────
    # On Arduino: replace np.linalg.inv with an 11×11 Cholesky solver.
    K = P_pred @ H.T @ np.linalg.inv(S)

    # ── STEP 9 — State update  x_upd = x_pred + K·ν ──────────────────────
    K_nu = K @ innov   # (16,)
    x_upd = np.zeros(16)
    x_upd[0]  = x_pred[0]  + K_nu[0]
    x_upd[1]  = x_pred[1]  + K_nu[1]
    x_upd[2]  = x_pred[2]  + K_nu[2]
    x_upd[3]  = x_pred[3]  + K_nu[3]
    x_upd[4]  = x_pred[4]  + K_nu[4]
    x_upd[5]  = x_pred[5]  + K_nu[5]
    x_upd[6]  = x_pred[6]  + K_nu[6]
    x_upd[7]  = x_pred[7]  + K_nu[7]
    x_upd[8]  = x_pred[8]  + K_nu[8]
    x_upd[9]  = x_pred[9]  + K_nu[9]
    x_upd[10] = x_pred[10] + K_nu[10]
    x_upd[11] = x_pred[11] + K_nu[11]
    x_upd[12] = x_pred[12] + K_nu[12]
    x_upd[13] = x_pred[13] + K_nu[13]
    x_upd[14] = x_pred[14] + K_nu[14]
    x_upd[15] = x_pred[15] + K_nu[15]

    # Renormalise quaternion after update
    q_n2 = np.sqrt(x_upd[9]**2 + x_upd[10]**2 + x_upd[11]**2 + x_upd[12]**2)
    if q_n2 > 1e-12:
        x_upd[9]  /= q_n2;  x_upd[10] /= q_n2
        x_upd[11] /= q_n2;  x_upd[12] /= q_n2

    # ── STEP 10 — Covariance update (Joseph form) ─────────────────────────
    # P_upd = (I - K·H)·P_pred·(I - K·H)ᵀ + K·R·Kᵀ
    I_KH  = np.eye(16) - K @ H
    P_upd = I_KH @ P_pred @ I_KH.T + K @ R @ K.T

    return x_upd, P_upd


# ── GUIDANCE LAW ───────────────────────────────────────────────────────────
def compute_guidance(state, target_enu, sim_time):
    """
    Proportional Navigation guidance law.
    Accepts either the raw RocketPy state vector or a KF-estimated state
    of the same layout: [x,y,z, vx,vy,vz, e0,e1,e2,e3, w1,w2,w3].

    Returns (cm_cmd, cn_cmd, pos_error_3d, pos_error_lateral_2d).

    """

    x,y,z         = state[0],state[1],state[2]
    vx,vy,vz      = state[3],state[4],state[5]
    e0,e1,e2,e3   = state[9],state[10],state[11],state[12]

    tau = 0.04  # ~100 ms
    x = x + vx * tau
    y = y + vy * tau
    z = z + vz * tau

    speed = np.linalg.norm([vx,vy,vz])

    # Full position error (always computed — used for miss logging)
    r = target_enu - np.array([x, y, z])

    # Suppress guidance on rail — but still
    # return real position error so miss logging is never zero.
    if speed < 20.0:               return 0., 0.

    R_norm = np.linalg.norm(r)

    print(f"[guidance] R_norm: {R_norm} | r[0]: {r[0]} | r[1]: {r[1]} | r[2]: {r[2]}")

    V_norm = np.linalg.norm([vx, vy, vz])

    # Line-of-sight unit vector and rate
    los      = r / R_norm
    los_rate = np.cross(r, np.array([vx,vy,vz])) / (R_norm**2)

    # ── Clamp LOS rate to prevent singularity at close range ──────────────
    LOS_RATE_MAX = 0.5   # rad/s 
    los_rate_mag = np.linalg.norm(los_rate)
    if los_rate_mag > LOS_RATE_MAX:
        los_rate = los_rate * (LOS_RATE_MAX / los_rate_mag)

    # Proportional Navigation (N=4)
    progress = 1 - (R_norm / TOTAL_DISTANCE)
    N     = 5.5 * (1 - progress)

    a_cmd = N * V_norm * np.cross(los_rate, los)

    g_inertial = np.array([0.0, 0.0, -9.81])
    a_cmd = a_cmd + g_inertial

    # Rotate command into body frame
    R_att  = quat_to_R(e0, e1, e2, e3)
    a_body = R_att.T @ a_cmd

    # Dynamic pressure gate — avoid noisy commands at low-q
    rho    = 1.225 * np.exp(-z / 8500)
    q_dyn  = 0.5 * rho * V_norm**2

    moment_scale = q_dyn * REF_AREA * REF_LENGTH

    cm = np.clip( a_body[1] * PITCH_INERTIA / moment_scale, -CM_CLIP, CM_CLIP)
    cn = np.clip(-a_body[0] * PITCH_INERTIA / moment_scale, -CM_CLIP, CM_CLIP)

    return float(cm), float(cn)

# ── ROLL CONTROLLER ────────────────────────────────────────────────────────
_prev_spin_val = 0.0
_prev_roll_t   = None

def compute_roll(state, sim_time):
    """
    PD roll controller locking the rocket to ROLL_SETPOINT degrees.

    Three terms:
      ROLL_KP_ANGLE × roll_angle_error  — eliminates steady roll offset
      ROLL_KP_RATE  × roll_rate         — damps spin
      ROLL_KD       × roll_acceleration — damps oscillation

    state layout: [..., e0,e1,e2,e3, w1, w2, w3]
      state[6:10] — quaternion  → roll angle via rotation matrix
      state[10]   — w1 [rad/s] → roll rate about body-Z (nose axis)
    """
    global _prev_spin_val, _prev_roll_t
    e0, e1, e2, e3 = state[9], state[10], state[11], state[12]
    w1 = state[13]

    # Extract roll angle from rotation matrix
    R_att      = quat_to_R(e0, e1, e2, e3)
    roll_angle = np.degrees(np.arctan2(R_att[2, 1], R_att[2, 2]))

    # Shortest-path roll error, wrapped to [-180, 180] deg

    roll_error = (roll_angle - ROLL_SETPOINT + 180.0) % 360.0 - 180.0

    # Finite-difference roll acceleration
    if _prev_roll_t is not None:
        dt    = max(sim_time - _prev_roll_t, 1e-9)
        w1dot = (w1 - _prev_spin_val) / dt
    else:
        w1dot = 0.0
    _prev_spin_val = w1
    _prev_roll_t   = sim_time

    cant = float(np.clip(
        -(ROLL_KP_ANGLE * roll_error + ROLL_KP_RATE * w1 + ROLL_KD * w1dot),
        -MAX_CANT, MAX_CANT,
    ))
    return cant

###################
# LIVE PLOT SETUP #
###################
def live_plot_setup():
    plt.ion()
    fig_live = plt.figure(figsize=(15, 7))
    gs = fig_live.add_gridspec(1, 2, width_ratios=[2, 1])

    # Left: Trajectory

    ax_live  = fig_live.add_subplot(gs[0, 0], projection="3d")
    ax_live.set_xlabel("x [m] (East)"); ax_live.set_ylabel("y [m] (North)")
    ax_live.set_zlabel("z [m] (Up)");   ax_live.set_title("Nemesis — Live")
    ax_live.set_xlim(-3000,3000); ax_live.set_ylim(-3000,3000); ax_live.set_zlim(0,3500)
    ax_live.set_box_aspect((1,1,1)); ax_live.view_init(elev=25, azim=-60)
    ax_live.scatter(*TARGET_ENU, color="red", s=120, marker="*", zorder=10, label="Waypoint")

    # ── Draw 5m interception sphere ──
    u = np.linspace(0, 2*np.pi, 30)
    v = np.linspace(0, np.pi, 20)

    xs = TARGET_ENU[0] + INTERCEPT_RADIUS * np.outer(np.cos(u), np.sin(v))
    ys = TARGET_ENU[1] + INTERCEPT_RADIUS * np.outer(np.sin(u), np.sin(v))
    zs = TARGET_ENU[2] + INTERCEPT_RADIUS * np.outer(np.ones_like(u), np.cos(v))

    ax_live.plot_surface(xs, ys, zs, color='red', alpha=0.15, linewidth=0)

    ax_live.plot([TARGET_ENU[0]]*2,[TARGET_ENU[1]]*2,[0,TARGET_ENU[2]],"r--",lw=0.8,alpha=0.5)
    ax_live.scatter(0,0,0,color="green",s=80,marker="^",zorder=10,label="Launch")
    ax_live.legend(loc="upper left", fontsize=9)
    _norm = mcolors.Normalize(vmin=0, vmax=1500)
    _sm   = mplcm.ScalarMappable(cmap="plasma_r", norm=_norm); _sm.set_array([])
    fig_live.tight_layout()

    # Right: Rocket attitude + fin cant
    ax_fins = fig_live.add_subplot(gs[0, 1], projection="3d")
    _AX_LIM = 0.40
    ax_fins.set_xlim(-_AX_LIM,_AX_LIM); ax_fins.set_ylim(-_AX_LIM,_AX_LIM); ax_fins.set_zlim(-_AX_LIM,_AX_LIM)
    ax_fins.set_xlabel("East",fontsize=7,labelpad=1)
    ax_fins.set_ylabel("North",fontsize=7,labelpad=1)
    ax_fins.set_zlabel("Up",fontsize=7,labelpad=1)
    ax_fins.tick_params(labelsize=5)
    ax_fins.view_init(elev=20, azim=30)

    return ax_live, ax_fins

# ── CONTROLLER CLASS ────────────────────────────────────────────────────────
class GuidanceController:
    def __init__(self, fin_set, target_enu, ax_live, ax_fins, csv_path=None, csv_hz=50.0):
        self.fin_set    = fin_set
        self.target_enu = target_enu

        self.ax_live = ax_live
        self.ax_fins = ax_fins

        self.log = {k: [] for k in
                    ["t","x","y","z","miss","cm","cn","cant","spin","enx","eny",
                     "ekf_x","ekf_y","ekf_z","ekf_vx","ekf_vy","ekf_vz",
                     # noisy measurements (11 channels)
                     "meas_z","meas_ax","meas_ay","meas_az",
                     "meas_e0","meas_e1","meas_e2","meas_e3",
                     "meas_wx","meas_wy","meas_wz",
                     # EKF estimates of those same quantities
                     "ekf_z","ekf_ax","ekf_ay","ekf_az",
                     "ekf_e0","ekf_e1","ekf_e2","ekf_e3",
                     "ekf_wx","ekf_wy","ekf_wz"]}
        self._last_plot = -99.; self._plot_dt = 1./PLOT_HZ
        self._last_csv  = -99.; self._csv_dt  = 1./csv_hz
        self._cf = self._cw = None
        self.intercepted = False

        # ── EKF persistent state ──────────────────────────────────────────
        # x_ekf: [x, y, z, vx, vy, vz, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
        self.ekf_x = np.zeros(16)
        self.ekf_x[9] = 1.0
        # P: 16×16 initial covariance
        self.ekf_P = np.diag([
            1.0,   1.0,   1.0,    # x,  y,  z      — small, launch is at origin
            1.0,   1.0,   1.0,    # vx, vy, vz     — small, starts at rest
            100.0, 100.0, 100.0,  # ax, ay, az     — large, thrust unknown
            1e-3,  1e-3,  1e-3,  1e-3,  # quaternion — small, known close to identity
            0.1,   0.1,   0.1,   # wx, wy, wz     — moderate
        ])
        self._ekf_prev_t     = None   # timestamp of last EKF call
        self._ekf_prev_state = None   # RocketPy state at previous step (for accel FD)

        # Single Poly3DCollection for all rocket geometry.
        # Matplotlib's 3D engine sorts faces by centroid depth within one
        # collection, so fins will never be buried behind the body.
        # Per-face colours are set via facecolors list in set_verts().
        self.rocket_coll = Poly3DCollection(
            [], alpha=1.0, linewidth=0.4,
            edgecolors='#aaaaaa')
        self.ax_fins.add_collection3d(self.rocket_coll)
        _L = 0.37
        self.ax_fins.plot([0,_L],[0,0],[0,0],'r--',lw=1.0,alpha=0.5)
        self.ax_fins.plot([0,0],[0,_L],[0,0],'g--',lw=1.0,alpha=0.5)
        self.ax_fins.plot([0,0],[0,0],[0,_L],'b--',lw=1.0,alpha=0.5)
        self.ax_fins.text(_L+0.01,0,0,'E',color='red',fontsize=7)
        self.ax_fins.text(0,_L+0.01,0,'N',color='green',fontsize=7)
        self.ax_fins.text(0,0,_L+0.01,'Up',color='blue',fontsize=7)
        self._att_txt = ax_fins.text2D(0.02,0.02,'',transform=ax_fins.transAxes,
            fontsize=7,va='bottom',family='monospace',
            bbox=dict(facecolor='white',alpha=0.80,pad=2))

        if csv_path:
            self._cf = open(csv_path,"w",newline="")
            self._cw = csv.writer(self._cf)
            self._cw.writerow(["t","x","y","z","miss","cm","cn","cant","spin"])
            self._cf.flush()

        # ── Serial port (HIL only) ────────────────────────────────────────
        self._ser = None
        if HIL_ENABLED:
            self._ser = serial.Serial(HIL_PORT, HIL_BAUD, timeout=HIL_TIMEOUT)
            time.sleep(2.0)               # wait for Arduino reboot after DTR toggle
            self._ser.reset_input_buffer()
            print(f"[HIL] Serial open on {HIL_PORT} @ {HIL_BAUD} baud")
        # RTT statistics (HIL only)
        self._rtt_samples = []

        self._lx = []
        self._ly = []
        self._lz = []
        self._lm = []
        _norm = mcolors.Normalize(vmin=0, vmax=1500)
        self._lsc = self.ax_live.scatter([],[],[], c=[], cmap="plasma_r", norm=_norm, s=3, zorder=3)

    def control_loop(self, sim_time, sampling_rate, state,
                     state_history, observed_variables, interactive_objects):
        x, y, z         = state[0], state[1], state[2]
        vx, vy, vz      = state[3], state[4], state[5]
        e0, e1, e2, e3  = state[6], state[7], state[8], state[9]

        speed = np.linalg.norm(np.array([vx, vy, vz]))

        speed = 3.6 * speed

        # ── NOISE INJECTION ────────────────────────────────────────────────────
        if self._ekf_prev_t is None:
            ekf_dt = 1.0 / SAMPLING_RATE
        else:
            ekf_dt = max(sim_time - self._ekf_prev_t, 1e-6)

        ekf_meas = build_noisy_measurement(state, self._ekf_prev_state, ekf_dt)
        # ekf_meas = [z_baro, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]

        self._ekf_prev_state = np.array(state)
        self._ekf_prev_t     = sim_time

        # ── EKF, GUIDANCE & ROLL ────────────────────────────────────────────────
        if self._ser is not None:
            # TX: send 11 floats (ekf_meas) as raw little-endian binary
            _t0 = time.perf_counter()
            self._ser.write(struct.pack("<B11f", 0x03, *ekf_meas))

            # RX: read 3 floats back (cm_cmd, cn_cmd, cant)
            raw = self._ser.read(76)   # 19 × 4 bytes [cm_cmd, cn_cmd, cant, x, y, z, vx, vy, vz, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
            self._rtt_samples.append(time.perf_counter() - _t0)
            cm_cmd, cn_cmd, cant, ekf_state[0], ekf_state[1], ekf_state[2], ekf_state[3], ekf_state[4], ekf_state[5], ekf_state[6], ekf_state[7], ekf_state[8], ekf_state[9], ekf_state[10], ekf_state[11], ekf_state[12], ekf_state[13], ekf_state[14], ekf_state[15] = struct.unpack("<19f", raw)
        else:
            # ── KALMAN FILTER CALL ─────────────────────────────────────────────────
            self.ekf_x, self.ekf_P = ekf_step(
                self.ekf_x, self.ekf_P, ekf_meas, ekf_dt, EKF_Q, EKF_R
            )

            # ekf_state layout: [x,y,z, vx,vy,vz, ax,ay,az, e0,e1,e2,e3, wx,wy,wz]
            ekf_state = self.ekf_x
            cm_cmd, cn_cmd  = compute_guidance(ekf_state, self.target_enu, sim_time)
            cant            = compute_roll(ekf_state, sim_time)

        _gs["cm"] = cm_cmd
        _gs["cn"] = cn_cmd
        self.fin_set.cant_angle = cant

        # ── MISS DISTANCE (horizontal E-N only) ────────────────────────────
        err_full       = self.target_enu - np.array([x, y, z])
        distance_to_target = float(np.linalg.norm(err_full))
        miss           = float(np.linalg.norm(err_full[:2]))

        # ── INTERCEPT CHECK ───────────────────────────────────────────────
        if distance_to_target <= INTERCEPT_RADIUS and not self.intercepted:
            print(f"\n>>> TARGET INTERCEPTED at T+{sim_time:.2f}s "
                f"(distance = {distance_to_target:.2f} m) <<<\n")
            self.intercepted = True

        # ── TELEMETRY LOG ─────────────────────────────────────────────────
        t = sim_time
        self.log["t"].append(t);         self.log["x"].append(x)
        self.log["y"].append(y);         self.log["z"].append(z)
        self.log["miss"].append(miss);   self.log["cm"].append(cm_cmd)
        self.log["cn"].append(cn_cmd);   self.log["cant"].append(cant)
        self.log["spin"].append(state[10])
        self.log["enx"].append(float(err_full[0]))
        self.log["eny"].append(float(err_full[1]))
        # EKF position and velocity estimates
        self.log["ekf_x"].append(float(ekf_state[0]))
        self.log["ekf_y"].append(float(ekf_state[1]))
        self.log["ekf_z"].append(float(ekf_state[2]))
        self.log["ekf_vx"].append(float(ekf_state[3]))
        self.log["ekf_vy"].append(float(ekf_state[4]))
        self.log["ekf_vz"].append(float(ekf_state[5]))
        # Noisy measurements [z, ax, ay, az, e0, e1, e2, e3, wx, wy, wz]
        self.log["meas_z"].append(float(ekf_meas[0]))
        self.log["meas_ax"].append(float(ekf_meas[1]));  self.log["meas_ay"].append(float(ekf_meas[2]))
        self.log["meas_az"].append(float(ekf_meas[3]))
        self.log["meas_e0"].append(float(ekf_meas[4]));  self.log["meas_e1"].append(float(ekf_meas[5]))
        self.log["meas_e2"].append(float(ekf_meas[6]));  self.log["meas_e3"].append(float(ekf_meas[7]))
        self.log["meas_wx"].append(float(ekf_meas[8]));  self.log["meas_wy"].append(float(ekf_meas[9]))
        self.log["meas_wz"].append(float(ekf_meas[10]))
        # EKF estimates of those same quantities (state indices)
        # state: [x,y,z,vx,vy,vz, ax,ay,az, e0,e1,e2,e3, wx,wy,wz]
        #          0 1 2  3  4  5   6  7  8   9 10 11 12  13 14 15
        self.log["ekf_ax"].append(float(ekf_state[6]));  self.log["ekf_ay"].append(float(ekf_state[7]))
        self.log["ekf_az"].append(float(ekf_state[8]))
        self.log["ekf_e0"].append(float(ekf_state[9]));  self.log["ekf_e1"].append(float(ekf_state[10]))
        self.log["ekf_e2"].append(float(ekf_state[11])); self.log["ekf_e3"].append(float(ekf_state[12]))
        self.log["ekf_wx"].append(float(ekf_state[13])); self.log["ekf_wy"].append(float(ekf_state[14]))
        self.log["ekf_wz"].append(float(ekf_state[15]))

        # ── LIVE ATTITUDE DISPLAY ─────────────────────────────────────────
        R_att  = quat_to_R(e0, e1, e2, e3)
        cyl_f  = apply_rotation(get_cylinder_polygons(),               R_att)
        nose_f = apply_rotation(get_nose_polygons(),                   R_att)
        tail_f = apply_rotation(get_tail_polygons(),                   R_att)
        fin_f  = apply_rotation(get_fin_polygons(PLOTSCALE_FINS*cant), R_att)
        all_verts  = cyl_f + nose_f + tail_f + fin_f
        all_colors = (
            ['#aaaaaa']     * len(cyl_f)  +
            ['#aaaaaa']  * len(nose_f) +
            ['#aaaaaa']     * len(tail_f) +
            ['deepskyblue'] * len(fin_f)
        )
        self.rocket_coll.set_verts(all_verts)
        self.rocket_coll.set_facecolor(all_colors)
        _p = float(np.degrees(np.arcsin(np.clip(-R_att[2, 0], -1., 1.))))
        _y = float(np.degrees(np.arctan2(R_att[1, 0], R_att[0, 0])))
        _r = float(np.degrees(np.arctan2(R_att[2, 1], R_att[2, 2])))

        ax_fins.set_title('Cant {:+.2f} deg'.format(cant), fontsize=8)

        if (t - self._last_csv) >= self._csv_dt and self._cw:
            self._last_csv = t
            self._cw.writerow([f"{v:.4f}" for v in
                                [t, x, y, z, miss, cm_cmd, cn_cmd, cant, state[10]]])
            self._cf.flush()

        if (t - self._last_plot) >= self._plot_dt:
            self._last_plot = t
            self._lx.append(x); self._ly.append(y); self._lz.append(z); self._lm.append(miss)
            self._lsc._offsets3d = (np.array(self._lx), np.array(self._ly), np.array(self._lz))
            self._lsc.set_array(np.array(self._lm))

            if self.intercepted:
                self.ax_live.set_title(f"T+{t:.1f}s | z: {z:.0f} m | speed: {speed:.1f} kmh | MISSION SUCCESSFUL | "
                                  f"cm: {cm_cmd:.2f} cn: {cn_cmd:.2f}", fontsize=10)
            elif self._ser is not None:
                self.ax_live.set_title(f"T+{t:.1f}s | z: {z:.0f} m | speed: {speed:.1f} kmh | DTT: {distance_to_target:.0f}m | "
                                  f"cm: {cm_cmd:.2f} cn: {cn_cmd:.2f} | Source: HIL", fontsize=10)
            else:
                self.ax_live.set_title(f"T+{t:.1f}s | z: {z:.0f} m | speed: {speed:.1f} kmh | DTT: {distance_to_target:.0f}m | "
                                  f"cm: {cm_cmd:.2f} cn: {cn_cmd:.2f} | Source: SIM", fontsize=10)

            plt.pause(0.001)
        return 0.0

    def close(self):
        if self._cf: self._cf.close(); self._cf = self._cw = None
        if self._ser is not None:
            self._ser.close()
            self._ser = None
        if self._rtt_samples:
            rtt = np.array(self._rtt_samples) * 1e3   # → milliseconds
            print(f"\n[HIL] Round-trip time over {len(rtt)} samples:")
            print(f"      mean   = {rtt.mean():.3f} ms")
            print(f"      std    = {rtt.std():.3f} ms")
            print(f"      var    = {rtt.var():.4f} ms²")
            print(f"      min    = {rtt.min():.3f} ms")
            print(f"      max    = {rtt.max():.3f} ms")

def _null_drag(d, mach): return 0.0

# ── POST-FLIGHT ─────────────────────────────────────────────────────────────
def plot_results(log, tenu):
    t    = np.array(log["t"]);   z    = np.array(log["z"])
    x    = np.array(log["x"]);   y    = np.array(log["y"])
    miss = np.array(log["miss"]); cm   = np.array(log["cm"])
    cn   = np.array(log["cn"]);  cant = np.array(log["cant"])
    spin = np.degrees(np.array(log["spin"]))
    enx  = np.array(log["enx"]); eny  = np.array(log["eny"])
    ekf_x  = np.array(log["ekf_x"]);  ekf_y  = np.array(log["ekf_y"])
    ekf_z  = np.array(log["ekf_z"])
    ekf_vx = np.array(log["ekf_vx"]); ekf_vy = np.array(log["ekf_vy"])
    ekf_vz = np.array(log["ekf_vz"])

    # 3-D trajectory
    fig1 = plt.figure(figsize=(12, 8))
    ax   = fig1.add_subplot(111, projection="3d")
    sc   = ax.scatter(x, y, z, c=miss, cmap="plasma_r",
                      vmin=0, vmax=np.percentile(miss, 95), s=2, zorder=3)
    ax.scatter(*tenu, color="red", s=140, marker="*", zorder=10, label="Waypoint")
    u = np.linspace(0, 2*np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    xs = tenu[0] + INTERCEPT_RADIUS * np.outer(np.cos(u), np.sin(v))
    ys = tenu[1] + INTERCEPT_RADIUS * np.outer(np.sin(u), np.sin(v))
    zs = tenu[2] + INTERCEPT_RADIUS * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(xs, ys, zs, color='red', alpha=0.15, linewidth=0)
    ax.plot([tenu[0]]*2, [tenu[1]]*2, [0, tenu[2]], "r--", lw=0.8, alpha=0.5)
    ax.scatter(0, 0, 0, color="green", s=80, marker="^", zorder=10, label="Launch")
    ax.plot(x, y, np.zeros_like(z), color="gray", lw=0.6, alpha=0.4)
    idx = int(np.argmax(z))
    ax.scatter(x[idx], y[idx], z[idx], color="orange", s=80, marker="D",
               zorder=10, label=f"Apogee {z[idx]:.0f}m")
    ax.set_xlabel("x [m] (East)"); ax.set_ylabel("y [m] (North)")
    ax.set_zlabel("z [m] (Up)")
    ax.set_title("Nemesis — Waypoint Guidance (GenericSurface PN)")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=25, azim=-60)
    plt.tight_layout()

    # ── Noisy measurement vs EKF estimate — all 11 channels ───────────────
    channels = [
        # (log key noisy,  log key ekf,   title,                  y-label)
        ("meas_z",  "ekf_z",  "Altitude z",         "m"),
        ("meas_ax", "ekf_ax", "Body accel X (ax)",  "m/s²"),
        ("meas_ay", "ekf_ay", "Body accel Y (ay)",  "m/s²"),
        ("meas_az", "ekf_az", "Body accel Z (az)",  "m/s²"),
        ("meas_e0", "ekf_e0", "Quaternion e0",      "–"),
        ("meas_e1", "ekf_e1", "Quaternion e1",      "–"),
        ("meas_e2", "ekf_e2", "Quaternion e2",      "–"),
        ("meas_e3", "ekf_e3", "Quaternion e3",      "–"),
        ("meas_wx", "ekf_wx", "Angular rate wx",    "rad/s"),
        ("meas_wy", "ekf_wy", "Angular rate wy",    "rad/s"),
        ("meas_wz", "ekf_wz", "Angular rate wz",    "rad/s"),
    ]

    fig4, axes4 = plt.subplots(4, 3, figsize=(18, 14), sharex=True)
    fig4.suptitle("EKF: Noisy Measurement vs Filter Estimate", fontsize=13)
    axes4_flat = axes4.flat

    for ax, (mk, ek, title, unit) in zip(axes4_flat, channels):
        m_arr = np.array(log[mk])
        e_arr = np.array(log[ek])
        ax.plot(t, m_arr, color="steelblue", lw=0.7, alpha=0.6, label="Noisy")
        ax.plot(t, e_arr, color="tomato",    lw=1.4,             label="EKF")
        ax.set_title(title, fontsize=9)
        ax.set_ylabel(unit,  fontsize=8)
        ax.grid(True, ls="--", alpha=0.4)
        ax.legend(fontsize=7, loc="upper right")

    # hide the unused 12th panel
    # next(axes4_flat).set_visible(False)

    for ax in axes4[-1]: ax.set_xlabel("Time [s]", fontsize=8)
    plt.tight_layout()

if __name__ == "__main__":
    
    ###############
    # ENVIRONMENT #
    ###############

    env = Environment(latitude=LAUNCH_LAT, longitude=LAUNCH_LON, elevation=LAUNCH_ALT)
    env.set_date((2026,2,19,12))
    env.set_atmospheric_model(
        type="custom_atmosphere",
        wind_u=[(0,0),(4500,0)],
        wind_v=[(0,0),(4500,0)],
    )
    env.max_expected_height = 4500

    #########
    # MOTOR #
    #########

    Pro75M8187 = SolidMotor(
        thrust_source    = str(BASE_DIR/"Cesaroni_8187M1545_P.csv"),
        dry_mass=0, dry_inertia=(0,0,0),
        nozzle_radius=29e-3, grain_number=6, grain_density=1758.7,
        grain_outer_radius=35.9e-3, grain_initial_inner_radius=18.1e-3,
        grain_initial_height=156.17e-3, grain_separation=3e-3,
        grains_center_of_mass_position=-0.7343,
        center_of_dry_mass_position=0, nozzle_position=-1.296,
        burn_time=5.3, throat_radius=20e-3,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )

    ##########
    # ROCKET #
    ##########

    Nemesis = Rocket(
        radius=ROCKET_RADIUS, mass=ROCKET_MASS,
        inertia=(PITCH_INERTIA, PITCH_INERTIA, 0.078),
        power_off_drag=str(BASE_DIR/"Nemesis150_v4.0_RAS_CDMACH_pwrOFF.csv"),
        power_on_drag =str(BASE_DIR/"Nemesis150_v4.0_RAS_CDMACH_pwrON.csv"),
        center_of_mass_without_motor=0,
        coordinate_system_orientation="tail_to_nose",
    )

    Nemesis.set_rail_buttons(upper_button_position=0.980,
                            lower_button_position=-0.239, angular_position=0)
    Nemesis.add_motor(Pro75M8187, position=0)
    Nemesis.add_nose(length=0.45, kind="vonKarman", position=1.635)

    fin_set = Nemesis.add_trapezoidal_fins(
        n=3, root_chord=0.30, tip_chord=0.093, span=0.16,
        position=-0.855, cant_angle=0.0, sweep_angle=58,
    )
    Nemesis.add_tail(top_radius=0.075, bottom_radius=0.046,
                    length=0.116, position=-1.155)

    # Guidance surface: cm/cn direct moments, placed at CDM (z=0)
    # CP at CDM means no extra moment from cross-product — pure direct moments only.
    # This avoids sign ambiguity from cp × F and makes cm/cn the sole steering input.

    guidance_surface = GenericSurface(
        reference_area   = REF_AREA,
        reference_length = REF_LENGTH,
        coefficients     = {"cm": _cm_func, "cn": _cn_func},
        center_of_pressure = (0, 0, 0),
        name             = "GuidanceSurface",
    )
    Nemesis.add_surfaces(guidance_surface, positions=(0, 0, -1))

    ###################
    # LIVE PLOT SETUP #
    ###################

    ax_live, ax_fins = live_plot_setup()

    ###########
    # WIRE UP #
    ###########

    ctrl = GuidanceController(fin_set, TARGET_ENU, ax_live, ax_fins, csv_path=str(BASE_DIR/"poc_telemetry.csv"), csv_hz=PLOT_HZ)

    Nemesis.add_air_brakes(
        drag_coefficient_curve = _null_drag,
        controller_function    = ctrl.control_loop,
        sampling_rate          = SAMPLING_RATE,
        clamp                  = True,
    )

    ########################################
    #   SENDING SIGNAL TO START SIMULATION #
    ########################################

    if ctrl._ser is not None:
        ctrl._ser.write(struct.pack("<B3f", 0x01, TARGET_ENU[0], TARGET_ENU[1], TARGET_ENU[2]))
        ack = ctrl._ser.read(1)
        if ack == b'\x88':
            print("[HIL] Connection confirmed (0x88 received)")
        else:
            raise RuntimeError(f"[HIL] Unexpected ACK: {ack.hex() if ack else 'timeout'}")

    ##########
    # FLIGHT #
    ##########

    inclination = np.atan(2 * (TARGET_ENU[2] / np.linalg.norm(np.array(TARGET_ENU[0], TARGET_ENU[1])))) * (180.0 / np.pi)
    heading = np.atan(TARGET_ENU[0] / TARGET_ENU[1]) * (180.0 / np.pi)

    if TARGET_ENU[1] < 0:
        heading = -heading

    print(f"inclination: {inclination:.2f} | heading: {heading:.2f}")

    print("\nRunning simulation …\n")
    test_flight = Flight(
        rocket      = Nemesis,
        environment = env,
        rail_length = 12,
        inclination = inclination,
        heading     = heading,
    )

    #########################################
    #   SENDING SIGNAL TO STOP SIMULATION   #
    #########################################

    if ctrl._ser is not None:
        ctrl._ser.write(struct.pack("<B", 0x02))

    ctrl.close()
    plt.ioff()

    plot_results(ctrl.log, TARGET_ENU)
    plt.show()
    Nemesis.draw()

    test_flight.export_kml(str(BASE_DIR/"trajectory_poc.kml"),
                        extrude=True, altitude_mode="relative_to_ground")
    print("\n>>> DONE <<<")
