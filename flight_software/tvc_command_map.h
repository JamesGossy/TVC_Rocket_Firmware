/*
 * tvc_command_map.h  -  TVC_Rocket_Gimbal_v3
 *
 * 1) Clips the requested gimbal tilt to an 8 deg CIRCLE (not per-axis),
 *    keeping the cartridge ~0.75 deg clear of the 8.75 deg hard-stop ring.
 * 2) Maps (yaw, pitch) gimbal angles to servo horn angles using a 2-D table
 *    generated from the v3 CAD kinematics (horns clocked -14.47 deg,
 *    pushrods 33.5 mm c-c). The table includes the small cross-coupling
 *    between axes. Re-measure on the real hardware and replace the table.
 *
 * Conventions (match Fusion joint names):
 *   yaw   = rotation of outer ring about +Y   (deg)
 *   pitch = rotation of cartridge about +X    (deg)
 *   yaw servo   +  -> yaw   +
 *   pitch servo +  -> pitch -
 *   Servo angles are relative to the CAD neutral (horn clocked, pushrod set to 33.5 mm).
 */
#ifndef TVC_COMMAND_MAP_H
#define TVC_COMMAND_MAP_H
#include <math.h>

#define TVC_MAX_TILT_DEG 8.0f

/* Scale the (yaw, pitch) vector so its magnitude never exceeds TVC_MAX_TILT_DEG. */
static inline void tvc_clip_circle(float *yaw_deg, float *pitch_deg)
{
    float m = sqrtf((*yaw_deg) * (*yaw_deg) + (*pitch_deg) * (*pitch_deg));
    if (m > TVC_MAX_TILT_DEG) {
        float k = TVC_MAX_TILT_DEG / m;
        *yaw_deg *= k;
        *pitch_deg *= k;
    }
}

/* Grid: -8..8 deg in 2 deg steps. Row index = pitch, column index = yaw. */
static const float TVC_GRID0 = -8.0f, TVC_GRID_STEP = 2.0f;
#define TVC_N 9

static const float TVC_YAW_SERVO[TVC_N][TVC_N] = {
    {-38.27f,-27.74f,-17.53f, -7.49f,  2.49f, 12.55f, 22.87f, 33.71f, 45.55f},
    {-39.95f,-29.16f,-18.78f, -8.63f,  1.42f, 11.51f, 21.81f, 32.59f, 44.27f},
    {-41.30f,-30.26f,-19.72f, -9.47f,  0.64f, 10.76f, 21.06f, 31.79f, 43.35f},
    {-42.27f,-31.02f,-20.34f,-10.00f,  0.17f, 10.31f, 20.61f, 31.31f, 42.78f},
    {-42.84f,-31.41f,-20.62f,-10.21f,  0.00f, 10.16f, 20.46f, 31.13f, 42.55f},
    {-42.98f,-31.43f,-20.56f,-10.10f,  0.13f, 10.31f, 20.61f, 31.26f, 42.64f},
    {-42.68f,-31.05f,-20.14f, -9.66f,  0.58f, 10.76f, 21.05f, 31.70f, 43.07f},
    {-41.92f,-30.28f,-19.37f, -8.90f,  1.33f, 11.50f, 21.79f, 32.43f, 43.81f},
    {-40.71f,-29.13f,-18.26f, -7.82f,  2.39f, 12.54f, 22.82f, 33.47f, 44.88f},
};

static const float TVC_PITCH_SERVO[TVC_N][TVC_N] = {
    { 44.62f, 43.74f, 43.11f, 42.71f, 42.55f, 42.61f, 42.90f, 43.41f, 44.15f},
    { 33.10f, 32.25f, 31.64f, 31.27f, 31.13f, 31.23f, 31.56f, 32.13f, 32.92f},
    { 22.48f, 21.60f, 20.97f, 20.59f, 20.46f, 20.58f, 20.96f, 21.58f, 22.45f},
    { 12.36f, 11.40f, 10.71f, 10.30f, 10.16f, 10.30f, 10.71f, 11.40f, 12.35f},
    {  2.49f,  1.42f,  0.64f,  0.17f,  0.00f,  0.13f,  0.58f,  1.33f,  2.39f},
    { -7.28f, -8.51f, -9.41f, -9.98f,-10.21f,-10.09f, -9.61f, -8.79f, -7.61f},
    {-17.04f,-18.49f,-19.58f,-20.30f,-20.62f,-20.54f,-20.05f,-19.15f,-17.84f},
    {-26.89f,-28.65f,-30.00f,-30.94f,-31.41f,-31.41f,-30.92f,-29.94f,-28.46f},
    {-36.92f,-39.10f,-40.85f,-42.11f,-42.84f,-42.99f,-42.53f,-41.45f,-39.76f},
};

static inline float tvc_bilerp(const float t[TVC_N][TVC_N], float yaw, float pitch)
{
    float fx = (yaw - TVC_GRID0) / TVC_GRID_STEP, fy = (pitch - TVC_GRID0) / TVC_GRID_STEP;
    if (fx < 0) fx = 0; if (fx > TVC_N - 1.001f) fx = TVC_N - 1.001f;
    if (fy < 0) fy = 0; if (fy > TVC_N - 1.001f) fy = TVC_N - 1.001f;
    int ix = (int)fx, iy = (int)fy; float ax = fx - ix, ay = fy - iy;
    float a = t[iy][ix] * (1 - ax) + t[iy][ix + 1] * ax;
    float b = t[iy + 1][ix] * (1 - ax) + t[iy + 1][ix + 1] * ax;
    return a * (1 - ay) + b * ay;
}

/* Full pipeline: requested gimbal angles in, servo horn angles out (deg). */
static inline void tvc_command(float yaw_deg, float pitch_deg, float *yaw_servo_deg, float *pitch_servo_deg)
{
    tvc_clip_circle(&yaw_deg, &pitch_deg);
    *yaw_servo_deg   = tvc_bilerp(TVC_YAW_SERVO,   yaw_deg, pitch_deg);
    *pitch_servo_deg = tvc_bilerp(TVC_PITCH_SERVO, yaw_deg, pitch_deg);
}

#endif
