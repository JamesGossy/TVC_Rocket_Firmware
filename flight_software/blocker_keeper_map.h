/*
 * blocker_keeper_map.h  -  made by tools/make_flight_maps.py from vehicle/vehicle.json (the v3 CAD).
 * Do not edit by hand: change the CAD or the script and run it again.
 *
 * Blocker: share of the full-close blocking (0 = claws open, 1 = fully closed) -> servo horn angle (deg).
 * Full close blocks 1 - throttle_min of the landing motor's thrust (rocket.cfg), so the flight computer
 * works out the share from the throttle it wants. Both claws get the same command.
 * Keeper: hold angle, and the release angle that drives the ring onto its hard stop (the pegs let go at
 * about 9.1 deg of ring, the stop is at 9.5 deg; a horn angle of 62 deg asks for 9.62 deg).
 */
#ifndef BLOCKER_KEEPER_MAP_H
#define BLOCKER_KEEPER_MAP_H

#define BLOCKER_OPEN_DEG -80.0f
#define BLOCKER_N 21
static const float BLOCKER_SERVO_DEG[BLOCKER_N] = {
    -80.00f, -52.12f, -49.61f, -47.44f, -45.40f, -43.45f, -41.66f,
    -39.78f, -37.86f, -36.06f, -34.22f, -32.21f, -30.14f, -28.03f,
    -25.79f, -23.39f, -20.70f, -17.74f, -14.41f, -10.13f, -0.50f,
};

#define KEEPER_HOLD_DEG 0.0f
#define KEEPER_RELEASE_DEG 62.0f

/* Horn angle for the share of full-close blocking wanted (0 to 1) */
static inline float blocker_servo_for_share(float share)
{
    if (share <= 0.0f) return BLOCKER_OPEN_DEG;
    if (share >= 1.0f) return BLOCKER_SERVO_DEG[BLOCKER_N - 1];
    float f = share * (BLOCKER_N - 1);
    int i = (int)f;
    float a = f - i;
    return BLOCKER_SERVO_DEG[i] * (1 - a) + BLOCKER_SERVO_DEG[i + 1] * a;
}

#endif
