#pragma once
#include <algorithm>
#include <cmath>

// Small vector and quaternion helpers. Just enough for the sim, and simple
// enough to also run on the flight computer.

const double PI = 3.14159265358979323846;
const double DEG = PI / 180.0;  // degrees * DEG = radians
const double GRAVITY = 9.81;

struct Vec3 {
    double x = 0, y = 0, z = 0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator/(Vec3 a, double s) { return {a.x / s, a.y / s, a.z / s}; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }

inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }

inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline Vec3 normalize(Vec3 a) {
    double len = length(a);
    return len > 0 ? a / len : a;
}

// A rotation. In this project it always turns body frame vectors into world frame vectors.
struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
};

inline Quat operator*(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quat normalize(Quat q) {
    double len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / len, q.x / len, q.y / len, q.z / len};
}

inline Quat conjugate(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }

// Body frame -> world frame
inline Vec3 rotate(Quat q, Vec3 v) {
    Quat p = q * Quat{0, v.x, v.y, v.z} * conjugate(q);
    return {p.x, p.y, p.z};
}

// World frame -> body frame
inline Vec3 rotateBack(Quat q, Vec3 v) { return rotate(conjugate(q), v); }

inline Quat fromAxisAngle(Vec3 axis, double angle) {
    axis = normalize(axis);
    double s = std::sin(angle / 2);
    return {std::cos(angle / 2), axis.x * s, axis.y * s, axis.z * s};
}

// Smallest rotation that turns direction a into direction b
inline Quat rotationBetween(Vec3 a, Vec3 b) {
    a = normalize(a);
    b = normalize(b);
    Vec3 axis = cross(a, b);
    if (length(axis) < 1e-9) return Quat{};  // already lined up
    double angle = std::acos(std::clamp(dot(a, b), -1.0, 1.0));
    return fromAxisAngle(axis, angle);
}

// How far the nose is tipped away from straight up, in degrees
inline double tiltDegrees(Quat attitude) {
    Vec3 nose = rotate(attitude, {1, 0, 0});
    return std::acos(std::clamp(nose.z, -1.0, 1.0)) / DEG;
}
