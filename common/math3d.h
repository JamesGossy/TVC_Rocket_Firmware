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

// A 3x3 matrix, for inertia tensors and the rotations of the moving parts of the rocket.
struct Mat3 {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

inline Mat3 zeroMatrix() {
    Mat3 a;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) a.m[i][j] = 0;
    return a;
}
inline Mat3 operator+(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][j] + b.m[i][j];
    return r;
}
inline Mat3 operator-(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][j] - b.m[i][j];
    return r;
}
inline Mat3 operator*(const Mat3& a, double s) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][j] * s;
    return r;
}
inline Mat3& operator+=(Mat3& a, const Mat3& b) { a = a + b; return a; }
inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r = zeroMatrix();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}
inline Vec3 operator*(const Mat3& a, Vec3 v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
            a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}
inline Mat3 transpose(const Mat3& a) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
    return r;
}
inline Mat3 diagonal(double x, double y, double z) {
    Mat3 r = zeroMatrix();
    r.m[0][0] = x; r.m[1][1] = y; r.m[2][2] = z;
    return r;
}

// Inverse by cofactors (the inertia tensors here are well conditioned)
inline Mat3 inverse(const Mat3& a) {
    const auto& m = a.m;
    Mat3 c;
    c.m[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    c.m[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
    c.m[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];
    c.m[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    c.m[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
    c.m[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
    c.m[2][0] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    c.m[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
    c.m[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    double det = m[0][0] * c.m[0][0] + m[0][1] * c.m[1][0] + m[0][2] * c.m[2][0];
    return c * (1.0 / det);
}

// Rotation about an axis (right-handed), as a matrix
inline Mat3 rotationMatrix(Vec3 axis, double angle) {
    axis = normalize(axis);
    double c = std::cos(angle), s = std::sin(angle), t = 1 - c;
    Mat3 r;
    r.m[0][0] = t * axis.x * axis.x + c;          r.m[0][1] = t * axis.x * axis.y - s * axis.z; r.m[0][2] = t * axis.x * axis.z + s * axis.y;
    r.m[1][0] = t * axis.x * axis.y + s * axis.z; r.m[1][1] = t * axis.y * axis.y + c;          r.m[1][2] = t * axis.y * axis.z - s * axis.x;
    r.m[2][0] = t * axis.x * axis.z - s * axis.y; r.m[2][1] = t * axis.y * axis.z + s * axis.x; r.m[2][2] = t * axis.z * axis.z + c;
    return r;
}

// Rotation matrix of the smallest turn that takes direction a to direction b
inline Mat3 rotationBetweenMatrix(Vec3 a, Vec3 b) {
    a = normalize(a);
    b = normalize(b);
    Vec3 axis = cross(a, b);
    if (length(axis) < 1e-12) return Mat3{};
    return rotationMatrix(axis, std::acos(std::clamp(dot(a, b), -1.0, 1.0)));
}

// Inertia a point mass adds about a point d away from it (parallel axis theorem)
inline Mat3 parallelAxis(double mass, Vec3 d) {
    Mat3 r = diagonal(1, 1, 1) * (dot(d, d) * mass);
    double v[3] = {d.x, d.y, d.z};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] -= mass * v[i] * v[j];
    return r;
}
