#pragma once

#include <cmath>
#include <limits>

//----------------------------------------------
// Math Utilities
//----------------------------------------------
struct Vec3 {
    float x, y, z;
    
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vec3 operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }
};

struct Mat3 {
    float m[9];
    
    Mat3() {
        for(int i = 0; i < 9; i++) m[i] = 0.0f;
        m[0] = m[4] = m[8] = 1.0f; // Identity matrix
    }
};

//----------------------------------------------
// Math Helpers
//----------------------------------------------
static inline float deg2rad(float deg) {
    return deg * 3.14159265358979323846f / 180.0f;
}

static inline float rad2deg(float rad) {
    return rad * 180.0f / 3.14159265358979323846f;
}

static inline Vec3 normalize(const Vec3 &v) {
    float len = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if(len < 1e-12f) {
        return {0.f, 0.f, 0.f};
    }
    return { v.x/len, v.y/len, v.z/len };
}

static inline float dot(const Vec3 &a, const Vec3 &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline float length(const Vec3 &v) {
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static inline float length_squared(const Vec3 &v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

// Multiply 3x3 matrix by Vec3
static inline Vec3 mat3_mul_vec3(const Mat3 &M, const Vec3 &v) {
    Vec3 r;
    r.x = M.m[0]*v.x + M.m[1]*v.y + M.m[2]*v.z;
    r.y = M.m[3]*v.x + M.m[4]*v.y + M.m[5]*v.z;
    r.z = M.m[6]*v.x + M.m[7]*v.y + M.m[8]*v.z;
    return r;
}

// Transpose a 3x3 row-major matrix (needed for world<-camera rotations)
static inline Mat3 transpose(const Mat3 &M) {
    Mat3 t;
    t.m[0] = M.m[0]; t.m[1] = M.m[3]; t.m[2] = M.m[6];
    t.m[3] = M.m[1]; t.m[4] = M.m[4]; t.m[5] = M.m[7];
    t.m[6] = M.m[2]; t.m[7] = M.m[5]; t.m[8] = M.m[8];
    return t;
}

//----------------------------------------------
// Euler -> Rotation Matrix
//----------------------------------------------
inline Mat3 rotation_matrix_yaw_pitch_roll(float yaw_deg, float pitch_deg, float roll_deg) {
    float y = deg2rad(yaw_deg);
    float p = deg2rad(pitch_deg);
    float r = deg2rad(roll_deg);

    // Build each sub-rotation
    // Rz(yaw)
    float cy = std::cos(y), sy = std::sin(y);
    float Rz[9] = {
        cy, -sy, 0.f,
        sy,  cy, 0.f,
        0.f, 0.f, 1.f
    };

    // Ry(roll)
    float cr = std::cos(r), sr = std::sin(r);
    float Ry[9] = {
        cr,  0.f, sr,
        0.f, 1.f, 0.f,
        -sr, 0.f, cr
    };

    // Rx(pitch)
    float cp = std::cos(p), sp = std::sin(p);
    float Rx[9] = {
        1.f,  0.f,  0.f,
        0.f,  cp,  -sp,
        0.f,  sp,   cp
    };

    // Helper to multiply 3x3
    auto matmul3x3 = [&](const float A[9], const float B[9], float C[9]){
        for(int row=0; row<3; ++row) {
            for(int col=0; col<3; ++col) {
                C[row*3+col] =
                    A[row*3+0]*B[0*3+col] +
                    A[row*3+1]*B[1*3+col] +
                    A[row*3+2]*B[2*3+col];
            }
        }
    };

    float Rtemp[9], Rfinal[9];
    matmul3x3(Rz, Ry, Rtemp);    // Rz * Ry
    matmul3x3(Rtemp, Rx, Rfinal); // (Rz*Ry)*Rx

    Mat3 out;
    for(int i=0; i<9; i++){
        out.m[i] = Rfinal[i];
    }
    return out;
}

//----------------------------------------------
// Focal Length Calculation
//----------------------------------------------
inline float compute_focal_length(int image_width, float fov_degrees) {
    float fov_rad = deg2rad(fov_degrees);
    return (image_width * 0.5f) / std::tan(fov_rad * 0.5f);
}

//----------------------------------------------
// Safe Division
//----------------------------------------------
static inline float safe_div(float num, float den) {
    float eps = 1e-12f;
    if(std::fabs(den) < eps) {
        return std::numeric_limits<float>::infinity();
    }
    return num / den;
}
