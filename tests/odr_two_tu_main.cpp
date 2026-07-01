// First translation unit of the ODR regression test. Includes every header
// and calls the same free functions the real mapper uses. If any header-defined
// free function is NOT marked inline, the linker will produce multiple-definition
// errors when this is linked with odr_two_tu_other.cpp.
#include "math.hpp"
#include "grid.hpp"
#include "pose.hpp"
#include "motion.hpp"
#include "ray_marching.hpp"
#include "clustering.hpp"
#include "navigation.hpp"

#include <iostream>

// Defined in odr_two_tu_other.cpp
int other_tu_force_emit();

int main() {
    Mat3 r = rotation_matrix_yaw_pitch_roll(10.0f, 5.0f, -3.0f);
    float f = compute_focal_length(640, 60.0f);
    ImageGray a(4, 4), b(4, 4);
    MotionMask mm = detect_motion(a, b, 0.5f);
    MotionStats st = compute_motion_stats(mm);
    (void)r; (void)f; (void)mm; (void)st;

    int other = other_tu_force_emit();
    if(other == -1) return 1;

    std::cout << "ODR two-TU link test passed.\n";
    return 0;
}