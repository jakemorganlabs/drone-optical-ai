// Phase 1 pilot_main stub. The real onboard main loop (manual Part 4.5)
// composes DronePoseProvider + LiveVoxelMapper + FcBridge + CfCPilot. For
// Phase 1 this stub just instantiates the perception provider and exits 0;
// the FcBridge TU is linked in by the build system when MAVSDK is available
// (Part 2/3), and CfCPilot/ONNX land in Part 4. The stub's job now is to give
// the new perception translation unit a standalone link entry point so the
// two-TU ODR gate is implicitly exercised, and to mirror the shape of the
// Part 9.2 CMake `pilot_main` target.

#include "perception/drone_pose_provider.hpp"

#include <iostream>

int main() {
    DronePoseProvider provider(640, 480, 60.0f);
    std::cout << "pilot_main stub: provider active? "
              << (provider.is_active() ? "yes" : "no") << "\n";
    return 0;
}