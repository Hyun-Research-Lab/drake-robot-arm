//Entrypoint

#include <gflags/gflags.h>
// #include "aerodynamics.h"
// #include <drake/systems
// #include "drake/examples/acrobot/acrobot_geometry.h"
namespace drake {
namespace flapping {
// #include "drake/multibody/plant/multibody_plant.h"

//Can add define constants
int DoMain() {
    return 0;
}
}
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::flapping::DoMain();
}