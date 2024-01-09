//Entrypoint

// #include "aerodynamics.h"
// #include <drake/systems

namespace drake {
namespace flapping {
using drake::multibody::MultibodyPlant;
//Can add define constants
int DoMain() {
    return 0
}
}
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::flapping::DoMain();
}