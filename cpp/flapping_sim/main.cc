//Entrypoint

#include "aerodynamics.h"

namespace drake {
namespace flapping {
using drake::multibody::MultibodyPlant;
//Can add define constants
int DoMain() {
    
}
}
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::flapping::DoMain();
}