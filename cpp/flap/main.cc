//Entrypoint

#include <gflags/gflags.h>
#include <iostream>
#include <filesystem>
// #include "aerodynamics.h"
// #include <drake/systems
// #include "drake/examples/acrobot/acrobot_geometry.h"
#include <drake/multibody/plant/multibody_plant.h>
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
namespace drake {
namespace flapping {
using Eigen::Vector3d;
using geometry::SceneGraph;

//Can add define constants
int DoMain() {
    systems::DiagramBuilder<double> builder;

    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0001);
    plant.set_name("plant");

    // std::cout << "Current path is " << current_path << std::endl;
    const std::string sdf_url = "package://drake/flap/URDF_LargeWings/urdf/URDF_LargeWings.sdf";
    // std::filesystem::path sdf_package_path = current_path.parent_path() / "URDF_LargeWings";
    Parser parser(&plant, &scene_graph);

    std::filesystem::path source_file_path(__FILE__);
    std::cout << "current path: " << source_file_path << std::endl;
    parser.package_map().Add("URDF_LargeWings", source_file_path.parent_path() / "URDF_LargeWings");
    
    parser.AddModelsFromUrl(sdf_url);
    
    // std::cout << "Added models from " << sdf_url << std::endl;
    // for (const std::string& package_name : parser.package_map().GetPackageNames()) {
    //     std::cout << "Package name: " << package_name << std::endl;
    // }
    // std::cout << "package names: " << parser.package_map().GetPackageNames() << std::endl;
    plant.Finalize();

    
    return 0;
}
}
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::flapping::DoMain();
}