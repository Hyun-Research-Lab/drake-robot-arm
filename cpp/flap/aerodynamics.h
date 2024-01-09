#pragma once

//FIX REDUNDANCY AND CLEAN UP LATER
#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/body.h>
#include <drake/geometry/scene_graph.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace drake {
namespace flap {

using drake::systems::LeafSystem;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::SpatialVelocity;
using drake::geometry::RigidTransform;
using drake::geometry::SceneGraph;
using Eigen::Vector3d;

class AerodynamicsSystem : public LeafSystem<double> {
public:
    explicit AerodynamicsSystem(const MultibodyPlant<double>& plant);
    
protected:
    void CalcOutput(const drake::systems::Context<double>& context, 
                    drake::systems::SystemOutput<double>* output) const override;

private:
    const MultibodyPlant<double>& plant_;
    const Body<double>* right_wing_body_;
    const Body<double>* left_wing_body_;
    // ... other member variables ...
    // Example: Vector3d orthogonal_vec_;
    // double blade_area_;
    // std::vector<Vector3d> center_pressure_body_lw_;
    // std::vector<Vector3d> center_pressure_body_rw_;
    // double lw_cp_length_;
    // double rw_cp_length_;
    // double air_density_;
    // double drag_coef_;
};

}  // namespace flap
}  // namespace drake
