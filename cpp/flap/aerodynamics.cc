#include "aerodynamics.h"

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/body.h>
#include <drake/geometry/scene_graph.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace drake {
namespace flap {

using systems::LeafSystem;
using multibody::MultibodyPlant;
using multibody::Body;
using multibody::SpatialVelocity;
using geometry::RigidTransform;
using geometry::SceneGraph;

using Eigen::Vector3d;
// Define the Aerodynamics class
class Aerodynamics : public LeafSystem<double> { //Use explicit?
 public:
    Aerodynamics(const MultibodyPlant<double>& plant)
        : plant_(plant) {
        this->DeclareAbstractInputPort("body_spatial_velocities", 
                                       Value<std::vector<SpatialVelocity<double>>>());
        this->DeclareAbstractInputPort("body_poses", 
                                       Value<std::vector<RigidTransform<double>>>());
        // ... other member initializations ...
        // Initialize body references
        right_wing_body_ = &plant_.GetBodyByName("RW_Pitch");
        left_wing_body_ = &plant_.GetBodyByName("LW_Pitch");
    }

    void DoCalcOutput(const systems::Context<double>& context,
                                        systems::SystemOutput<double>* output) const override {
        // Calculate the output based on the input and other computations
        const auto& input = context.get_continuous_state()->get_vector().GetAtIndex(input_port_index_);
        auto output_vector = output->GetMutableVectorData(output_port_index_);
        
        // Perform the aerodynamics calculations here
        
        // Set the output vector
        output_vector->SetFromVector(/* calculated output vector */);
    }

private:
    const MultibodyPlant<double>& plant_;
    const Body<double>* right_wing_body_;
    const Body<double>* left_wing_body_;
};

}  // namespace flap
}  // namespace drake
