from copy import copy
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    DirectCollocation,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    FiniteHorizonLinearQuadraticRegulator,
    LogVectorOutput,
    MakeFiniteHorizonLinearQuadraticRegulator,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    MakeFiniteHorizonLinearQuadraticRegulator,
    Parser,
    PiecewisePolynomial,
    PlanarSceneGraphVisualizer,
    SceneGraph,
    Simulator,
    SnoptSolver,
    Solve,
    TrajectorySource,
)

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    Propeller,
    PropellerInfo,
    RigidTransform,
    RobotDiagramBuilder,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
    namedview,
    ConstantVectorSource,
)
from pydrake.examples import (
    AcrobotGeometry,
    AcrobotInput,
    AcrobotPlant,
    AcrobotState,
    QuadrotorGeometry,
    QuadrotorPlant,
    StabilizingLQRController,
)


from pydrake.solvers import MathematicalProgram, Solve
from libs.scenarios  import AddFloatingRpyJoint
import os
from pydrake.multibody.tree import JointIndex, JointActuatorIndex

meshcat = StartMeshcat()

def MultiBodyParser(): #change proj dir and model from path/name
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    #Parse urdf
    project_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "URDF_LargeWings")
    parser = Parser(plant, scene_graph) #also adds geometry to scene_graph
    parser.package_map().Add("URDF_LargeWings", os.path.join(project_dir))
    parser.AddModelFromFile(os.path.join(project_dir, "urdf", "URDF_LargeWings.sdf"))
    # (model_instance,) = parser.AddModelsFromUrl("package://drake/examples/quadrotor/quadrotor.urdf")
    # AddFloatingRpyJoint( #this is just to change the states from quaternion naming scheme (13 states) to rpy naming scheme (12 states)
    #     plant,
    #     plant.GetFrameByName("base_link"),
    #     model_instance,
    #     use_ball_rpy=False,
    # )
    plant.Finalize()

    #Info check
    print(f"Number of bodies: {plant.num_bodies()}")
    print(f"Number of joints: {plant.num_joints()}")
    print(f"Number of actuators: {plant.num_actuators()}")
    print(f"Number of model instances: {plant.num_model_instances()}")
    print(f"Number of positions: {plant.num_positions()}")
    print(f"Number of velocities: {plant.num_velocities()}")
    print(f"Number of multibody states: {plant.num_multibody_states()}")
    print(f"Number of continuous states: {plant.num_continuous_states()}")
    # For more detailed information, you can iterate through bodies, joints, and actuators:
    for body_index in plant.GetBodyIndices(plant.world_frame().model_instance()):
        body = plant.get_body(body_index)
        print(f"Body: {body.name()}, Model Instance: {body.model_instance()}")

    for joint_index in range(plant.num_joints()):
        joint_idx = JointIndex(joint_index)
        joint = plant.get_joint(joint_idx)
        # if joint.num_actuators() > 0:
        #     for actuator_idx in range(joint.num_actuators()):
        #         actuator = joint.get_actuator(actuator_idx)
        #         print(f"Joint: {joint.name()}, Actuator: {actuator.name()}")
        print(f"Joint: {joint.name()}, Model Instance: {joint.model_instance()}")

    print("Actuators: " + str(plant.num_actuators()))
    #broken
    # actuation_model = plant.get_actuation_input_port()
    # for actuator_index in range(plant.num_actuators()):
    #     actuator_index_obj = JointActuatorIndex(actuator_index)
    #     actuator_name = actuation_model.get_name()
    #     print(f"Actuator {actuator_index}: {actuator_name}")

    # diagram = builder.Build()
    # print(f"Number of input ports: {diagram.num_input_ports()}")
    # print(f"Number of output ports: {diagram.num_output_ports()}")

    # # Get details about input ports
    # for i in range(diagram.num_input_ports()):
    #     input_port = diagram.get_input_port(i)
    #     print(f"Input Port {i}: {input_port.get_name()}")

    # # Get details about output ports
    # for i in range(diagram.num_output_ports()):
    #     output_port = diagram.get_output_port(i)
    #     print(f"Output Port {i}: {output_port.get_name()}")

    return plant, scene_graph, builder
MultiBodyParser()

