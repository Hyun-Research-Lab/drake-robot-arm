from copy import copy
import matplotlib.pyplot as plt
import numpy as np
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

def MakeMultibodyQuadrotor():
    
    builder = DiagramBuilder()
    # The MultibodyPlant handles f=ma, but doesn't know about propellers.
    plant = builder.AddSystem(MultibodyPlant(0.0))
    parser = Parser(plant)
    (model_instance,) = parser.AddModelsFromUrl(
        "package://drake/examples/quadrotor/quadrotor.urdf"
    )
    # By default the multibody has a quaternion floating base.  To match
    # QuadrotorPlant, we can manually add a FloatingRollPitchYaw joint. We set
    # `use_ball_rpy` to false because the BallRpyJoint uses angular velocities
    # instead of ṙ, ṗ, ẏ.
    AddFloatingRpyJoint(
        plant,
        plant.GetFrameByName("base_link"),
        model_instance,
        use_ball_rpy=False,
    )
    plant.Finalize()

    # Now we can add in propellers as an external force on the MultibodyPlant.
    body_index = plant.GetBodyByName("base_link").index()
    # Default parameters from quadrotor_plant.cc:
    L = 0.15  # Length of the arms (m).
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

    # Note: Rotors 0 and 2 rotate one way and rotors 1 and 3 rotate the other.
    prop_info = [
        PropellerInfo(body_index, RigidTransform([L, 0, 0]), kF, kM),
        PropellerInfo(body_index, RigidTransform([0, L, 0]), kF, -kM),
        PropellerInfo(body_index, RigidTransform([-L, 0, 0]), kF, kM),
        PropellerInfo(body_index, RigidTransform([0, -L, 0]), kF, -kM),
    ]
    propellers = builder.AddSystem(Propeller(prop_info))
    builder.Connect(
        propellers.get_output_port(),
        plant.get_applied_spatial_force_input_port(),
    )
    builder.Connect(
        plant.get_body_poses_output_port(),
        propellers.get_body_poses_input_port(),
    )
    builder.ExportInput(propellers.get_command_input_port(), "u")

    #Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    QuadrotorGeometry.AddToBuilder(
        builder, plant.get_output_port(0), scene_graph
    )       
    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # QuadrotorGeometry.AddToBuilder(
    #     builder, plant.get_output_port(0), scene_graph
    # )
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(2)
    meshcat.PublishRecording()
    return
    # return builder.Build(), plant

MakeMultibodyQuadrotor()
# This test demonstrates that the MultibodyPlant version has identical dynamics
# to the QuadrotorPlant version (except that the state variables are permuted).
# TODO(russt): Move this to Drake as a unit test.
# def MultibodyQuadrotorExample():
#     mbp_plant, mbp = MakeMultibodyQuadrotor()

#     hand_derived_plant = QuadrotorPlant()

#     # Compare the dynamics at a handful of states.
#     mbp_plant_context = mbp_plant.CreateDefaultContext()
#     mbp_context = mbp.GetMyContextFromRoot(mbp_plant_context)
#     hand_derived_context = hand_derived_plant.CreateDefaultContext()

#     # Permute mbp <=> hand_derived states.
#     P = np.array(
#         [
#             [1, 0, 0, 0, 0, 0],
#             [0, 1, 0, 0, 0, 0],
#             [0, 0, 1, 0, 0, 0],
#             [0, 0, 0, 0, 0, 1],
#             [0, 0, 0, 0, 1, 0],
#             [0, 0, 0, 1, 0, 0],
#         ]
#     )
#     PP = np.block([[P, np.zeros((6, 6))], [np.zeros((6, 6)), P]])

#     rng = np.random.default_rng(seed=1037)
#     for i in range(5):
#         u = rng.random((4,))
#         mbp_x = rng.random((12,))
#         mbp_plant.get_input_port().FixValue(mbp_plant_context, u)
#         mbp_context.SetContinuousState(mbp_x)
#         mbp_xdot = mbp_plant.EvalTimeDerivatives(
#             mbp_plant_context
#         ).CopyToVector()

#         hand_derived_x = PP @ mbp_x
#         hand_derived_plant.get_input_port().FixValue(hand_derived_context, u)
#         hand_derived_context.SetContinuousState(hand_derived_x)
#         hand_derived_xdot = (
#             PP
#             @ hand_derived_plant.EvalTimeDerivatives(
#                 hand_derived_context
#             ).CopyToVector()
#         )

#         assert np.allclose(
#             mbp_xdot, hand_derived_xdot
#         ), f"\nmbp\t\t = {mbp_xdot}\nhand_derived\t = {hand_derived_xdot}"


# MultibodyQuadrotorExample()