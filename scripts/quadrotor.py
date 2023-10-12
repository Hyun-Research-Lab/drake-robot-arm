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
meshcat = StartMeshcat()

def MakeMultibodyQuadrotor():
    
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)


    #Parse urdf
    parser = Parser(plant, scene_graph)
    parser.AddModelsFromUrl("package://drake/examples/quadrotor/quadrotor.urdf")
    plant.Finalize()


    #visualize  
    MeshcatVisualizer.AddToBuilder(builder,scene_graph, meshcat)
    # builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Start the simulation with visualization
    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(2)
    meshcat.PublishRecording()

    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    print("rdy")
    while True:
        pass
    # return builder.Build(), plant


def MakeMultibodyQuadrotorLQR():
    
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)


    #Parse urdf
    parser = Parser(plant, scene_graph)
    (model_instance,) = parser.AddModelsFromUrl("package://drake/examples/quadrotor/quadrotor.urdf")
    AddFloatingRpyJoint(
        plant,
        plant.GetFrameByName("base_link"),
        model_instance,
        use_ball_rpy=False,
    )
    plant.Finalize()


    #adding propeller external output 
    # Default parameters from quadrotor_plant.cc:
    body_index = plant.GetBodyByName("base_link").index()
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
    builder.ExportOutput(plant.get_state_output_port(), "x")
    builder.ExportOutput(scene_graph.get_query_output_port(), "query")

    diagram = builder.Build()
    state_names = plant.GetStateNames(False)
    print(state_names, len(state_names))
    StateView = namedview("state", state_names)
    for i in range(plant.num_input_ports()):
        print(f"Input Port {i}: {plant.get_input_port(i).get_name()}")

    for i in range(plant.num_output_ports()):
        print(f"Output Port {i}: {plant.get_output_port(i).get_name()}")
    
    world_builder = DiagramBuilder()
    world_builder.AddSystem(diagram)

    # Create the LQR controller
    context = diagram.CreateDefaultContext()
    nominal_state = StateView.Zero()
    nominal_state.z_x = 1.0  # height is 1.0m
    # nominal_state. = 1  # no rotation
    context.SetContinuousState(nominal_state[:])
    mass = plant.CalcTotalMass(plant.GetMyContextFromRoot(context))
    gravity = plant.gravity_field().gravity_vector()[2]
    nominal_input = [-mass * gravity / 4] * 4
    diagram.get_input_port().FixValue(context, nominal_input)
    
    #check states
    n = plant.num_positions() + plant.num_velocities()
    print(n)
    Q = np.diag(np.concatenate(([10] * (n//2), [1] * (n//2 + n %2 ))))
    print(Q)
    print(f"Number of actuators: {plant.num_actuators()}")
    m = plant.num_actuators()
    R = np.eye(4) #properllers are external, so they are not part of the plant actuators

    controller = world_builder.AddSystem(LinearQuadraticRegulator(diagram, context, Q, R))
    world_builder.Connect(controller.get_output_port(), diagram.get_input_port())
    world_builder.Connect(diagram.GetOutputPort("x"),
                    controller.get_input_port())

    # MeshcatVisualizer.AddToBuilder(world_builder, scene_graph, meshcat)
    MeshcatVisualizer.AddToBuilder(world_builder, diagram.GetOutputPort("query"), meshcat)
    # builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))
    world_diagram = world_builder.Build()
    simulator = Simulator(world_diagram)
    simulator.set_target_realtime_rate(1.0)
    # simulator.set_publish_every_time_step(False)
    context = simulator.get_mutable_context()

    # Simulate
    for i in range(5):
        context.SetTime(0.0)
        context.SetContinuousState(
            0.5
            * np.random.randn(
                12,
            )
        )
        simulator.Initialize()
        simulator.AdvanceTo(0.1)

    # Start the simulation with visualization
    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(2)
    meshcat.PublishRecording()

    print("rdy")
    while True:
        pass



def quadrotor_example():
    builder = DiagramBuilder()

    # plant = builder.AddSystem(QuadrotorPlant())
    plant = builder.AddSystem(MultibodyPlant(0.0))
    parser = Parser(plant)
    (model_instance,) = parser.AddModelsFromUrl(
        "package://drake/examples/quadrotor/quadrotor.urdf"
    )
    controller = builder.AddSystem(StabilizingLQRController(plant, [0, 0, 1]))
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

    # Set up visualization in MeshCat
    scene_graph = builder.AddSystem(SceneGraph())
    QuadrotorGeometry.AddToBuilder(
        builder, plant.get_output_port(0), scene_graph
    )
    meshcat.Delete()
    meshcat.ResetRenderMode()
    meshcat.SetProperty("/Background", "visible", False)
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # end setup for visualization

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    # Simulate
    for i in range(5):
        context.SetTime(0.0)
        context.SetContinuousState(
            0.5
            * np.random.randn(
                12,
            )
        )
        simulator.Initialize()
    # simulator.AdvanceTo(2)
    # meshcat.PublishRecording()
    while True:
        pass


# quadrotor_example()
# MakeMultibodyQuadrotor()
MakeMultibodyQuadrotorLQR()
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