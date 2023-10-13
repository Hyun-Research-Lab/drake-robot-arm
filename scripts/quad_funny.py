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

def lemniscate(t, a=1.0):
    x = a * np.cos(t)
    z = a * np.sin(t) * np.cos(t)
    return x, z

def funny_traj(plant, init_state, manual_points,minimum_timestep=0.1,maximum_timestep=1):
    x_traj = None
    u_traj = None
    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(
        plant,
        context,
        num_time_samples=len(manual_points),
        minimum_timestep=minimum_timestep,
        maximum_timestep=maximum_timestep,
    )

    #input constraints
    prop_limit = 4
    u = dircol.input() #input decision variables
    for i in range(4):
        dircol.AddConstraintToAllKnotPoints(u[i] <= prop_limit)
        dircol.AddConstraintToAllKnotPoints(-prop_limit <= u[i])
    dircol.AddEqualTimeIntervalsConstraints()

    for i, (time, state) in enumerate(manual_points):
        dircol.AddBoundingBoxConstraint(state, state, dircol.state(i))
        # dircol.AddBoundingBoxConstraint(time, time, dircol.time(i))


manual_points = [
    (0.0, np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])),
    # (2.0, np.array([0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])),
    (4.0, np.array([1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0])),
    # (6.0, np.array([0, -1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0])),
    # (8.0, np.array([-1, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0])),
    (6.0, np.array([0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
]

def traj_opt(plant, manual_points, num_time_samples=21, minimum_time=0.1, maximum_time=40.0, StateView=None):
    #potentially add multiple runs #For loop off https://deepnote.com/workspace/Underactuated-2ed1518a-973b-4145-bd62-1768b49956a8/project/10-Trajectory-Optimization-05031f8c-3586-4b47-be79-7f4893cf2f9d/notebook/perching-fafc9a0c7de2430294ed93b619830ef2?
    #overwrite x_traj, u_traj
    x_traj = None
    u_traj = None
    context = plant.CreateDefaultContext()
    dircol = DirectCollocation(
        plant,
        context,
        # num_time_samples=len(manual_points) + 3,
        num_time_samples=len(manual_points) + 10 ,
        minimum_timestep=minimum_time, 
        maximum_timestep=maximum_time,
    )
    #input constraints
    prop_limit = 2
    u = dircol.input() #input decision variables
    for i in range(4):
        dircol.AddConstraintToAllKnotPoints(u[i] <= prop_limit)
        dircol.AddConstraintToAllKnotPoints(-prop_limit <= u[i])

    dir_prog = dircol.prog() #math prog
    for i, (time, state) in enumerate(manual_points[1:len(manual_points)-1]):
        dir_prog.AddBoundingBoxConstraint(state, state, dircol.state(i+3))
    
    dir_prog.AddBoundingBoxConstraint(manual_points[0][1], manual_points[0][1], dircol.initial_state())


    # statev =StateView(dircol.state(3))
    # dir_prog.AddBoundingBoxConstraint(np.pi-1, np.pi+.2, statev.rz_q)
    final = [1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    dir_prog.AddBoundingBoxConstraint(final, final, dircol.final_state())

        # dir_prog.AddBoundingBoxConstraint(time, time, dircol.time(i))
    #initial state constraint
    # dir_prog.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())
    #dircol.AddLinearConstraint(dircol.final_state() == final_state)
    #cost function
    # R = 10 * np.eye(4)  # Cost on input "effort". (4 propellers)
    R = 10 
    dircol.AddRunningCost(R * u[0]**2)

    # Add a final cost equal to the total duration.
    dircol.AddFinalCost(dircol.time())

    #initial guess trajectory, if multi iter, can erase this to use prev
    # initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
    #     [0.0, 4.0], np.column_stack(([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
    # )
    break_times = [point[0] for point in manual_points]
    #insert halftime points for every point

    states = [point[1] for point in manual_points]
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(break_times, np.column_stack(states))
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)
    print("set init traj")
    #solver
    solver = SnoptSolver()
    solver_id = solver.solver_id()
    major_tol = .01
    minor_tol = .01
    dir_prog.SetSolverOption(solver_id, "Feasibility tolerance", major_tol)
    dir_prog.SetSolverOption(solver_id, "Major feasibility tolerance", major_tol)
    dir_prog.SetSolverOption(solver_id, "Major optimality tolerance", major_tol)
    dir_prog.SetSolverOption(solver_id, "Minor feasibility tolerance", minor_tol)
    dir_prog.SetSolverOption(solver_id, "Minor optimality tolerance", minor_tol)
    result = Solve(dir_prog)
    # for binding in dircol.prog().GetAllConstraints():
    #     constraint = binding.evaluator()
    #     val = constraint.Eval(result.GetSolution(binding.variables()))
    #     if not constraint.CheckSatisfied(val):
    #         print("Unsatisfied constraint:", constraint)
    
    details = result.get_solver_details()
    print("Solver Exit Condition:", details.info)
    print("Final Objective Value:", result.get_optimal_cost())
    # print("Number of Major Iterations:", details.iterations)
    # print("Condition Number of the Final Reduced Hessian:", details.condhz)

    if not result.is_success():
        raise ValueError("Optimization failed!")
    print("Optimization successful.")
    x_traj = dircol.ReconstructStateTrajectory(result)
    u_traj = dircol.ReconstructInputTrajectory(result)

    if StateView:
        ts = np.linspace(u_traj.start_time(), u_traj.end_time(), 301)
        desired_state = StateView(x_traj.vector_values(ts))

        fig, ax = plt.subplots(figsize=(14, 6))
        ax.plot(desired_state.x_x, desired_state.z_x, label="desired")
        ax.set_xlabel("x_x")
        ax.set_ylabel("z_x")
        ax.axis("equal")
        return x_traj, u_traj, ax
    return x_traj, u_traj

# def finite_horizon_lqr(x_traj, u_traj):
#     options = FiniteHorizonLinearQuadraticRegulatorOptions()
#     options.x0 = x_traj
#     options.u0 = u_traj

#     builder = DiagramBuilder()



def MakeMultibodyQuadrotorLQR():
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)


    #Parse urdf
    parser = Parser(plant, scene_graph) #also adds geometry to scene_graph
    (model_instance,) = parser.AddModelsFromUrl("package://drake/examples/quadrotor/quadrotor.urdf")
    AddFloatingRpyJoint( #this is just to change the states from quaternion naming scheme (13 states) to rpy naming scheme (12 states)
        plant,
        plant.GetFrameByName("base_link"),
        model_instance,
        use_ball_rpy=False,
    )
    plant.Finalize()


    #adding propeller external output, so 0 internal actuators, 4 external
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
    
    #Exporting output, so u would be to propellers, x would be to plant for LQR feedback, 
    #Query output port is for visualization, must be exported, cant just directly get query and connect
    builder.ExportInput(propellers.get_command_input_port(), "u")
    builder.ExportOutput(plant.get_state_output_port(), "x")
    builder.ExportOutput(scene_graph.get_query_output_port(), "query")

    #We have to build now to grab the context from the diagram and set nominal states 
    #We use another builder (world_builder) as a wrapper, add the prev builder in and then 
    #add the LQR controller to the wrapper along with visualization
    diagram = builder.Build()
    state_names = plant.GetStateNames(False)
    print(state_names, len(state_names))
    StateView = namedview("state", state_names)
    for i in range(plant.num_input_ports()):
        print(f"Input Port {i}: {plant.get_input_port(i).get_name()}")

    for i in range(plant.num_output_ports()):
        print(f"Output Port {i}: {plant.get_output_port(i).get_name()}")
    
    for i in range(diagram.num_input_ports()):
        print(f"in Port {i}: {diagram.get_input_port(i).get_name()}")
    world_builder = DiagramBuilder()
    world_builder.AddSystem(diagram)

    # Create the LQR controller
    context = diagram.CreateDefaultContext()
    nominal_state = StateView.Zero()
    nominal_state.z_x = 5.0  # height is 1.0m
    # nominal_state. = 1  # no rotation
    context.SetContinuousState(nominal_state[:])
    mass = plant.CalcTotalMass(plant.GetMyContextFromRoot(context))
    gravity = plant.gravity_field().gravity_vector()[2]
    nominal_input = [-mass * gravity / 4] * 4
    diagram.get_input_port().FixValue(context, nominal_input)

    
    #check states
    n = plant.num_positions() + plant.num_velocities()
    print(n)
    Q = np.diag(np.concatenate(([10] * (n//2), [1] * (n//2 + n % 2))))
    print(Q)
    print(f"Number of actuators: {plant.num_actuators()}")
    m = plant.num_actuators()
    R = np.eye(4) #properllers are external, so they are not part of the plant actuators

    # controller = world_builder.AddSystem(LinearQuadraticRegulator(diagram, context, Q, R))
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.Qf = Q
    final_state = np.array(nominal_state[:]) #test without array transform after
    init_state = np.array(StateView.Zero()[:])
    print(init_state, final_state)
    x_traj, u_traj, ax = traj_opt(diagram, manual_points, StateView=StateView)
    options.x0 = x_traj
    options.u0 = u_traj
    controller = world_builder.AddSystem(MakeFiniteHorizonLinearQuadraticRegulator(
        system=diagram,
        context=context,
        t0=x_traj.start_time(),
        tf=x_traj.end_time(),
        Q=Q,
        R=R,
        options=options,
    )) 

    world_builder.Connect(controller.get_output_port(), diagram.get_input_port())
    world_builder.Connect(diagram.GetOutputPort("x"),
                    controller.get_input_port())
    logger = LogVectorOutput(diagram.GetOutputPort("x"), builder=world_builder)

    # MeshcatVisualizer.AddToBuilder(world_builder, scene_graph, meshcat)
    MeshcatVisualizer.AddToBuilder(world_builder, diagram.GetOutputPort("query"), meshcat)
    # builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))
    world_diagram = world_builder.Build()
    simulator = Simulator(world_diagram)
    simulator.set_target_realtime_rate(1.0)
    # simulator.set_publish_every_time_step(False)
    context = simulator.get_mutable_context()

    ts = np.linspace(u_traj.start_time(), u_traj.end_time(), 301)
    desired_state = StateView(x_traj.vector_values(ts))

    # Plotting desired state
    fig1, ax1 = plt.subplots(figsize=(14, 6))
    ax1.plot(desired_state.x_x, desired_state.z_x, label="desired")
    ax1.set_xlabel("x_x")
    ax1.set_ylabel("z_x")
    ax1.axis("equal")
    ax1.legend()

    # Plotting u forces across time
    u_values = u_traj.vector_values(ts)
    fig2, ax2 = plt.subplots(figsize=(14, 6))
    for i in range(u_values.shape[0]):
        ax2.plot(ts, u_values[i, :], label=f"u_{i}")
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Control Forces (u)")
    ax2.legend()


    rng = np.random.default_rng(123)
    
    meshcat.StartRecording()
    # Simulate
    for i in range(1):
        context.SetTime(x_traj.start_time())
        initial_state = StateView(x_traj.value(x_traj.start_time()))
        initial_state.x_x += rng.standard_normal() * .01
        initial_state.y_x += rng.standard_normal() * .01
        context.SetContinuousState(
            initial_state[:]
        )
        simulator.Initialize()
        simulator.AdvanceTo(x_traj.end_time())
        log = logger.FindLog(context)
        state = StateView(log.data())
        log.Clear()
        ax.plot(state.x_x, state.z_x, label="actual")
    ax.legend()

    meshcat.PublishRecording()

    print("rdy")
    plt.show()
    while True:
        pass



if __name__ == "__main__":
    MakeMultibodyQuadrotorLQR()
