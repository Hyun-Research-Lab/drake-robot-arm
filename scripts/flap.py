from copy import copy
import matplotlib.pyplot as plt
import numpy as np
import time
#concurrent futures python

from pydrake.all import (
    LeafSystem,
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
    Sine,
    SnoptSolver,
    Solve,
    TrajectorySource,
)

from pydrake.all import (
    Cylinder,
    SpatialVelocity,
    SpatialForce,
    AbstractValue,
    ExternallyAppliedSpatialForce,
    RotationMatrix,
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
    Multiplexer,
    
    # RegisterGeometry
    
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

from pydrake.geometry import GeometryInstance, MakePhongIllustrationProperties, GeometryFrame, IllustrationProperties

from pydrake.solvers import MathematicalProgram, Solve
from libs.scenarios  import AddFloatingRpyJoint
from libs.abstract_logger import AbstractValueLogger
import os
from pydrake.systems.framework import BasicVector

from pydrake.multibody.tree import JointIndex, JointActuatorIndex, JointActuator

meshcat = StartMeshcat()
def process_externally_applied_spatial_force(value: ExternallyAppliedSpatialForce):
    #only 1 force for now so 
    spatial_force = value[0]
    rw = {
        'body_idx': spatial_force.body_index,
        'position_vec': spatial_force.p_BoBq_B.tolist(),
        'force_vec': spatial_force.F_Bq_W.get_coeffs().tolist()[3:], #[0:2] is torque, [3:] is force
    }
    spatial_force = value[1]
    lw = {
        'body_idx': spatial_force.body_index,
        'position_vec': spatial_force.p_BoBq_B.tolist(),
        'force_vec': spatial_force.F_Bq_W.get_coeffs().tolist()[3:], #[0:2] is torque, [3:] is force
    }
    return [rw, lw]

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
    # plant.mutable_gravity_field().set_gravity_vector([0, 0, 0])
    
    left_wing_body = plant.GetBodyByName("LW_Pitch")
    # cylinder_pose_lw = RigidTransform(RotationMatrix(), [-0.06, -0.02, 0.05]) #z is facing, y is up , x is in
    cylinder_pose_lw = RigidTransform(RotationMatrix(), [0.0, -0.0, 0.02]) #z is facing, y is up , x is in
    cylinder_pose_rw = RigidTransform(RotationMatrix(), [0.0, -0.0, 0.02]) #z is facing, y is up , x is in
    # cylinder_pose_rw = RigidTransform(RotationMatrix(), [-0.06, -0.02, 0.05]) #z is facing, y is up , x is in
    cylinder_shape = Cylinder(0.001, 0.04)
    geometry_id_lw = plant.RegisterVisualGeometry(left_wing_body, cylinder_pose_lw, cylinder_shape, "wing_cylinder", [0, 1, 0, 0.5])
    
    right_wing_body = plant.GetBodyByName("RW_Pitch")
    geometry_id_rw = plant.RegisterVisualGeometry(right_wing_body, cylinder_pose_rw, cylinder_shape, "wing_cylinder", [0, 1, 0, 0.5])

    #can visually verify that the cylinder is in the right place (cp)
    center_pressure_body_lw = [[-0.00833, -0.025, 0], [-0.02499, -0.025, 0], [-0.04165, -0.025, 0], [-0.05831, -0.025, 0], [-0.07497, -0.025, 0], [-0.09163, -0.025, 0], [-0.10829, -0.025, 0], [-0.12495, -0.025, 0], [-0.14161, -0.025, 0]]
    center_pressure_body_rw = [[0.00833, -0.025, 0], [0.02499, -0.025, 0], [0.04165, -0.025, 0], [0.05831, -0.025, 0], [0.07497, -0.025, 0], [0.09163, -0.025, 0], [0.10829, -0.025, 0], [0.12495, -0.025, 0], [0.14161, -0.025, 0]]


    for i,cp in enumerate(center_pressure_body_lw):
        cp[2] = 0.02
        cylinder_pose_lw = RigidTransform(RotationMatrix(), cp)
        geometry_id_lw = plant.RegisterVisualGeometry(left_wing_body, cylinder_pose_lw, cylinder_shape, "wing_cylinder" + str(i), [0, 1, 1, 0.5])
    for i,cp in enumerate(center_pressure_body_rw):
        cp[2] = 0.02

        cylinder_pose_lw = RigidTransform(RotationMatrix(), cp)
        geometry_id_lw = plant.RegisterVisualGeometry(right_wing_body, cylinder_pose_lw, cylinder_shape, "wing_cylinder" + str(i), [0, 1, 1, 0.5])
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
        print(f"Joint: {joint.name()}, Model Instance: {joint.model_instance()}")

    print("Actuator names: ", plant.GetActuatorNames(add_model_instance_prefix=False))
    print("Actuators: " + str(plant.num_actuators()))

    return plant, scene_graph, builder

def Flap():
    plant, scene_graph, builder = MultiBodyParser()

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    aerodynamics = builder.AddSystem(AerodynamicsSystem(plant))
    builder.Connect(aerodynamics.get_output_port(), plant.get_applied_spatial_force_input_port())
    builder.Connect(plant.get_body_poses_output_port(), aerodynamics.GetInputPort("body_poses"))
    builder.Connect(plant.get_body_spatial_velocities_output_port(), aerodynamics.GetInputPort("body_spatial_velocities"))
    aero_logger = builder.AddSystem(AbstractValueLogger(model_value=[ExternallyAppliedSpatialForce()], publish_period_seconds=0.05 ))
    builder.Connect(aerodynamics.get_output_port(), aero_logger.get_input_port())


    #passive joint effort in limit in sdf set to 0 to make passive

    sine_wave = Sine(3, 30.0, 1.0, 1)
    sine_wave2 = Sine(-3, 30.0, 1.0, 1)
    sin = builder.AddSystem(sine_wave)
    sin2 = builder.AddSystem(sine_wave2)
    #mux so we can have opposite amp sin wave input to both
    sinmux = builder.AddSystem(Multiplexer([1, 1])) #2 ports. each with vector size 1
    builder.Connect(sin.get_output_port(0), sinmux.get_input_port(1))
    builder.Connect(sin2.get_output_port(0), sinmux.get_input_port(0))
    builder.Connect(sinmux.get_output_port(0), plant.get_actuation_input_port())
    diagram = builder.Build()

    state_names = plant.GetStateNames(False)
    print("states:", state_names, len(state_names))
    StateView = namedview("state", state_names)
    for i in range(plant.num_input_ports()):
        print(f"Input Port {i}: {plant.get_input_port(i).get_name()}")

    for i in range(plant.num_output_ports()):
        print(f"Output Port {i}: {plant.get_output_port(i).get_name()}")
    
    for i in range(diagram.num_input_ports()):
        print(f"in Port {i}: {diagram.get_input_port(i).get_name()}")

    context = diagram.CreateDefaultContext()
    actuation_input_port = plant.get_actuation_input_port()
    print(actuation_input_port.get_name())
    print("dia ports", diagram.num_input_ports())
    

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()
    meshcat.StartRecording()
    simulator.set_target_realtime_rate(0.25)
    simulator.Initialize()
    print("init")
    simulator.AdvanceTo(2)
    print("Done calc")
    meshcat.PublishRecording()
    print("Done record")
    aero_logger.WriteCSV("aero_logger.csv", process_externally_applied_spatial_force)
    print("Done Log")
    while True:
        pass


class AerodynamicsSystem(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self.plant = plant
        self.body_spatial_velocity_input_port = self.DeclareAbstractInputPort("body_spatial_velocities", AbstractValue.Make([SpatialVelocity()]))
        self.body_poses_input_port = self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform()]))
        # self.state_input_port = self.DeclareVectorInputPort("state", BasicVector(plant.num_multibody_states()))#(plant.num_positions() + plant.num_velocities()))
        self.force_output_port = self.DeclareAbstractOutputPort("force", lambda: AbstractValue.Make([ExternallyAppliedSpatialForce()]), self.CalcOutput)
        self.right_wing_body = plant.GetBodyByName("RW_Pitch")
        self.left_wing_body = plant.GetBodyByName("LW_Pitch")
        self.right_wing_body_index = self.right_wing_body.index()
        self.left_wing_body_index = self.left_wing_body.index()
        self.orthogonal_vec = np.array([0,0,1])
        self.blade_area = 0.00765
        self.center_pressure_body_lw = [[-0.00833, -0.025, 0], [-0.02499, -0.025, 0], [-0.04165, -0.025, 0], [-0.05831, -0.025, 0], [-0.07497, -0.025, 0], [-0.09163, -0.025, 0], [-0.10829, -0.025, 0], [-0.12495, -0.025, 0], [-0.14161, -0.025, 0]]
        self.center_pressure_body_rw = [[0.00833, -0.025, 0], [0.02499, -0.025, 0], [0.04165, -0.025, 0], [0.05831, -0.025, 0], [0.07497, -0.025, 0], [0.09163, -0.025, 0], [0.10829, -0.025, 0], [0.12495, -0.025, 0], [0.14161, -0.025, 0]]
        self.lw_cp_length = len(self.center_pressure_body_lw)
        self.rw_cp_length = len(self.center_pressure_body_rw)
        self.air_density = 1.293
        self.drag_coef = 1.28
    def CalcOutput(self, context, output):
        poses = self.body_poses_input_port.Eval(context)
        velocities = self.body_spatial_velocity_input_port.Eval(context)
        #vel of rw
        
        velocity_world_rw = velocities[self.right_wing_body_index] 
        velocity_world_lw = velocities[self.left_wing_body_index]
        
        #We take the orthogonal vector to the wing and then project the linear velocity onto it to get the velocity component for drag
        pose_world_rw = poses[self.right_wing_body_index]
        pose_world_lw = poses[self.left_wing_body_index]
        # rigidbody.rotationmatrix, multiply by orth

        up_vector_rw = pose_world_rw.rotation().multiply(self.orthogonal_vec) #orthogonal unit vector in world frame
        up_vector_lw = pose_world_lw.rotation().multiply(self.orthogonal_vec) #orthogonal unit vector in world frame

        # blade_area_list = [0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085]


        # center_pressure_body_rw = [[0.06, -0.02, 0]]  # relative location on wing for cp from origin
        blade_area_list_rw = [self.blade_area / self.rw_cp_length] * self.rw_cp_length
        # center_pressure_body_lw = [[-0.06, -0.02, 0]]  # relative location on wing for cp from origin
        # center_pressure_body_lw = [[-0.060, -0.025, 0], [-0.045, -0.025, 0], [-0.030, -0.025, 0], [-0.015, -0.025, 0], [0.000, -0.025, 0], [0.015, -0.025, 0], [0.030, -0.025, 0], [0.045, -0.025, 0], [0.060, -0.025, 0] ]
        blade_area_list_lw = [self.blade_area / self.lw_cp_length] * self.lw_cp_length
        
        rw_info = (self.right_wing_body , self.center_pressure_body_rw, blade_area_list_rw, velocity_world_rw, up_vector_rw)
        lw_info = (self.left_wing_body, self.center_pressure_body_lw, blade_area_list_lw, velocity_world_lw, up_vector_lw)

        external_force_list = []
        init_time = time.time()
        for body_info in [rw_info, lw_info]:
            body = body_info[0]
            center_pressure_body = body_info[1]
            blade_area_list = body_info[2]
            velocity_world = body_info[3]
            up_vector = body_info[4]
            # first_time = time.time()
            for i, cp_coord in enumerate(center_pressure_body):
                #we take center pressure, get linear velocity and rotational velocity at wing origin, then map rotational to linear from origin to
                # center pressure, then add the two linear velocities together to get total linear velocity at center pressure
                wing_rotational_v_mapped = np.cross(velocity_world.rotational(), cp_coord) #additional velocity at cp from origin
                wing_cp_linear_v = velocity_world.translational() + wing_rotational_v_mapped #total lin velocity at cp
                vel_dot_up = np.dot(wing_cp_linear_v, up_vector) #projected linear vel onto up vec

                #1.293 is air density, 0.00765 is wing area, 1.28 drag coef
                # wing_area = 0.00765
                flap_drag_scalar = 0.5 * self.air_density * blade_area_list[i] * self.drag_coef * vel_dot_up**2 #drag force scalar
                flap_drag_force = flap_drag_scalar * up_vector
                if vel_dot_up < 0:  #fixes direction
                    flap_drag_force = -flap_drag_force

                external_force = ExternallyAppliedSpatialForce()
                external_force.body_index = body.index()
                external_force.p_BoBq_B = cp_coord
                external_force.F_Bq_W = SpatialForce([0,0,0], flap_drag_force) 
                external_force_list.append(external_force)
            # wing_time = time.time()
        # end_time = time.time()
        # print("time taken", end_time - init_time)
        # print("time taken wing", wing_time - first_time)
        output.set_value(external_force_list)


Flap()

