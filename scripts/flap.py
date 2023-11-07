from copy import copy
import matplotlib.pyplot as plt
import numpy as np
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
    value = value[0]
    return {
        'body_idx': value.body_index,
        'position_vec': value.p_BoBq_B.tolist(),
        'force_vec': value.F_Bq_W.get_coeffs().tolist()[3:], #[0:2] is torque, [3:] is force
    }
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
    plant.mutable_gravity_field().set_gravity_vector([0, 0, 0])
    
    left_wing_body = plant.GetBodyByName("LW_Pitch")
    cylinder_pose_lw = RigidTransform(RotationMatrix(), [-0.06, -0.02, 0.05]) #z is facing, y is up , x is in
    cylinder_pose_rw = RigidTransform(RotationMatrix(), [0.06, -0.02, 0.05]) #z is facing, y is up , x is in
    cylinder_shape = Cylinder(0.001, 0.1)
    geometry_id_lw = plant.RegisterVisualGeometry(left_wing_body, cylinder_pose_lw, cylinder_shape, "wing_cylinder", [0, 1, 0, 0.5])
    
    right_wing_body = plant.GetBodyByName("RW_Pitch")
    geometry_id_rw = plant.RegisterVisualGeometry(right_wing_body, cylinder_pose_rw, cylinder_shape, "wing_cylinder", [0, 1, 0, 0.5])
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

    print("Actuator names: ", plant.GetActuatorNames(add_model_instance_prefix=False))
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

def Flap():
    plant, scene_graph, builder = MultiBodyParser()
    # builder.ExportInput(plant.get_input_port(4), "actuation")
    #sine(amp, freq, phase, vector size)
    # plant_context = plant.CreateDefaultContext()
    # builder.Connect(plant.get_state_output_port(), aerodynamics.get_input_port())
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    aerodynamics = builder.AddSystem(AerodynamicsSystem(plant))
    builder.Connect(aerodynamics.get_output_port(), plant.get_applied_spatial_force_input_port())
    builder.Connect(plant.get_body_poses_output_port(), aerodynamics.GetInputPort("body_poses"))
    builder.Connect(plant.get_body_spatial_velocities_output_port(), aerodynamics.GetInputPort("body_spatial_velocities"))
    aero_logger = builder.AddSystem(AbstractValueLogger(model_value=[ExternallyAppliedSpatialForce()], publish_period_seconds=0.05 ))
    builder.Connect(aerodynamics.get_output_port(), aero_logger.get_input_port())
    # aero_logger = LogVectorOutput(aerodynamics.get_output_port(), builder)

    # Aerodynamics(plant, plant_context)
    # left_wing_body = plant.GetBodyByName("LW_Pitch")
    # X_WB = plant.EvalBodyPoseInWorld(plant_context, left_wing_body)
    # orthogonal_vec = np.array([0,0,1])
    # orthogonal_vector_world_frame = X_WB.rotation().multiply(orthogonal_vec)
    # cylinder_shape = Cylinder(5, 10)   
    # cylinder_pose_trans = X_WB.translation() + orthogonal_vector_world_frame / 2
    # cylinder_pose = RigidTransform(RotationMatrix(X_WB.rotation()), cylinder_pose_trans)

    # cylinder_geometry = GeometryInstance(cylinder_pose, cylinder_shape, "cylinder")
   
    # cylinder_geometry.set_illustration_properties(MakePhongIllustrationProperties(np.array([0.9, 0.9, 0.9, 1.0])))
    # source_id = scene_graph.RegisterSource("cylinder")
    # frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("cylinder_frame", 0))
    # geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, cylinder_geometry)
    # geometry_id = plant.RegisterVisualGeometry(
    # left_wing_body, RigidTransform(), cylinder_shape, "wing_cylinder", [1, 0, 0, 0.5])
    
    
    # scene_graph.AssignRole( geometry_id, IllustrationProperties())
    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)





    sine_wave = Sine(2.0, 10.0, 1.0, 1)
    sine_wave2 = Sine(-2.0, 10.0, 1.0, 1)
    sin = builder.AddSystem(sine_wave)
    sin2 = builder.AddSystem(sine_wave2)
    # builder.Connect(sine_wave.get_output_port(0), plant.get_actuation_input_port())
    sinmux = builder.AddSystem(Multiplexer([1, 1])) #2 ports. each with vector size 1
    builder.Connect(sin.get_output_port(0), sinmux.get_input_port(1))
    builder.Connect(sin2.get_output_port(0), sinmux.get_input_port(0))

    # mux = builder.AddSystem(Multiplexer([2, 2])) #2 ports , each with size 2
    mux = builder.AddSystem(Multiplexer([3, 2])) # For pole
    builder.Connect(sinmux.get_output_port(0), mux.get_input_port(1))

    passive_in = builder.AddSystem(ConstantVectorSource([0.0, 0.0, 0.0])) #For pole
    # passive_in = builder.AddSystem(ConstantVectorSource([0.0, 0.0]))
   
    builder.Connect(passive_in.get_output_port(0), mux.get_input_port(0))
    builder.Connect(mux.get_output_port(0), plant.get_actuation_input_port())
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
    # plant_context = diagram.GetMutableSubsystemContext(plant, context)
    actuation_input_port = plant.get_actuation_input_port()
    print(actuation_input_port.get_name())
    
    #can also use GetJointActuatorByName()
    # rw_passive = plant.GetJointActuatorByName("joint_RW_J_Pitch")
    # lw_passive = plant.GetJointActuatorByName("joint_LW_J_Pitch")
    # assert isinstance(rw_passive, JointActuator)
    # assert rw_passive.joint().num_velocities() == 1
    # print(lw_passive.joint().num_velocities())
    # print(rw_passive.num_inputs())
    # agh =[[]]
    # u_inst = np.array([[0.0]],)
    # print(u_inst.shape)
    # #print u_inst type
    # print(u_inst.dtype)
    # print(type(u_inst))
    # rw_passive.set_actuation_vector(u_inst)
    
    #set actuations
    
    # plant.get_actuation_input_port().FixValue(plant_context, np.array([[0.0,0.0,1,1]]))
    # plant.get_actuation_input_port().FixValue(plant_context, np.array([[0.0,0.0,1,1]]))
    #print diagram input ports
    print("dia ports", diagram.num_input_ports())
    
    # vector = BasicVector([0.0, 0.0, 1, 1])
    # diagram.get_input_port().FixValue(context, vector)
    #visualize

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()
    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(5)
    print("Done calc")
    meshcat.PublishRecording()
    print("Done record")
    #save to csv
    # aero_logger.FindLog(context) 
    aero_logger.WriteCSV("aero_logger.csv", process_externally_applied_spatial_force)
    print("Done Log")
    while True:
        pass

def Aerodynamics(plant,context):
        #get body obj, https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_body.html
    right_wing_body = plant.GetBodyByName("RW_Pitch")

    # cylinder_pose_lw = RigidTransform(RotationMatrix(), [-0.06, -0.02, 0.05]) #z is facing leftright, y is up , x is in
    # cylinder_pose_rw = RigidTransform(RotationMatrix(), [0.06, -0.02, 0.05]) #z is facing leftright, y is up , x is in

    # body_points = 
        #get velocity at speicfic point of the body
    # Get the body's spatial velocity at point Q, expressed in the world frame.
    velocity_world_rw = plant.EvalBodySpatialVelocityInWorld(context, right_wing_body) #velocity of origin of body
    center_pressure_body_rw = np.array([-0.06, -0.02, 0])  # relative location on wing for cp from origin
    wing_rotational_v = np.cross(velocity_world_rw.rotational(), center_pressure_body_rw) #additional velocity at cp from origin

    wing_cp_linear_v = velocity_world_rw.translational() + wing_rotational_v #total lin velocity at cp
    

    orthogonal_vec_rw = np.array([0,0,1])
    pose_world_rw = plant.EvalBodyPoseInWorld(context, right_wing_body)
    #rigidbody.rotationmatrix, multiply by orth
    up_vector = pose_world_rw.rotation().multiply(orthogonal_vec_rw) #orthogonal unit vector in world frame

    #projected linear vel onto up vec
    vel_dot_up = np.dot(wing_cp_linear_v, up_vector) 
    #1.293 is air density, 0.00765 is wing area, 1.28 drag coef
    wing_area = 0.00765
    air_density = 1.293
    drag_coef = 1.28
    flap_drag_scalar = 0.5 * air_density * wing_area * drag_coef * vel_dot_up**2 #drag force scalar
    blade_area_list = [0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085]
    print("blade area list: ", sum(blade_area_list))
    print("flap drag scalar: ", flap_drag_scalar)
    flap_drag_force = flap_drag_scalar * up_vector
    if vel_dot_up < 0:
        flap_drag_force = -flap_drag_force
    print("flap drag force: ", flap_drag_force)
    
    external_force = ExternallyAppliedSpatialForce()
    external_force.body_index = right_wing_body.index()
    external_force.p_BoBq_B = center_pressure_body_rw
    external_force.F_Bq_W = flap_drag_force

class AerodynamicsSystem(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self.plant = plant
        self.body_spatial_velocity_input_port = self.DeclareAbstractInputPort("body_spatial_velocities", AbstractValue.Make([SpatialVelocity()]))
        self.body_poses_input_port = self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform()]))
        # self.state_input_port = self.DeclareVectorInputPort("state", BasicVector(plant.num_multibody_states()))#(plant.num_positions() + plant.num_velocities()))
        self.force_output_port = self.DeclareAbstractOutputPort("force", lambda: AbstractValue.Make([ExternallyAppliedSpatialForce()]), self.CalcOutput)

    def CalcOutput(self, context, output):
        poses = self.body_poses_input_port.Eval(context)
        velocities = self.body_spatial_velocity_input_port.Eval(context)
        # body_poses = 
        # # plant_context = self.plant.CreateDefaultContext()
        # self.plant.SetPositionsAndVelocities(plant_context, state)

        right_wing_body = self.plant.GetBodyByName("RW_Pitch")
        #vel of rw
        
        velocity_world_rw = velocities[right_wing_body.index()] 
        # velocity_world_rw = self.plant.EvalBodySpatialVelocityInWorld(context, right_wing_body) #velocity of origin of body
        center_pressure_body_rw = np.array([-0.06, -0.02, 0])  # relative location on wing for cp from origin
        wing_rotational_v = np.cross(velocity_world_rw.rotational(), center_pressure_body_rw) #additional velocity at cp from origin

        wing_cp_linear_v = velocity_world_rw.translational() + wing_rotational_v #total lin velocity at cp
        

        orthogonal_vec_rw = np.array([0,0,1])
        pose_world_rw = poses[right_wing_body.index()]
        # pose_world_rw = self.plant.EvalBodyPoseInWorld(context, right_wing_body)
        # rigidbody.rotationmatrix, multiply by orth
        up_vector = pose_world_rw.rotation().multiply(orthogonal_vec_rw) #orthogonal unit vector in world frame

        # #projected linear vel onto up vec
        vel_dot_up = np.dot(wing_cp_linear_v, up_vector) 
        #1.293 is air density, 0.00765 is wing area, 1.28 drag coef
        wing_area = 0.00765
        air_density = 1.293
        drag_coef = 1.28
        flap_drag_scalar = 0.5 * air_density * wing_area * drag_coef * vel_dot_up**2 #drag force scalar
        blade_area_list = [0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085,0.00085]
        # print("blade area list: ", sum(blade_area_list))
        # print("flap drag scalar: ", flap_drag_scalar)
        flap_drag_force = flap_drag_scalar * up_vector
        if vel_dot_up < 0:
            flap_drag_force = -flap_drag_force
        # print("flap drag force: ", flap_drag_force)

        external_force = ExternallyAppliedSpatialForce()
        external_force.body_index = right_wing_body.index()
        external_force.p_BoBq_B = center_pressure_body_rw
        external_force.F_Bq_W = SpatialForce([0,0,0], flap_drag_force) 
        output.set_value([external_force])


Flap()

