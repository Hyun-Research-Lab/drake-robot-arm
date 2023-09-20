import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from copy import copy
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    DiagramBuilder,
    MeshcatVisualizer,
    LeafSystem,
    Simulator,
    Simulator,
    StartMeshcat,
    CoulombFriction,
    HalfSpace,
    LogVectorOutput,
    SpatialVelocity,
    BasicVector,
    AffineSystem,
    ConstantVectorSource,
)

def AddGround(plant):
    # Constants
    transparent_color = np.array([0.5,0.5,0.5,0])
    nontransparent_color = np.array([0.5,0.5,0.5,0.1])

    p_GroundOrigin = [0, 0.0, 0.0]
    R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
    X_GroundOrigin = RigidTransform(R_GroundOrigin,p_GroundOrigin)

    # Set Up Ground on Plant
    surface_friction = CoulombFriction(
            static_friction = 0.7,
            dynamic_friction = 0.5)
    plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

class BlockHandlerSystem(LeafSystem):
    def __init__(self,plant):
        LeafSystem.__init__(self)

        # Add the Block to the given plant
        self.plant = plant
        self.block_as_model = Parser(plant).AddModelFromFile("./sdf/cylinder_no_gravity.sdf")
        
        # Add ground to the plant
        AddGround(self.plant)

        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        # Create Input Port for the Slider Block System
        self.desired_pose_port = self.DeclareVectorInputPort("desired_pose",BasicVector(6))

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
                "measured_block_pose",
                BasicVector(6),
                self.SetBlockPose,
                {self.time_ticket()})   # indicate that this doesn't depend on any inputs, but should still be updated each timestep

    def SetBlockPose(self, context, output):
        """
        Description:
            This function sets the desired pose of the block.
        """

        # Get Desired Pose from Port
        plant_context = self.context
        pose_as_vec = self.desired_pose_port.Eval(context)

        self.plant.SetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName("cylinder_link", self.block_as_model),
            RigidTransform(RollPitchYaw(pose_as_vec[:3]),pose_as_vec[3:])
        )

        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("cylinder_link", self.block_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            plant_context
            )

        X_WBlock = self.plant.GetFreeBodyPose(
            plant_context,
            self.plant.GetBodyByName("cylinder_link", self.block_as_model)
        )

        pose_as_vector = np.hstack([RollPitchYaw(X_WBlock.rotation()).vector(), X_WBlock.translation()])

        # Create Output
        output.SetFromVector(pose_as_vector)

    def SetInitialBlockState(self,diagram_context):
        """
        Description:
            Sets the initial position to be slightly above the ground (small, positive z value)
            to be .
        """

        # Set Pose
        p_WBlock = [0.0, 0.0, 1]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi/4.1)
        X_WBlock = RigidTransform(R_WBlock,p_WBlock)
        self.plant.SetFreeBodyPose(
            self.plant.GetMyContextFromRoot(diagram_context),
            self.plant.GetBodyByName("cylinder_link", self.block_as_model),
            X_WBlock)

        # Set Velocities
        self.plant.SetFreeBodySpatialVelocity(
            self.plant.GetBodyByName("cylinder_link", self.block_as_model),
            SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
            self.plant.GetMyContextFromRoot(diagram_context))

# Connect System To Handler
# Create system that outputs the slowly updating value of the pose of the block.
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
block_handler_system = builder.AddSystem(BlockHandlerSystem(plant))

# log the output of the block handler system
state_logger = LogVectorOutput(block_handler_system.GetOutputPort("measured_block_pose"),builder)
state_logger.set_name("state_logger")

A = np.zeros((6,6))
B = np.zeros((6,1))
f0 = np.array([1,1,1,0,0,0])
C = np.eye(6)
D = np.zeros((6,1))
y0 = np.zeros((6,1))
x0 = np.array([0,0,0,0,0,1])
target_source2 = builder.AddSystem(AffineSystem(A,B,f0,C,D,y0))
target_source2.configure_default_state(x0)

# log the output of target source 2
command_logger = LogVectorOutput(target_source2.get_output_port(), builder)
command_logger.set_name("command_logger")

u0 = np.array([20])
affine_system_input = builder.AddSystem(ConstantVectorSource(u0))

# connect u0 -> target_source2 input
builder.Connect(
    affine_system_input.get_output_port(),
    target_source2.get_input_port()    
)

# connect target_source2 output -> 
builder.Connect(target_source2.get_output_port(), block_handler_system.GetInputPort("desired_pose"))

# connect Meshcat to our scene
meshcat = StartMeshcat()
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# Set initial pose and vectors
block_handler_system.SetInitialBlockState(diagram_context)

# Set up simulation
simulator = Simulator(diagram, diagram_context)
block_handler_system.context = block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)

simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(False)

# Run simulation
meshcat.StartRecording()
simulator.Initialize()
simulator.AdvanceTo(15)
meshcat.PublishRecording()

# state_log = state_logger.FindLog(diagram_context)
# log_times  = state_log.sample_times()
# state_data = state_log.data()
# print(state_data)

# command_log = command_logger.FindLog(diagram_context)
# log_times_c = command_log.sample_times()
# command_data = command_log.data()
# print(command_data)

while True:
    pass