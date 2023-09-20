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
    Simulator,
    Simulator,
    StartMeshcat,
    CoulombFriction,
    HalfSpace,
    LogVectorOutput,
    SpatialVelocity
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
    # plant.RegisterVisualGeometry(
    #         plant.world_body(),
    #         X_GroundOrigin,
    #         HalfSpace(),
    #         "ground_visual",
    #         nontransparent_color)

# create the builder and load the cylinder model
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
cylinder_model = Parser(plant).AddModelFromFile("./sdf/cylinder.sdf")
AddGround(plant) # https://www.kwesirutledge.info/thoughts/drake1
plant.Finalize()

# set up a logger to log the state of the the cylinder model
state_logger = LogVectorOutput(plant.get_state_output_port(cylinder_model), builder)
state_logger.set_name("state_logger")

# connect Meshcat to our scene
meshcat = StartMeshcat()
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
diagram = builder.Build()

# set free body pose
diagram_context = diagram.CreateDefaultContext()
p_WBlock = [0.0, 0.0, 2]
R_WBlock = RotationMatrix.MakeXRotation(np.pi/4.1)
X_WBlock = RigidTransform(R_WBlock,p_WBlock)
plant.SetFreeBodyPose(plant.GetMyContextFromRoot(diagram_context),plant.GetBodyByName("cylinder_link", cylinder_model),X_WBlock)

plant.SetFreeBodySpatialVelocity(
    plant.GetBodyByName("cylinder_link", cylinder_model),
    SpatialVelocity(np.zeros(3),np.array([0.0,0.0,0.0])),
    plant.GetMyContextFromRoot(diagram_context))

# Set up simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_every_time_step(True)

# Run simulation
meshcat.StartRecording()
simulator.Initialize()
simulator.AdvanceTo(2)
meshcat.PublishRecording()

# Collect Data
state_log = state_logger.FindLog(diagram_context)
log_times  = state_log.sample_times()
state_data = state_log.data()
print(state_data.shape)

# use matplotlib to plot the state_data as a function of time
plt.figure()
plt.plot(log_times,state_data[0,:])
plt.plot(log_times,state_data[1,:])
plt.plot(log_times,state_data[2,:])
plt.plot(log_times,state_data[3,:])
plt.plot(log_times,state_data[4,:])
plt.plot(log_times,state_data[5,:])
# enable the grid
plt.grid()
# create legend
plt.legend(["x","y","z","roll","pitch","yaw"])
plt.show()


while True:
    pass
