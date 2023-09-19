import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer

def create_scene(sim_time_step):
    # Clean up the Meshcat instance.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=sim_time_step)
    parser = Parser(plant)

    # Loading models.
    # Load the table top and the cylinder we created.
    parser.AddModels("./sdf/table_wide.sdf")
    parser.AddModels("./sdf/cylinder.sdf")

    # Weld the table to the world so that it's fixed during the simulation.
    table_frame = plant.GetFrameByName("table_top_center")
    plant.WeldFrames(plant.world_frame(), table_frame)
    # Finalize the plant after loading the scene.
    plant.Finalize()
    # We use the default context to calculate the transformation of the table
    # in world frame but this is NOT the context the Diagram consumes.
    plant_context = plant.CreateDefaultContext()

    # Set the initial pose for the free bodies, i.e., the custom cylinder,
    # the cracker box, and the sugar box.
    cylinder = plant.GetBodyByName("cylinder_link")
    X_WorldTable = table_frame.CalcPoseInWorld(plant_context)
    X_TableCylinder = RigidTransform(
        RollPitchYaw(np.asarray([90, 0, 0]) * np.pi / 180), p=[0,0,3])
    X_WorldCylinder = X_WorldTable.multiply(X_TableCylinder)
    plant.SetDefaultFreeBodyPose(cylinder, X_WorldCylinder)
    
    # Add visualization to see the geometries.
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    return diagram

def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    return simulator

def run_simulation(sim_time_step):
    diagram = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    meshcat.StartRecording()
    finish_time = 5.0
    simulator.AdvanceTo(finish_time)
    meshcat.PublishRecording()

# Run the simulation with a small time step. Try gradually increasing it!
meshcat = StartMeshcat()
meshcat.AddButton('say_hello')
run_simulation(sim_time_step=0.001)

while True:
    pass