# python libraries
import numpy as np
from IPython.display import HTML, display

# pydrake imports
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    LogVectorOutput,
    MeshcatVisualizer,
    ModelVisualizer,
    Parser,
    Simulator,
    StartMeshcat,
    RotationMatrix,
)
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from libs.meshcat_utils import MeshcatSliders

def create_scene(sim_time_step):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=sim_time_step)
    parser = Parser(plant)

    # Loading models.
    # Load the table top and the cylinder we created.
    parser.AddModels("./resources/pend.urdf")


    plant.Finalize()
    
    plant_context = plant.CreateDefaultContext()
    state_vector = plant.GetStateNames(True)
    plant.DeclareContinuousState(6)
    print(len(state_vector))
    # Create a source of constant zero actuation (replace `num_actuators` with the actual number of actuators in your system)
    # zero_actuation = ConstantVectorSource([1])

    # Add this source to the builder
    # actuation_source = builder.AddSystem(zero_actuation)

    # Connect the source to the MultibodyPlant's actuation input port
    # builder.Connect(actuation_source.get_output_port(0), plant.get_actuation_input_port())
    
    # Setup slider input
    meshcat.AddSlider("u", min=-30.0, max=30, step=0.1, value=0.0)
    force_system = builder.AddSystem(MeshcatSliders(meshcat, ["u"]))
    builder.Connect(
        force_system.get_output_port(), plant.get_actuation_input_port()
    )

    # Add visualization to see the geometries.
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    meshcat.Set2dRenderMode(
        X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    context.SetContinuousState([1, 0, 0, 0, 0, 0])
    

    return diagram


def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    # context.DeclareContinuousState(6)

    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    return simulator

def run_simulation(sim_time_step):
    diagram = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    
    # meshcat.StartRecording()
    # finish_time = 5.0
    # simulator.AdvanceTo(finish_time)
    # meshcat.PublishRecording()
    meshcat.AddButton("Stop Simulation (Q)", "KeyQ")
    meshcat.AddButton("Restart (Space)", "Space")
    numClicks = 0
    while True:
        
        if meshcat.GetButtonClicks("Stop Simulation (Q)") == 1:
            break

        if meshcat.GetButtonClicks("Restart (Space)") > numClicks:
            numClicks += 1
            # context.SetContinuousState([0.1, 0])
            # context.SetTime(0)
            simulator.Initialize()
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1)
        
    meshcat.DeleteAddedControls()
# Run the simulation with a small time step. Try gradually increasing it!
meshcat = StartMeshcat()

meshcat.AddButton('say_hello')
run_simulation(sim_time_step=0.001)
