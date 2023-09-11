import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import os
import pydrake.geometry as mut

#import scene_graph
from pydrake.geometry import MeshcatVisualizer, SceneGraph
from libs.meshcat_utils import StartMeshcat
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MultibodyPlant,
    Parser,
    Propeller,
    PropellerInfo,
    RigidTransform,
    RobotDiagramBuilder,
    Saturation,
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


def MakeUr3eRobot():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, MultibodyPlant(time_step=0.0))
    # plant = builder.AddSystem(MultibodyPlant(0.0))
    parser = Parser(plant)
    curr_file_path = os.path.dirname(os.path.abspath(__file__))
    parser.AddModelFromFile(os.path.join(curr_file_path, "../urdf/ur3e_robot.urdf"))
    plant.Finalize()
    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build(), plant

def Ur3eRobotExample():
    diagram, plant = MakeUr3eRobot()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)
    print("starting simulation")
    # meshcat.AddButton("Reset Simulation")
    # while meshcat.GetButtonClicks("Reset Simulation") < 1:
    while True:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
    # meshcat.DeleteAddedControls()

if __name__ == "__main__":
    Ur3eRobotExample()
# from pydrake.visualization.model_visualizer import MeshcatVisualizer

# Build a system diagram for the simulation.
# builder = DiagramBuilder()
# pendulum = builder.AddSystem(PendulumPlant())

# # Create a MeshCat visualizer and add it to the diagram.
# vis = builder.AddSystem(MeshcatVisualizer(meshcat.Visualizer()))
# builder.Connect(pendulum.get_output_port(0), vis.get_input_port(0))

# # Complete the diagram and create a simulator.
# diagram = builder.Build()
# simulator = Simulator(diagram)
# context = simulator.get_mutable_context()

# # Set the initial state for the pendulum (angle and angular velocity).
# pendulum_context = diagram.GetMutableSubsystemContext(pendulum, context)
# pendulum_context.SetContinuousState([1.0, 0.0])  # 1.0 radian initial angle.

# # Start the simulation.
# simulator.AdvanceTo(10.0)
