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

import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    StartMeshcat,
)
from pydrake.examples import PendulumGeometry, PendulumPlant
from libs.meshcat_utils import MeshcatSliders
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

def pendulum_simulation():
    builder = DiagramBuilder()
    pendulum = builder.AddSystem(PendulumPlant())
    meshcat = StartMeshcat()
    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    PendulumGeometry.AddToBuilder(
        builder, pendulum.get_state_output_port(), scene_graph
    )
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    meshcat.Delete()
    meshcat.Set2dRenderMode(
        X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
    )

    # Setup slider input
    meshcat.AddSlider("u", min=-5, max=5, step=0.1, value=0.0)
    torque_system = builder.AddSystem(MeshcatSliders(meshcat, ["u"]))
    builder.Connect(torque_system.get_output_port(), pendulum.get_input_port())

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    meshcat.AddButton("Stop Simulation")

    # Set the initial conditions
    context.SetContinuousState([0.5, 0])  # theta, thetadot

    simulator.set_target_realtime_rate(1.0)

    print("Use the slider in the MeshCat controls to apply elbow torque.")
    print("Press 'Stop Simulation' in MeshCat to continue.")
    while True:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)

if __name__ == "__main__":
    pendulum_simulation()
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
