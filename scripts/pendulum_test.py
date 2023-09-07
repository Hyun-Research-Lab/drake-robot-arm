import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
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



def pendulum_simulation():
    builder = DiagramBuilder()
    pendulum = builder.AddSystem(PendulumPlant())

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

    if running_as_notebook:  # Then we're not just running as a test on CI.
        simulator.set_target_realtime_rate(1.0)

        print("Use the slider in the MeshCat controls to apply elbow torque.")
        print("Press 'Stop Simulation' in MeshCat to continue.")
        while meshcat.GetButtonClicks("Stop Simulation") < 1:
            simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
    else:
        simulator.AdvanceTo(0.1)

    meshcat.DeleteAddedControls()


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
