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


import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
)
from pydrake.examples import PendulumGeometry, PendulumPlant
from libs.meshcat_utils import MeshcatSliders#, #StartMeshcat
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

from pydrake.examples import PendulumGeometry, PendulumParams, PendulumPlant
from copy import copy

import matplotlib.pyplot as plt
import mpld3
import numpy as np
from IPython.display import display
from pydrake.all import (
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    VectorLogSink,
    VectorSystem,
    wrap_to,
)

class EnergyShapingController(VectorSystem):
    def __init__(self, pendulum):
        VectorSystem.__init__(self, 2, 1)
        self.pendulum = pendulum
        self.pendulum_context = pendulum.CreateDefaultContext()
        self.SetPendulumParams(PendulumParams())

    def SetPendulumParams(self, params):
        self.pendulum_context.get_mutable_numeric_parameter(0).SetFromVector(
            params.CopyToVector()
        )
        self.pendulum_context.SetContinuousState([np.pi, 0])
        self.desired_energy = self.pendulum.EvalPotentialEnergy(
            self.pendulum_context
        )

    def DoCalcVectorOutput(self, context, pendulum_state, unused, output):
        self.pendulum_context.SetContinuousState(pendulum_state)
        params = self.pendulum_context.get_numeric_parameter(0)
        theta = pendulum_state[0]
        thetadot = pendulum_state[1]
        total_energy = self.pendulum.EvalPotentialEnergy(
            self.pendulum_context
        ) + self.pendulum.EvalKineticEnergy(self.pendulum_context)
        output[:] = params.damping() * thetadot - 0.1 * thetadot * (
            total_energy - self.desired_energy
        )


def PhasePlot(pendulum):
    phase_plot = plt.figure()
    ax = phase_plot.gca()
    theta_lim = [-np.pi, 3.0 * np.pi]
    ax.set_xlim(theta_lim)
    ax.set_ylim(-10.0, 10.0)

    theta = np.linspace(theta_lim[0], theta_lim[1], 601)  # 4*k + 1
    thetadot = np.zeros(theta.shape)
    context = pendulum.CreateDefaultContext()
    params = context.get_numeric_parameter(0)
    context.SetContinuousState([np.pi, 0])
    E_upright = pendulum.EvalPotentialEnergy(context)
    E = [E_upright, 0.1 * E_upright, 1.5 * E_upright]
    for e in E:
        for i in range(theta.size):
            v = (
                e
                + params.mass()
                * params.gravity()
                * params.length()
                * np.cos(theta[i])
            ) / (0.5 * params.mass() * params.length() * params.length())
            if v >= 0:
                thetadot[i] = np.sqrt(v)
            else:
                thetadot[i] = float("nan")
        ax.plot(theta, thetadot, color=[0.6, 0.6, 0.6])
        ax.plot(theta, -thetadot, color=[0.6, 0.6, 0.6])

    return ax


def energy_shaping_demo():
    builder = DiagramBuilder()

    pendulum = builder.AddSystem(PendulumPlant())
    ax = PhasePlot(pendulum)
    saturation = builder.AddSystem(Saturation(min_value=[-3], max_value=[3]))
    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    controller = builder.AddSystem(EnergyShapingController(pendulum))
    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(
        controller.get_output_port(0), saturation.get_input_port(0)
    )

    logger = builder.AddSystem(VectorLogSink(2))
    builder.Connect(pendulum.get_output_port(0), logger.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    for i in range(5):
        context.SetTime(0.0)
        context.SetContinuousState(
            np.random.randn(
                2,
            )
        )
        simulator.Initialize()
        simulator.AdvanceTo(4)
        log = logger.FindLog(context)
        ax.plot(log.data()[0, :], log.data()[1, :])
        log.Clear()

    display(mpld3.display())


# energy_shaping_demo()
def BalancingLQR(pendulum):
    context = pendulum.CreateDefaultContext()

    pendulum.get_input_port(0).FixValue(context, [0])
    context.SetContinuousState([np.pi, 0])

    Q = np.diag((10.0, 1.0))
    R = [1]

    linearized_pendulum = Linearize(pendulum, context)
    (K, S) = LinearQuadraticRegulator(
        linearized_pendulum.A(), linearized_pendulum.B(), Q, R
    )
    return (K, S)


class SwingUpAndBalanceController(VectorSystem):
    def __init__(self, pendulum):
        VectorSystem.__init__(self, 2, 1)
        (self.K, self.S) = BalancingLQR(pendulum)
        self.energy_shaping = EnergyShapingController(pendulum)
        self.energy_shaping_context = (
            self.energy_shaping.CreateDefaultContext()
        )

        # TODO(russt): Add a witness function to tell the simulator about the
        # discontinuity when switching to LQR.

    def DoCalcVectorOutput(self, context, pendulum_state, unused, output):
        xbar = copy(pendulum_state)
        xbar[0] = wrap_to(xbar[0], 0, 2.0 * np.pi) - np.pi

        # If x'Sx <= 2, then use the LQR controller
        if xbar.dot(self.S.dot(xbar)) < 2.0:
            output[:] = -self.K.dot(xbar)
        else:
            self.energy_shaping.get_input_port(0).FixValue(
                self.energy_shaping_context, pendulum_state
            )
            output[:] = self.energy_shaping.get_output_port(0).Eval(
                self.energy_shaping_context
            )


def swing_up_and_balance_demo(show=False):
    builder = DiagramBuilder()
    meshcat = StartMeshcat()
    pendulum = builder.AddSystem(PendulumPlant())
    ax = PhasePlot(pendulum)
    saturation = builder.AddSystem(Saturation(min_value=[-3], max_value=[3]))
    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    controller = builder.AddSystem(SwingUpAndBalanceController(pendulum))
    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(
        controller.get_output_port(0), saturation.get_input_port(0)
    )


    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    PendulumGeometry.AddToBuilder(
        builder, pendulum.get_state_output_port(), scene_graph
    )
    meshcat.Set2dRenderMode(
        X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
    )
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # logger = builder.AddSystem(VectorLogSink(2))
    # builder.Connect(pendulum.get_output_port(0), logger.get_input_port(0))

    meshcat.AddSlider("u", min=-5, max=5, step=0.1, value=0.0)
    # torque_system = builder.AddSystem(MeshcatSliders(meshcat, ["u"]))
    # builder.Connect(torque_system.get_output_port(1), pendulum.get_input_port(1))


    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()



    simulator.set_target_realtime_rate(1.0)
    context.SetContinuousState([0.5, 0]) 
    for i in range(5):
        context.SetTime(0.0)
        context.SetContinuousState(
            np.random.randn(
                2,
            )
        )
        simulator.Initialize()
        for z in range(100):
            simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
        # log = logger.FindLog(context)
        # ax.plot(log.data()[0, :], log.data()[1, :])
        # log.Clear()
    # while True:
    #     simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
    # ax.set_xlim(np.pi - 3.0, np.pi + 3.0)
    # ax.set_ylim(-5.0, 5.0)
    # display(mpld3.display())




if __name__ == "__main__":
    swing_up_and_balance_demo()
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
