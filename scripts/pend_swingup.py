
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Saturation,
    SceneGraph,
    Simulator,
    VectorLogSink,
    VectorSystem,
    wrap_to,
)

from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    StartMeshcat,
)

from pydrake.examples import PendulumGeometry, PendulumParams, PendulumPlant

from libs.meshcat_utils import MeshcatSliders

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

def energy_sim():
    builder = DiagramBuilder()
    pendulum = builder.AddSystem(PendulumPlant())

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    PendulumGeometry.AddToBuilder(
        builder, pendulum.get_state_output_port(), scene_graph
    )

    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # meshcat.Delete()
    meshcat.Set2dRenderMode(
        X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
    )

    # wire it all up
    saturation = builder.AddSystem(Saturation(min_value=[-3], max_value=[3]))
    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    controller = builder.AddSystem(EnergyShapingController(pendulum))
    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(
        controller.get_output_port(0), saturation.get_input_port(0)
    )

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    meshcat.AddButton("Stop Simulation")

    # Set the initial conditions
    context.SetContinuousState([0.5, 0])  # theta, thetadot

    simulator.set_target_realtime_rate(1.0)

    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)

    meshcat.DeleteAddedControls()

if __name__ == '__main__':
    energy_sim()