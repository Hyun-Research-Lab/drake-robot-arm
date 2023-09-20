
from copy import copy
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
    meshcat.Set2dRenderMode(
        X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
    )

    # wire it all up
    saturation = builder.AddSystem(Saturation(min_value=[-1], max_value=[1]))
    controller = builder.AddSystem(SwingUpAndBalanceController(pendulum))

    builder.Connect(saturation.get_output_port(0), pendulum.get_input_port(0))
    builder.Connect(pendulum.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))
    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    meshcat.AddButton("Stop Simulation (Q)", "KeyQ")
    meshcat.AddButton("Restart (Space)", "Space")
    numClicks = 0

    # Set the initial conditions
    context.SetContinuousState([0.1, 0])  # theta, thetadot

    simulator.set_target_realtime_rate(1.0)

    while True:
        
        if meshcat.GetButtonClicks("Stop Simulation (Q)") == 1:
            break

        if meshcat.GetButtonClicks("Restart (Space)") > numClicks:
            numClicks += 1
            context.SetContinuousState([0.1, 0])
            context.SetTime(0)
            simulator.Initialize()
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1)
        
    meshcat.DeleteAddedControls()

if __name__ == '__main__':
    energy_sim()