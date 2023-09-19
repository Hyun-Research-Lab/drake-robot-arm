import numpy as np
from pydrake.common.containers import namedview
from pydrake.common.value import Value
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, LeafSystem

from copy import copy
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    LogVectorOutput,
    Saturation,
    SceneGraph,
    Simulator,
    VectorSystem
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

# Define the system. xddot = u
class BlockDin(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareContinuousState(2) # Two state variables, x and xdot

        # pass a dependency ticket when declaring the vector output port
        self.DeclareVectorOutputPort(name="y", size=1, calc=self.CalcOutput)
        self.DeclareVectorInputPort("u", 1)
        
    def DoCalcTimeDerivatives(self, context, derivatives):

        # get state from context
        x = context.get_continuous_state_vector().GetAtIndex(0)
        xdot = context.get_continuous_state_vector().GetAtIndex(1)

        # get input port from context
        u = self.EvalVectorInput(context, 0).GetAtIndex(0)
        xddot = u

        # set the derivative of xdot equal to xddot
        derivatives.get_mutable_vector().SetAtIndex(0, xdot)
        derivatives.get_mutable_vector().SetAtIndex(1, xddot)

    def CalcOutput(self, context, output):
        y = context.get_continuous_state_vector().GetAtIndex(0)
        output = y


class StupidController(VectorSystem):
    def __init__(self):
        VectorSystem.__init__(self,1,1)

    def DoCalcVectorOutput(self, context, state, unused, output):
        if state > 0:
            output[:] = -1
        else:    
            output[:] = 1

# create the diagram
builder = DiagramBuilder()

plant = builder.AddSystem(BlockDin())
controller = builder.AddSystem(StupidController())

# wire it all up
builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
builder.Connect(controller.get_output_port(0), plant.get_input_port(0))

# Setup visualization
scene_graph = builder.AddSystem(SceneGraph())
meshcat = StartMeshcat()
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
meshcat.Set2dRenderMode(
    X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
)

# create logger to capture the plant output
logger = LogVectorOutput(plant.get_output_port(0), builder)

diagram = builder.Build() # error!

# # Set up a simulator to run this diagram
# simulator = Simulator(diagram)
# simulator.set_target_realtime_rate(1.0)

# context = simulator.get_mutable_context()
# context.SetContinuousState([3,0])

# while True:
#     simulator.AdvanceTo(simulator.get_context().get_time() + 1)

# # Run the simulation.
# simulator.AdvanceTo(4.0)

# # print sample times and data
# log = logger.FindLog(context)
# print(log.sample_times())
# print(log.data())