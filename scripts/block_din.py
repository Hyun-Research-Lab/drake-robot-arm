import numpy as np
from pydrake.common.containers import namedview
from pydrake.common.value import Value
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, Context, LeafSystem, SystemOutput

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

class BlockDin(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("u_din", 1)
        state_index = self.DeclareContinuousState(1,1,0) # Two state variables, x and xdot
        self.DeclareStateOutputPort("x", state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):

        # get state from context
        x = context.get_continuous_state_vector().CopyToVector()

        # get input port from context
        u = self.EvalVectorInput(context, 0).CopyToVector()[0]

        xdot = x[1]
        xddot = u

        # set the derivative of xdot equal to xddot
        derivatives.get_mutable_vector().SetFromVector(np.array([xdot,xddot]))

class StupidController(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareVectorOutputPort("y_control", 1, calc=self.CalcOutput)
        self.DeclareVectorInputPort("u_control", 2)

    def CalcOutput(self, context, output):
        # get the current value of the vector input port
        u = self.EvalVectorInput(context, 0).CopyToVector()[0]
        y = -1 if u > 0 else 1
        output.SetAtIndex(0, y)

# create the diagram
builder = DiagramBuilder()

plant = builder.AddSystem(BlockDin())
controller = builder.AddSystem(StupidController())

# wire it all up
builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
builder.Connect(controller.get_output_port(0), plant.get_input_port(0))

# Setup visualization
# scene_graph = builder.AddSystem(SceneGraph())
# meshcat = StartMeshcat()
# MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
# meshcat.Set2dRenderMode(
#     X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 1, 0])
# )

# create logger to capture the plant output
logger = LogVectorOutput(plant.get_output_port(0), builder)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# # Set up a simulator to run this diagram
simulator = Simulator(diagram, diagram_context)
context = simulator.get_mutable_context()
# simulator.set_target_realtime_rate(1.0)

plant_context = diagram.GetMutableSubsystemContext(plant, context)
plant_context.get_mutable_continuous_state_vector().SetFromVector([np.pi/2, 0.0])

# Run the simulation.
simulator.Initialize()
simulator.AdvanceTo(10.0)

# print sample times and data
log = logger.FindLog(context)
print(log.sample_times())
print(log.data())

# plot the data with matplotlib
plt.figure()
plt.plot(log.sample_times(), log.data().transpose())
plt.legend(["x", "x_dot"])
plt.xlabel("time (s)")
plt.grid()
plt.show()
