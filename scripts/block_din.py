import numpy as np
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, Context, LeafSystem, SystemOutput
from pydrake.geometry import Box, GeometryFrame, FramePoseVector, GeometryInstance, IllustrationProperties
from pydrake.common.value import AbstractValue

from pydrake.all import (
    DiagramBuilder,
    LogVectorOutput,
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
        self.u_port = self.DeclareVectorInputPort("u_din", 1)
        state_index = self.DeclareContinuousState(1,1,0) # Two state variables, x and xdot
        # you must use DeclareContinuousState(1,1,0)
        # AND NOT DeclareContinuousState(2) because this function requires 3 inputs:
        # num q, num v, num z. If you only have q terms (position) there will be not integration
        # and this will lead directly to algebraic loops (which suck really bad)

        self.DeclareStateOutputPort("x", state_index)
        # you must use the DeclareStateOutputPort and not DeclareVectorOutputPort
        # as otherwise you will cause an algebraic loop. Note that the states need to
        # be integrated, while a vector output is literally just a vector which has 
        # nothing to do with integration.... this was a hard bug to track down.
        # see https://github.com/RussTedrake/underactuated/blob/b0fdc68b862df7bf7ccaec6b4c513762597e0ce7/underactuated/quadrotor2d.py#L23
        
    def DoCalcTimeDerivatives(self, context, derivatives):

        # get state from context
        x = context.get_continuous_state_vector().CopyToVector()

        # get input port from context
        u = self.u_port.Eval(context)[0]

        # state space model
        xdot = x[1]
        xddot = u

        # set the derivative of xdot equal to xddot
        derivatives.get_mutable_vector().SetFromVector(np.array([xdot,xddot]))

class BoxVisualizer(LeafSystem):
    def __init__(self,frame_id):
        LeafSystem.__init__(self)
        self.frame_id = frame_id
        self.x_port = self.DeclareVectorInputPort("x", BasicVector(2)) # position and velocity from BlockDin
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        # YOU MUST DECLARE OUTPUT PORT AS AN ABSTRACT OUTPUT PORT
        # do not use a vector port, as the scene_graph input expects a frameposevector
        # see https://github.com/RussTedrake/drake/blob/c185d5dd5321ce59fb6c298e1f5e412f55750d6b/examples/acrobot/acrobot_geometry.cc#L55
        # see https://github.com/RussTedrake/drake/blob/c185d5dd5321ce59fb6c298e1f5e412f55750d6b/bindings/pydrake/systems/jupyter_widgets_examples.ipynb#L53

    def CalcFramePoseOutput(self, context, output):
        position = self.x_port.Eval(context)[0]
        output.get_mutable_value().set_value(self.frame_id, RigidTransform(
            RotationMatrix().Identity(),
            np.array([position,0,0])
        ))

class StupidController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("y_control", 1, calc=self.CalcOutput)
        self.u_port = self.DeclareVectorInputPort("u_control", 2)

    def CalcOutput(self, context, output):
        # u is the position of the block
        u = self.u_port.Eval(context)[0]
        y = -1 if u > 0 else 1
        output.SetAtIndex(0, y)

def main():
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(BlockDin())
    controller = builder.AddSystem(StupidController())

    # wire it all up
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))

    # create logger_plant to capture the plant output
    logger_plant = LogVectorOutput(plant.get_output_port(0), builder)
    logger_controller = LogVectorOutput(controller.get_output_port(0), builder)

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # set initial conditions
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant_context.get_mutable_continuous_state_vector().SetFromVector([1.0, 0.0])

    # Run the simulation.
    simulator.Initialize()
    simulator.AdvanceTo(10.0)

    # print sample times and data
    log1 = logger_plant.FindLog(context)
    log2 = logger_controller.FindLog(context)

    print(log1.sample_times())
    print(log1.data())

    # plot the data with matplotlib
    plt.figure()
    plt.plot(log1.sample_times(), log1.data().transpose())
    plt.plot(log2.sample_times(), log2.data().transpose())
    plt.legend(["x", "x_dot","u"])
    plt.xlabel("time (s)")
    plt.grid()
    plt.show()

def main_meshcat(render_mode_2D=False):
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(BlockDin())
    controller = builder.AddSystem(StupidController())

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    meshcat = StartMeshcat()

    # create a frame
    source_id = scene_graph.RegisterSource("my_block_source")
    frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
    geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(RigidTransform(), Box(1.0, 1.0, 1.0), "my_geometry_instance"))

    # must assign an illustration role to see the geometry
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())

    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    if render_mode_2D:
        meshcat.Set2dRenderMode(X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 0, 0]))

    box_viz = builder.AddSystem(BoxVisualizer(frame_id))

    # wire it all up
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))

    builder.Connect(plant.get_output_port(0), box_viz.get_input_port(0))
    builder.Connect(box_viz.get_output_port(0), scene_graph.get_source_pose_port(source_id))

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # set the block initial conditions
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant_context.get_mutable_continuous_state_vector().SetFromVector([1.0, 0.0])

    # Run the simulation.
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(30.0)
    meshcat.PublishRecording()

    while True:
        pass


if __name__ == "__main__":
    #main()
    main_meshcat(render_mode_2D=False)
    