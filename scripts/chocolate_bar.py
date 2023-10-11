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
        position_state_index = self.DeclareContinuousState(3,3,0) # Position and velocity, x,y,z in world frame
        # see https://github.com/RussTedrake/underactuated/blob/b0fdc68b862df7bf7ccaec6b4c513762597e0ce7/underactuated/quadrotor2d.py#L23
        
        # TODO: add rotation

        self.DeclareStateOutputPort("x", position_state_index)
        
    # https://github.com/RussTedrake/underactuated/blob/b0fdc68b862df7bf7ccaec6b4c513762597e0ce7/underactuated/quadrotor2d.py#L23
    def DoCalcTimeDerivatives(self, context, derivatives):

        # get state from context
        x = context.get_continuous_state_vector().CopyToVector()
        u = self.EvalVectorInput(context, 0).CopyToVector()

        # dynamics
        q = x[:3] # position: x,y,z
        qdot = x[3:] # velocity, xdot, ydot, zdot
        qddot = np.array(
            [
                0,
                0,
                -9.81
            ]
        )

        derivatives.get_mutable_vector().SetFromVector(
            np.concatenate((qdot, qddot))
        )

class ChocolateBarVisualizer(LeafSystem):
    def __init__(self,frame_id):
        LeafSystem.__init__(self)
        self.frame_id = frame_id
        self.x_port = self.DeclareVectorInputPort("x", BasicVector(6)) # position and velocity, for x,y,z
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    def CalcFramePoseOutput(self, context, output):
        x,y,z = self.x_port.Eval(context)[0:3]
        output.get_mutable_value().set_value(self.frame_id, RigidTransform(
            RotationMatrix().Identity(),
            np.array([x,y,z])
        ))

class StupidController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("y_control", 1, calc=self.CalcOutput)
        self.u_port = self.DeclareVectorInputPort("u_control", 6)

    def CalcOutput(self, context, output):
        # z position of block
        z = self.u_port.Eval(context)[2]
        if z < 0:
            output.SetAtIndex(0, 10)
        else:
            output.SetAtIndex(0, 0) # no input

def main_meshcat(render_mode_2D=False):
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(BlockDin())
    controller = builder.AddSystem(StupidController())

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())

    # Register the geometry with the scene graph.
    # This is a non-trivial process. See https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html
    # in particular, the section Working with SceneGraph > Producer > Registering Geometry
    # 1. Acquire a source ID for the new geometry
    # 2. Register a frame. We used the world frame (0).
    # 3. Register non-deformable geometry (i.e. a box) with the frame.
    # 4. Assign the geometry a role (illustration)
    source_id = scene_graph.RegisterSource("my_block_source")
    frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
    geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(RigidTransform(), Box(5.75, 2.75, 0.25), "my_geometry_instance"))

    # must assign an illustration role to see the geometry
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
    
    # must add the scene graph to the visualizer to see anything
    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    if render_mode_2D:
        meshcat.Set2dRenderMode(X_WC=RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0, 0, 0]))

    # In order for the frame "my_frame" to move, we must create a leaf system
    # which tells the scene_graph what pose to assign to the frame for each point in time.
    # See the ChocolateBarVisualizer class above.
    box_viz = builder.AddSystem(ChocolateBarVisualizer(frame_id))

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
    plant_context.get_mutable_continuous_state_vector().SetFromVector([0,0,0,0,0,10])

    # Run the simulation.
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(3.0)
    meshcat.PublishRecording()

    while True:
        pass


if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    #main()
    main_meshcat(render_mode_2D=False)
