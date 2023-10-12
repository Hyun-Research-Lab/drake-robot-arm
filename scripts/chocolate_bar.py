import numpy as np
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RotationMatrix, ClosestQuaternion
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, Context, LeafSystem, SystemOutput
from pydrake.geometry import Box, GeometryFrame, FramePoseVector, GeometryInstance, IllustrationProperties
from pydrake.common.value import AbstractValue
from pydrake.common.eigen_geometry import Quaternion

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

WIDTH = 5.75 # x0
DEPTH = 2.75 # y0
HEIGHT = 0.25 # z0
MASS = 1.0

class ChocolateBar(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        
        # input provides acceleration on body z axis
        self.u_port = self.DeclareVectorInputPort("u_din", 1)

        # see An Introduction to Physically Based Modeling
        # David Baraff (Carnegie Mellon University)
        # Chapter 2-4

        # state variables are as follows:
        # 0:2 - x,y,z (position)
        # 3:6 - unit quaternion representing rotation
        # 7:9 - x_dot, y_dot, z_dot (linear velocity)
        # 10:13 - q_dot
        position_state_index = self.DeclareContinuousState(7,7,0)

        # inertia tensor in body frame (3x3 matrix)
        self.I_body = MASS/12 * np.diag([DEPTH**2 + HEIGHT**2, WIDTH**2 + HEIGHT**2, WIDTH**2 + DEPTH**2])
        self.I_body_inv = np.linalg.inv(self.I_body)

        self.DeclareStateOutputPort("x", position_state_index)
        
    # https://github.com/RussTedrake/underactuated/blob/b0fdc68b862df7bf7ccaec6b4c513762597e0ce7/underactuated/quadrotor2d.py#L23
    def DoCalcTimeDerivatives(self, context, derivatives):

        # get state from context
        state = context.get_continuous_state_vector().CopyToVector()
        u = self.EvalVectorInput(context, 0).CopyToVector()[0]
        
        # unpack state variables
        x = state[:3]   # positions
        q = state[3:7]  # quaternion (rotation)
        x_dot = state[7:10] # velocity
        q_dot = state[10:]  # d/dt quaternion

        # normalize quaternion if necessary
        try:
            R = RotationMatrix(Quaternion(q))
        except:
            q = q / np.linalg.norm(q)
            R = RotationMatrix(Quaternion(q))

        # body acceleration (in spatial frame)
        u = 1 # overwrite the controller
        x_ddot = R @ np.array([0,0,u]) # + np.array([0,0,-9.81])

        # see table 5.18 in Quaternion.pdf
        q0,q1,q2,q3 = q
        G = np.array([
            [-q1, q0, q3, -q2],
            [-q2, -q3, q0, q1],
            [-q3, q2, -q1, q0]
        ])

        q0,q1,q2,q3 = q_dot
        G_dot = np.array([
            [-q1, q0, q3, -q2],
            [-q2, -q3, q0, q1],
            [-q3, q2, -q1, q0]
        ])

        # omega is in body frame
        omega = 2 * G @ q_dot

        # omega_dot is from the Euler equation in Chapter 4 of MLS
        # Iw = -w x Iw + tau
        tau = np.array([0, 0, 1]) # spin about z axis
        omega_dot = self.I_body_inv @ (np.cross(-omega, self.I_body @ omega) + tau)

        # q_dot = 1/2 G' omega, then take a derivative
        q_ddot = 0.5 * (G_dot.transpose() @ omega + G.transpose() @ omega_dot)

        derivatives.get_mutable_vector().SetFromVector(
            np.concatenate((x_dot, q_dot, x_ddot, q_ddot))
        )

class ChocolateBarPoseGenerator(LeafSystem):
    def __init__(self,frame_id):
        LeafSystem.__init__(self)
        self.frame_id = frame_id
        self.state_port = self.DeclareVectorInputPort("x", BasicVector(14))
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    # note that we are actually moving and rotating the FRAME
    # and because the chocolate bar is attached to the frame,
    # the chocolate bar will move and rotate with the frame
    def CalcFramePoseOutput(self, context, output):
        state = self.state_port.Eval(context)
        x = state[:3]
        q = Quaternion(state[3:7])

        output.get_mutable_value().set_value(self.frame_id, RigidTransform(
            RotationMatrix(quaternion=q),
            x
        ))

class StupidController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("y_control", 1, calc=self.CalcOutput)
        self.u_port = self.DeclareVectorInputPort("u_control", 14)

    def CalcOutput(self, context, output):
        # z position of block
        z = self.u_port.Eval(context)[2]
        # if z < 0:
        #     output.SetAtIndex(0, 3*9.81) # apply force upwards
        # else:
        #     output.SetAtIndex(0, 0) # no input (let block fall down)
        output.SetAtIndex(0, 0)

def main_meshcat():
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(ChocolateBar())
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
        GeometryInstance(RigidTransform(), Box(WIDTH, DEPTH, HEIGHT), "my_geometry_instance"))

    # must assign an illustration role to see the geometry
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
    
    # must add the scene graph to the visualizer to see anything
    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # In order for the frame "my_frame" to move, we must create a leaf system
    # which tells the scene_graph what pose to assign to the frame for each point in time.
    # See the ChocolateBarPoseGenerator class above.
    box_viz = builder.AddSystem(ChocolateBarPoseGenerator(frame_id))

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
    
    R = RotationMatrix.MakeXRotation(np.pi/4)
    q = R.ToQuaternion().wxyz()

    plant_context.get_mutable_continuous_state_vector().SetFromVector(
        [0,0,1,    # position
         q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
         0,0,0,    # velocity
         0,0,0,0]  # d/dt quaternion
    )

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
    main_meshcat()
