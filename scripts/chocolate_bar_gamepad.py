import numpy as np

from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, LeafSystem
from pydrake.geometry import Meshcat, GeometryFrame, Box, FramePoseVector, FrameId, GeometryInstance, IllustrationProperties, Mesh, Cylinder, MakePhongIllustrationProperties
from pydrake.common.value import AbstractValue
from pydrake.common.eigen_geometry import Quaternion

from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
)

# https://github.com/kwesiRutledge/OzayGroupExploration/blob/main/drake/manip_tests/config_demo.py
def AddTriad(source_id, frame_id, scene_graph, length=.25, radius=0.01, opacity=1., X_FT=RigidTransform(), name="frame"):
    """
    Adds illustration geometry representing the coordinate frame, with the
    x-axis drawn in red, the y-axis in green and the z-axis in blue. The axes
    point in +x, +y and +z directions, respectively.
    Args:
      source_id: The source registered with SceneGraph.
      frame_id: A geometry::frame_id registered with scene_graph.
      scene_graph: The SceneGraph with which we will register the geometry.
      length: the length of each axis in meters.
      radius: the radius of each axis in meters.
      opacity: the opacity of the coordinate axes, between 0 and 1.
      X_FT: a RigidTransform from the triad frame T to the frame_id frame F
      name: the added geometry will have names name + " x-axis", etc.
    """
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2), [length / 2., 0, 0])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length), name + " x-axis")
    geom.set_illustration_properties(MakePhongIllustrationProperties([1, 0, 0, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # y-axis
    X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2),[0, length / 2., 0])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),name + " y-axis")
    geom.set_illustration_properties(MakePhongIllustrationProperties([0, 1, 0, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.])
    geom = GeometryInstance(X_FT.multiply(X_TG), Cylinder(radius, length),name + " z-axis")
    geom.set_illustration_properties(MakePhongIllustrationProperties([0, 0, 1, opacity]))
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

# SI units
WIDTH = 0.129  # m
DEPTH = 0.071  # m
HEIGHT = 0.005 # m
MASS = 0.043   # kg

class ChocolateBar(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        
        # input provides force and torque
        self.u_port = self.DeclareVectorInputPort("u_din", 6)

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
        u = self.EvalVectorInput(context, 0).CopyToVector()[0:3]
        tau = self.EvalVectorInput(context, 0).CopyToVector()[3:]
        
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
        x_ddot = 1/MASS * (R @ u + np.array([0,0,-9.81*MASS]))

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
        omega_dot = self.I_body_inv @ (np.cross(-omega, self.I_body @ omega) + tau)

        # q_dot = 1/2 G' omega, then take a derivative
        q_ddot = 0.5 * (G_dot.transpose() @ omega + G.transpose() @ omega_dot)

        derivatives.get_mutable_vector().SetFromVector(
            np.concatenate((x_dot, q_dot, x_ddot, q_ddot))
        )

class ChocolateBarPoseGenerator(LeafSystem):
    def __init__(self,frame_id,meshcat):
        LeafSystem.__init__(self)
        self.frame_id = frame_id
        self.meshcat = meshcat
        self.state_port = self.DeclareVectorInputPort("x", BasicVector(14))
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    # note that we are actually moving and rotating the FRAME
    # and because the chocolate bar is attached to the frame,
    # the chocolate bar will move and rotate with the frame
    def CalcFramePoseOutput(self, context, output):
        state = self.state_port.Eval(context)
        x = state[:3]
        q = state[3:7]

        try:
            R = RotationMatrix(Quaternion(q))
        except:
            q = q / np.linalg.norm(q)
            R = RotationMatrix(Quaternion(q))

        T = RigidTransform(R,x)

        output.get_mutable_value().set_value(self.frame_id, T)

        # update the camera pose to follow the chocolate bar
        # create a frame at the center of chocolate bar whose x-y plane is always aligned with the world
        # that is, a frame with only a yaw relative to the world.
        rpy = R.ToRollPitchYaw()
        R = RotationMatrix.MakeZRotation(rpy.yaw_angle())
        T = RigidTransform(R,x)
        y_bar = T @ np.array([0.1,-1,0.5,1])
        self.meshcat.SetCameraPose(camera_in_world=y_bar[0:3], target_in_world=x)


def main():
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(ChocolateBar())

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())
    meshcat = Meshcat()
    meshcat.SetCamera(meshcat.PerspectiveCamera(fov=60,aspect=16/9,zoom=1))
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Register the chocolate bar geometry with the scene graph.
    source_id = scene_graph.RegisterSource("my_block_source")
    frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
    geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(
            RigidTransform(RotationMatrix(), [0,0,-HEIGHT/2]),
            Mesh('meshes/chocolate_bar/Chocolate_Bar.obj', scale=0.001), 
            "my_geometry_instance")
    )
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
    box_viz = builder.AddSystem(ChocolateBarPoseGenerator(frame_id,meshcat))

    # add triad to chocolate bar frame
    AddTriad(source_id, frame_id, scene_graph, length=WIDTH, radius=HEIGHT)

    builder.Connect(plant.get_output_port(0), box_viz.get_input_port(0))
    builder.Connect(box_viz.get_output_port(0), scene_graph.get_source_pose_port(source_id))
    builder.ExportInput(plant.get_input_port())

    # now, add some obstacles
    source_id = scene_graph.RegisterSource("obstacle_1")
    geometry_id = scene_graph.RegisterAnchoredGeometry(source_id, 
        GeometryInstance(
            RigidTransform(RotationMatrix(), [0,5,0.5]),
            Box(5,1,1),
            "obstacle_1"
        )
    )
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())

    source_id = scene_graph.RegisterSource("obstacle_2")
    geometry_id = scene_graph.RegisterAnchoredGeometry(source_id, 
        GeometryInstance(
            RigidTransform(RotationMatrix(), [0,5,0.5+2]),
            Box(5,1,1),
            "obstacle_2"
        )
    )
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # set the block initial conditions
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    
    R = RotationMatrix()
    q = R.ToQuaternion().wxyz()

    meshcat.AddButton(name='Reset', keycode="KeyQ")

    print('To connect gamepad, switch to browser tab with Meshcat (refresh if necessary), and then spam the A button on the gamepad.')
    while(meshcat.GetGamepad().index == None):
        pass
    print('Connected!')
    print('Now, press and release both triggers on the gamepad to start the simulation.')
    while(meshcat.GetGamepad().axes[2] != -1 or meshcat.GetGamepad().axes[5] != -1):
        pass
    print('Simulation starting!')

    plant_context.get_mutable_continuous_state_vector().SetFromVector(
        [0,0,0,    # position
        q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
        0,0,0,    # velocity
        0,0,0,0]  # d/dt quaternion
    )


    # Run the simulation.
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    simulator.Initialize()

    ###################################
    ### GAMEPAD BUTTON VALUES TABLE ###
    ###################################
    # 0: A
    # 1: B
    # 2: X
    # 3: Y
    # 4: LB
    # 5: RB
    # 6: view button (middle left)
    # 7: menu button (middle right)
    # 8: xbox button (middle X)
    # 9: L3 (press left stick down)
    # 10: R3 (press right stick down)
    ###################################

    ###################################
    #### GAMEPAD AXIS VALUES TABLE ####
    ###################################
    # 0: left stick horizontal (-1  is left)
    # 1: left stick vertical (-1 is up)
    # 2: left trigger (-1 when not pressed)
    # 3: right stick horizontal (-1 is left)
    # 4: right stick vertical (-1 is up)
    # 5: right trigger (-1 when not pressed)
    # 6:7 represent dpad states:
    # (0,0) is neutral
    # (-1,0) is left
    # (1,0) is right
    # (0,-1) is up
    # (0,1) is down
    ###################################

    # this loop runs until we hit reset
    while True:

        # you must call meshcat.GetGamepad() every frame
        # in order to get the latest values
        gamepad = meshcat.GetGamepad()

        # B button pressed
        if meshcat.GetButtonClicks("Reset") > 0 or gamepad.button_values[1] == 1:

            # delete all sliders and buttons
            meshcat.DeleteAddedControls()
            meshcat.AddButton(name='Reset', keycode="KeyQ")

            # reset the simulation
            plant_context.get_mutable_continuous_state_vector().SetFromVector(
                [0,0,0,    # position
                q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
                0,0,0,    # velocity
                0,0,0,0]  # d/dt quaternion
            )
            
        
        def deadzone(x):
            y = np.zeros_like(x)
            for i in range(len(x)):
                if abs(x[i]) < 0.2:
                    y[i] = 0
                else:
                    y[i] = x[i]
            return y
            
        # apply wrench to body frame
        axes = deadzone(gamepad.axes)

        diagram.get_input_port(0).FixValue(context, 
            [
                0,
                0,
                MASS*9.81*(axes[2]+axes[5]+2),
                axes[1] * 1e-4*1.5, # left stick vertical controls Tx
                0,
                -axes[3] * 1e-4, # right stick horizontal controls Tz
            ]
        )
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1/60)


if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    main()
