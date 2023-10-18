import numpy as np
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RotationMatrix, ClosestQuaternion
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, Context, LeafSystem, SystemOutput
from pydrake.geometry import Box, GeometryFrame, FramePoseVector, GeometryInstance, IllustrationProperties, Mesh
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
        q = state[3:7]

        try:
            q = Quaternion(q)
        except:
            q = q / np.linalg.norm(q)
            q = Quaternion(q)

        output.get_mutable_value().set_value(self.frame_id, RigidTransform(
            RotationMatrix(quaternion=q),
            x
        ))

def main():
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(ChocolateBar())

    # Setup visualization
    scene_graph = builder.AddSystem(SceneGraph())

    # Register the geometry with the scene graph.
    source_id = scene_graph.RegisterSource("my_block_source")
    frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
    geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(RigidTransform(), Mesh('meshes/chocolate_bar/Chocolate_Bar.obj', scale=0.01), "my_geometry_instance"))
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
    box_viz = builder.AddSystem(ChocolateBarPoseGenerator(frame_id))
    
    # must add the scene graph to the visualizer to see anything
    meshcat = StartMeshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    builder.Connect(plant.get_output_port(0), box_viz.get_input_port(0))
    builder.Connect(box_viz.get_output_port(0), scene_graph.get_source_pose_port(source_id))

    builder.ExportInput(plant.get_input_port())

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

        print(gamepad.axes)

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
                axes[3] * 1e-4, # right stick horizontal controls Tz
            ]
        )
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1/30)


if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    main()
