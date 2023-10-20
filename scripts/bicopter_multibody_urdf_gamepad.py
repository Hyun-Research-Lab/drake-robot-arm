import numpy as np
from pydrake.all import (
    DiagramBuilder,
    Simulator,
    Parser,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    Meshcat,
    JointIndex,
    BodyIndex,
    LeafSystem,
    RotationMatrix,
    RigidTransform,
    Quaternion,
    BasicVector,
    AbstractValue,
    FramePoseVector,
    SceneGraph,
    MultibodyPlant,
    IllustrationProperties,
)

# bicopter parameters (inertial calculations)
WIDTH = 180 * 1e-3
DEPTH = 30 * 1e-3
HEIGHT = 60 * 1e-3
MASS = 0.350  # kg

class Bicopter(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        
        # input vector:
        # u[0] = left BLDC motor force magnitude
        # u[1] = right BLDC motor force magnitude
        # u[2] = left servo motor angle (radians)
        # u[3] = right servo motor angle (radians)
        self.u_port = self.DeclareVectorInputPort("u_din", 4)

        # state variables are as follows:
        # 0:2 - x,y,z (position)
        # 3:6 - unit quaternion representing rotation
        # 7:9 - x_dot, y_dot, z_dot (linear velocity)
        # 10:13 - q_dot
        position_state_index = self.DeclareContinuousState(7,7,0)

        # inertia tensor in body frame (3x3 matrix)
        # currently using same inertia tensor as chocolate bar.
        # TODO: use parallel axis theorem to obtain a better estimate
        self.I_body = MASS/12 * np.diag([DEPTH**2 + HEIGHT**2, WIDTH**2 + HEIGHT**2, WIDTH**2 + DEPTH**2])
        self.I_body_inv = np.linalg.inv(self.I_body)

        self.DeclareStateOutputPort("x", position_state_index)
        
    # https://github.com/RussTedrake/underactuated/blob/b0fdc68b862df7bf7ccaec6b4c513762597e0ce7/underactuated/quadrotor2d.py#L23
    def DoCalcTimeDerivatives(self, context, derivatives):

        # get state from context
        state = context.get_continuous_state_vector().CopyToVector()
        u = self.EvalVectorInput(context, 0).CopyToVector()

        # calculate body frame force applied by motors and servos (Adjoint Transform)
        # L = distance from Bicopter center of mass to force in (x) direction
        # h = distance from Bicopter CoM to pivot point in (z) direction
        # d = distance from pivot point to thrust vector location
        L = 90 * 1e-3
        #h = 20 * 1e-3
        h = 0 # 20
        d = 0 * 1e-3 # 38

        fz = u[0]
        theta = u[2]
        tz = 0 # no torque for now

        Fb_L = np.array([
            fz*(d*np.sin(theta) - h*np.sin(theta)),
            tz*np.sin(theta) + L*fz*np.cos(theta),
            tz*np.cos(theta) - L*fz*np.sin(theta),
            0,
            fz*np.sin(theta),
            fz*np.cos(theta)
        ])

        fz = u[1]
        theta = u[3]
        tz = 0 # no torque for now
        
        Fb_R = np.array([
            fz*(d*np.sin(theta) - h*np.sin(theta)),
            tz*np.sin(theta) - L*fz*np.cos(theta),
            tz*np.cos(theta) + L*fz*np.sin(theta),
            0,
            fz*np.sin(theta),
            fz*np.cos(theta)
        ])

        Fb = Fb_L + Fb_R
        
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
        x_ddot = 1/MASS*(R @ Fb[3:] + np.array([0,0,-9.81*MASS]))
        x_ddot = np.array([0,0,0])
        x_ddot = 1/MASS*(R @ Fb[3:])

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
        # tau is Fb[0:3]
        omega_dot = self.I_body_inv @ (np.cross(-omega, self.I_body @ omega) + Fb[0:3])

        # q_dot = 1/2 G' omega, then take a derivative
        q_ddot = 0.5 * (G_dot.transpose() @ omega + G.transpose() @ omega_dot)

        derivatives.get_mutable_vector().SetFromVector(
            np.concatenate((x_dot, q_dot, x_ddot, q_ddot))
        )

class BicopterPoseSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.state_port = self.DeclareVectorInputPort("plant_state", BasicVector(14))
        self.source_pose_port = self.DeclareAbstractInputPort("source_pose", AbstractValue.Make(FramePoseVector()))
        self.DeclareAbstractOutputPort("y", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    def CalcFramePoseOutput(self, context, output):
        state = self.state_port.Eval(context)
        x = state[:3]
        q = state[3:7]

        # normalize q to obtain a unit quaternion
        q /= np.linalg.norm(q)
        R = RotationMatrix(Quaternion(q))

        frame_pose_vectors = self.source_pose_port.Eval(context)
        X_WB = RigidTransform(R,x).GetAsMatrix4()

        for frame_id in frame_pose_vectors.ids():
            old_pose = frame_pose_vectors.value(frame_id).GetAsMatrix4()
            new_pose = RigidTransform(X_WB @ old_pose)
            frame_pose_vectors.set_value(frame_id, new_pose)
        output.set_value(frame_pose_vectors)


def main():
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    bicopter_model = builder.AddSystem(MultibodyPlant(time_step=0.0))
    poser = builder.AddSystem(BicopterPoseSystem())
    plant = builder.AddSystem(Bicopter())
    
    # very important! Call RegisterAsSourceForSceneGraph before calling AddModels()
    source_id = bicopter_model.RegisterAsSourceForSceneGraph(scene_graph)
    Parser(bicopter_model).AddModels("./resources/bicopter.urdf")
    bicopter_model.Finalize()
    
    # We go off script here. Instead of directly connecting the bicopter model to the scene graph,
    # we insert the BicopterPoseSystem in between. This will allow us to set the pose of the bicopter model
    # whenever the bicopter model (MultibodyPlant) gives a input query for the pose.
    builder.Connect(scene_graph.get_query_output_port(), bicopter_model.get_geometry_query_input_port())
    builder.Connect(bicopter_model.get_geometry_poses_output_port(), poser.GetInputPort("source_pose"))
    builder.Connect(poser.get_output_port(), scene_graph.get_source_pose_port(source_id))

    builder.Connect(plant.get_output_port(), poser.GetInputPort("plant_state"))
    builder.ExportInput(plant.get_input_port(), "exported_plant_input")

    inspector = scene_graph.model_inspector()
    print(inspector.GetAllGeometryIds())
    print(inspector.num_sources())
    print(inspector.GetAllSourceIds())

    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    bicopter_model.mutable_gravity_field().set_gravity_vector([0, 0, 0])
    J1 = bicopter_model.GetJointByName("J1") # right sevro
    J3 = bicopter_model.GetJointByName("J3") # left servo
    propeller_right  = bicopter_model.GetJointByName("J2")
    propeller_left  = bicopter_model.GetJointByName("J4")

    # https://github.com/RobotLocomotion/drake/blob/67671ab0c97be45434a8b6f9609901f685c55462/doc/_pages/troubleshooting.md
    # github.com/RobotLocomotion/drake/ --> drake/doc/_pages/troubleshooting.md
    root_context = diagram.CreateDefaultContext()
    bicopter_model_context = bicopter_model.GetMyContextFromRoot(root_context=root_context)
    plant_context = plant.GetMyContextFromRoot(root_context=root_context)


    # bicopter_body = bicopter_model.GetBodyByName("body")
    # bicopter_model.SetFreeBodyPose(bicopter_model_context, bicopter_body, RigidTransform([0, 0, 0]))
    
    # initial conditions for plant
    R = RotationMatrix()
    q = R.ToQuaternion().wxyz()
    plant_context.get_mutable_continuous_state_vector().SetFromVector(
        [0,0,0,    # position
        q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
        0,0,0,    # velocity
        0,0,0,0]  # d/dt quaternion
    )

    # Gamepad
    print('To connect gamepad, switch to browser tab with Meshcat (refresh if necessary), and then spam the A button on the gamepad.')
    while(meshcat.GetGamepad().index == None):
        pass
    print('Connected!')
    print('Now, press and release both triggers on the gamepad to start the simulation.')
    while(meshcat.GetGamepad().axes[2] != -1 or meshcat.GetGamepad().axes[5] != -1):
        pass
    print('Simulation starting!')
    
    simulator = Simulator(diagram, root_context)
    simulator.Initialize()

    simulator.set_target_realtime_rate(1.0)

    def deadzone(x):
        y = np.zeros_like(x)
        for i in range(len(x)):
            if abs(x[i]) < 0.1:
                y[i] = 0
            else:
                y[i] = x[i]
        return y

    while True:
        gamepad = meshcat.GetGamepad()
        axes = deadzone(gamepad.axes)

        # # B button pressed
        if gamepad.button_values[1] == 1:

            # reset the simulation
            plant_context.get_mutable_continuous_state_vector().SetFromVector(
                [0,0,0,    # position
                q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
                0,0,0,    # velocity
                0,0,0,0]  # d/dt quaternion
            )
            
        # visual only
        J1.set_angle(bicopter_model_context, axes[4]*np.pi/2)
        J3.set_angle(bicopter_model_context, axes[1]*np.pi/2)

        propeller_left.set_angle(bicopter_model_context, -simulator.get_context().get_time()*10)
        propeller_right.set_angle(bicopter_model_context, simulator.get_context().get_time()*10)

        # FL = MASS*9.81*(1+axes[2])/2
        # FR = MASS*9.81*(1+axes[5])/2
        # F = 0.5*(FL + FR)
        # diagram.get_input_port(0).FixValue(simulator.get_context(),
        #     [
        #         F + (FL-FR)*0.01, # get left trigger value
        #         F + (FR-FL)*0.01, # get right trigger value
        #         axes[1] * 0.01, # get left stick vertical
        #         axes[4] * 0.01, # right stick vertical
        #     ]
        # )

        diagram.get_input_port(0).FixValue(simulator.get_context(),
            [
                (1+axes[2])/2 * 0.1, # get left trigger value
                (1+axes[5])/2 * 0.1, # get right trigger value
                -axes[1] * np.pi/2, # get left stick vertical
                -axes[4] * np.pi/2, # right stick vertical
            ]
        )
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1/60)

if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    main()
