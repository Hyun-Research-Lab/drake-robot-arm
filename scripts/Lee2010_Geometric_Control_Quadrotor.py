import numpy as np
from scipy.linalg import logm
from pydrake.all import (
    Context,
    ContinuousState,
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

# helper functions
def VeeMap(R):
    return np.array([R[2,1], R[0,2], R[1,0]])

def HatMap(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

# Geometric tracking control of a quadrotor UAV on SE(3). (Lee, 2010)
# https://ieeexplore.ieee.org/abstract/document/5717652

class Quadrotor(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        
        # input vector:
        # u(0) = f = f_1 + f_2 + f_3 + f_4
        # u(1:) = M = [M_1, M_2, M_3]
        self.u_port = self.DeclareVectorInputPort("u_din", 4)

        # state variables are as follows:
        # 0:2 - x,y,z (position)
        # 3:6 - unit quaternion representing rotation
        # 7:9 - x_dot, y_dot, z_dot (linear velocity)
        # 10:13 - q_dot
        position_state_index = self.DeclareContinuousState(7,7,0)

        # inertia tensor in body frame (3x3 matrix)
        self.J = np.diag([0.0820, 0.0845, 0.1377])
        self.J_inv = np.linalg.inv(self.J)
        self.m = 4.34
        self.g = 9.81

        self.DeclareStateOutputPort("x", position_state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        # get state from context
        state = context.get_continuous_state_vector().CopyToVector()

        # extract f, M from the input port
        u = self.EvalVectorInput(context, 0).CopyToVector()
        f = u[0]
        M = u[1:]
        
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

        # Accleration in Spatial Frame
        e3 = np.array([0,0,1])
        # print(f)
        x_ddot = 1/self.m * (self.m*self.g * e3 - f * (R @ e3))

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
        Omega = 2 * G @ q_dot

        # omega_dot is from the Euler equation in Chapter 4 of MLS
        # Jw_dot + w x Jw = M
        Omega_dot = self.J_inv @ (np.cross(-Omega, self.J @ Omega) + M)

        # q_dot = 1/2 G^T Omega, then take a derivative
        q_ddot = 0.5 * (G_dot.transpose() @ Omega + G.transpose() @ Omega_dot)

        derivatives.get_mutable_vector().SetFromVector(
            np.concatenate((x_dot, q_dot, x_ddot, q_ddot))
        )

# controller is a discrete-time system
class QuadrotorController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # input vector: current states from the plant
        self.DeclareVectorInputPort("u", BasicVector(14))

        # get access to the previous values of the controller
        self.DeclareDiscreteState(1+3+3+3+9) # time, Omega, Omega_d, Omega_d_dot, Rd
        self.DeclarePeriodicDiscreteUpdateEvent(period_sec=0.001, offset_sec=0.0, update=self.MyUpdate)
        
        # output = [f, M] where f is a scalar and M is a 3x1 vector
        self.DeclareVectorOutputPort("y", BasicVector(4), self.CalcOutput)

        # controller paramters
        self.m = 4.34           # mass of quadrotor
        self.g = 9.81           # gravity
        self.k_x = 16*self.m     # position gain
        self.k_v = 5.6*self.m    # velocity gain
        self.k_R = 8.81          # attitude gain
        self.k_Omega = 2.54          # angular velocity gain
        self.J = np.diag([0.0820, 0.0845, 0.1377])
        self.J_inv = np.linalg.inv(self.J)

        # desired trajectory information
        self.xd = np.array([0,0,0])
        self.xd_dot = np.array([0,0,0])
        self.xd_ddot = np.array([0,0,0])
        self.b1d = np.array([0,0,0])

    def ComputeDesiredTrajectory(self, context):
        # this function must be changed for a different target trajectory
        t = context.get_time()
        self.xd = np.array([0.4*t, 0.4*np.sin(np.pi*t), 0.6*np.cos(np.pi*t)])
        self.xd_dot = np.array([0.4, 0.4*np.pi*np.cos(np.pi*t), -0.6*np.pi*np.sin(np.pi*t)])
        self.xd_ddot = np.array([0.0, -0.4*np.pi**2*np.sin(np.pi*t), -0.6*np.pi**2*np.cos(np.pi*t)])
        self.b1d = np.array([np.cos(np.pi*t), np.sin(np.pi*t), 0])

    # update every time step
    def MyUpdate(self, context, discrete_state):

        prev_state = context.get_discrete_state().value()
        prev_time = prev_state[0]
        prev_Omega = prev_state[1:4]
        prev_Omega_d = prev_state[4:7]
        prev_Omega_d_dot = prev_state[7:10]
        prev_Rd = prev_state[10:].reshape(3,3)

        
        # update time
        next_state = discrete_state.get_mutable_value()
        next_state[0] = 1.005*prev_time + 1

        # print(prev_state)
        
        # next_time = 6
        # discrete_state.set_value(6)

        # get the most recent state of the controller
        # prev_time = context.get_discrete_state()[0]

        # next_time[0] = 1.005*prev_time+1
        # # next_Omega[:] = 1.005*prev_Omega + np.ones(3)
        
        # print(next_time[0])

        # get the current of the quadrotor plant
        # quadrotor_state = self.EvalVectorInput(context, 0).CopyToVector()
        # x = quadrotor_state[:3]   # positions
        # q = quadrotor_state[3:7]  # quaternion (rotation)
        # x_dot = quadrotor_state[7:10] # velocity
        # q_dot = quadrotor_state[10:]  # d/dt quaternion

        # q = q / np.linalg.norm(q)
        # R = RotationMatrix(Quaternion(q)).matrix()

        # # get angular velocity Omega from quaternion and its derivative
        # q0,q1,q2,q3 = q
        # G = np.array([
        #     [-q1, q0, q3, -q2],
        #     [-q2, -q3, q0, q1],
        #     [-q3, q2, -q1, q0]
        # ])
        # Omega = 2 * G @ q_dot

        # # get the desired trajectory
        # self.ComputeDesiredTrajectory(context)
        
        # # calculate b3d
        # e_x = x - self.xd
        # e_v = x_dot - self.xd_dot

        # e3 = np.array([0,0,1])
        # num = -self.k_x*e_x - self.k_v*e_v - self.m*self.g*e3 + self.m*self.xd_ddot
        # b3d = -num / np.linalg.norm(num)

        # # calculate total thrust
        # f = np.dot(-num, R @ e3)
        # # print(f)

        # # calculate b2d
        # num = np.cross(b3d, self.b1d)
        # b2d = num / np.linalg.norm(num)

        # # calculate Rd
        # b1 = np.cross(b2d, b3d)
        # Rd = np.array([b1, b2d, b3d])

        # # calculate Omega_d, Omega_d_dot
        # Omega_d_hat = 1/dt * logm(self.Rd_prev.transpose() @ Rd)
        # Omega_d = self.VeeMap(Omega_d_hat)
        # Omega_d_dot = 1/dt * (Omega_d - self.Omega_d_prev)

        # # compute error terms
        # e_R = 0.5*self.VeeMap(Rd.transpose() @ R - R.transpose() @ Rd)
        # e_Omega = Omega - R.transpose() @ Rd @ Omega_d

        # # compute total moment
        # M = -self.k_R*e_R - self.k_Omega*e_Omega + np.cross(Omega, self.J @ Omega) - \
        #     self.J @ (self.HatMap(Omega) @ R.transpose() @ Rd @ Omega_d - R.transpose() @ Rd @ Omega_d_dot)
        
        # # update previous values
        # self.t_prev = t
        # self.Omega_prev = Omega
        # self.Omega_d_prev = Omega_d
        # self.Rd_prev = Rd

    def CalcOutput(self, context, output):

        # output to plant
        # output.set_value(np.array([f,M[0],M[1],M[2]]))
        output.set_value(np.array([50,0,0,0]))

class PoseSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.state_port = self.DeclareVectorInputPort("quadrotor_state", BasicVector(14))
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
        X = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]]) # world frame to Lee's aerospace spatial frame
        X_WB = X @ RigidTransform(R,x).GetAsMatrix4()

        for frame_id in frame_pose_vectors.ids():
            old_pose = frame_pose_vectors.value(frame_id).GetAsMatrix4()
            new_pose = RigidTransform(X_WB @ old_pose)
            frame_pose_vectors.set_value(frame_id, new_pose)
        output.set_value(frame_pose_vectors)


def main():
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    quadrotor_model = builder.AddSystem(MultibodyPlant(time_step=0.0))
    poser = builder.AddSystem(PoseSystem())
    plant = builder.AddSystem(Quadrotor())
    controller = builder.AddSystem(QuadrotorController())
    
    # very important! Call RegisterAsSourceForSceneGraph before calling AddModels()
    source_id = quadrotor_model.RegisterAsSourceForSceneGraph(scene_graph)
    Parser(quadrotor_model).AddModels("./resources/quadrotor_Lee2010.urdf")
    quadrotor_model.Finalize()
    
    # We go off script here. Instead of directly connecting the bicopter model to the scene graph,
    # we insert the PoseSystem in between. This will allow us to set the pose of the bicopter model
    # whenever the bicopter model (MultibodyPlant) gives a input query for the pose.
    builder.Connect(scene_graph.get_query_output_port(), quadrotor_model.get_geometry_query_input_port())
    builder.Connect(quadrotor_model.get_geometry_poses_output_port(), poser.GetInputPort("source_pose"))
    builder.Connect(poser.get_output_port(), scene_graph.get_source_pose_port(source_id))

    builder.Connect(controller.get_output_port(), plant.get_input_port())
    builder.Connect(plant.get_output_port(), poser.GetInputPort("quadrotor_state"))
    builder.Connect(plant.get_output_port(), controller.get_input_port())

    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    quadrotor_model.mutable_gravity_field().set_gravity_vector([0, 0, 0])

    # https://github.com/RobotLocomotion/drake/blob/67671ab0c97be45434a8b6f9609901f685c55462/doc/_pages/troubleshooting.md
    # github.com/RobotLocomotion/drake/ --> drake/doc/_pages/troubleshooting.md
    root_context = diagram.CreateDefaultContext()
    quadrotor_model_context = quadrotor_model.GetMyContextFromRoot(root_context=root_context)
    plant_context = plant.GetMyContextFromRoot(root_context=root_context)
    
    # initial conditions for plant
    R = RotationMatrix()
    q = R.ToQuaternion().wxyz()
    plant_context.get_mutable_continuous_state_vector().SetFromVector(
        [0,0,0,    # position
        q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
        0,0,0,    # velocity
        0,0,0,0]  # d/dt quaternion
    )
    
    simulator = Simulator(diagram, root_context)
    simulator.Initialize()

    meshcat.StartRecording()
    # simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()

    while True:
        pass

if __name__ == "__main__":
    main()
