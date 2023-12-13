import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import logm
from pydrake.all import (
    DiagramBuilder,
    Simulator,
    Parser,
    MeshcatVisualizer,
    Meshcat,
    LeafSystem,
    RotationMatrix,
    RigidTransform,
    Quaternion,
    BasicVector,
    AbstractValue,
    FramePoseVector,
    SceneGraph,
    MultibodyPlant,
    LogVectorOutput,
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
        # u = [f1 f2 f3 f4]
        self.u_port = self.DeclareVectorInputPort("u", 4)

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

        # mapping thrusts to f, M
        d = 0.315
        c = 8.004e-4
        self.T = np.array([
            [1,1,1,1],
            [0,-d,0,d],
            [d,0,-d,0],
            [-c,c,-c,c]
        ])

        self.DeclareStateOutputPort("x", position_state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        # get state from context
        state = context.get_continuous_state_vector().CopyToVector()

        # extract f, M from the input port
        u = self.T @ self.EvalVectorInput(context, 0).CopyToVector()
        f = u[0]
        M = u[1:]
        
        # unpack state variables
        q = state[3:7]  # quaternion (rotation)
        x_dot = state[7:10] # velocity
        q_dot = state[10:]  # d/dt quaternion

        # normalize quaternion if necessary
        try:
            R = RotationMatrix(Quaternion(q))
        except:
            q = q / np.linalg.norm(q)
            R = RotationMatrix(Quaternion(q))

        # Acceleration in Spatial Frame
        # note that +z is down, so mg acts in the +z world direction
        # and our force f acts in the -z body direction
        e3 = np.array([0,0,1])
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

        # omega_dot is from the Euler equation in Chapter 4 of MLS
        # Jw_dot + w x Jw = M
        # q_dot = 1/2 G^T omega, then take a derivative
        omega = 2 * G @ q_dot
        omega_dot = self.J_inv @ (np.cross(-omega, self.J @ omega) + M)
        q_ddot = 0.5 * (G_dot.transpose() @ omega + G.transpose() @ omega_dot)

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
        initial_states = np.concatenate((
            np.zeros(1), # time
            np.zeros(3), # omega_d
            np.eye(3).reshape(9), # Rd
            np.zeros(4) # output
        ))
        self.DeclareDiscreteState(initial_states)
        #self.DeclarePeriodicDiscreteUpdateEvent(period_sec=1e-2, offset_sec=5e-3, update=self.MyUpdate)
        self.DeclarePerStepDiscreteUpdateEvent(update=self.MyUpdate)
        
        # output = [f1, f2, f3, f4]
        self.DeclareVectorOutputPort("y", BasicVector(4), self.CalcOutput)

        # functions to log for comparison to Lee2010
        self.DeclareVectorOutputPort("psi", BasicVector(1), self.OutputErrorFunction)
        self.DeclareVectorOutputPort("omega", BasicVector(6), self.OutputOmegaFunction)
        self.DeclareVectorOutputPort("lyapunov", BasicVector(3), self.OutputLyapunovFunction)

        # controller paramters
        self.m = 4.34           # mass of quadrotor
        self.g = 9.81           # gravity
        self.k_x = 16*self.m     # position gain
        self.k_v = 5.6*self.m    # velocity gain
        self.k_R = 8.81          # attitude gain
        self.k_omega = 2.54          # angular velocity gain
        self.J = np.diag([0.0820, 0.0845, 0.1377])
        self.J_inv = np.linalg.inv(self.J)

        # mapping thrusts to f, M
        d = 0.315
        c = 8.004e-4
        self.T = np.array([
            [1,1,1,1],
            [0,-d,0,d],
            [d,0,-d,0],
            [-c,c,-c,c]
        ])
        self.T_inv = np.linalg.inv(self.T)

    # update every 0.001 seconds
    def MyUpdate(self, context, discrete_state):

        # desired trajectory
        t = context.get_time()

        # parse previous state
        prev_state = context.get_discrete_state().value()
        prev_time = prev_state[0]
        prev_omega_d = prev_state[1:4]
        prev_Rd = prev_state[4:13].reshape(3,3)
        dt = t - prev_time

        if SIM_NUMBER == 1:
            # Elliptical Helix Trajectory
            xd = np.array([0.4*t, 0.4*np.sin(np.pi*t), 0.6*np.cos(np.pi*t)])
            xd_dot = np.array([0.4, 0.4*np.pi*np.cos(np.pi*t), -0.6*np.pi*np.sin(np.pi*t)])
            xd_ddot = np.array([0.0, -0.4*np.pi**2*np.sin(np.pi*t), -0.6*np.pi**2*np.cos(np.pi*t)])
            #b1d = np.array([np.cos(np.pi*t), np.sin(np.pi*t), 0])
            b1d = np.array([1,0,0])
        elif SIM_NUMBER == 2:
            # Flip back over trajectory
            xd = np.zeros(3)
            xd_dot = np.zeros(3)
            xd_ddot = np.zeros(3)
            b1d = np.array([1,0,0])
        elif SIM_NUMBER == 3:
            # Flip back over trajectory
            A = 5.0
            B = -1.0
            xd = np.array([0, A*np.cos(t), B*np.sin(2*t)])
            xd_dot = np.array([0, -A*np.sin(t), 2*B*np.cos(2*t)])
            xd_ddot = np.array([0, -A*np.cos(t), -4*B*np.sin(2*t)])
            b1d = np.array([1,0,0])
        
        # get the current of the quadrotor plant
        quadrotor_state = self.EvalVectorInput(context, 0).CopyToVector()
        x = quadrotor_state[:3]   # positions
        q = quadrotor_state[3:7]  # quaternion (rotation)
        x_dot = quadrotor_state[7:10] # velocity
        q_dot = quadrotor_state[10:]  # d/dt quaternion

        try:
            R = RotationMatrix(Quaternion(q)).matrix()
        except:
            q = q / np.linalg.norm(q)
            R = RotationMatrix(Quaternion(q)).matrix()

        # q = q / np.linalg.norm(q)
        q0,q1,q2,q3 = q
        G = np.array([
            [-q1, q0, q3, -q2],
            [-q2, -q3, q0, q1],
            [-q3, q2, -q1, q0]
        ])
        omega = 2 * G @ q_dot
        
        # calculate total thrust
        e_x = x - xd
        e_v = x_dot - xd_dot
        e3 = np.array([0,0,1])
        tmp = -self.k_x*e_x - self.k_v*e_v - self.m*self.g*e3 + self.m*xd_ddot
        total_thrust = np.dot(-tmp, R @ e3)
        b3d = -tmp / np.linalg.norm(tmp)

        # calculate Rd
        b2d = np.cross(b3d, b1d) / np.linalg.norm( np.cross(b3d, b1d) )
        Rd = np.array([np.cross(b2d, b3d), b2d, b3d]).transpose()

        # estimate omega_d, omega_d_dot
        if t < 5e-2:
            omega_d = np.zeros(3)
            omega_d_dot = np.zeros(3)
        else:
            omega_d_hat = 1/dt * logm(prev_Rd.transpose() @ Rd)
            omega_d = VeeMap(omega_d_hat)
            omega_d_dot = 1/dt * (omega_d - prev_omega_d)

        # compute error terms
        e_R = 0.5*VeeMap(Rd.transpose() @ R - R.transpose() @ Rd)
        e_omega = omega - R.transpose() @ Rd @ omega_d

        # compute total moment
        M = -self.k_R*e_R - self.k_omega*e_omega + np.cross(omega, self.J @ omega) - \
            self.J @ (HatMap(omega) @ R.transpose() @ Rd @ omega_d - R.transpose() @ Rd @ omega_d_dot)

        # update previous values
        next_state = discrete_state.get_mutable_value()
        next_state[0] = t
        next_state[1:4] = omega_d
        next_state[4:13] = Rd.reshape(9)
        next_state[13:] = np.array([total_thrust, M[0], M[1], M[2]]) # save the control output

    # runs sparatically with no fixed dt
    # use zero-order hold from state information
    def CalcOutput(self, context, output):
        y = context.get_discrete_state().value()[13:]
        output.set_value(self.T_inv @ y) # give thrust outputs

    def OutputErrorFunction(self, context, output):
        # get the rotation matrix of the quadrotor plant
        quadrotor_state = self.EvalVectorInput(context, 0).CopyToVector()
        q = quadrotor_state[3:7]
        q = q / np.linalg.norm(q)
        R = RotationMatrix(Quaternion(q)).matrix()

        # parse previous state
        prev_state = context.get_discrete_state().value()
        prev_Rd = prev_state[4:13].reshape(3,3)

        psi = 0.5*np.trace(np.eye(3) - prev_Rd.transpose() @ R)
        output.set_value(np.array([psi]))

    def OutputLyapunovFunction(self, context, output):
        # V1 = 0.5*kx||ex||^2 + 0.5*m||ev||^2 + c1* ex . ev
        # V2 = 0.5*e_omega . J @ e_omega + kr * Psi(R,Rd) + c2 * e_r . e_omega
        # V3 = 0.5*||ex||^2 + 0.5*m||ev||^2

        # desired trajectory
        t = context.get_time()

        # parse previous state
        prev_state = context.get_discrete_state().value()
        prev_time = prev_state[0]
        prev_Rd = prev_state[4:13].reshape(3,3)
        dt = t - prev_time

        if SIM_NUMBER == 1:
            # Elliptical Helix Trajectory
            xd = np.array([0.4*t, 0.4*np.sin(np.pi*t), 0.6*np.cos(np.pi*t)])
            xd_dot = np.array([0.4, 0.4*np.pi*np.cos(np.pi*t), -0.6*np.pi*np.sin(np.pi*t)])
            xd_ddot = np.array([0.0, -0.4*np.pi**2*np.sin(np.pi*t), -0.6*np.pi**2*np.cos(np.pi*t)])
            #b1d = np.array([np.cos(np.pi*t), np.sin(np.pi*t), 0])
            b1d = np.array([1,0,0])
        elif SIM_NUMBER == 2:
            # Flip back over trajectory
            xd = np.zeros(3)
            xd_dot = np.zeros(3)
            xd_ddot = np.zeros(3)
            b1d = np.array([1,0,0])
        elif SIM_NUMBER == 3:
            # Flip back over trajectory
            A = 5.0
            B = -1.0
            xd = np.array([0, A*np.cos(t), B*np.sin(2*t)])
            xd_dot = np.array([0, -A*np.sin(t), 2*B*np.cos(2*t)])
            xd_ddot = np.array([0, -A*np.cos(t), -4*B*np.sin(2*t)])
            b1d = np.array([1,0,0])
        
        # get the current of the quadrotor plant
        quadrotor_state = self.EvalVectorInput(context, 0).CopyToVector()
        x = quadrotor_state[:3]   # positions
        q = quadrotor_state[3:7]  # quaternion (rotation)
        x_dot = quadrotor_state[7:10] # velocity
        q_dot = quadrotor_state[10:]  # d/dt quaternion

        try:
            R = RotationMatrix(Quaternion(q)).matrix()
        except:
            q = q / np.linalg.norm(q)
            R = RotationMatrix(Quaternion(q)).matrix()

        # q = q / np.linalg.norm(q)
        q0,q1,q2,q3 = q
        G = np.array([
            [-q1, q0, q3, -q2],
            [-q2, -q3, q0, q1],
            [-q3, q2, -q1, q0]
        ])
        omega = 2 * G @ q_dot
        
        # calculate total thrust
        e_x = x - xd
        e_v = x_dot - xd_dot
        e3 = np.array([0,0,1])
        tmp = -self.k_x*e_x - self.k_v*e_v - self.m*self.g*e3 + self.m*xd_ddot
        b3d = -tmp / np.linalg.norm(tmp)

        # calculate Rd
        b2d = np.cross(b3d, b1d) / np.linalg.norm( np.cross(b3d, b1d) )
        Rd = np.array([np.cross(b2d, b3d), b2d, b3d]).transpose()

        # estimate omega_d, omega_d_dot
        if t < 5e-2:
            omega_d = np.zeros(3)
        else:
            omega_d_hat = 1/dt * logm(prev_Rd.transpose() @ Rd)
            omega_d = VeeMap(omega_d_hat)

        # compute error terms
        e_R = 0.5*VeeMap(Rd.transpose() @ R - R.transpose() @ Rd)
        e_omega = omega - R.transpose() @ Rd @ omega_d

        # V1 = 0.5*kx||ex||^2 + 0.5*m||ev||^2 + c1* ex . ev
        # V2 = 0.5*e_omega . J @ e_omega + kr * Psi(R,Rd) + c2 * e_r . e_omega
        # V3 = 0.5*||ex||^2 + 0.5*m||ev||^2

        # calculate Lyapunov functions
        c1 = 1
        V1 = 0.5*self.k_x*np.linalg.norm(e_x)**2 + 0.5*self.m*np.linalg.norm(e_v)**2 + c1*np.dot(e_x, e_v)

        psi = np.trace(np.eye(3) - Rd.transpose() @ R)
        c2 = 1
        V2 = 0.5*np.dot(e_omega, self.J @ e_omega) + self.k_R * psi + c2*np.dot(e_R, e_omega)

        V3 = 0.5*np.linalg.norm(e_x)**2 + 0.5*self.m*np.linalg.norm(e_v)**2

        output.set_value(np.array([V1,V2,V3]))

    def OutputOmegaFunction(self, context, output):
        # get the current of the quadrotor plant
        quadrotor_state = self.EvalVectorInput(context, 0).CopyToVector()
        q = quadrotor_state[3:7]  # quaternion (rotation)
        q_dot = quadrotor_state[10:]  # d/dt quaternion

        q = q / np.linalg.norm(q)
        q0,q1,q2,q3 = q
        G = np.array([
            [-q1, q0, q3, -q2],
            [-q2, -q3, q0, q1],
            [-q3, q2, -q1, q0]
        ])
        omega = 2 * G @ q_dot

        # parse previous state
        prev_state = context.get_discrete_state().value()
        prev_Omega_d = prev_state[1:4]

        # output both vectors
        output.set_value(np.concatenate((omega, prev_Omega_d)))

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
    Parser(quadrotor_model).AddModels("quadrotor_Lee2010.urdf")
    quadrotor_model.Finalize()
    
    # We go off script here. Instead of directly connecting the bicopter model to the scene graph,
    # we insert the PoseSystem in between. This will allow us to set the pose of the bicopter model
    # whenever the bicopter model (MultibodyPlant) gives a input query for the pose.
    builder.Connect(scene_graph.get_query_output_port(), quadrotor_model.get_geometry_query_input_port())
    builder.Connect(quadrotor_model.get_geometry_poses_output_port(), poser.GetInputPort("source_pose"))
    builder.Connect(poser.get_output_port(), scene_graph.get_source_pose_port(source_id))

    builder.Connect(controller.GetOutputPort("y"), plant.get_input_port())
    builder.Connect(plant.get_output_port(), poser.GetInputPort("quadrotor_state"))
    builder.Connect(plant.get_output_port(), controller.get_input_port())

    # create loggers
    psi_logger = LogVectorOutput(controller.GetOutputPort("psi"), builder)
    position_logger = LogVectorOutput(plant.get_output_port(), builder)
    omega_logger = LogVectorOutput(controller.GetOutputPort("omega"), builder)
    thrust_logger = LogVectorOutput(controller.GetOutputPort("y"), builder)
    lyapunov_logger = LogVectorOutput(controller.GetOutputPort("lyapunov"), builder)

    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    # fix quadrotor_model in place at the origin
    quadrotor_model.mutable_gravity_field().set_gravity_vector([0, 0, 0])

    # https://github.com/RobotLocomotion/drake/blob/67671ab0c97be45434a8b6f9609901f685c55462/doc/_pages/troubleshooting.md
    # github.com/RobotLocomotion/drake/ --> drake/doc/_pages/troubleshooting.md
    root_context = diagram.CreateDefaultContext()
    quadrotor_model_context = quadrotor_model.GetMyContextFromRoot(root_context=root_context)
    plant_context = plant.GetMyContextFromRoot(root_context=root_context)
    
    if SIM_NUMBER == 1:
        R = RotationMatrix()
        q = R.ToQuaternion().wxyz()
        plant_context.get_mutable_continuous_state_vector().SetFromVector(
            [0,0,0,    # position
            q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
            0,0,0,    # velocity
            0,0,0,0]  # d/dt quaternion
        )
    elif SIM_NUMBER == 2:
        R = RotationMatrix(np.array([
            [1,0,0],
            [0,-0.9995,-0.0314],
            [0,0.0314,-0.9995]
        ]))
        q = R.ToQuaternion().wxyz()
        plant_context.get_mutable_continuous_state_vector().SetFromVector(
            [0,0,0,    # position
            q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
            0,0,0,    # velocity
            0,0,0,0]  # d/dt quaternion
        )
    elif SIM_NUMBER == 3:
        R = RotationMatrix()
        q = R.ToQuaternion().wxyz()
        plant_context.get_mutable_continuous_state_vector().SetFromVector(
            [0,5,0,    # position
            q[0], q[1], q[2], q[3],  # unit quaternion (rotation)
            0,0,0,    # velocity
            0,0,0,0]  # d/dt quaternion
        )
    
    simulator = Simulator(diagram, root_context)
    simulator.Initialize()
    meshcat.StartRecording()
    #simulator.set_target_realtime_rate(1.0)

    if SIM_NUMBER == 1:
        simulator.AdvanceTo(10.0)
    elif SIM_NUMBER == 2:
        simulator.AdvanceTo(6.0)
    elif SIM_NUMBER == 3:
        simulator.AdvanceTo(4*np.pi)
    
    meshcat.StopRecording()
    meshcat.PublishRecording()


    # plot the psi log data
    if SIM_NUMBER == 1:
        psi_log = psi_logger.FindLog(root_context)
        fig, axs = plt.subplots(1)
        fig.suptitle('Psi')
        t = psi_log.sample_times()
        data = psi_log.data()
        axs.plot(t, data[0,:])
        axs.set_ylabel('Attitude Error')
        plt.show()

        # plot the position data
        position_log = position_logger.FindLog(root_context)
        fig, axs = plt.subplots(3)
        fig.suptitle('Position')
        t = position_log.sample_times()
        data = position_log.data()
        axs[0].plot(t, data[0,:])
        axs[0].set_ylabel('x')
        axs[0].set_ylim([0,4])
        axs[1].plot(t, data[1,:])
        axs[1].set_ylabel('y')
        axs[1].set_ylim([-0.5,0.5])
        axs[2].plot(t, data[2,:])
        axs[2].set_ylabel('z')
        axs[2].set_ylim([-1,1])
        plt.show()

        # plot omega and omega_d
        omega_log = omega_logger.FindLog(root_context)
        fig, axs = plt.subplots(3)
        fig.suptitle('Omega')
        t = omega_log.sample_times()
        data = omega_log.data()
        axs[0].plot(t, data[0,:], t, data[3,:],'--')
        axs[0].set_ylim([-5,5])
        axs[0].legend(['omega', 'omega_d'])
        axs[1].plot(t, data[1,:], t, data[4,:],'--')
        axs[1].set_ylim([-5,5])
        axs[1].legend(['omega', 'omega_d'])
        axs[2].plot(t, data[2,:], t, data[5,:],'--')
        axs[2].set_ylim([0,10])
        axs[2].legend(['omega', 'omega_d'])
        plt.show()

        # plot the thrust inputs
        thrust_log = thrust_logger.FindLog(root_context)
        fig, axs = plt.subplots(4)
        fig.suptitle('Thrust Inputs')
        t = thrust_log.sample_times()
        data = thrust_log.data()
        axs[0].plot(t, data[0,:])
        axs[0].set_ylim([-200,200])
        axs[0].set_ylabel('f1')
        axs[1].plot(t, data[1,:])
        axs[1].set_ylabel('f2')
        axs[1].set_ylim([-200,200])
        axs[2].plot(t, data[2,:])
        axs[2].set_ylabel('f3')
        axs[2].set_ylim([-200,200])
        axs[3].plot(t, data[3,:])
        axs[3].set_ylabel('f4')
        axs[3].set_ylim([-200,200])
        plt.show()

        # plot the lypunov functions
        lyapunov_log = lyapunov_logger.FindLog(root_context)
        fig, axs = plt.subplots(3)
        fig.suptitle('Lyapunov Functions')
        t = lyapunov_log.sample_times()
        data = lyapunov_log.data()
        axs[0].plot(t, data[0,:])
        axs[0].set_ylabel('V1')
        axs[1].plot(t, data[1,:])
        axs[1].set_ylabel('V2')
        axs[2].plot(t, data[2,:])
        axs[2].set_ylabel('V3')
        plt.show()


    elif SIM_NUMBER == 2:
        # plot the psi log data
        psi_log = psi_logger.FindLog(root_context)
        fig, axs = plt.subplots(1)
        fig.suptitle('Psi')
        t = psi_log.sample_times()
        data = psi_log.data()
        axs.plot(t, data[0,:])
        axs.set_ylabel('Attitude Error')
        plt.show()

        # plot the position data
        position_log = position_logger.FindLog(root_context)
        fig, axs = plt.subplots(3)
        fig.suptitle('Position')
        t = position_log.sample_times()
        data = position_log.data()
        axs[0].plot(t, data[0,:])
        axs[0].set_ylabel('x')
        axs[0].set_ylim([-1,1])
        axs[1].plot(t, data[1,:])
        axs[1].set_ylabel('y')
        axs[1].set_ylim([-1,1])
        axs[2].plot(t, data[2,:])
        axs[2].set_ylabel('z')
        axs[2].set_ylim([-1,1])
        plt.show()

        # plot omega and omega_d
        omega_log = omega_logger.FindLog(root_context)
        fig, axs = plt.subplots(3)
        fig.suptitle('Omega')
        t = omega_log.sample_times()
        data = omega_log.data()
        axs[0].plot(t, data[0,:], t, data[3,:],'--')
        axs[0].set_ylim([-10,10])
        axs[0].legend(['omega', 'omega_d'])
        axs[1].plot(t, data[1,:], t, data[4,:],'--')
        axs[1].set_ylim([-1,1])
        axs[1].legend(['omega', 'omega_d'])
        axs[2].plot(t, data[2,:], t, data[5,:],'--')
        axs[2].set_ylim([-1,1])
        axs[2].legend(['omega', 'omega_d'])
        plt.show()


        # plot the thrust inputs
        thrust_log = thrust_logger.FindLog(root_context)
        fig, axs = plt.subplots(4)
        fig.suptitle('Thrust Inputs')
        t = thrust_log.sample_times()
        data = thrust_log.data()
        axs[0].plot(t, data[0,:])
        axs[0].set_ylim([-50,50])
        axs[0].set_ylabel('f1')
        axs[1].plot(t, data[1,:])
        axs[1].set_ylabel('f2')
        axs[1].set_ylim([-50,50])
        axs[2].plot(t, data[2,:])
        axs[2].set_ylabel('f3')
        axs[2].set_ylim([-50,50])
        axs[3].plot(t, data[3,:])
        axs[3].set_ylabel('f4')
        axs[3].set_ylim([-50,50])
        plt.show()

        # plot the lypunov functions
        lyapunov_log = lyapunov_logger.FindLog(root_context)
        fig, axs = plt.subplots(3)
        fig.suptitle('Lyapunov Functions')
        t = lyapunov_log.sample_times()
        data = lyapunov_log.data()
        axs[0].plot(t, data[0,:])
        axs[0].set_ylabel('V1')
        axs[1].plot(t, data[1,:])
        axs[1].set_ylabel('V2')
        axs[2].plot(t, data[2,:])
        axs[2].set_ylabel('V3')
        plt.show()

    while True:
        pass

if __name__ == "__main__":
    # Simulation Mode #
    # 1 => Elliptic helix trajectory
    # 2 => Flip back over trajectory
    # 3 => Lissajous trajectory
    SIM_NUMBER = 2
    main()