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
        self.I_body = np.diag([0.0820, 0.0845, 0.1377])
        self.I_body_inv = np.linalg.inv(self.I_body)
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
        omega = 2 * G @ q_dot

        # omega_dot is from the Euler equation in Chapter 4 of MLS
        # Iw_dot + w x Iw = M
        omega_dot = self.I_body_inv @ (np.cross(-omega, self.I_body @ omega) + M)

        # q_dot = 1/2 G' omega, then take a derivative
        q_ddot = 0.5 * (G_dot.transpose() @ omega + G.transpose() @ omega_dot)

        derivatives.get_mutable_vector().SetFromVector(
            np.concatenate((x_dot, q_dot, x_ddot, q_ddot))
        )

class QuadrotorController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("y", BasicVector(4), self.CalcOutput)

    def CalcOutput(self, context, output):
        output.set_value(np.array([0,0,0,0]))

class PoseSystem(LeafSystem):
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
        X_WB = np.diag([1,1,-1,1]) @ RigidTransform(R,x).GetAsMatrix4()

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
    builder.Connect(plant.get_output_port(), poser.GetInputPort("plant_state"))

    inspector = scene_graph.model_inspector()
    print(inspector.GetAllGeometryIds())
    print(inspector.num_sources())
    print(inspector.GetAllSourceIds())

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
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()

    while True:
        pass

if __name__ == "__main__":
    main()
