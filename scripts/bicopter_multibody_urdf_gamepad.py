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

class BicopterPoseSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.state_port = self.DeclareVectorInputPort("plant_state", BasicVector(14))
        self.source_pose_port = self.DeclareAbstractInputPort("source_pose", AbstractValue.Make(FramePoseVector()))
        self.DeclareAbstractOutputPort("y", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    def CalcFramePoseOutput(self, context, output):
        # state = self.state_port.Eval(context)
        # x = state[:3]
        # q = state[3:7]

        t = context.get_time()

        frame_pose_vectors = self.source_pose_port.Eval(context)
        X_WB = RigidTransform(RotationMatrix().MakeXRotation(t),[0,0,0]).GetAsMatrix4()

        for frame_id in frame_pose_vectors.ids():
            old_pose = frame_pose_vectors.value(frame_id).GetAsMatrix4()
            new_pose = RigidTransform(X_WB @ old_pose)
            frame_pose_vectors.set_value(frame_id, new_pose)
        output.set_value(frame_pose_vectors)


        # # normalize q to obtain a unit quaternion
        # q /= np.linalg.norm(q)
        # R = RotationMatrix(Quaternion(q))
        # t = context.get_time()

        # output.get_mutable_value().set_value(1, RigidTransform(
        #     RotationMatrix(),
        #     [0, 0, t/10]
        # ))


def main():
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    bicopter_model = builder.AddSystem(MultibodyPlant(time_step=0.0))
    poser = builder.AddSystem(BicopterPoseSystem())
    
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

    inspector = scene_graph.model_inspector()
    print(inspector.GetAllGeometryIds())
    print(inspector.num_sources())
    print(inspector.GetAllSourceIds())

    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    # https://github.com/RobotLocomotion/drake/blob/67671ab0c97be45434a8b6f9609901f685c55462/doc/_pages/troubleshooting.md
    # github.com/RobotLocomotion/drake/ --> drake/doc/_pages/troubleshooting.md
    root_context = diagram.CreateDefaultContext()
    bicopter_model_context = bicopter_model.GetMyContextFromRoot(root_context=root_context)

    J1 = bicopter_model.GetJointByName("J1") # right sevro
    J3 = bicopter_model.GetJointByName("J3") # left servo

    bicopter_model.mutable_gravity_field().set_gravity_vector([0, 0, 0])
    bicopter_body = bicopter_model.GetBodyByName("body")
    bicopter_model.SetFreeBodyPose(bicopter_model_context, bicopter_body, RigidTransform([0, 0, 0]))

    
    # Gamepad
    print('To connect gamepad, switch to browser tab with Meshcat (refresh if necessary), and then spam the A button on the gamepad.')
    while(meshcat.GetGamepad().index == None):
        pass
    print('Connected!')
    
    simulator = Simulator(diagram, root_context)
    simulator.Initialize()

    simulator.set_target_realtime_rate(1.0)

    def deadzone(x):
        y = np.zeros_like(x)
        for i in range(len(x)):
            if abs(x[i]) < 0.2:
                y[i] = 0
            else:
                y[i] = x[i]
        return y

    while True:
        gamepad = meshcat.GetGamepad()
        axes = deadzone(gamepad.axes)
        #axes = gamepad.axes

        # # B button pressed
        # if gamepad.button_values[1] == 1:
        #     pass
            
        # # visual only
        J1.set_angle(bicopter_model_context, axes[4])
        J3.set_angle(bicopter_model_context, axes[1])
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1/60)

if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    main()
