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
)


def main():
    builder = DiagramBuilder()
    bicopter_model, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0)
    Parser(bicopter_model).AddModels("./resources/bicopter.urdf")
    bicopter_model.Finalize()

    # For more detailed information, you can iterate through bodies, joints, and actuators:
    for body_index in range(bicopter_model.num_bodies()):
        body = bicopter_model.get_body(BodyIndex(body_index))
        print(f"Body: {body.name()}, Model Instance: {body.model_instance()}")

    for i in range(bicopter_model.num_joints()):
        joint = bicopter_model.get_joint(JointIndex(i))
        print(f"Joint: {joint.name()}, Model Instance: {joint.model_instance()}")

    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()

    # https://github.com/RobotLocomotion/drake/blob/67671ab0c97be45434a8b6f9609901f685c55462/doc/_pages/troubleshooting.md
    # github.com/RobotLocomotion/drake/ --> drake/doc/_pages/troubleshooting.md
    root_context = diagram.CreateDefaultContext()
    context = bicopter_model.GetMyContextFromRoot(root_context=root_context)

    J1 = bicopter_model.GetJointByName("J1")
    J3 = bicopter_model.GetJointByName("J3")

    body = bicopter_model.GetBodyByName("body")
    bicopter_model.mutable_gravity_field().set_gravity_vector([0, 0, 0])
    #bicopter_model.SetFreeBodyPose(context, body, RigidTransform(RotationMatrix().MakeXRotation(1), [0, 0, 0.5]))

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
        axes = gamepad.axes
        print(axes)

        # B button pressed
        if gamepad.button_values[1] == 1:
            pass
            
        J1.set_angle(context, axes[4])
        J3.set_angle(context, axes[1])
        
        
        simulator.AdvanceTo(simulator.get_context().get_time() + 1/60)

if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    main()
