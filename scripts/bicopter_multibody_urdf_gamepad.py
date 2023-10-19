import numpy as np
import matplotlib.pyplot as plt

from pydrake.math import RigidTransform, RotationMatrix, ClosestQuaternion
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, Context, LeafSystem, SystemOutput
from pydrake.geometry import Box, GeometryFrame, FramePoseVector, GeometryInstance, IllustrationProperties, Mesh
from pydrake.common.value import AbstractValue
from pydrake.common.eigen_geometry import Quaternion

from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from copy import copy
import matplotlib.pyplot as plt
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    DiagramBuilder,
    MeshcatVisualizer,
    LeafSystem,
    Simulator,
    Simulator,
    CoulombFriction,
    HalfSpace,
    LogVectorOutput,
    SpatialVelocity,
    BasicVector,
    AffineSystem,
    ConstantVectorSource,
)

from pydrake.all import (
    DiagramBuilder,
    LogVectorOutput,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    Meshcat,
    MultibodyPlant,
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
    J1.set_angle(context, 0.2)
    J3.set_angle(context, -0.2)
    body = bicopter_model.GetBodyByName("body")
    bicopter_model.SetFreeBodyPose(context, body, RigidTransform(RotationMatrix().MakeXRotation(1), [0, 0, 0.5]))
    bicopter_model.mutable_gravity_field().set_gravity_vector([0, 0, 0])
    
    simulator = Simulator(diagram, root_context)
    simulator.Initialize()

    # meshcat.StartRecording()
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1.0)
    # meshcat.StopRecording()
    # meshcat.PublishRecording()

    while True:
        pass

if __name__ == "__main__":
    print('NOTE: Meshcat sometimes takes a long time to start up. Please be patient.')
    main()
