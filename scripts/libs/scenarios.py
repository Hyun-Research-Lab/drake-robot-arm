#Directly ripped from Russ:
#https://github.com/RussTedrake/underactuated/blob/aa443993ecda6e623f4f89c0dc2dfdf7cb08397a/underactuated/scenarios.py#L87

"""
This file contains a number of helper utilities to set up our various
experiments with less code.
"""
import numpy as np
from pydrake.all import (
    BallRpyJoint,
    Box,
    CoulombFriction,
    Cylinder,
    PrismaticJoint,
    RevoluteJoint,
    RigidTransform,
    SpatialInertia,
    Sphere,
    UnitInertia,
)

# https://github.com/RobotLocomotion/drake/issues/14949
def AddFloatingRpyJoint(plant, frame, instance, use_ball_rpy=True):
    inertia = SpatialInertia(
        mass=0, p_PScm_E=[0.0, 0.0, 0.0], G_SP_E=UnitInertia(0, 0, 0)
    )
    x_body = plant.AddRigidBody("x", instance, inertia)
    plant.AddJoint(
        PrismaticJoint(
            "x", plant.world_frame(), x_body.body_frame(), [1, 0, 0]
        )
    )
    y_body = plant.AddRigidBody("y", instance, inertia)
    plant.AddJoint(
        PrismaticJoint(
            "y", x_body.body_frame(), y_body.body_frame(), [0, 1, 0]
        )
    )
    z_body = plant.AddRigidBody("z", instance, inertia)
    plant.AddJoint(
        PrismaticJoint(
            "z", y_body.body_frame(), z_body.body_frame(), [0, 0, 1]
        )
    )
    if use_ball_rpy:
        plant.AddJoint(BallRpyJoint("ball", z_body.body_frame(), frame))
    else:
        # RollPitchYaw is body z-y-x
        rz_body = plant.AddRigidBody("rz", instance, inertia)
        plant.AddJoint(
            RevoluteJoint(
                "rz", z_body.body_frame(), rz_body.body_frame(), [0, 0, 1]
            )
        )
        ry_body = plant.AddRigidBody("ry", instance, inertia)
        plant.AddJoint(
            RevoluteJoint(
                "ry", rz_body.body_frame(), ry_body.body_frame(), [0, 1, 0]
            )
        )
        plant.AddJoint(
            RevoluteJoint("rx", ry_body.body_frame(), frame, [1, 0, 0])
        )