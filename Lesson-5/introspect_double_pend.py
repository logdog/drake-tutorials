import numpy as np

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    LeafSystem,
    LogVectorOutput,
    Meshcat,
    MeshcatParams,
    RigidTransform,
    Parser,
    SpatialForce,
    AddMultibodyPlantSceneGraph, 
    Propeller, 
    ExternallyAppliedSpatialForceMultiplexer,
    Simulator,
    DiagramBuilder,
    AddDefaultVisualization,
    PdControllerGains,
    AbstractValue,
    JointIndex,
    FrameIndex,
    BodyIndex,
    RpyFloatingJoint,
    GeometryInstance,
    RotationMatrix,
    Cylinder,
    IllustrationProperties,
    Rgba,
    MakePhongIllustrationProperties,
    Sphere,
)

import os
import matplotlib.pyplot as plt

if __name__ == "__main__":
    
    # TODO: change to DP-1.urdf, DP-2.urdf, DP-3.urdf, DP-4.urdf
    urdf_path = os.path.join(os.path.dirname(__file__), "DP-4.urdf")
    
    meshcat = Meshcat()
    meshcat.SetCameraPose(
        camera_in_world=np.array([0, -2, 0.5]),
        target_in_world=np.array([0, 0, 0]),
    )

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.AddModels(urdf_path)
    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    
    plot_system_graphviz(diagram)
    plt.show()

    # Introspect the MultibodyPlant
    # print number of joints and their names
    for i in range(plant.num_joints()):
        joint = plant.get_joint(JointIndex(i))
        print(f"Joint {i}: {joint.name()}")
        print(f"  Type: {joint.type_name()}")
        print(f"  Index: {joint.index()}")
    print()
        
    # print the number of actuators
    print(f"Number of actuators: {plant.num_actuators()}")
    print()

    # print the frames
    context = plant.CreateDefaultContext()
    context_rotated = plant.CreateDefaultContext()
    context_rotated.SetContinuousState(np.deg2rad([30, 30, 0, 0]))
    for i in range(plant.num_frames()):
        frame = plant.get_frame(FrameIndex(i))
        print(f"Frame {i}: {frame.name()}")
        print(f"  Index: {frame.index()}")
        print(f"  World Frame?: {frame.is_world_frame()}")
        print(f"  Body Frame?: {frame.is_body_frame()}")
        print(f"  Position: {frame.CalcPoseInWorld(context)}")
        print(f"  Rotated Position: {frame.CalcPoseInWorld(context_rotated)}")
    print()
        
    # numebr of bodies
    print(f"Number of bodies: {plant.num_bodies()}")
    print()
    # print the bodies
    for i in range(plant.num_bodies()):
        body = plant.get_body(BodyIndex(i))
        print(f"Body {i}: {body.name()}")
        print(f"  Mass: {body.default_mass()} kg")
        print(f"  Inertia: {body.default_unit_inertia()} kg*m^2")
        print(f"  COM: {body.default_com()} m")
    print()
