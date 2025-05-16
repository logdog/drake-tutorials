import numpy as np

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    LeafSystem,
    LogVectorOutput,
    Meshcat,
    MeshcatParams,
    RigidTransform,
    ConstantVectorSource,
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
    
    # add actuation to the multibody plant
    if 'DP-4.urdf' in urdf_path:
        vector_source = builder.AddSystem(ConstantVectorSource([0.0, 5.0]))
        builder.Connect(vector_source.get_output_port(), plant.get_actuation_input_port())
        

    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    
    plot_system_graphviz(diagram)
    plt.show()

    # manually add axes to the upper_arm/lower_arm frames
    # this is useful for debugging joints in your URDF file
    inspector = scene_graph.model_inspector()
    plant_source_id = plant.get_source_id()
    for frame_id in inspector.FramesForSource(plant_source_id):
        frame_name = inspector.GetName(frame_id)
        
        radius = 0.002
        length = 0.25
        x_axis = scene_graph.RegisterGeometry(plant_source_id, frame_id, 
            GeometryInstance(RigidTransform(RotationMatrix.MakeYRotation(np.pi/2), np.array([length/2,0,0])), Cylinder(radius,length), frame_name+"_xaxis"))
        scene_graph.AssignRole(plant_source_id, x_axis, MakePhongIllustrationProperties([1, 0, 0, 1.0]))
        
        y_axis = scene_graph.RegisterGeometry(plant_source_id, frame_id, 
            GeometryInstance(RigidTransform(RotationMatrix.MakeXRotation(np.pi/2), np.array([0,length/2,0])), Cylinder(radius,length), frame_name+"_yaxis"))
        scene_graph.AssignRole(plant_source_id, y_axis, MakePhongIllustrationProperties([0, 1, 0, 1.0]))
        
        z_axis = scene_graph.RegisterGeometry(plant_source_id, frame_id, 
            GeometryInstance(RigidTransform(RotationMatrix(), np.array([0,0,length/2])), Cylinder(radius,length), frame_name+"_zaxis"))
        scene_graph.AssignRole(plant_source_id, z_axis, MakePhongIllustrationProperties([0, 0, 1, 1.0]))
        

    # Set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetContinuousState(np.deg2rad([30, 30, 0, 0]))

    # simulate the multibody plant
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.) # optional
    meshcat.StartRecording()
    simulator.AdvanceTo(5.0)
    meshcat.PublishRecording()

    while True:
        pass

