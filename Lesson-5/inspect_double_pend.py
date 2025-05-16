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
import matplotlib.pyplot as plt
import os

if __name__ == "__main__":
    # TODO: change to DP-1.urdf, DP-2.urdf, DP-3.urdf, DP-4.urdf
    urdf_path = os.path.join(os.path.dirname(__file__), "DP-3.urdf")
    
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

    # SceneGraphInspector
    inspector = scene_graph.model_inspector()
    source_ids = inspector.GetAllSourceIds()
    frame_ids = inspector.GetAllFrameIds()
    geometry_ids = inspector.GetAllGeometryIds()
    print()
    print(f"Source Ids: {source_ids}")
    print(f"Frame Ids: {frame_ids}")
    print(f"Geometry Ids: {geometry_ids}")
    print()
    # sources and their properties
    for source_id in source_ids:
        print(f"Source ID: {source_id}")
        print(f"  {inspector.SourceIsRegistered(source_id)=}")
        print(f"  {inspector.GetName(source_id)=}")
        print(f"  {inspector.NumFramesForSource(source_id)=}")
        print(f"  {inspector.FramesForSource(source_id)=}")
        
        for frame_id in inspector.FramesForSource(source_id):
            print(f"    Frame ID: {frame_id}")
            print(f"    {inspector.GetName(frame_id)=}")
            print(f"    {inspector.NumGeometriesForFrame(frame_id)=}")
            
            for geometry_id in inspector.GetGeometries(frame_id):
                print(f"      Geometry ID: {geometry_id}")
                print(f"      {inspector.GetName(geometry_id)=}")
                print(f"      {inspector.GetShape(geometry_id)=}")
                print(f"      {inspector.GetPoseInFrame(geometry_id)=}")
        print()

