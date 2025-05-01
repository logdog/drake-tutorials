import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    LeafSystem,
    LogVectorOutput,
    MeshcatVisualizer,
    RigidTransform,
    RotationMatrix,
    Box,
    GeometryFrame,
    FramePoseVector,
    GeometryInstance,
    IllustrationProperties,
    AbstractValue,
    SceneGraph,
    Simulator,
    Meshcat,
)

# simple sliding block with friction
class Block(LeafSystem):
    def __init__(self, frame_id):
        LeafSystem.__init__(self)
        state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("x", state_index)
        
        # visualization
        self.frame_id = frame_id
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        x, x_dot = context.get_continuous_state_vector().CopyToVector()
        x_ddot = -2*x_dot

        derivatives.SetFromVector(
            np.array([x_dot, x_ddot])
        )
        
    def CalcFramePoseOutput(self, context, output):
        x, x_dot = context.get_continuous_state_vector().CopyToVector()
        
        T = RigidTransform()
        T.set_translation([x,0,0])

        output.get_mutable_value().set_value(self.frame_id, T)

def main():
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Add a frame "my_frame" to the scene_graph and add a geometry "my_geometry_instance" to it
    source_id = scene_graph.RegisterSource("my_block_source")
    frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
    geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(RigidTransform(RotationMatrix(), np.array([0,0,0.05])), Box(0.1, 0.1, 0.1), "my_geometry_instance"))
    # we used a box, but there are other options: Capsule, Cylinder, Ellipsoid, HalfSpace, Sphere, Convex, Mesh
    # documentation: https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1GeometryInstance.html
    
    # the purpose of this geometry is for visualization, so we assign IllustrationProperties role
    # there are other roles such as ProximityProperties and PerceptionProperties
    # documentation: https://drake.mit.edu/doxygen_cxx/group__geometry__roles.html
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
    
    # add the meshcat visualizer to the diagram
    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    # add the plant to the diagram (be sure to pass in the frame id!)
    plant = builder.AddSystem(Block(frame_id))
    
    # connect the "my_pose" output port to the scene graph
    builder.Connect(plant.GetOutputPort("my_pose"), scene_graph.get_source_pose_port(source_id))

    # Log the state output port
    logger = LogVectorOutput(plant.GetOutputPort("x"), builder)
    
    diagram = builder.Build()
    plot_system_graphviz(diagram)
    plt.show()
    
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # set the block initial conditions. x = 0.0, x_dot = 5.0
    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant_context.SetContinuousState([0.0, 5.0])

    # Run the simulation.
    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(5.0)
    meshcat.PublishRecording()
    print('Simulation complete!')
    
    while True:
        pass

if __name__ == "__main__":
    main()