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
    Simulator,
    Meshcat,
    Rgba,
)

# simple sliding block with friction
class Block(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("x", state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        x, x_dot = context.get_continuous_state_vector().CopyToVector()
        x_ddot = -2*x_dot

        derivatives.SetFromVector(
            np.array([x_dot, x_ddot])
        )

def main():
    builder = DiagramBuilder()
    plant = builder.AddSystem(Block())

    # Log the state output port
    logger = LogVectorOutput(plant.GetOutputPort("x"), builder)
    
    diagram = builder.Build()
    
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # set the block initial conditions. x = 0.0, x_dot = 5.0
    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant_context.SetContinuousState([0.0, 5.0])

    simulator.Initialize()
    simulator.AdvanceTo(5.0)
    print('Simulation complete!')
    
    
    # create animation
    log = logger.FindLog(context)
    t = log.sample_times()
    x = log.data()[0,:].transpose()
    
    meshcat = Meshcat()
    
    # create the object in the meshcat visualizer
    meshcat.SetObject("my_block", Box(0.1, 0.1, 0.1), Rgba(0.5, 0.5, 0.5, 1))
    
    # start recording the animation. Including the third argument t[i] will associate
    # the transform with the time t[i] in the animation so you can use the playback controls to replay the animation
    # without having the rerun the simulation.
    meshcat.StartRecording(frames_per_second=64)
    for i in range(len(t)):
        T = RigidTransform()
        T.set_translation([x[i],0,0.05])
        meshcat.SetTransform("my_block", T, t[i])
        
    # Because the simulation time step is faster than the meshcat frame rate, there
    # can be a problem with the initial condition not being displayed correctly in the animation.
    # This is because meshcat is chopping time into chunks of 1/64 seconds, and the most recent call to SetTransform(..., ..., ..., t[1])
    # will overwrite the previous call to SetTransform(..., ..., ..., t[0]) if t[1] and t[0] are both less than 1/64.
    # As a workaround, we can simply set the transform to the initial condition at the beginning of the animation.
    # To really understand this, you can try commenting out the line below and see what happens to the initial condition in the animation.
    # You can also read this SO post for more details: https://stackoverflow.com/questions/77916511/meshcat-not-visualizing-initial-conditions-correctly-drake
    meshcat.SetTransform("my_block", RigidTransform(RotationMatrix(), np.array([x[0],0,0.05])), t[0])
    
    meshcat.StopRecording()
    meshcat.PublishRecording()
    
    # adding this line will keep the meshcat visualizer open after the simulation is complete
    # otherwise, the meshcat visualizer will close when the script is done
    # and you will be left scratching your head wondering where why your browser cannot connect to the meshcat server
    while True:
        pass

if __name__ == "__main__":
    main()