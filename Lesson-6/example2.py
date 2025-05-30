import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
)

class ExampleDiscreteSystem(LeafSystem):
    
    # use a step size of 1/50 seconds 
    kPeriod = 1.0/50
    kOffset = 0.0
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareDiscreteState(1)
        self.DeclarePeriodicDiscreteUpdateEvent(ExampleDiscreteSystem.kPeriod, ExampleDiscreteSystem.kOffset, self.Update)
        self.DeclareVectorOutputPort("y", 1, self.Output)
        
    def Update(self, context, discrete_values):
        # get the current state
        x = context.get_discrete_state().get_vector().GetAtIndex(0)
        
        # update the state
        x_new = x + 1
        discrete_values.set_value([x_new])
        
        print(f"Update(): t={context.get_time()}, x={x}, x_new={x_new}")
        
    
    def Output(self, context, output):
        x = context.get_discrete_state().get_vector().GetAtIndex(0)
        y = 10 * x
        output.SetFromVector([y])
        
        print(f"Output(): t={context.get_time()}, x={x}, y={y}")
        
    
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(ExampleDiscreteSystem())
    logger = LogVectorOutput(plant.get_output_port(), builder, publish_period=0.03) # leave publish_period=0.0 for per-step logging
    diagram = builder.Build()

    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetDiscreteState(np.array([0]))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # run the simulation
    simulator.AdvanceTo(3* ExampleDiscreteSystem.kPeriod)  # Run for 3 time steps
    
    # log the data
    log = logger.FindLog(context)
    t = log.sample_times()
    y = log.data()[0,:].transpose()
    
    for tt, yy in zip(t, y):
        print(f"t={tt:0.2f}, y={yy:0.2f}")

    
    
    