import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    ConstantVectorSource,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
)

# double integrator system with a continuous state
class DoubleIntegrator(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        
        # declare the input port (1-dimensional)
        self.DeclareVectorInputPort("u", 1)
        
        # declare the continuous state (1 position state, 1 velocity state, 0 abstract states)
        self.state_index = self.DeclareContinuousState(1,1,0)
        
        # declare the output port to be the state vector
        self.DeclareStateOutputPort("y", self.state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        # read the input
        u = self.get_input_port().Eval(context)[0]
    
        # read the output
        state = context.get_continuous_state().get_vector()
        x = state.GetAtIndex(0)
        x_dot = state.GetAtIndex(1)
        
        # state space equations
        x_ddot = u
        
        # update the derivatives for the continous-time integrator
        # print(f"DoCalcTimeDerivatives(): t={context.get_time()}, u={u}, x={x}, x_dot={x_dot}, x_ddot={x_ddot}")
        derivatives.get_mutable_vector().SetFromVector(
            np.array([x_dot, x_ddot])
        )
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(DoubleIntegrator())
    plant.set_name("double integrator")
    
    vector_input = builder.AddSystem(ConstantVectorSource(np.array([-1.0])))
    vector_input.set_name("constant vector input source")
    
    builder.Connect(vector_input.get_output_port(), plant.get_input_port())
    
    logger = LogVectorOutput(plant.get_output_port(), builder)
    logger.set_name("output state logger")
    
    diagram = builder.Build()
    diagram.set_name("Double Integrator System")
    
    # visualize the diagram (optional)
    plot_system_graphviz(diagram)
    plt.show()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetContinuousState(np.array([1.0, 2.0]))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # run the simulation
    print("Running simulation...")
    simulator.AdvanceTo(5.0)
    print("Simulation complete. Press Ctrl+C to exit.")
    
    # create plots
    log = logger.FindLog(context)
    
    x = log.data()[0,:]
    x_dot = log.data()[1,:]
    
    plt.figure()
    plt.plot(log.sample_times(), x, label="x")
    plt.title("Position vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.grid()
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(log.sample_times(), x_dot, label="x_dot")
    plt.title("Velocity vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.grid()
    plt.legend()
    plt.show()
