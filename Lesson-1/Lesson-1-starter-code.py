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
        x_ddot = 1/2 * (-0.5 * x_dot + u)
        
        # update the derivatives for the continous-time integrator
        # print(f"DoCalcTimeDerivatives(): t={context.get_time()}, u={u}, x={x}, x_dot={x_dot}, x_ddot={x_ddot}")
        derivatives.get_mutable_vector().SetFromVector(
            np.array([x_dot, x_ddot])
        )


class Controller(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("u", 1, self.MyOutput)

    def MyOutput(self, context, output):
        t = context.get_time()
        u = np.sin(t) # modify this line
        output.SetFromVector([u])
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(DoubleIntegrator())
    plant.set_name("double integrator")
    
    controller = builder.AddSystem(Controller())
    
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    
    logger = LogVectorOutput(plant.get_output_port(), builder)
    # logger = LogVectorOutput(plant.get_output_port(), builder, publish_period=0.1) # if you want to log only at specific intervals
    logger.set_name("output state logger")

    input_logger = LogVectorOutput(controller.get_output_port(), builder)
    
    diagram = builder.Build()
    diagram.set_name("Double Integrator System")
    
    # visualize the diagram (optional)
    # plot_system_graphviz(diagram)
    # plt.show()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetContinuousState(np.array([0.0, 0.0]))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # run the simulation
    print("Running simulation...")
    simulator.AdvanceTo(4*np.pi)
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

    input_log = input_logger.FindLog(context)
    u = input_log.data()[0,:]
    plt.figure()
    plt.plot(input_log.sample_times(), u, label="u")
    plt.title("Control Input vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input (N)")
    plt.grid()
    plt.legend()
    plt.show()
