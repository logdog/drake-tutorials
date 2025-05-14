import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    ConstantVectorSource,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
    Sine,
)

# double integrator system with a continuous state
class DoubleIntegrator(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("u", 1)
        self.state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("y", self.state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        # read the input
        u = self.get_input_port().Eval(context)[0]
        
        # read the output
        state = context.get_continuous_state().get_vector()
        x = state.GetAtIndex(0)
        x_dot = state.GetAtIndex(1)
        
        # state space equations
        # mass = 2 kg
        # damping = 0.5 N-s/m
        x_ddot = 1/2 * (u - 0.5*x_dot)
        
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
        u = np.sin(t)
        output.SetFromVector([u])
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(DoubleIntegrator())
    plant.set_name("double integrator")
    
    # use a LeafSystem to create a sinusoidal input
    controller = builder.AddSystem(Controller())
    controller.set_name("My Controller")
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    
    # or comment out the 3 lines above, and uncomment
    # the 3 lines below to use the Sine source for the extension
    # controller = builder.AddSystem(Sine(1.0, 1, 0.0, 1, True))
    # controller.set_name("Sine Controller")
    # builder.Connect(controller.GetOutputPort("y0"), plant.get_input_port())
    
    logger = LogVectorOutput(plant.get_output_port(), builder)
    logger.set_name("output state logger")
    
    # if using LeafSystem, uncomment the lines below
    logger2 = LogVectorOutput(controller.get_output_port(), builder)
    logger2.set_name("input logger")
    
    # if using Sine, uncomment the line blow
    # logger2 = LogVectorOutput(controller.GetOutputPort("y0"), builder)
    # logger2.set_name("input logger")
    
    diagram = builder.Build()
    diagram.set_name("Double Integrator System (Solution)")
    
    plot_system_graphviz(diagram)
    plt.show()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetContinuousState(np.array([0, 0]))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # run the simulation
    print("Running simulation...")
    simulator.AdvanceTo(5.0)
    print("Simulation complete. Press Ctrl+C to exit.")
    
    # create plots
    log = logger.FindLog(context)
    log2 = logger2.FindLog(context)
    
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
    
    plt.figure()
    plt.plot(log2.sample_times(), log2.data()[0,:], label="u")
    plt.title("Input vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Input (m/s^2)")
    
    