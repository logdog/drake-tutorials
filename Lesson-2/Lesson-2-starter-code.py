import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
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
        x_ddot = u
        
        # update the derivatives for the continous-time integrator
        derivatives.get_mutable_vector().SetFromVector(
            np.array([x_dot, x_ddot])
        )
        
class Controller(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        
        # TODO: declare input port (2-dimensional)
        # TODO: declare output port (1-dimensional)
        
        self.kp = 10.0
        self.kd = 5.0
        
        self.target_x = 1.0
        self.target_x_dot = 0.0

    def Output(self, context, output):
        # TODO: read the input to get x and x_dot
        
        # PD controller
        u = -self.kp * (x - self.target_x) - self.kd * (x_dot - self.target_x_dot)
        output.SetFromVector([u])
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(DoubleIntegrator())
    plant.set_name("double integrator")
    
    controller = builder.AddSystem(Controller())
    controller.set_name("PD controller")
    
    # TODO: connect the controller output to the plant input
    # TODO: connect the plane output to the controller input
    
    logger = LogVectorOutput(plant.get_output_port(), builder)
    logger.set_name("output state logger")
    
    logger2 = LogVectorOutput(controller.get_output_port(), builder)
    logger2.set_name("controller logger")
    
    diagram = builder.Build()
    diagram.set_name("Closed Loop Double Integrator System (Solution)")
    
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
    
    