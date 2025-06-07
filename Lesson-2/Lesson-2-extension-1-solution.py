import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    PidController,
    ConstantVectorSource,
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
        
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(DoubleIntegrator())

    # read the documentation for PidController to see how to set it up
    # https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_pid_controller.html
    controller = builder.AddSystem(PidController(kp=[10.0], ki=[0.0], kd=[5.0]))
    reference = builder.AddSystem(ConstantVectorSource([1.0, 0.0]))
    
    # read the documentation for PidController to see how to set it up
    # https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_pid_controller.html
    builder.Connect(controller.get_output_port_control(), plant.get_input_port())
    builder.Connect(plant.get_output_port(), controller.get_input_port_estimated_state())
    builder.Connect(reference.get_output_port(), controller.get_input_port_desired_state())
    
    logger = LogVectorOutput(plant.get_output_port(), builder)
    logger2 = LogVectorOutput(controller.get_output_port_control(), builder)
    
    diagram = builder.Build()
    
    plot_system_graphviz(diagram)
    plt.show()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)

    # the integral term in the PID controller is technically a state, too
    # so it can be set like this (optional)
    controller_context = controller.GetMyContextFromRoot(context)
    controller_context.SetContinuousState(np.array([0]))

    # when multiple systems have state, you need to be careful
    # to set the state of the correct system. Use the GetMyContextFromRoot
    # method to get the context for the specific system.
    plant_context = plant.GetMyContextFromRoot(context)
    plant_context.SetContinuousState(np.array([0, 0]))
    
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
    plt.grid()
    plt.legend()
    plt.show()
    