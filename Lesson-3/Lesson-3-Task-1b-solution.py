import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
)

class Pendulum(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("u", 1)
        self.state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("y", self.state_index)
        
        self.m = 1.0  # mass of the pendulum
        self.b = 0.1  # damping coefficient
        self.g = 9.81  # acceleration due to gravity
        self.l = 1.0  # length of the pendulum
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        # unpack the state
        state = context.get_continuous_state_vector().CopyToVector()
        theta, theta_dot = state
        
        # read the input
        u = self.get_input_port().Eval(context)[0]
        
        # state space equations
        theta_ddot = 1/(self.m*self.l**2) * (u - self.b*theta_dot - self.m*self.g*self.l*np.sin(theta))
        
        derivatives.get_mutable_vector().SetFromVector(
            np.array([theta_dot, theta_ddot])
        )
        
class LQRController(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("x", 2)
        self.DeclareVectorOutputPort("u", 1, self.MyOutput)

        # calculated from starter code
        self.K = np.array([20.11708979,  6.32216315])

    def MyOutput(self, context, output):
        # unpack the state
        state = self.get_input_port().Eval(context)

        # LQR controller
        u = -self.K @ (state - np.array([np.pi, 0]))
        output.SetFromVector([u])
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(Pendulum())
    plant.set_name("Pendulum Plant")
    
    controller = builder.AddSystem(LQRController())
    controller.set_name("LQR controller")
    
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    builder.Connect(plant.get_output_port(), controller.get_input_port())
    
    logger = LogVectorOutput(plant.GetOutputPort("y"), builder)
    logger.set_name("output state logger")
    
    logger2 = LogVectorOutput(controller.get_output_port(), builder)
    logger2.set_name("controller logger")
    
    diagram = builder.Build()
    diagram.set_name("Closed Loop System (Solution)")
    
    plot_system_graphviz(diagram)
    plt.show()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetContinuousState(np.deg2rad(np.array([170, 0])))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # run the simulation
    print("Running simulation...")
    simulator.AdvanceTo(10.0)
    print("Simulation complete. Press Ctrl+C to exit.")
    
    # create plots
    log = logger.FindLog(context)
    log2 = logger2.FindLog(context)
    
    theta = log.data()[0,:]
    theta_dot = log.data()[1,:]
    
    plt.figure()
    plt.plot(log.sample_times(), np.rad2deg(theta), label="theta")
    plt.title("Angle vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (Degrees)")
    plt.grid()
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(log.sample_times(), np.rad2deg(theta_dot), label="theta_dot")
    plt.title("Angular Velocity vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (Degrees/s)")
    plt.grid()
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(log2.sample_times(), log2.data()[0,:], label="u")
    plt.title("Input vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N-m)")
    plt.grid()
    plt.legend()
    plt.show()
    
    
    