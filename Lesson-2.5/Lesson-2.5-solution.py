import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
    ApplySimulatorConfig,
    SimulatorConfig,
)

# double integrator system with a continuous state
class InvertedPendulum(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("y", self.state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
       
        # read the state
        state = context.get_continuous_state().get_vector()
        x = state.GetAtIndex(0)
        x_dot = state.GetAtIndex(1)
        
        # state space equations
        x_ddot = 10*np.sin(x)
        
        # update the derivatives for the continous-time integrator
        derivatives.get_mutable_vector().SetFromVector(
            np.array([x_dot, x_ddot])
        )
        

def do_simulation(time_step=0):
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(InvertedPendulum())
    logger = LogVectorOutput(plant.get_output_port(), builder)
    diagram = builder.Build()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetContinuousState(np.array([np.deg2rad(1), 0]))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # based on the time step, set the simulator configuration
    if (time_step > 0):
        simulator_config = SimulatorConfig(
            max_step_size=time_step,
            use_error_control=False,
        )
        ApplySimulatorConfig(simulator_config, simulator)
        
    simulator.Initialize()
    
    # run the simulation
    simulator.AdvanceTo(10.0)

    # create plots
    log = logger.FindLog(context)
    
    t = log.sample_times()
    x = np.rad2deg(log.data()[0,:])
    x_dot = np.rad2deg(log.data()[1,:])
    
    return t, x, x_dot
    

if __name__ == "__main__":
    
    t, x, x_dot = do_simulation(0)  # original simulation with no time step
    t0, x0, x0_dot = do_simulation(0.001)
    t1, x1, x1_dot = do_simulation(0.01)
    t2, x2, x2_dot = do_simulation(0.1)
    

    plt.figure()
    plt.plot(t, x, '*-',label="original")
    plt.plot(t0, x0, '*-',label="dt = 0.001")
    plt.plot(t1, x1, '*-',label="dt = 0.01")
    plt.plot(t2, x2, '*-',label="dt = 0.1")
    plt.title("Angle vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.grid()
    plt.legend()
    plt.show()
    
    plt.figure()
    plt.plot(t, x_dot, '*-',label="original")
    plt.plot(t0, x0_dot, '*-',label="dt = 0.001")
    plt.plot(t1, x1_dot, '*-',label="dt = 0.01")
    plt.plot(t2, x2_dot, '*-',label="dt = 0.1")
    plt.title("Velocity vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (deg/s)")
    plt.grid()
    plt.legend()
    plt.show()
    
    # what is the time step for the error controlled integrator?
    delta_t = np.diff(t)
    
    plt.title("Time Step vs Sample Index")
    plt.plot(delta_t, '*-')
    plt.xlabel("Sample Index")
    plt.ylabel("Time Step (s)")
    plt.grid()
    plt.show()
    
    plt.hist(delta_t, bins=20, alpha=0.5, label="original")
    plt.title("Time Step Histogram")
    plt.xlabel("Time Step (s)")
    plt.ylabel("Frequency")
    plt.grid()
    plt.legend()
    plt.show()
    
    
    
