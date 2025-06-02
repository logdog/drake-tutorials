import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    SimulatorConfig,
    ApplySimulatorConfig,
    ZeroOrderHold,
    LogVectorOutput,
)

dt = 1/50
        
class ExampleContinuousPlant(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("u", 1)
        self.state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareVectorOutputPort("y", 2, self.Output)
    
    def DoCalcTimeDerivatives(self, context, derivatives):
        t = context.get_time()
        u = self.get_input_port().Eval(context)[0]
        theta = context.get_continuous_state_vector().GetAtIndex(0)
        theta_dot = context.get_continuous_state_vector().GetAtIndex(1)
        
        # pendulum dynamics
        theta_ddot = -9.81 * np.sin(theta) + u
        derivatives.SetFromVector([theta_dot, theta_ddot])
        print(f"Plant.DoCalcTimeDerivatives(): \tt={t:0.6f},                                     \tu={u:0.6f}")
        
        
    def Output(self, context, output):
        t = context.get_time()
        theta = context.get_continuous_state_vector().GetAtIndex(0)
        theta_dot = context.get_continuous_state_vector().GetAtIndex(1)
        
        output.SetFromVector([theta, theta_dot])
        print(f"Plant.Output(): \t\tt={t:0.6f}, theta={theta:0.6f}, theta_dot={theta_dot:0.6f}")
        
class ExampleDiscreteController(LeafSystem):
    
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("y_sampled", 2)
        self.DeclareVectorOutputPort("u", 1, self.Output)
    
    def Output(self, context, output):
        t = context.get_time()
        theta = self.get_input_port().Eval(context)[0]
        theta_dot = self.get_input_port().Eval(context)[1]
        
        # simple proportional controller
        u = -2.0 * theta - 0.5 * theta_dot
        output.SetFromVector([u])
        print(f"Controller.Output(): \t\tt={t:0.6f}, theta={theta:0.6f}, theta_dot={theta_dot:0.6f}  \tu={u:0.6f}")
        
        
class MyZeroOrderHold(LeafSystem):
    
    def __init__(self, dt, size):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("x", size)
        self.state_index = self.DeclareDiscreteState(size)
        self.DeclarePeriodicDiscreteUpdateEvent(dt, 0.0, self.LatchInputVectorToState)
        self.DeclareStateOutputPort("y", self.state_index)
    
    def LatchInputVectorToState(self, context, discrete_state):
        t = context.get_time()
        x = self.get_input_port().Eval(context)
        discrete_state.set_value(x)
        print(f"ZOH.LatchInputVectorToState(): \tt={t:0.6f}, theta={x[0]:0.6f}, theta_dot={x[1]:0.6f}")
        
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    plant = builder.AddSystem(ExampleContinuousPlant())
    zoh = builder.AddSystem(MyZeroOrderHold(dt, 2))
    controller = builder.AddSystem(ExampleDiscreteController())
    builder.Connect(plant.get_output_port(), zoh.get_input_port())
    builder.Connect(zoh.get_output_port(), controller.get_input_port())
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    
    # plant_logger = LogVectorOutput(plant.get_output_port(), builder)
    controller_logger = LogVectorOutput(controller.get_output_port(), builder, dt)
    # zoh_logger = LogVectorOutput(zoh.get_output_port(), builder)
    
    diagram = builder.Build()
    
    # plot_system_graphviz(diagram)
    # plt.show()

    # set initial conditions
    # Initial state: 30 degrees, 0 rad/s
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant_context.SetContinuousState(np.array([0.1, 0]))
    
    # zoh_context = diagram.GetMutableSubsystemContext(zoh, context)
    # zoh_context.SetDiscreteState(np.array([0.111111, 0]))  # Initial state for ZOH
    
    # create the simulator
    simulator = Simulator(diagram, context)
    simulator_config = SimulatorConfig(max_step_size=dt/2)
    ApplySimulatorConfig(simulator_config, simulator)

    # run the simulation
    simulator.Initialize()
    print('Initialize complete!\nStarting simulation...')
    simulator.AdvanceTo(3*dt)  # Run for 3 time steps
    print('Simulation complete.')
    
    controller_log = controller_logger.FindLog(context)
    t = controller_log.sample_times()
    u = controller_log.data()[0,:]
    
    for tt, uu in zip(t, u):
        print(f"t={tt:0.6f}, u={uu:0.6f}")
    
    # plant_log = plant_logger.FindLog(context)
    # zoh_log = zoh_logger.FindLog(context)
    
    # plt.stem(plant_log.sample_times(), plant_log.data()[0,:], 'bo',label='Theta (rad)')
    # plt.stem(zoh_log.sample_times(), zoh_log.data()[0,:], 'ro', label='ZOH Theta')
    # plt.xlabel('Time (s)')
    # plt.ylabel('State Values')
    # plt.title('Plant States and ZOH Outputs')
    # plt.legend()
    # plt.grid()
    # plt.show()
    
    
    