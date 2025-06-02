import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    TriggerType,
    ZeroOrderHold,
    LogVectorOutput,
)

        
class PendulumPlant(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("u", 1)
        self.state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("y", self.state_index)
    
    def DoCalcTimeDerivatives(self, context, derivatives):
        u = self.get_input_port().Eval(context)[0]
        theta = context.get_continuous_state_vector().GetAtIndex(0)
        theta_dot = context.get_continuous_state_vector().GetAtIndex(1)
        
        # pendulum dynamics
        theta_ddot = -9.81 * np.sin(theta) + u
        derivatives.get_mutable_vector().SetFromVector([theta_dot, theta_ddot])
        
class Controller(LeafSystem):
    
    def __init__(self):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("y", 2)
        self.DeclareVectorOutputPort("u", 1, self.Output)
    
    def Output(self, context, output):
        theta = self.get_input_port().Eval(context)[0]
        theta_dot = self.get_input_port().Eval(context)[1]
        
        # simple proportional controller
        u = -2.0 * theta - 0.5 * theta_dot
        output.SetFromVector([u])


def sim_discrete_system(dt, tfinal=10):
    assert dt > 0, "dt must be positive"
    
    builder = DiagramBuilder()
    plant = builder.AddSystem(PendulumPlant())
    zoh = builder.AddSystem(ZeroOrderHold(dt, 2))
    controller = builder.AddSystem(Controller())
    builder.Connect(plant.get_output_port(), zoh.get_input_port())
    builder.Connect(zoh.get_output_port(), controller.get_input_port())
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    
    controller_logger = LogVectorOutput(controller.get_output_port(0), builder, set([TriggerType.kForced]))
    plant_logger = LogVectorOutput(plant.get_output_port(), builder)
    
    diagram = builder.Build()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant_context.SetContinuousState(np.array([np.deg2rad(30), 0]))
    # create the simulator
    simulator = Simulator(diagram, context)

    # run the simulation
    simulator.Initialize()
    
    for i in range(1, int(tfinal/dt) + 1):
        simulator.AdvancePendingEvents()
        controller_logger.ForcedPublish(controller_logger.GetMyContextFromRoot(context))
        simulator.AdvanceTo(i*dt)
    
    plant_log = plant_logger.FindLog(context)
    t_continuous = plant_log.sample_times()
    theta_continuous = plant_log.data()[0,:]
    
    controller_log = controller_logger.FindLog(context)
    t_discrete = controller_log.sample_times()
    u_discrete = controller_log.data()[0,:]
    
    return t_continuous, theta_continuous, t_discrete, u_discrete

def sim_continuous_system(tfinal=10):
    
    builder = DiagramBuilder()
    plant = builder.AddSystem(PendulumPlant())
    controller = builder.AddSystem(Controller())
    builder.Connect(plant.get_output_port(), controller.get_input_port())
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    
    controller_logger = LogVectorOutput(controller.get_output_port(), builder)
    plant_logger = LogVectorOutput(plant.get_output_port(), builder)
    
    diagram = builder.Build()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)
    plant_context.SetContinuousState(np.array([np.deg2rad(30), 0]))
    # create the simulator
    simulator = Simulator(diagram, context)

    # run the simulation
    simulator.Initialize()
    simulator.AdvanceTo(tfinal)
    
    plant_log = plant_logger.FindLog(context)
    t = plant_log.sample_times()
    theta = plant_log.data()[0,:]
    
    controller_log = controller_logger.FindLog(context)
    u = controller_log.data()[0,:]
    
    return t, theta, u

        
if __name__ == "__main__":
    
    sim_continuous = sim_continuous_system()
    sim_1000 = sim_discrete_system(dt=1/1000)
    sim_100 = sim_discrete_system(dt=1/100)
    sim_10 = sim_discrete_system(dt=1/10)
    
    # plot the states
    plt.figure()
    plt.plot(sim_continuous[0], np.rad2deg(sim_continuous[1]), label='Continuous', color='blue')
    plt.plot(sim_1000[0], np.rad2deg(sim_1000[1]), label='dt=1/1000', color='orange')
    plt.plot(sim_100[0], np.rad2deg(sim_100[1]), label='dt=1/100', color='green')
    plt.plot(sim_10[0], np.rad2deg(sim_10[1]), label='dt=1/10', color='red')
    plt.title("Pendulum Angle vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")
    plt.grid()
    plt.legend()
    plt.show()
    
    # plot the control input
    plt.figure()
    plt.plot(sim_continuous[0], sim_continuous[2], label='Continuous', color='blue')
    plt.step(sim_1000[2], sim_1000[3], label='dt=1/1000', where='post', color='orange')
    plt.step(sim_100[2], sim_100[3], label='dt=1/100', where='post', color='green')
    plt.step(sim_10[2], sim_10[3], label='dt=1/10', where='post', color='red')
    plt.title("Control Input vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input (N)")
    plt.grid()
    plt.legend()
    plt.show()
    
    
    