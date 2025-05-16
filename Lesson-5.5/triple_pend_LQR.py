import copy
import numpy as np

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    LogVectorOutput,
    Meshcat,
    Parser,
    AddMultibodyPlantSceneGraph, 
    Simulator,
    DiagramBuilder,
    AddDefaultVisualization,
    Linearize,
    Adder,
    LinearQuadraticRegulator,
    ConstantVectorSource,
)

import os
import matplotlib.pyplot as plt

if __name__ == "__main__":
    
    urdf_path = os.path.join(os.path.dirname(__file__), "TP.urdf")

    meshcat = Meshcat()
    meshcat.SetCameraPose(
        camera_in_world=np.array([0, -6, 2]),
        target_in_world=np.array([0, 0, 2]),
    )
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0)
    parser = Parser(plant)
    parser.AddModels(urdf_path)

    # add joint actuators
    shoulder_act = plant.AddJointActuator(
        name="shoulder_act",
        joint=plant.GetJointByName("shoulder")
    )
    elbow_act = plant.AddJointActuator(
        name="elbow_act",
        joint=plant.GetJointByName("elbow")
    )
    wrist_act = plant.AddJointActuator(
        name="wrist_act",
        joint=plant.GetJointByName("wrist")
    )
    
    # if using a discrete-time system, you might need to use the line below to remove
    # the abstract sample variable "s" from our context for linearization purposes
    # plant.SetUseSampledOutputPorts(False) # this removes the abstract sample variable "s" from our context for linerization
    plant.Finalize()

    # create the LQR controller
    context = plant.CreateDefaultContext()
    context.SetContinuousState(np.array([np.pi, 0, 0, 0, 0, 0])) # equilibrium state (upside)
    plant.get_actuation_input_port().FixValue(context, np.array([0,0,0])) # equilibrium input

    # linearize the diagram about its input and output ports
    sys = Linearize(plant, 
                    context, 
                    input_port_index=plant.get_actuation_input_port().get_index(), 
                    output_port_index=plant.get_state_output_port().get_index()
    )
    # print the A, B, C, D matrices of the linear system
    print("A matrix:")
    print(sys.A())
    print("B matrix:")
    print(sys.B())
    print("C matrix:")
    print(sys.C())
    print("D matrix:")
    print(sys.D())
    
    # do LQR
    Q = 10*np.diag([1, 1, 1, 1, 1, 1])
    R = np.eye(3)
    K = LinearQuadraticRegulator(sys.A(), sys.B(), Q, R)[0]
    
    print("LQR gain matrix:")
    print(K)
    
    # Just for fun, let's simulate a finite horizon LQR controller 
    # connect the LQR controller to the plant
    lqr = builder.AddSystem(LinearQuadraticRegulator(sys, Q, R))
    adder = builder.AddSystem(Adder(2, 6))
    
    adder_const = builder.AddSystem(ConstantVectorSource(np.array([-np.pi, 0, 0, 0, 0, 0])))
    builder.Connect(adder_const.get_output_port(), adder.get_input_port(1))
    
    builder.Connect(lqr.get_output_port(), plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(), adder.get_input_port(0))
    builder.Connect(adder.get_output_port(), lqr.get_input_port())
    
    # add a logger to log the state
    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    
    # add a logger to log the input
    logger_input = LogVectorOutput(lqr.get_output_port(), builder)
    
    # Add visualization to see the geometries.
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    
    # simulate the system
    context = diagram.CreateDefaultContext()
    context.SetContinuousState(np.array([np.pi+np.pi/12, 0, 0, 0, 0, 0]))
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    
    meshcat.StartRecording()
    simulator.AdvanceTo(10.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    print(f"{meshcat.web_url()=}")
    
    # system diagram
    plot_system_graphviz(diagram)
    plt.show()
    
    # plot the results
    logger_data = logger.FindLog(context)
    logger_input_data = logger_input.FindLog(context)
    time = logger_data.sample_times()
    
    state = logger_data.data()
    input = logger_input_data.data()
    
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(time, state[0,:], label="theta")
    plt.plot(time, state[1,:], label="theta_dot")
    plt.plot(time, state[2,:], label="phi")
    plt.plot(time, state[3,:], label="phi_dot")
    plt.plot(time, state[4,:], label="psi")
    plt.plot(time, state[5,:], label="psi_dot")
    plt.legend()
    # plt.xlabel("time (s)")
    plt.ylabel("state (rad, rad/s)")
    plt.title("State response")
    plt.grid()
    
    plt.subplot(2, 1, 2)
    plt.plot(time, input[0,:], label="shoulder")
    plt.plot(time, input[1,:], label="elbow")
    plt.plot(time, input[2,:], label="wrist")
    plt.legend()
    plt.xlabel("time (s)")
    plt.ylabel("torque")
    plt.title("Input response")
    plt.grid()
    
    plt.show()
    
    while True:
        pass
    
    
    