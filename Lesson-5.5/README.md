## Lesson 5.5: Linearize Multibody Plant

This is a short lesson which shows you how to linearize a MultibodyPlant. We will use the model of the triple pendulum we created in Lesson 5, Task 4. We are linearizing about the equilibrium point where $\theta = (\pi, 0, 0)$ and $\dot \theta = (0,0,0)$ (the pendulum is vertical and in the air). At this equilibrium point we need $u = (0,0,0)$ (the torque applied to each joint).

First, we load the URDF into a MultibodyPlant, add the joint actuators, and then finalize the plant. (You could also skip the `AddJointActuator()` calls if you added `<transmission>` into the URDF file.)

```python
    urdf_path = os.path.join(os.path.dirname(__file__), "TP.urdf")

    meshcat = Meshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0)
    parser = Parser(plant)
    parser.AddModels(urdf_path)

    # add joint actuators 
    # (necessary because we didn't include <transmission> in .urdf file)
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
```

Now we can do the linearization.
```python
    # create the LQR controller
    context = plant.CreateDefaultContext()
    context.SetContinuousState(np.array([np.pi, 0, 0, 0, 0, 0])) # equilibrium state (upside down)
    plant.get_actuation_input_port().FixValue(context, np.array([0,0,0])) # equilibrium input

    # linearize the diagram about its input and output ports
    sys = Linearize(plant, 
                    context, 
                    input_port_index=plant.get_actuation_input_port().get_index(), 
                    output_port_index=plant.get_state_output_port().get_index()
    )
```

If you want to inspect the linear system, you can do that. Here we also get the matrix $K$.
```python
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
```

If instead, you want to create an LQR controller as a system block to add into your diagram, you can use the following code. Note that we need to make sure the input of the LQR controller is in the error coodinates $e(t) = x(t) - x^*(t)$ so we use an `Adder` block.
```python
    # Just for fun, let's simulate a finite horizon LQR controller 
    # connect the LQR controller to the plant
    lqr = builder.AddSystem(LinearQuadraticRegulator(sys, Q, R))
    adder = builder.AddSystem(Adder(2, 6))
    
    adder_const = builder.AddSystem(ConstantVectorSource(np.array([-np.pi, 0, 0, 0, 0, 0])))
    builder.Connect(adder_const.get_output_port(), adder.get_input_port(1))
    
    builder.Connect(lqr.get_output_port(), plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(), adder.get_input_port(0))
    builder.Connect(adder.get_output_port(), lqr.get_input_port())
```

### Task 1

Simulate the triple pendulum with LQR control. All you have to do is run [triple_pend_LQR.py](triple_pend_LQR.py).

### Task 2

Remove the "wrist" input from the model. Create a new LQR controller, and see if you can still make it balance.

Hint: the point of this exercise is really to make sure you know how many dimensions the state and input vectors should be.

### Extensions

Linearize some other MultibodyPlant. Use that model to create an LQR controller to stabilize said plant.