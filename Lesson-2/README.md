## Lesson 2: State Feedback

In [Lesson 1](../Lesson-1) you learned how to simulate a system with an open loop controller. In this lesson, you will close the loop by building a controller which depends on the output state.

All you have to do is add an input port to your controller with `DeclareVectorInputPort()` and then use `builder.Connect()` to connect the output of your plant to the input of your controller. Because your controller is now a function of the plant's state, you will also have to read in this input port's value via `self.get_input_port().Eval(context)`. For help, take a look at [some drake code on github](https://github.com/RobotLocomotion/drake/blob/bf0f76af4a7f29d5edcf36ebfd6da5255aa3c782/examples/manipulation_station/end_effector_teleop_sliders.py#L137).

Again, we will use a double-integrator as our system (plant) but we will design a PD controller which drives the plant to state $$(x^*, \dot{x}^\*) = (1.0, 0.0)$$ (the blocks is at 1 meter at rest).

Use a feedback controller of the following form:

$$
u = -k_p (x - x^\*)  - k_d (\dot{x} - \dot{x}^\*)
$$

where $$k_p = 10$$ and $$k_d = 5$$.

## Task

1. Finish implementing the controller (fill in the TODOs).
2. Connect the plant output to controller input, and controller output to plant input. (Fill in the TODOs).
3. Update the dynamics of your plant to include the mass and linear viscuous drag. Play around with different values for $$k_p$$ and $$k_d$$ which work well for your updated model.

## Extensions

1. Replace our LeafSystem implementation of a PD controller using Drake's [PID Controller](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_pid_controller.html). You will also need to connect some input to the desired state port using a [ConstantVectorSource](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_constant_vector_source.html). 

2. Add [saturation](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_saturation.html) to the output of your controller using a `Saturation` block. Starter code is provided.
```python
sat = builder.AddSystem(Saturation([-min_value],[max_value]))
...
builder.Connect(controller.get_output_port(), sat.get_input_port())
builder.Connect(sat.get_output_port(), plant.get_input_port())
...
```

3. Choose a new dynamical system (torque-controlled pendulum, etc.) and use a PD controller to make it go towards a goal (maybe hold a 10 degree angle with zero velocity).

### Next Steps

[Lesson 3](../Lesson-3/)