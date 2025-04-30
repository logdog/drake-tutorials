## Lesson 1: Simulate Continuous Time ODE

In this lesson, you will learn to model systems of the form

$$\dot x = f(x, u)$$

$$x(0) = x_0$$ 

where $$x_0$$ is given as some initial condition and $$u$$ is the input. In Drake, you would define this as a [LeafSystem](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_leaf_system.html). 

> Note: I will provide links to the documentaiton but it will take time to understand how to read the documentation, especially because we are using python but the documentation is auto-generated from C++ code. Almost everything in the C++ documentation has been ported to python using bindings, but not everything. If you need help seeing how something is done in Python, search the [Drake github page](https://github.com/RobotLocomotion/drake). For your first pass through this tutorial, you may skip the documentation, but later on you should at least become familiar with all the functions you use in Drake so you know what else is available.

Specifically, you will simulate a double integrator with dynamics:

$$\dot{x}_1 = x_2$$

$$\dot{x}_2 = u$$

with $$u(t) = -1$$ for sake of example. Your task is to go through the code and understand it. When you are done, you will be asked to make the following changes:

## Tasks

1. Change the initial conditions and constant input vector to something else.

2. Modify the system dynamics to change the block's mass and/or incorporate linear viscuous drag force (proportional to the velocity)

3. Replace the `ConstantVectorSource` with your own leaf system that outputs a sinusoid. Connect the output of this leaf system to the double integrator. Also, add another logger to visualize the input to the double integrator. *Hint: here is some more starter code.*

```python

class MyReferenceSignal(LeafSystem):
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("u", 1, self.MyOutput)

    def MyOutput(self, context, output):
        t = context.get_time()
        u = 10*t # modify this line
        output.SetFromVector([u])
```

