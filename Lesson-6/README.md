## Lesson 6: Discrete System and the Simulator


## Learning about Discrete Systems

A mandatory read is the documentation on [Discrete Systems](https://drake.mit.edu/doxygen_cxx/group__discrete__systems.html). The code provided in the documentation is written in C++, but this tutorial provides equivalent code in Python so that you can easily run it on your machine and follow along.

### Simple Difference Equation

We will implement a simple difference equation $$x[k+1] = x[k] + 1$$ and $$y[k] = 10 x[k]$$. Run the script `example1.py` and observe the output carefully. The output should not be too surprising.
```
t = 0.00, y = 0.00
t = 0.02, y = 10.00
t = 0.04, y = 20.00
t = 0.06, y = 30.00
```
Let's really look at what is going on under the hood. Here is the code for our example Discrete System.

```python
class ExampleDiscreteSystem(LeafSystem):
    
    # use a step size of 1/50 seconds 
    kPeriod = 1.0/50
    kOffset = 0.0
    
    def __init__(self):   
        LeafSystem.__init__(self)
        self.DeclareDiscreteState(1)
        self.DeclarePeriodicDiscreteUpdateEvent(ExampleDiscreteSystem.kPeriod, ExampleDiscreteSystem.kOffset, self.Update)
        self.DeclareVectorOutputPort("y", 1, self.Output)

    def Update(self, context, discrete_values):
        # get the current state
        x = context.get_discrete_state().get_vector().GetAtIndex(0)
        
        # update the state
        x_new = x + 1
        discrete_values.set_value([x_new])
        
        print(f"Update(): t={context.get_time()}, x={x}, x_new={x_new}")
        

    def Output(self, context, output):
        x = context.get_discrete_state().get_vector().GetAtIndex(0)
        y = 10 * x
        output.SetFromVector([y])
        
        print(f"Output(): t={context.get_time()}, x={x}, y={y}")
```

There are a few things to notice:
* We declared a discrete state with 1 element.
* We declared a periodic discrete update event that calls `self.Update()` at $$t = 0.0, 0.02, 0.04, 0.06, ...$$. Discrete Update Events can only modify discrete states. (As opposed to Unrestricted Update Events).
* We declared a vector output port to output $$y = 10 x$$.
* The function `self.Update()` modifies the discrete state.
* The function `self.Output()` sets the output port to 10 times the current value of the discrete state.

Later in the code, we connect a `MyLogger` to the output of our discrete system. This ensures that the `self.Output()` function gets called. Without something connected to the output port, there is no reason to execute the `self.Output()` function, and Drake is smart enough to know that!


We see that the printed output of our code contains the following information. 
```
Output(): t=0.0, x=0.0, y=0.0
Publish(): t=0.0, input=0.0
Update(): t=0.0, x=0.0, x_new=1.0
Output(): t=0.02, x=1.0, y=10.0
Publish(): t=0.02, input=10.0
Update(): t=0.02, x=1.0, x_new=2.0
Output(): t=0.04, x=2.0, y=20.0
Publish(): t=0.04, input=20.0
Update(): t=0.04, x=2.0, x_new=3.0
Output(): t=0.06, x=3.0, y=30.0
Publish(): t=0.06, input=30.0
```
It is important to know exactly what is going on here. Why does `Output()` get called 4 times, but `Update()` only gets called 3 times? What about `Publish()`? What about the order in which the functions are called? To understand why this order is happening, you need to have a basic understanding of the simulator.

### Simulator Basics

You are encouraged to read the documentation about the [simulator](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html). I will provide a brief summary below.

The fundamental issue with Discrete or Hybrid systems is that the discrete state $$x_d$$ can have two different values when a discrete update occurs. The documentation uses the notation $$x_d^-(t)$$ and $$x_d^+(t)$$ to denote the value of $$x_d$$ before or after the discrete update occurs. For example, in our example system, at $$t = 0.02$$ we have that $$x_d^-(t) = 1$$ but $$x_d^+(t) = 2$$. (Our code calls these values `x` and `x_new`). It is very important to note if you are logging the data before or after this discrete update occurs. (Currently, we log $$x_d^-(t)$$.)

The simulator advances through time In the `Simulator.Step()` function within the Simulator, there are a few steps:
    * Unrestricted update events occur (not applicable to us)
    * Discrete Update events occur (our call to `self.Update()`)
    * Continuous states are integrated up to the next event time (we have no continuous states, but event times occur every 1/50 seconds based on our code)
    * The value of time is advanced
    * Publish events occur (applicable)

Before any `Simulator.Step()` events occur, the `Simulator.Initialize()` function is called. This function gets the initial trajectory value (using the initial conditions) and triggers any publish events using these initial conditions as context.

### ExampleLogger

We also present a very simple logger which declares a per-step publish event.

```python
class ExampleLogger(LeafSystem):
    
    def __init__(self, input_port):
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("input", input_port.size())
        self.DeclarePerStepPublishEvent(self.Publish)
        
    def Publish(self, context):
        input_value = self.get_input_port().Eval(context)[0]
        print(f"Publish(): t={context.get_time()}, input={input_value}")
```

At the end of `Simulator.Initialize()` and at the end of `Simulator.Step()`, publish events will occur. Because the `ExampleLogger` has a per-step publish event, at this point in the code the `Publish()` function can be called. However, Drake knows that the output port of `ExampleDiscreteSystem` must be evaluated before `Publish()` can run, hence `Output()` is called just before `Publish()`. 

Piecing this altogether, you can understand the order in which these print statements occur.

1. At the end of `Simulator.Initialize()`, the `Output()` and `Publish()` functions are called.
2. In the first `Simulator.Step()`, we first update the discrete state and call `Update()`.
3. The time-step is advanced to the next event time (`t+=0.02`).
4. The `Output()` and `Publish()` functions are called.
5. Steps 2-4 are repeated until the simulation is complete.


### LogVectorOutput

The `ExampleLogger` class was presented to give clarity about the `DeclarePerStepPublishEvent`. When you call the `LogVectorOutput` function with `publish_period=0.0`, a `VectorLogSink` object is created that declares a per-step pubilsh event just like our `ExampleLogger` does. Run the 2nd example python script to see an implementation without `ExampleLogger`. You will get the following output:

```
Output(): t=0.0, x=0.0, y=0.0
Update(): t=0.0, x=0.0, x_new=1.0
Output(): t=0.02, x=1.0, y=10.0
Update(): t=0.02, x=1.0, x_new=2.0
Output(): t=0.04, x=2.0, y=20.0
Update(): t=0.04, x=2.0, x_new=3.0
Output(): t=0.06, x=3.0, y=30.0
t=0.00, y=0.00
t=0.02, y=10.00
t=0.04, y=20.00
t=0.06, y=30.00
```

You can see that the exact same order for the `Output()` and `Update()` functions have been called, and that we are able to print out the output of the `ExampleDiscreteSystem` at each time step. Compare the values of `y` here with the values of `input` in the previous example. They are the same because they are quite literally doing the same thing in code!

### Task 1

* In `example2.py`, change the value of `publish_period=0.01`. Make a prediction about how many times `Output()` and `Update()` will be called. Was your guess correct? 
* Do the same thing, but change the value of `publish_period=0.03`. What will happen now?

## Mixing Continuous and Discrete Systems

TODO