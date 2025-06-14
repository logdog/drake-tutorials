## Lesson 2.5 - Logger and Simulation Options

## Logger Options

A useful feature of Drake is that you can make the logger only record data at fixed intervals by using the `publish_period` keyword argument in the `LogVectorOutput()` function. See the documentation [here](https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1systems.html#af2f7dd25823e78194ea36eabbd525cea) for details.

As a typical use case, let's say you want to evaluate your simulation at 60 Hz, so that you can make a nice video for YouTube. You can set the publish period to `publish_period=1/60` and the logger will only record at this period. Because of how the simulator actually works under the hood (we will get into this more in Lesson 6), setting a publish period creates an upper bound for the simulator step size, which by default is chosen dynamically in order to minimize integration error.

In the section below, you will learn how to force the simulator to use a fixed step-size, if you so want. Just be aware that combining the fixed step-size integration options with the `publish_period` logger options doesn't really make sense, unless the `publish_period` is an integer multiple of the fixed step-size (because otherwise you will force the integrator to do smaller step sizes occasionally, which could be annoying to sort out). For 99% of use cases, I would leave the simulation options alone and just change the `publish_period`.

## Simulation Options
Until now, you have just been using the default options for the `Simulator`. The Simulator is actually quite sophisticated, but we won't fully explore its abilities until we start simulating hybrid systems. For now, just note that the `Simulator` object contains a pointer to an `Integrator` object which is responsible for numerically integrating our ODEs. Here is a snippet from the `Simulator` constructor in `simulator.cc` (which you can find [here](https://github.com/RobotLocomotion/drake/blob/master/systems/analysis/simulator.cc#L46-L51) on Drake's github page).
```cpp
// lines 46-51 of simulator.cc (C++ code)
integrator_ = std::unique_ptr<IntegratorBase<T>>(
      new RungeKutta3Integrator<T>(system_, context_.get()));
  integrator_->request_initial_step_size_target(kDefaultInitialStepSizeTarget);
  integrator_->set_maximum_step_size(SimulatorConfig{}.max_step_size);
  integrator_->set_target_accuracy(SimulatorConfig{}.accuracy);
  integrator_->Initialize();
```
From the code above, you can see that there is a `SimulatorConfig` class which contains default settings used to set the `max_step_size`, and `target_accuracy`. This should spark your curiosity: is there a way we can set these settings ourselves for our own simulations? The answer is yes!


In Python, we can create a `SimulatorConfig` object and then use the `ApplySimulatorConfig` function to apply these settings to the integrator. For example, if we want to set the maximum step size, this is how we do it. Be sure to import `SimulatorConfig` and `ApplySimulatorConfig` before you use them!

```python
# simulator = Simulator(diagram, context)
sim_config = SimulatorConfig(max_step_size=1e-3)
ApplySimulatorConfig(sim_config, simulator)
# simulator.Initialize()
```

For a full list of SimulatorConfig options, see the applicable documentation/source code.
* Documentation: [Simulator Configuration](https://drake.mit.edu/doxygen_cxx/group__simulator__configuration.html)
* Documentation: [SimulatorConfig Struct Reference](https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1_simulator_config.html)
* Code: [simulator_config.h](https://github.com/RobotLocomotion/drake/blob/master/systems/analysis/simulator_config.h)
* Code: [ApplySimulatorConfig() function](https://github.com/RobotLocomotion/drake/blob/master/systems/analysis/simulator_config_functions.cc#L213-L230)
* Code: [Simulator() constructor](https://github.com/RobotLocomotion/drake/blob/master/systems/analysis/simulator.cc#L46-L51)

_Advanced:_ If you are curious how we are able to call the `ApplySimulatorConfig()` function from Python (when its source code is written in C++) see the [python binding](https://github.com/RobotLocomotion/drake/blob/master/bindings/pydrake/systems/analysis_py.cc#L421C4-L426C75). You can see that there are two arguments named `config` and `simulator`, which are consistent with the names of the C++ function definition. Their types are implicitly inferred based on the signiture of the C++ function. These python bindings auto-generate Python documentation, which you can view [here](https://drake.mit.edu/pydrake/pydrake.systems.analysis.html). I don't particularly like reading the Python documentation, but it is useful to verify whether a particular C++ function has associated Python bindings.

## Tasks

1. Read the documentation for the `SimulatorConfig` to learn about all of the options. Compare this to the source code for `simulator_config.h`. Also browse through the C++ source code for `ApplySimulatorConfig()`.

2. You may have noticed the following line of code in the `ApplySimulatorConfig()` function:
```cpp
integrator.set_fixed_step_mode(!config.use_error_control);
```
This seems to imply that if we disable `use_error_control` then we can actually make the integrator use a fixed step size. You task is to experiment, and try to make the simulator use a fixed-step size. Consider the inverted pendulum as a plant

$$
\ddot x = 10 \sin x
$$

where $$x=0$$ corresponds to the pendulum in the upright position. Compare the differences betweeen using the default simulator options and versus manually setting the step sizes to `dt=0.001, dt=0.01, dt=0.1`. Investigate `dt` for the default simulator. Starter code is provided. **Make sure to disable the `publish_period` in your logger whenever you use a fixed step-size in your simulator options.** (For a learning opportunity, you can see what happens when you have both a fixed step-size simulator and a `publish_period` of 1/60. Can you explain what's happening under the hood?)

3. By default, Runge-Kutta 3 integration scheme will be used. There are other options available:

By running the following code, you can see all of the integration options.
```python
from pydrake.all import GetIntegrationSchemes
print(GetIntegrationSchemes())
```
This is the output:
```python
['bogacki_shampine3', 'explicit_euler', 'implicit_euler', 'radau1', 'radau3', 'runge_kutta2', 'runge_kutta3', 'runge_kutta5', 'semi_explicit_euler', 'velocity_implicit_euler']
```

You can also verify that `runge_kutta3` is the default for the simulator.
```python
print(ExtractSimulatorConfig(simulator))
```

For task 3, use the simulator config options to change the integration scheme. Do you notice any numerical differences between the different methods?