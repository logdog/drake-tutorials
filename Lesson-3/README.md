## Lesson 3: Linearize Leaf System

As you know, linearization is a very useful technique in optimal control. For example, we can linearize a bicopter about the hovering equilibrium point and use an LQR controller (based on the linearized dynamics) to fly the bicopter autonomously surprisingly well.

In this lesson, you will create a leaf system implementing the dynamics of a pendulum, and then use the auto-diff tools built into drake to linearize the pendulum about both its equilibrium points. As a word of caution, using auto-diff in Python with Drake is quite smelly, but it's worth it for how amazing the results are.

Read through the [starter code](Lesson-3-starter-code.py) and focus on what you are familiar with. You should recognize a lot of the stuff in that file from Lessons 1 and 2, but there is a lot of weird stuff going on as well.

Here is the structure of the Pendulum "class":

```python
@TemplateSystem.define("Pendulum_")
def Pendulum_(T):
    
    class Impl(LeafSystem_[T]):
        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter)
            ...
            
        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)

        def DoCalcTimeDerivatives(self, context, derivatives):
            ...

    return Impl

Pendulum = Pendulum_[None] # default instantiation
```

where the `...` is filled in with code you already know how to write. This structure is what allows the auto-diff to take derivatives of the dynamics with respect to inputs, outputs, and state variables. For further reading, see [an official tutorial](https://github.com/RobotLocomotion/drake/blob/bf0f76af4a7f29d5edcf36ebfd6da5255aa3c782/tutorials/authoring_leaf_systems.ipynb#L505) and the [cpp documentation](https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html) on system scalar conversion.

### Task 1

Use the starter code to get your matrix `K`. Then, in a separate file, create a simulation of a pendulum with an LQR state feedback controller which drives the system state to the origin. Make sure to start the pendulum at a non-zero initial condition, and note that all angles are in radians. The feedback control law is given by $$u = - K x$$ where $$x = (\theta, \dot\theta)$$ is the pendulum state and $$K$$ is the LQR gain matrix you were given in the starter code. (Just copy and paste the values from the terminal with `Ctrl+Shift+C`).

### Task 1b

Next, you can modify your code to make the pendulum balance at the upright equilibrium point. Simply linearize the pendulum about the top equilibrium point to get a new matrix $$K$$, update your control law $$u = -K(x - x^\*)$$ where $$x^\* = (\pi, 0)$$ and set the initial condition to be close to the upright equilibrium point (the controller is linear after all, and going too far away from the equilibrium point may cause your controller to fail).

### Task 2 (Hard!)

Now, you will create a closed-loop system which can do swing-up control of the inverted pendulum using energy shaping. The idea is simple: you want to get the total energy of the system to be equal to $$E = 2 m g \ell$$ (the potential energy of the pendulum in the upright position). So if the total energy is too low, pump energy into the system by adding torque in the direction the pendulum is swinging. If the total energy is too high, do the opposite. Practically, you will want to define an energy error term to be the difference between your current total energy and the target energy level, and add energy proportional to this error term. I recommend you add a function in your pendulum plant which calculates the total amount of energy in the system (see hint below).

> Hint: create functions `EvalPotentialEnergy(self, context)` and `EvalKineticEnergy(self, context)` in your pendulum class which will return (not output - no need to use an output port here) the potential and kinetic energy. Your energy shaping controller can call these functions (with the correct context passed in) in its `DoOutput()` function. Make sure that when the pendulum state is at the origin, the total energy is zero.

When the energy error is small and the pendulum is near the upright position, you can switch to an LQR controller. You will need to linearize the pendulum about the upright equilibrium point in order to get the correct K matrix. Also, the feedback controller should be $$u = -K(x - x^\*)$$ where $$x^\* = (\pi, 0)$$ is the target upright state of the pendulum.

> Note: you can either make a few difference controllers (energy shaping, LQR) and then create a third system (mux) to dynamically switch which controller output is actually going to the plant based on the energy level and plant state, or you can just define one massive controller to do everything (what I did). The choice is yours.

> Warning: because the pendulum dynamics technically evolve on a circle, there can be issues with "wrapping" the angle $$\theta$$. We have defined the target position to be $$x^\* = (\pi, 0)$$ so if the pendulum is upright at $$x = (-\pi, 0)$$, the controller won't actually realize the pendulum is in the upright state and will apply counterclockwise torque to increase the value of $$\theta$$. You can either account for this by wrapping $$\theta$$ to be on the interval $$[0,2\pi)$$ inside your controller, or you can just cheat and use an initial condition that just "works" by avoiding this problem (this might take a few attempts to guess good initial conditions - I found `(-10 deg, 0)` did the trick.). Wrapping can be accomplished with `np.mod(theta, np.pi*2)`.

This is a hard task, but you can find inspiration [in Chapter 2 of underactuated.mit.edu](https://underactuated.mit.edu/pend.html#section3) and its [deepnote notebook solution](https://deepnote.com/workspace/Underactuated-2ed1518a-973b-4145-bd62-1768b49956a8/project/314062d5-b839-4089-b02f-6c21e42e9581/notebook/energy_shaping-fa3f57d134b6498ea52d50dec2128d03). If you need to peek at my solution, go ahead, but instead of copy and pasting immediately, try to read and understand my plant and controller.

> Warning: make sure you set the initial condition to a non-zero value! Otherwise the energy shape control will no nothing because $$\dot\theta(t) = 0$$ so the output torque will be zero!

> Pro Tip: if you are having trouble with energy shaping, you can just set the initial condition to something like 179 degrees and see if your LQR controller can balance the pendulum upright.

### Next Steps

[Lesson 4](../Lesson-4/)