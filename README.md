# Drake Tutorials

Logan Dihel

Learn Drake 2025

## Foreword

Drake is a very powerful suite of tools which can help you do many things in robotics. Here are some of the highlights:

* Simulation
    * Uses block diagrams to connect controllers and plants, similar to Simulink
    * Simulate hybrid systems with discrete-events
    * Simulate systems with continuous, discrete, and abstract state components
    * Create Multibody plant
* Mathematical Programming
    * Solve convex and non-convex optimization problems quickly with fast solvers
    * Built-in tools for trajectory optimization
* Emphasis on Control
    * Use AutoDiff tools to linearize nonlinear plants about equilibrium points automatically
    * Easy to create controllers like LQR, FiniteHorizonLQR, PID, etc.
* Accessibility
    * Free and open-source
    * Written in C++ with Python Bindings
    * Easily integrates with other Python libraries

The main drawback of Drake is that it has a higher learning curve than MATLAB, and can be daunting to learn on your own. However, this tutorial is designed to get you up to speed quickly, to the point where you can comfortably create simulations and solve mathematical programs on your own.

Another main skill of Drake is learning how to read documentation and its source code. It can be overwhelming to jump right in, so I will do my best to feed in documentation and source code in small bites. The goal is to get you to the point where, if you have a question, you can independently find the answer either in the documentation or the source code.

The best way to learn anything is to experiment. There are listed Tasks and Extensions throughout these tutorials which you should at least attempt. However, if you are curious about how something works (such as reversing the gravity field in a multibody plant), you should try it out! I guarentee you will learn more from being curious and playing with things than watching YouTube tutorials and doom-scrolling documentation.

## Table of Contents

* [Lesson 0: Installation](./Lesson-0)
* [Lesson 1: Simulate Continuous Time ODE](./Lesson-1)
    * Simulate Double Integrator with Open Loop Control
* [Lesson 2: Simulate Closed Loop System](./Lesson-2)
    * PD Controller for Double Integrator
* [Lesson 2.5: Simulator Configuration](./Lesson-2.5/)
    * Setting Maximum Step Size
    * Setting Fixed Integrator Step Size
* [Lesson 3: Linearize Leaf System with Auto Diff](./Lesson-3/)
    * Linearize Pendulum About Two Equilibrium Points
    * Design and Simulate LQR Controllers
    * Implement Swingup Controller for Pendulum
* [Lesson 4: Adding Visualization using Meshcat and Scene Graph](./Lesson-4/)
    * Use Meshcat to add visualization objects decoupled from simulation
    * Use Scene Graph and Meshcat Visualizer to add geometry into simulation
* [Lesson 5: Simulate with Multibody Plant from URDF](./Lesson-5/)
<!-- * [Lesson 6: Linearize Multibody Plant]
* [Lesson 7: Discrete-Time Controllers with Continuous-Time Plants] -->


## Useful Links
* [Official Drake Tutorials](https://github.com/RobotLocomotion/drake/tree/master/tutorials).
* [Official Drake Documentation](https://drake.mit.edu/doxygen_cxx/index.html)