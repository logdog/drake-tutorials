## Lesson 4: Adding Visualization using Meshcat and Scene Graph

A really nice feature of Drake is that it is compatible with third party remotely controllable 3D viewer, [Meshcat](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_meshcat.html). We can connect our LeafSystem to Meshcat in order to, for example, visualize the state of our system. There are additional featuers such as the ability to add buttons, sliders, and connect joysticks such as an xbox controller, which can be used as input sources to our leaf system! In this way, you can even create your own quadrotor flight simulator! We will get to this in a future tutorial, but for now you will learn how to use Meshcat for visualization purposes only.

### Task 1

Your first task is to run the script [Lesson-4-Task-1.py](./Lesson-4-Task-1.py). Connect your browser window to [localhost:7000](http://localhost:7000) and watch the animation. Then, in the top right corner of the browser you will see an `Open Controls` button. Under the `Animations -> defualt` dropdown, you can experiment with the play, pause, reset, time, and timeScale buttons.

Next, expand the `Scene -> drake` dropdown. Note the name of the object `my_block`. You can press the checkbox to disable the visibility of the object within the animation.

Now, open up the code. You will see that we first create and simulate a sliding block system. After the simulation is over, we get the time `t` and position `x` data from the logger. Then, we create a meshcat instance and create our `my_block` object.

```python
meshcat = Meshcat()
meshcat.SetObject("my_block", Box(0.1, 0.1, 0.1), Rgba(0.5, 0.5, 0.5, 1))
```

We can then start the recording, and use `meshcat.SetTransform()` to move the block.
```python
meshcat.StartRecording(frames_per_second=64)
for i in range(len(t)):
    T = RigidTransform()
    T.set_translation([x[i],0,0.05])
    meshcat.SetTransform("my_block", T, t[i])
```

Finally, we can stop recording and publish the recording:
```
meshcat.StopRecording()
meshcat.PublishRecording()
```

> Note: the meshcat instance will die when the python script finishes running. I recommend adding a `while(True)` statement at the end of the file to keep the code running until you press `Ctrl+C`.


---

Using what you learned in Task 1 thus far, create an animation for the pendulum. You can use a Cylinder and Sphere for the pendulum arm and mass at the end of the rod. A list of all shapes available to you in Drake is given [here](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_shape.html) (scroll down to VariantShapeConstPtr). You will also find the [RotationMatrix documentation](https://drake.mit.edu/doxygen_cxx/classdrake_1_1math_1_1_rotation_matrix.html) helpful (especially the `MakeYRotation()` function). You may need to add `np.pi` to the pendulum angle if you find it in the wrong orientation. Finally, you can take advantage of the tree-like structure in Meshcat by setting the transform for the `pendulum` in your animation loop.

```python
meshcat = Meshcat()
meshcat.SetObject("pendulum/arm", ...)
meshcat.SetTransform("pendulum/arm", ...)
meshcat.SetObject("pendulum/mass", ...)
meshcat.SetTransform("pendulum/mass", ...)

meshcat.StartRecording()
for i in range(...):
    meshcat.SetTransform("pendulum", ..., t[i])
meshcat.SetTransform("pendulum", ..., t[0])

meshcat.StopRecording()
meshcat.PublishRecording()
```

> Note: it is normal for meshcat to take a bit of time (less than 1 minute) to initialize and display the pendulum object to the screen. Just be patient.

### Task 2

While it is possible to exclusively use Meshcat for adding and animating objects (via the `meshcat.SetObject()` and `meshcat.SetTransform()` functions), there is another way which opens up a whole world of possibilities. The [Scene Graph](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html) is how Drake associates geometries with objects, and the `MeshcatVisualizer` can connect to the output of scene graph in order to visualize these geometries. Scene graph is incredibly powerful because it is not just used for creating geometries so that we can see it in an animation: scene graph also allows systems (such as a camera or lidar sensor) to view the object in simulation or determine whether there was a collision between two different geometries (especially useful for hybrid systems like walking robots, which Drake was literally designed to simulate).

In this task, you will learn how to add a Scene Graph to your builder, register a geometry (such as a block) to the scene graph, and use the Meshcat Visualizer to actually see the animation on the screen. The key difference between Task 1 and Task 2 is that meshcat will now be updated *during* the simulation itself, and it will part of your diagram which you construct.

---

Start by running the example [Lesson-4-Task-2.py](Lesson-4-Task-2.py). Then take a look at the system diagram. You will notice the Scene Graph, Meshcat Visualizer, and Meshcat. Now look at the code to see how our `Block` system is connected to the `Scene Graph`.

```python
builder.Connect(plant.GetOutputPort("my_pose"), scene_graph.get_source_pose_port(source_id))
```

(The reason we cannot use `plant.get_output_port()` is because there are multiple output ports in the `Block` LeafSystem. You can use the port index e.g., 0 or 1 to say which port you want `plant.get_output_port(1)` or you can use the `plant.GetOutputPort("my_pose")` function. I prefer the named ports to remove ambiguity.)

In the `Block` class you will see our first example of an `AbstractOutputPort` which, in this case, expects us to return a `FramePoseVector()`. This is what the scene graph needs to know where our Frame (position and orientation of the block) should be displayed.

```python
class Block(LeafSystem):
    def __init__(self, frame_id):
        ...
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)

    def CalcFramePoseOutput(self, context, output):
        ...
```

We need to register a frame and give it a corresponding geometry. See the code's comments for more details.
```python
source_id = scene_graph.RegisterSource("my_block_source")
frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
geometry_id = scene_graph.RegisterGeometry(source_id, frame_id, 
    GeometryInstance(RigidTransform(RotationMatrix(), np.array([0,0,0.05])), Box(0.1, 0.1, 0.1), "my_geometry_instance"))
scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
```

The Meshcat Visualizer is added to the builder via the following function call:
```python
meshcat = Meshcat()
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
```

Documentation for all of this is spread out all across Drake. I recommend you look at the following pages for continued learning:
* [Scene Graph](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html)
* [Geometry Queries and Roles](https://drake.mit.edu/doxygen_cxx/group__geometry__roles.html)
* Read the documentation for each function call `SceneGraph.RegisterSource` etc. that used in this Step.

---

For your assignment, create an animation for the swingup pendulum simulation using what you learned in task 2. You will need to associate the same `frame_id` for two different geometry instances: `pendulum_mass` and `pendulum_arm`.

If you would like another challenge, try to change the color of the mass and arm. 
* Hint: there is a helper function to do this. Search for Phong at [this link](https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1geometry.html).
* If you want to know why I would ever search "Phong", it's because I read [this documentation about Drake materials](https://drake.mit.edu/doxygen_cxx/group__geometry__file__formats.html).
* In the solution, I provide two different methods of achieving the same goal.