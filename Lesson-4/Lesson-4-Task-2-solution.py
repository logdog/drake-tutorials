import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (
    plot_system_graphviz,
    DiagramBuilder,
    Simulator,
    LeafSystem,
    LogVectorOutput,
    Meshcat,
    RigidTransform,
    RotationMatrix,
    Cylinder,
    Sphere,
    Rgba,
    SceneGraph,
    MeshcatVisualizer,
    GeometryFrame,
    GeometryInstance,
    IllustrationProperties,
    AbstractValue,
    Box,
    FramePoseVector,
    MakePhongIllustrationProperties,
)

class Pendulum(LeafSystem):
    
    def __init__(self, frame_id):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("u", 1)
        self.state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("y", self.state_index)
        
        self.m = 1.0  # mass of the pendulum
        self.b = 0.01  # damping coefficient
        self.g = 9.81  # acceleration due to gravity
        self.l = 1.0  # length of the pendulum
        
        # visualization
        self.frame_id = frame_id
        self.DeclareAbstractOutputPort("my_pose", lambda: AbstractValue.Make(FramePoseVector()), self.CalcFramePoseOutput)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        # unpack the state
        state = context.get_continuous_state_vector().CopyToVector()
        theta, theta_dot = state
        
        # read the input
        u = self.get_input_port().Eval(context)[0]
        
        # state space equations
        theta_ddot = 1/(self.m*self.l**2) * (u - self.b*theta_dot - self.m*self.g*self.l*np.sin(theta))
        
        derivatives.get_mutable_vector().SetFromVector(
            np.array([theta_dot, theta_ddot])
        )
        
        
    # the functions CalcPotentialEnergy, CalcKineticEnergy are already declared,
    # so let's use EvalPotentialEnergy, EvalKineticEnergy instead to avoid name conflicts
    def EvalPotentialEnergy(self, context):
        state = context.get_continuous_state_vector().CopyToVector()
        theta, theta_dot = state
        potential_energy = self.m * self.g * self.l * (1 - np.cos(theta))
        return potential_energy
    
    def EvalKineticEnergy(self, context):
        state = context.get_continuous_state_vector().CopyToVector()
        theta, theta_dot = state
        kinetic_energy = 0.5 * self.m * (self.l*theta_dot)**2
        return kinetic_energy
        
    def EvalTotalEnergy(self, context):
        total_energy = self.EvalPotentialEnergy(context) + self.EvalKineticEnergy(context)
        return total_energy
    
    # visualization
    def CalcFramePoseOutput(self, context, output):
        x, x_dot = context.get_continuous_state_vector().CopyToVector()
        
        T = RigidTransform()
        T.set_rotation(RotationMatrix.MakeYRotation(np.pi + x))

        output.get_mutable_value().set_value(self.frame_id, T)
        
    
        
class SwingUpAndBalanceController(LeafSystem):
    
    def __init__(self, pendulum):   
        LeafSystem.__init__(self)
        self.DeclareVectorInputPort("x", 2)
        self.DeclareVectorOutputPort("u", 1, self.MyOutput)

        # calculated from starter code
        self.K = np.array([20.11708979,  6.32216315])

        # target energy for the pendulum
        self.pendulum = pendulum
        up_context = pendulum.CreateDefaultContext()
        up_context.SetContinuousState(np.array([np.pi, 0.0]))
        self.E_target = pendulum.EvalTotalEnergy(up_context)
        
        # we can use any context object to calculate energy
        # so let's hold on to one for later use
        self.pendulum_context = pendulum.CreateDefaultContext()

    def MyOutput(self, context, output):
        # unpack the state
        state = self.get_input_port().Eval(context)
        theta, theta_dot = state
        
        # wrap theta to [0, 2*pi)
        theta = np.mod(theta, 2*np.pi)
        
        # calculate the total energy of the pendulum using a context object
        self.pendulum_context.SetContinuousState(np.array([theta, theta_dot]))
        total_energy = self.pendulum.EvalTotalEnergy(self.pendulum_context)
        
        # energy shaping controller
        energy_diff = total_energy - self.E_target
        u_energy = -0.1 * theta_dot * energy_diff

        # LQR controller
        u_lqr = -self.K @ (np.array([theta, theta_dot]) - np.array([np.pi, 0.0]))
        
        # mux
        if np.abs(energy_diff) < 0.1 * self.E_target and np.abs(theta - np.pi) < np.deg2rad(10):
            u = u_lqr
        else:
            u = u_energy
        
        output.SetFromVector([u])
        
if __name__ == "__main__":
    
    # create the diagram
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())

    # Add a frame "my_frame" to the scene_graph and add a geometry "my_geometry_instance" to it
    source_id = scene_graph.RegisterSource("my_block_source")
    frame_id = scene_graph.RegisterFrame(source_id, GeometryFrame("my_frame", 0))
    
    # arm (bonus: add red color)
    arm_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(RigidTransform(RotationMatrix(), np.array([0,0,0.5])), Cylinder(0.05, 1.0), "cylinder_geometry"))
    props = IllustrationProperties()
    props.AddProperty("phong", "diffuse", Rgba(1, 0, 0, 1.0))
    scene_graph.AssignRole(source_id, arm_id, props)
    
    # mass (bonus: add cyan color)
    mass_id = scene_graph.RegisterGeometry(source_id, frame_id, 
        GeometryInstance(RigidTransform(RotationMatrix(), np.array([0,0,1])), Sphere(0.1), "sphere_geometry"))
    scene_graph.AssignRole(source_id, mass_id, MakePhongIllustrationProperties([0, 1, 1, 1.0]))
    
    # add the meshcat visualizer to the diagram
    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    # add the plant to the diagram (be sure to pass in the frame id!)
    plant = builder.AddSystem(Pendulum(frame_id))
    plant.set_name("Pendulum Plant")
    
    # connect the "my_pose" output port to the scene graph
    builder.Connect(plant.GetOutputPort("my_pose"), scene_graph.get_source_pose_port(source_id))


    controller = builder.AddSystem(SwingUpAndBalanceController(plant))
    controller.set_name("SwingUpAndBalance Controller")
    
    builder.Connect(controller.get_output_port(), plant.get_input_port())
    builder.Connect(plant.GetOutputPort("y"), controller.get_input_port())
    
    logger = LogVectorOutput(plant.GetOutputPort("y"), builder)
    logger.set_name("output state logger")
    
    logger2 = LogVectorOutput(controller.get_output_port(), builder)
    logger2.set_name("controller logger")
    
    diagram = builder.Build()
    diagram.set_name("Closed Loop System (Solution)")
    
    # plot_system_graphviz(diagram)
    # plt.show()
    
    # set initial conditions
    context = diagram.CreateDefaultContext()
    context.SetTime(0.0)
    context.SetContinuousState(np.deg2rad(np.array([-10, 0])))
    
    # create the simulator
    simulator = Simulator(diagram, context)
    
    # run the simulation
    print("Running simulation...")
    meshcat.StartRecording()
    simulator.Initialize()
    simulator.AdvanceTo(10.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    
    
    while True:
        pass
    
    