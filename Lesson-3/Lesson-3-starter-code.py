import numpy as np

from pydrake.all import (
    Linearize,
    TemplateSystem,
    LeafSystem_, 
    LinearQuadraticRegulator,
)

@TemplateSystem.define("Pendulum_")
def Pendulum_(T):
    
    class Impl(LeafSystem_[T]):
        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter)
            
            # declare input port
            self.DeclareVectorInputPort("u", 1) # torque (N-m)
            
            # declare continuous state
            state_index = self.DeclareContinuousState(1, 1, 0) # theta, theta_dot
            self.DeclareStateOutputPort("x", state_index)
            
            # write down some system parameters
            self.m = 1.0
            self.b = 0.1
            self.g = 9.81
            self.l = 1.0
            
        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)

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
    return Impl

Pendulum = Pendulum_[None] # default instantiation

if __name__ == "__main__":
    
    # linearize the pendulum  about the bottom equilibrium point
    pendulum = Pendulum()
    
    # choose the equilibrium point we wish to linearize about
    context = pendulum.CreateDefaultContext()
    context.SetContinuousState([np.pi, 0])
    pendulum.get_input_port().FixValue(context, [0]) 

    sys = Linearize(pendulum, context)
    
    # Compute the LQR controller for the pendulum (you can tune Q, R as you wish)
    Q = np.diag([10, 1])
    R = np.diag([1])
    K = LinearQuadraticRegulator(sys.A(), sys.B(), Q, R)[0]
    
    # print the results of linearization and LQR
    print(f"{sys.A()=}, \n{sys.B()=}, \n{sys.C()=}, \n{sys.D()=}")
    print(f"Computed LQR controller: {K=}")
    
    