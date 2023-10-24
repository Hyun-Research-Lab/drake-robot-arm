import numpy as np
from pydrake.all import (
    DiagramBuilder,
    Simulator,
    LeafSystem,
)

class Rock(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        position_state_index = self.DeclareContinuousState(1,1,0)
        self.DeclareStateOutputPort("x", position_state_index)
        
    def DoCalcTimeDerivatives(self, context, derivatives):
        state = context.get_continuous_state_vector().CopyToVector()
        x_dot = state[1]
        x_ddot = -9.81
        derivatives.get_mutable_vector().SetFromVector([x_dot, x_ddot])


def main():
    builder = DiagramBuilder()
    plant = builder.AddSystem(Rock())
    diagram = builder.Build()

    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(root_context=root_context)
    
    # initial conditions for plant
    plant_context.get_mutable_continuous_state_vector().SetFromVector([0,10])
    
    simulator = Simulator(diagram, root_context)
    simulator.Initialize()
    simulator.AdvanceTo(1.0)


if __name__ == "__main__":
    main()
