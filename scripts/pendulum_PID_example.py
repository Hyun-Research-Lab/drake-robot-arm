# https://deepnote.com/@jeh15/Drake-Tutorial-Modeling-Systems-59e84db9-b9b5-42c4-a84f-11763ed115df

# Import Libraries:
import pydrake.symbolic
from pydrake.symbolic import Variable
from pydrake.systems.primitives import SymbolicVectorSystem
import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput
from pydrake.systems.controllers import PidController
from pydrake.systems.framework import LeafSystem

def open_loop_pendulum():
    # create the pendulum as a symbolic vector system
    g = 9.81
    L = 1
    th = Variable("th") # state
    dth = Variable("dth") # state
    pendulum = SymbolicVectorSystem(
        state=[th, dth], 
        dynamics=[dth, -g / L * pydrake.symbolic.sin(th)], 
        output=[th, dth]
    )

    # Create Block Diagram containing our system:
    builder = DiagramBuilder()
    builder.AddSystem(pendulum)

    logger = LogVectorOutput(pendulum.get_output_port(0), builder)
    diagram = builder.Build()

    # Set Initial Conditions: [theta(0) = pi / 4, dtheta(0) = 0]
    context = diagram.CreateDefaultContext()
    context.SetContinuousState([np.pi / 4.0, 0.0])

    # Create Simulator and simulate for 10 seconds:
    simulator = Simulator(diagram, context)
    simulator.AdvanceTo(10)

    # Grab results from Logger:
    log = logger.FindLog(context)
    time = log.sample_times()
    data = log.data().transpose()
    theta = data[:, 0]
    dtheta = data[:, 1]

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(time, theta, color='r')
    plt.plot(time, dtheta, color='b')
    plt.xlabel("time (s)")
    plt.grid()
    plt.legend(["theta", "dtheta"])
    plt.show()

def pendulum_PID():

    # create the pendulum as a symbolic vector system
    g = 9.81
    L = 1
    u = Variable("u") # input
    th = Variable("th") # state
    dth = Variable("dth") # state
    pendulum = SymbolicVectorSystem(
        state=[th, dth], 
        input=[u],
        dynamics=[dth, -g / L * pydrake.symbolic.sin(th) + u], 
        output=[th, dth]
    )

    # Create Block Diagram containing our system:
    builder = DiagramBuilder()

    # Add Pendulum System to Diagram:
    builder.AddSystem(pendulum)

    # Add PID Controller to Diagram:
    controller = builder.AddSystem(PidController(kp=[10.0], ki=[1.0], kd=[1.0]))

    # Connect the Controller to the Plant:
    # this code is working
    # https://drake.mit.edu/pydrake/pydrake.systems.controllers.html#pydrake.systems.controllers.PidController
    builder.Connect(pendulum.get_output_port(), controller.get_input_port_estimated_state())
    builder.Connect(controller.get_output_port_control(), pendulum.get_input_port())

    # Make the Desired State Input of the Controller an Input to the Diagram:
    builder.ExportInput(controller.get_input_port_desired_state()) # size = 2
    # print(controller.get_input_port_desired_state().size())

    # Log the States of the Pendulum:
    logger = LogVectorOutput(pendulum.get_output_port(0), builder)
    logger.set_name("logger")

    diagram = builder.Build()
    diagram.set_name("diagram")

    # Set Simulator to Run Diagram:
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Desired Pendulum Angle:
    desired_angle = np.pi / 2.0

    # First Extract the subsystem context for the pendulum:
    pendulum_context = diagram.GetMutableSubsystemContext(pendulum, context)
    pendulum_context.get_mutable_continuous_state_vector().SetFromVector([0.0, 0.0])

    # Set Desired State to Diagram Input Port:
    diagram.get_input_port(0).FixValue(context, [desired_angle, 0.])

    # Simulate for 20 seconds.
    simulator.AdvanceTo(20)

    # Grab results from Logger:
    log = logger.FindLog(simulator.get_context())
    time = log.sample_times()
    data = log.data().transpose()
    theta = data[:, 0]
    dtheta = data[:, 1]

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(time, theta, color='r')
    plt.plot(time, dtheta, color='b')
    plt.xlabel("time (s)")
    plt.grid()
    plt.legend(["theta", "dtheta"])
    plt.show()


# FINALLY WORKS
# you must use LeafSystem.DeclareContinuousState(1,1,0) AND NOT LeafSystem.DeclareContinuousState(2)
# because the first argument is the number of positions, and the second argument is the number of velocities
# otherwise direct feedthrough is a problem and your code WILL NOT RUN
def pendulum_PID_vector_system():

    # create the pendulum as a leaf system
    # example: https://github.com/RussTedrake/underactuated/blob/master/underactuated/quadrotor2d.py#L44
    class Pendulum(LeafSystem):
        def __init__(self):
            LeafSystem.__init__(self)
            self.DeclareVectorInputPort("u", 1)
            state_index = self.DeclareContinuousState(1,1,0)
            self.DeclareStateOutputPort("x", state_index)

        def DoCalcTimeDerivatives(self, context, derivatives):
            x = context.get_continuous_state_vector().CopyToVector()
            u = self.EvalVectorInput(context, 0).CopyToVector()[0]

            qdot = x[1]
            qddot = -9.81 / 1 * np.sin(x[0]) + u

            derivatives.get_mutable_vector().SetFromVector(np.array([qdot, qddot]))


    # Create Block Diagram containing our system:
    builder = DiagramBuilder()

    # Add Pendulum System to Diagram:
    pendulum = builder.AddSystem(Pendulum())

    # Add PID Controller to Diagram:
    controller = builder.AddSystem(PidController(kp=[10.0], ki=[1.0], kd=[1.0]))

    # Connect the Controller to the Plant:
    # this code is working
    # https://drake.mit.edu/pydrake/pydrake.systems.controllers.html#pydrake.systems.controllers.PidController
    builder.Connect(pendulum.get_output_port(), controller.get_input_port_estimated_state())
    builder.Connect(controller.get_output_port_control(), pendulum.get_input_port())

    # Make the Desired State Input of the Controller an Input to the Diagram:
    builder.ExportInput(controller.get_input_port_desired_state()) # size = 2
    # print(controller.get_input_port_desired_state().size())

    # Log the States of the Pendulum:
    logger = LogVectorOutput(pendulum.get_output_port(0), builder)
    logger.set_name("logger")

    diagram = builder.Build()
    diagram.set_name("diagram")

    # Set Simulator to Run Diagram:
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    # Desired Pendulum Angle:
    desired_angle = np.pi / 2.0

    # First Extract the subsystem context for the pendulum:
    pendulum_context = diagram.GetMutableSubsystemContext(pendulum, context)
    pendulum_context.get_mutable_continuous_state_vector().SetFromVector([0.0, 0.0])

    # Set Desired State to Diagram Input Port:
    diagram.get_input_port(0).FixValue(context, [desired_angle, 0.])

    # Simulate for 20 seconds.
    simulator.AdvanceTo(20)

    # Grab results from Logger:
    log = logger.FindLog(simulator.get_context())
    time = log.sample_times()
    data = log.data().transpose()
    theta = data[:, 0]
    dtheta = data[:, 1]

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(time, theta, color='r')
    plt.plot(time, dtheta, color='b')
    plt.xlabel("time (s)")
    plt.grid()
    plt.legend(["theta", "dtheta"])
    plt.show()

if __name__ == "__main__":
    #open_loop_pendulum()
    #pendulum_PID()
    pendulum_PID_vector_system()