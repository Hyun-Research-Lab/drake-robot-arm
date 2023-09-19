# drake-robot-arm

Installs drake python and uses meshcat visualizer
https://github.com/meshcat-dev/meshcat-python
https://drake.mit.edu/pip.html

Create venv and install requirements:
```
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```


# things to note
https://drake.guzhaoyuan.com/introduction/drake-multibody 
One thing worth mentioning is that drake requires the model to have at least one transmission in the URDF, or it will fail to create input torque port for the plant. So if you do not have any transmission tag in the URDF, do create SimpleTransmission for each actuated joint.
One more thing, MultibodyPlant has a unique body called world_body. Every body created in Drake by default is a floating body, unless it is connected with some other bodies. Floating body is connected to the world_body with a floating joint. So if your robot should be connected to the ground, you would create new joints to connect the body and world. The floating joint will be overwritten.

 Code and functions taken from :
 https://github.com/RussTedrake/underactuated/blob/master/underactuated/meshcat_utils.py


 # new to drake?
 Drake is a comprehensive toolbox for analyzing the dynamics of robots and other systems. To properly understand the relationship between its various components, let's dive into the key concepts:

    MultibodyPlant:
        Role: Represents the physical system, including all its bodies, joints, actuators, collision geometries, and other elements. It provides a mathematical model of the system's dynamics.
        Properties/Uses:
            Describes the kinematic and dynamic properties of the system.
            Can add bodies, joints, actuators, etc., to this plant.
            Can be queried for state derivatives given a state and input.
            Generates output ports for system state and other quantities of interest.

    Simulator:
        Role: Uses the dynamics provided by a system (like MultibodyPlant) to advance its state forward in time.
        Properties/Uses:
            Integrates a system's dynamics over time.
            Can adjust integration methods and parameters.
            Interacts with the system by fetching its state, computing derivatives, and updating the state.

    DiagramBuilder:
        Role: A factory for creating a block-diagram of systems. In Drake, complex models often involve multiple interconnected systems.
        Properties/Uses:
            Lets you add systems to a diagram.
            Can specify interconnections between systems.
            Once the diagram structure is defined, it can "Build" the diagram to generate an interconnected system.

    Diagram:
        Role: Represents an interconnected system, possibly with many subsystems.
        Properties/Uses:
            Can be simulated just like any individual system.
            Allows hierarchical structuring: diagrams within diagrams.
            Aggregates input, output, and state across all its subsystems.

    SceneGraph:
        Role: Manages all the geometrical data in a simulation, such as the visual and collision geometries of bodies.
        Properties/Uses:
            Provides an interface for systems (like MultibodyPlant) to register geometries.
            Can be queried for collisions, raycasting, etc.
            Allows visualization of the system.

How they intertwine:

    A typical workflow might involve using a DiagramBuilder to create a new diagram.
    You add a MultibodyPlant to the builder to model the physical aspects of your robot or system. Simultaneously, you might add a SceneGraph to handle visualization and collision queries.
    The plant and SceneGraph are interconnected so that geometries from the plant are registered and can be visualized or checked for collisions.
    You might add other systems, like controllers or sensors, to the builder and wire them up to the plant and other systems as necessary.
    Once everything is set up, you "build" the diagram.
    You can then use a Simulator to simulate this entire interconnected system. The simulator queries the Diagram (which in turn queries all its subsystems, like the plant) to get dynamics, and it integrates these dynamics forward in time.

Regarding simulation properties:

    MultibodyPlant:
        Adjust physical properties (mass, inertia, etc.)
        Add/remove bodies, joints, etc.
        Define actuators and their properties.
    Simulator:
        Change the integration method.
        Adjust integration accuracy and timestep.
        Introduce any artificial system perturbations.
    DiagramBuilder and Diagram:
        Adjust the structure and interconnections of your overall system.
    SceneGraph:
        Adjust visualization details.
        Define collision properties of geometries.

Each component has a specific role in the modeling, simulation, and analysis of systems in Drake, and they all work together to provide a comprehensive environment for system simulation and analysis.
