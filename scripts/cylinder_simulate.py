import numpy as np
import os

from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer

meshcat = StartMeshcat()
meshcat.AddButton('say_hello')

visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels("./sdf/cylinder.sdf")

# Click the "Stop Running" button in MeshCat when you're finished.
visualizer.Run(loop_once=False)

# https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/authoring_multibody_simulation-1b91f35a734a426fb1d775a3a565d9af