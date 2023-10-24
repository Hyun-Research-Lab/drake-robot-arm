from pydrake.geometry import Meshcat
from pydrake.visualization import ModelVisualizer

meshcat = Meshcat()
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels("./resources/quadrotor_Lee2010.urdf")
visualizer.Run(loop_once=False)