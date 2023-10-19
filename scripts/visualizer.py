from pydrake.geometry import Meshcat
from pydrake.visualization import ModelVisualizer

meshcat = Meshcat()
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModels("./resources/bicopter.urdf")
visualizer.Run(loop_once=False)