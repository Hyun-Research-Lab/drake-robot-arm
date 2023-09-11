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