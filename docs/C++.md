# Download Drake
(https://drake.mit.edu/from_source.html#getting-drake)
`git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git`

# Ubuntu one-time platform-specific setup
`sudo ./setup/ubuntu/install_prereqs.sh`

To use the Python bindings from Drake externally, we recommend using CMake. As an example:

```
git clone https://github.com/RobotLocomotion/drake.git
mkdir drake-build
cd drake-build
cmake ../drake
make -j
```

Please note the additional CMake options which affect the Python bindings:

`-DWITH_GUROBI={ON, [OFF]}` - Build with Gurobi enabled.
`-DWITH_MOSEK={ON, [OFF]}` - Build with MOSEK™ enabled.
-`DWITH_SNOPT={ON, [OFF]}` - Build with SNOPT enabled.

e.g.
`cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON ../drake`

make sure path config prop
Ubuntu 22.04 (Jammy):

`cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.10/site-packages:${PYTHONPATH}`

Specific information found in the cpp directory