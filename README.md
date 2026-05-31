# Gazebo SpiRobs

Gazebo simulation of logarithmic spiral robot models based on models
generated using the
[Open-Spiral-Robots](https://github.com/ZhanchiWang/Open-Spiral-Robots) toolkit

## Usage

The build instructions below assume the project is cloned into a Pixi workspace
that includes Gazebo Jetty and ros-jazzy-ament-cmake as dependencies. The
colcon build command installs to the current environment and ament hooks ensure
the Gazebo environment variables are set correctly. A non-Pixi Gazebo or
ROS colon workspace will also work.

```bash
colcon build --merge-install --symlink-install --install-base $CONDA_PREFIX --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE:PATH=$CONDA_PREFIX/bin/python --packages-select gz_spirob_description gz_spirob_gazebo
```

Run the simulation

```bash
gz sim -v4 -r spirob.sdf
```

Run the grasping script

```bash
cd gz-spirob/gz_spirob_gazebo/scripts
python grasp_2d.py
```

## References

Models are designed and exported to MuJoCo XML using the [Open-Spiral-Robots](https://github.com/ZhanchiWang/Open-Spiral-Robots) toolkit. This forms a
template for the Gazebo model and Tendon plugin.

```
@software{wang_openspirob,
  title        = {OpenSpiRobs: Open Spiral Robots Toolkit},
  author       = {Wang, Zhanchi},
  year         = {2026},
  url          = {https://github.com/ZhanchiWang/Open-Spiral-Robots}
}
```
