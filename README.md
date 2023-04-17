# Estimate end-effector wrench

This ROS package implements a node that estimates the wrench applied at the end-effector (or any given robot link).

## Install

1. Change directory: `$ cd /path/to/your_ws/src`
2. Clone repository
   - (ssh) `$ git clone git@github.com:cmower/eff_wrench.git`
   - (https) `$ git clone https://github.com/cmower/eff_wrench.git`
3. Install [optas](https://github.com/cmower/optas#install). Note, currently OpTaS is not hosted on PyPi - this will change in the future.
4. Change directory: `$ cd /path/to/your_ws`
5. Build workspace: `$ colcon build`


## Node: `estimate_wrench`

### Parameters

* `urdf_file_name` [`str`]: full path to the URDF file.
* `link_name` [`str`]: name of link to estimate wrench (note, this must appear in the URDF).

### Subscribed topics

* `joint_states` [`sensor_msgs/JointState`]: robot joint states. This assumes the joint positions are given in `position`, and the external torque is given in `effort`. You may need to implement a parser and remap the `joint_states` topic for this node.

### Published topics

* `wrench` [`geometry_msgs/WrenchStamped`]: the estimated wrench.
