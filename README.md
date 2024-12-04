# kinovo_project
This repository is made to control the kinovo kortex in gazebo when end effector twist is given as the input commands 
- it uses inverse jacobian (mapping from end effector to the joint space velocity commands) which is built from scratch
- the next part which is currently ongoing is the image based visual servoing 
- its open to suggestions 

## How to run
- the repository for ros_kortex needs to be cloned from their github page website this repository is built over that to facilitate control in gazebo 
- clone the package into the src folder of the workspace 
- use catkin_make command in the root directory of your worskpace
- <code>rosun kinovo_project keyboard_node.py</code>
- <code>rosrun kinovo_project jacobian_calculation.py</code>
### controls 

note every command is with respect to end effector frame
- <b>w,s</b> for y direction linear velocity
- <b>a,d</b> for x direction linear velocity
- <b>r,f</b> for z direction linear velocity
- <b>i,k</b> for y direction angular velocity
- <b>j,l</b> for x direction angular velocity 
- <b>u,h</b> for z direction angular velocity



