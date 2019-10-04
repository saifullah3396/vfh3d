# VFH3D+

An octomap based VFH+ algorithm in 3D for local planning of UAVs (or any robot in 3D).

## Installation
- The build is only yet tested on ROS melodic but it should work on other distributions as well. Make a local cakin workspace:
```
mkdir -p ~/catkin_ws/src
```
- Build and source the workspace:
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
- Clone the repository to your workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/saifullah3396/vfh3d.git
```
- Install dependencies for the package with your ROS distribution:
```
rosdep install --from-paths src --ignore-src --rosdistro <ros-distro> -y.
```
- Build the package:
```
cd ~/catkin_ws
catkin_make
```
- Run test launch file for demonstration:
```
roslaunch vfh3d vfh3d_planner_test.launch
```