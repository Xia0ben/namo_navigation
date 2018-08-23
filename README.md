# Namo navigation ROS-based simulator

This repository contains our implementation of the algorithms presented in [our research report](https://github.com/Xia0ben/master-report). These algorithms are derivated from the following works:

- M. Levihn, M. Stilman, and H. Christensen. “Locally optimal navigation among movable obstacles in unknown environments”. In: 2014 IEEE-RAS International Conference on Humanoid Robots. Nov. 2014, pp. 86–91. DOI: 10.1109/HUMANOIDS.2014.7041342.

- Hai-Ning Wu, M. Levihn, and M. Stilman. “Navigation Among Movable Obstacles in unknown environments”. In: 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems. Oct.2010, pp. 1433–1438. DOI:10.1109/IROS.2010.5649744.

## Prerequisites

You will need a [ROS](http://www.ros.org/) https://github.com/occiware/OCCInterface.gitinstallation to run this simulation. For this you can either:

- Use a local install by following installation instructions from the original [ROS documentation](http://wiki.ros.org/kinetic/Installation/Ubuntu).

- Use a Docker container so that you can easily dispose of ROS later. For this, follow instructions in [our repository](https://github.com/Xia0ben/rosdocked-kinetic-pepper) to launch a container with all required ROS dependencies (stop before the "Getting started with Pepper" section).

## Quick start

Open 4 separate bash sessions. If you use the Docker approach, do this by executing (of course after starting the container):

``` bash
sudo docker exec -it <CONTAINER_NAME> bash
```

- Clone the present repository in the ~/catkin_ws/src directory

``` bash
git clone https://github.com/Xia0ben/namo_navigation.git
```

- Install python dependencies by executing:

``` bash
pip install --user -r ~/catkin_ws/src/namo_navigation/requirements.txt
```

- Build the ROS package:

``` bash
cd ~/catkin_ws
catkin build
```

- Execute the following command in the 4 separate bash sessions to get the ROS autocompletion and library linking to work:
``` bash
source devel/setup.bash
```

- Launch a roscore server in one bash session:
``` bash
roscore
```

- Launch the map server:
``` bash
roslaunch namo_navigation map_server.launch
```

- Launch the rviz visualization in another bash session:
``` bash
rviz rviz -d $HOME/catkin_ws/src/namo_navigation/basic_simulator_rviz_config.rviz
```

- Finally, start the simulator code either by executing the "basic_simulator.py" file in your favorite IDE, or by executing the command:
``` bash
python ~/catkin_ws/src/namo_navigation/src/basic_simulator.py
```

You can change the positions of obstacle by editing the main method of the same file.
