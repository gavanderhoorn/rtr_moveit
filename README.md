# rtr_moveit

Description: A MoveIt! planner interface for RapidPlan by [Realtime Robotics](http://rtr.ai/).

<img src="https://picknik.ai/images/logo.jpg" width="100">

Developed by Henning Kayser at [PickNik Consulting](http://picknik.ai/)

Travis:
[![Build Status](https://travis-ci.com/PickNikRobotics/rtr_moveit.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/rtr_moveit)

### Plugin Structure

The plugin consists of the three classes RTRPlannerManager, RTRPlanningContext and RTRPlannerInterface.
While the first two implement the plugin structure of Moveit!, the RTRPlannerInterface provides and interface for sending motion planning requests to RapidPlan.

RapidPlan's HardwareInterface and PathPlanner need to be initialized with the same roadmaps at each query.
At the moment, it is not possible to retrieve the currently loaded roadmap of the PathPlanner and the stored roadmaps on the Hardware.
The RTRPlannerInterface keeps track of loaded and stored roadmaps and handles write access.
To prevent race conditions, calls of the PathPlanner and HardwareInterface are synchronized with a mutex lock.
The interface hides all of this and only provides functions for initialization, availability and planning attempts.

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-kinetic-rtr_moveit

### Build from Source

These instructions assume you are running on Ubuntu 16.04:

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools

1. Install RapidPlan dependencies:

> Note: RapidPlan is not released yet. All dependencies are installed from [this](https://drive.google.com/uc?id=1cXFOaMUuUyq5Fic1vdgIpBeOah8T6olh) encrypted binary shell script.

        sudo apt-get install mcrypt python-pip
        pip install --user gdown
        gdown https://drive.google.com/uc?id=1cXFOaMUuUyq5Fic1vdgIpBeOah8T6olh
        cat rtr_0.1.2.deb.run.crypt | crypt "super secret picknik pass" -d > rtr_0.1.2.deb.run
        sudo bash rtr_0.1.2.deb.run
        rm rtr_0.1.2.deb.run rtr_0.1.2.deb.run.crypt

1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin/
        mkdir -p $CATKIN_WS
        cd $CATKIN_WS

1. Download the required repositories and install any dependencies:

        git clone git@github.com:PickNikRobotics/rtr_moveit.git
        wstool init src
        wstool merge -t src rtr_moveit/rtr_moveit.rosinstall
        wstool update -t src
        rosdep install --from-paths src --ignore-src --rosdistro kinetic

1. Configure and build the workspace:

        catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Run

See the [rtr_moveit tutorial](https://github.com/PickNikRobotics/rtr_moveit/blob/pr-tutorial/rtr_moveit_tutorial/rtr_moveit_tutorial.rst) on how to configure and run this plugin. 

## Developers: Quick update code repositories

To make sure you have the latest repos:

    cd $CATKIN_WS/src/rtr_moveit
    git checkout master
    git pull origin master
    cd ..
    wstool merge rtr_moveit/rtr_moveit.rosinstall
    wstool update
    rosdep install --from-paths . --ignore-src --rosdistro kinetic

## Code API

> Note: this package has not been released yet

See [the Doxygen documentation](http://docs.ros.org/kinetic/api/rtr_moveit/html/anotated.html)

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro kinetic

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests --no-deps --this -i
