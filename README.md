# R.A.I.N (WIP)
Robot for Autonomous Indoor Navigation

The project is mainly divided into the following verticals of work:
1. `Planning and Control`: Deals with the path planning and control of the Robot.
2. `Perception`: Deals with Perception using LiDAR and Depth Camera.
3. `Localisation and State Estimation`: Deals with State Estimation, Localisation and Mapping.

The main aim of the Project is to develop software that can be deployed on an Indoor Autonomous Robot. The crux of the project is to write deployable, reliable and repeatable software. The project has certain milestones to achieve in terms of Product development. A new release shall be made when each of the product stage is achieved.

## Product Stages
1. ### Mark I
- [x] AStar Algorithm for Global Planning with fixed waypoints in an indoor environment.
- [ ] PID Controller for waypoint to waypoint navigation.
- [ ] `move_base` based Global and Local Costmaps.
- [ ] `gmapping` based mapping framework and `amcl` for localisation.
- [x] `move_base_flex` inspired rospy backend for integration.

## Docs
The development process of the project shall be documented in the `docs` folder of the project as Markdown files. A supporting documentation for each of the Product stage will be made.

## Project Structure
The package is divided by the following structure
1. `rain_interface`: Package that contains the interfacing ROS Backend code, all messages, actions, services definitions and implementations
2. `rain_planning`: Package that contains all the code for Path Planning and Control Algorithms.
3. `rain_localisation`: Package that contains the files for Localisation and Mapping Algorithms.
4. `rain_perception`: Package that contains the files for Perception Algorithms.

Refer to the Individual Packages for more Details.
## Installation
1. Fork and clone the package into your workspace
```bash
cd catkin_ws/src
git clone git@github.com:your-github-username/RAIN.git
```
2. Build the package
```bash
catkin build
```
if you do not have `python-catkin-tools`, install them from [here](https://catkin-tools.readthedocs.io/en/latest/installing.html) 
## Contribution Guidelines
Refer to [CONTRIBUTION.md](CONTRIBUTION.md) for information on Contribution

## Contributors
Will be updated soon
