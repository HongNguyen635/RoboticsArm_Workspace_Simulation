# Robotics Arm Workspace Simulation

A simulation that plots all possible reachable points for a robotics arm.

# Demo

There are 2 modes: 

1. Plot all reachable points: plot reachable points on the graph with the model.

2. Real-time simulation: update the graph as the new point is generated.

## Example of a real-time simulation:

https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/1169fd6c-47da-434a-bba2-84e01f8a673e

## Precision

Depending on how "precise" or "close" you want your points to be. But more points don't always mean better visualization. Increasing the precision slows calculation time since possible configurations is calculated using permutation. Additionally, it can cluster your 3D graph.

Example of a **precision = 3** plot:
![3_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/b2d61397-3d70-40d8-87b9-df536fc3545f)

Example of a **precision = 4** plot:
![4_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/a9d59c68-fd1d-49a4-aeb0-8ae650a90d17)

Example of a **precision = 5** plot:
![5_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/a4711369-7264-4757-a32b-927d99b13463)

Example of a **precision = 6** plot:

![6_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/cf5c5ee6-e810-487e-8938-7ecd15017e39)

# How to Use

To use the simulation, a .urdf file of the model is needed. 

This project utilizes the IKPY library, to understand more about IKPY and its terminology, see [ikpy](https://github.com/Phylliade/ikpy).

Additionally, PyBullet was included to help with the simulation. See [PyBullet documentation](https://pybullet.org/wordpress/index.php/forum-2/).

Depending on the input precision, the distance between points would be sparse or dense. For example, for a joint limit between 0 and pi radian, if the precision is 3, 3 points will be generated inclusively (e.g. 0, 0.5 pi, and pi). Of course the more precision, the better. But since joints' position need to be in a specific order, **permutation** is used. Thus, **more precision = slower run time**.

# Credits

The arm.urdf is an example arm that belongs to the [TrickFire Robotics Club](https://www.trickfirerobotics.org/)

This file is only for testing purposes.
