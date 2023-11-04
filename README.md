# Robotics Arm Workspace Simulation

A simulation that plots all possible reachable points for a robotics arm.

# Demo

There are 2 modes: 

1. Plot all reachable points: plot reachable points on the graph with the model.

2. Real-time simulation: update the graph as the new point is generated.

## Example of a real-time simulation:

https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/1169fd6c-47da-434a-bba2-84e01f8a673e

## Precision

Depending on how "precise" or "close" you want your points to be. But more points don't always mean better visualization. Increasing the precision slows calculation time since possible configurations are calculated using permutation. Additionally, it can cluster your 3D graph.

Example of a **precision = 3** plot:
![3_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/00cd1177-4664-408e-abcf-c58e7bab3ce8 | width=100)

Example of a **precision = 4** plot:
![4_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/0261bbe4-3e67-4c37-8b65-01bdb02e9548 | width=100)

Example of a **precision = 5** plot:
![5_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/df7cd419-6fee-4707-a918-67b3d9f7088a | width=100)

Example of a **precision = 6** plot:
![6_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/83cebd5e-1a74-4b7d-aa04-967221c74f1c | width=100)

# How to Use

To use the simulation, a .urdf file of the model is needed. 

This project utilizes the IKPY library, to understand more about IKPY and its terminology, see [ikpy](https://github.com/Phylliade/ikpy).

Additionally, PyBullet was included to help with the simulation. See [PyBullet documentation](https://pybullet.org/wordpress/index.php/forum-2/).

Depending on the input precision, the distance between points would be sparse or dense. For example, for a joint limit between 0 and pi radian, if the precision is 3, 3 points will be generated inclusively (e.g. 0, 0.5 pi, and pi). Of course the more precision, the better. But since joints' position need to be in a specific order, **permutation** is used. Thus, **more precision = slower run time**.

# Credits

The arm.urdf is an example arm that belongs to the [TrickFire Robotics Club](https://www.trickfirerobotics.org/)

This file is only for testing purposes.
