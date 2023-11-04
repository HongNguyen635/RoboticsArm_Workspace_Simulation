# Robotics Arm Workspace Simulation

A simulation that plots all possible reachable points for a robotics arm.

# Demo

There are 2 modes: 

1. Plot all reachable points: plot reachable points on the graph with the model.

2. Real-time simulation: update the graph as the new point is generated.

## Example of a real-time simulation:

https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/1169fd6c-47da-434a-bba2-84e01f8a673e

## Precision

Depending on how "precise" or "close" you want your points to be. But more points don't always mean better visualization. Increasing the precision slows calculation time since possible configurations are calculated using permutation. Additionally, too high precision can cluster your 3D graph. 

For each graph below, notice that less precision = less dense. But the reachable points are sparse and it is difficult to visualize the reachable positions. Since the alpha of each scatter point is set to 0.1, the transparency of a point indicates the frequency of that point being reached. A solid point means that it has been reached multiple times. Likewise, an almost-transparent point shows that the arm only reaches it once or twice.

Example of a **precision = 3** plot:
![3_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/a440884b-f8c4-4189-a7f3-b09f8807ac0d)

Example of a **precision = 4** plot:
![4_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/b591503a-1aa8-4655-a917-7d7d2820ab17)

Example of a **precision = 5** plot:
![5_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/dfe0b81f-3e89-4ea2-b1eb-1431f989d76c)

Example of a **precision = 6** plot:
![6_precision_plot](https://github.com/HongNguyen635/RoboticsArm_Workspace_Simulation/assets/73915779/f519e7e6-eecf-4758-bd0b-5c2ba7c83aec)

# How to Use

To use the simulation, a .urdf file of the model is needed. 

## Brief description

**Class Graph**: Display a 3D graph of all reachable points with the input figure and scatter points.

**Class EndEffectorSim**: run the simulation. Each joint has a certain joint limit defined in the .urdf file. Based on that, **permutation** is used to generate a list of all possible joint positions (given a certain precision). Then, **forward kinematics** is used to calculate the end effector position.

**Note:** Depending on the input precision, the distance between points would be sparse or dense. For example, for a joint limit between 0 and pi radian, if the precision is 3, 3 points will be generated inclusively (e.g. 0, 0.5 pi, and pi). Of course the more precision, the better. But since joints' position need to be in a specific order, **permutation** is used. Thus, **more precision = slower run time**.

## Dependencies

This project utilizes the IKPY library, to understand more about IKPY and its terminology, see [ikpy](https://github.com/Phylliade/ikpy).

Additionally, PyBullet was included to help with the simulation. See [PyBullet documentation](https://pybullet.org/wordpress/index.php/forum-2/).

# Credits

The arm.urdf is an example arm that belongs to the [TrickFire Robotics Club](https://www.trickfirerobotics.org/)

This arm.urdf file is only for testing purposes.
