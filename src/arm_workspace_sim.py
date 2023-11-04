"""A simulation that plots possible reachable points for a robotics arm.

To use the simulation, a .urdf file of the model is needed. 
This module utilizes the ikpy library, read more about ikpy to 
understand the terminology uses in this module:
https://github.com/Phylliade/ikpy
"""

from itertools import product
import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
import pybullet
import pybullet_data
import numpy as np


class Graph:
    """Display a 3D graph of all reachable points with input figure

    Methods
    -------
    display_graph()
        displays the 3D graph of reachale points and figure.

    update_graph(target_position)
        update the graph if the figure and points has been changed.
    """

    def __init__(self, urdf_file_name, active_links_masks, model_start_position):
        """
        Parameters
        ----------
            urdf_file_name : str
                the name of the urdf file
            active_links_masks : list
                A list of boolean indicating that whether or not the 
                corresponding link is active
            model_start_position : list
                A list that contains the start position of each joint
        """

        self.start_position = model_start_position

        # figure = arm
        self.figure = ikpy.chain.Chain.from_urdf_file(urdf_file_name,
                                                      active_links_mask=active_links_masks)

        self.current_position = self.start_position
        self.target_position = self.figure.forward_kinematics(self.current_position)[
            :3, 3]

        # self.canvas, self.axes = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

        self.canvas, self.axes = plot_utils.init_3d_figure()

        # lists of coordinates for scatter points
        self.x_positions = []
        self.y_positions = []
        self.z_positions = []

        # append the first coordinate
        self.x_positions.append(self.target_position[0])
        self.y_positions.append(self.target_position[1])
        self.z_positions.append(self.target_position[2])

    def display_graph(self, real_time):
        """Displays the 3D graph of reachale points and figure

        Parameters
        ----------
        real_time : bool
            enable interactive mode. See if the graph will update.
        """

        matplotlib.pyplot.xlim(-1, 1)
        matplotlib.pyplot.ylim(-1, 1)
        self.axes.set_zlim(0, 1)

        # example: axes.scatter([0, 0.4, 0.8], [0, 0.4, 0.8], [0, 0.4, 0.8], s=10, c="green")

        # find the rotation -> convert to position -> feed to forward -> plot

        self.figure.plot(self.start_position, self.axes,
                         target=self.target_position)
        self.axes.scatter(self.x_positions, self.y_positions, self.z_positions,
                          s=10, c="green", alpha=0.1)

        # on if the graph will be update in real-time
        if real_time:
            matplotlib.pyplot.ion()

        matplotlib.pyplot.show()

    def update_graph(self, target_position):
        """Update the graph if the figure and points has been changed

        Parameters
        ----------
        target_position : list
            a list of the positions of the end effector 
        """

        self.axes.clear()

        self.figure.plot(self.start_position, self.axes,
                         target=target_position)

        matplotlib.pyplot.xlim(-1, 1)
        matplotlib.pyplot.ylim(-1, 1)
        self.axes.set_zlim(0, 1)

        self.axes.scatter(self.x_positions, self.y_positions, self.z_positions,
                          s=10, c="green", alpha=0.1)

        self.canvas.canvas.draw()

        self.canvas.canvas.flush_events()


# calculate the every possible end-effector position
class EndEffectorSim:
    """Run the simulation for the arm's reachable points

    Methods
    -------
    get_all_positional_configurations(precision)
        get all possible joint configurations

    run_simulation (all_positional_configurations)
        start the simulation with all the joint positions
    """

    def __init__(self, urdf_file_name, num_joints, model_start_position, active_links_masks):
        """
        Parameters
        ----------
            urdf_file_name : str
                name of the .urdf file
            num_joints : int
                the number of joints of the model
            model_start_position : list
                a list of the start position of each joint
            active_links_masks : list
                A list of boolean indicating that whether or not the 
                corresponding link is active
        """

        self.urdf_file_name = urdf_file_name
        self.num_joints = num_joints
        self.figure = ikpy.chain.Chain.from_urdf_file(urdf_file_name,
                                                      active_links_mask=active_links_masks)

        self.graph = Graph(urdf_file_name, active_links_masks=active_links_masks,
                           model_start_position=model_start_position)

    def get_all_positional_configurations(self, precision):
        """Get all possible joint configurations

        Parameters
        ----------
            precision : int
                given the joint limit from the urdf file. Define how 
                many in-between points you want to generate.
                Example: joint limit from 0 to pi radian. An input of 
                10 indicates you want to generate 10 points between 0
                and pi inclusively.

        Returns
        -------
        all_positional_configurations : list
            a list of all possible configurations of the arm
        """

        # connect to pybullet sim
        pybullet.connect(pybullet.GUI)

        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)
        # planeId = pybullet.loadURDF("plane.urdf")

        start_position = [0, 0, 0]
        start_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        # model = arm
        model = pybullet.loadURDF(
            self.urdf_file_name, start_position, start_orientation, useFixedBase=1)

        joint_positions = []
        for i in range(pybullet.getNumJoints(model)):
            # get the lower + upper joint limit -> generate 5 number in between
            position_each_joint = np.linspace(pybullet.getJointInfo(model, i)[8],
                                              pybullet.getJointInfo(model, i)[9], precision)

            joint_positions.insert(0, position_each_joint.tolist())

        # get all possible combination of joint positions
        all_positional_configurations = list(
            map(list, (product(*joint_positions))))

        # if line 189 use append the joint's position, then don't need the following loop
        for i in range(len(all_positional_configurations)):
            all_positional_configurations[i].reverse()

        # print(*all_positional_configurations, sep="\n")

        pybullet.disconnect()

        return all_positional_configurations

    # update the plot as well as the model
    def run_simulation(self, all_positional_configurations):
        """Start the simulation with all the joint positions
        The plot will be updated in real-time.

        Parameters
        ----------
            all_positional_configurations : list
            list of all possible joint configurations. Each item
            in a list is a list of joints' positions
        """

        # empty all coordinates
        self.graph.x_positions.clear()
        self.graph.y_positions.clear()
        self.graph.z_positions.clear()

        # display initial before updating real-time
        self.graph.display_graph(True)

        # update graph in real-time
        for position in all_positional_configurations:
            end_effector_position = self.figure.forward_kinematics(
                position + [0,])[:3, 3]

            self.graph.start_position = position + [0,]

            self.graph.x_positions.append(end_effector_position[0])
            self.graph.y_positions.append(end_effector_position[1])
            self.graph.z_positions.append(end_effector_position[2])

            self.graph.update_graph(end_effector_position)

    # plot all points with the model, no real-time running
    def show_workspace(self, all_positional_configurations):
        """Plot all reachable points in 3D space. No real-time running

        Args:
            all_positional_configurations : list
            list of all possible joint configurations. Each item
            in a list is a list of joints' positions
        """
        # empty all coordinates
        self.graph.x_positions.clear()
        self.graph.y_positions.clear()
        self.graph.z_positions.clear()

        # start to append (x, y, z) points
        for position in all_positional_configurations:
            end_effector_position = self.figure.forward_kinematics(
                position + [0,])[:3, 3]

            self.graph.x_positions.append(end_effector_position[0])
            self.graph.y_positions.append(end_effector_position[1])
            self.graph.z_positions.append(end_effector_position[2])

        self.graph.display_graph(False)


# test simulation
def main():
    """Test code to run the simulation
    """
    arm_simulation = EndEffectorSim("arm.urdf", 6, [0, 0, 1.5, -3.14, 0, 0, 0],
                                    active_links_masks=[False, True, True, True, True, True, True])

    positions = arm_simulation.get_all_positional_configurations(4)
    arm_simulation.run_simulation(positions)


if __name__ == "__main__":
    main()
