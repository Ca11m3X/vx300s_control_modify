#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import os
import numpy as np
from moveit_msgs.msg import RobotTrajectory

count = 0


def plotting_funct(trajectory: RobotTrajectory):
    joint_names = trajectory.joint_trajectory.joint_names
    points = trajectory.joint_trajectory.points
    global count
    count += 1

    for i in range(len(joint_names)):
        joint_name = joint_names[i]
        positions = []
        time = []

        for j in range(len(points)):
            positions.append(points[j].positions[i])
            time.append(points[j].time_from_start.to_sec())

        y_values = np.array(positions)
        x_values = np.array(time)

        plt.title(joint_name + " by Time")
        plt.ylabel("Position (rad)")
        plt.xlabel("Time (sec)")
        plt.grid(True)
        plt.tight_layout()
        plt.plot(x_values, y_values, marker=".")
        figure_saved_path = os.path.join(os.path.dirname(__file__), 'figure/')
        fig_name = f'path_{count}_{joint_name}.png'
        plt.savefig(figure_saved_path + fig_name, bbox_inches='tight')
        plt.close()


if __name__ == "__main__":
    rospy.init_node("Plot Node")

    sub = rospy.Subscriber("/plot_data", RobotTrajectory, plotting_funct)

    rospy.spin()
