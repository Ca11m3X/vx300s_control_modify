#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np
from moveit_msgs.msg import RobotTrajectory

count = 0


def plotting_funct(trajectory: RobotTrajectory):
    joint_names = trajectory.joint_trajectory.joint_names
    points = trajectory.joint_trajectory.points
    global count
    count += 1

    for i in range(joint_names):
        joint_name = joint_names[i]
        positions = []
        time = []

        for j in range(points):
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
        fig_name = f'figure/path_{count}_{joint_name}.png'
        plt.savefig(fig_name, bbox_inches='tight')


if __name__ == "__main__":
    rospy.init_node("Plot Node")

    sub = rospy.Subscriber("/plot_data", RobotTrajectory, plotting_funct)

    rospy.spin()
