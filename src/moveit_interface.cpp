#include "vx300s_control_modify/moveit_interface_obj.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_interface");
    // We need two spinners to run the InterbotixMoveItInterface node
    //    - One spinner allows ROS messages to be processed during the blocking 'move_group.move()' command
    //    - Another spinner allows ROS messages to be processed during a blocking service call (since planning
    //      can take some time and the service does not return until the planning is done)
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle n;
    // Create instance of MoveIt interface
    InterbotixMoveItInterface interface(&n);

    interface.moveit_scale_ee_velocity(0.5);

    // Go to home pose
    std::vector<double> joint_values = {0, 0, 0, 0, 0};
    interface.moveit_plan_joint_positions(joint_values);
    interface.moveit_execute_plan();
    interface.add_a_Box();

    std::cin.get();
    // Go to goal pose
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = 0.2;
    goal_pose.position.z = 0.1;
    tf::Quaternion quat;
    quat = tf::createQuaternionFromRPY(0.0, 1.57, 0.0);
    goal_pose.orientation.x = quat.getX();
    goal_pose.orientation.y = quat.getY();
    goal_pose.orientation.z = quat.getZ();
    goal_pose.orientation.w = quat.getW();

    interface.moveit_plan_ee_pose(goal_pose);
    interface.moveit_execute_plan();

    std::cin.get();
    // Cartesian Path
    geometry_msgs::Pose start_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;

    start_pose = interface.moveit_get_ee_pose();
    waypoints.push_back(start_pose);
    target_pose = start_pose;
    target_pose.position.z -= 0.08;
    waypoints.push_back(target_pose);
    interface.moveit_plan_cartesian_path(waypoints);
    interface.moveit_execute_plan();
    interface.attach_object();

    waypoints.clear();
    start_pose = interface.moveit_get_ee_pose();
    waypoints.push_back(start_pose);
    target_pose = start_pose;
    target_pose.position.z += 0.08;
    waypoints.push_back(target_pose);
    interface.moveit_plan_cartesian_path(waypoints);
    interface.moveit_execute_plan();

    waypoints.clear();
    start_pose = interface.moveit_get_ee_pose();
    waypoints.push_back(start_pose);
    target_pose = start_pose;
    target_pose.position.x -= 0.07;
    target_pose.position.y += 0.07;
    waypoints.push_back(target_pose);
    interface.moveit_plan_cartesian_path(waypoints);
    interface.moveit_execute_plan();

    waypoints.clear();
    start_pose = interface.moveit_get_ee_pose();
    waypoints.push_back(start_pose);
    target_pose = start_pose;
    target_pose.position.z -= 0.08;
    waypoints.push_back(target_pose);
    interface.moveit_plan_cartesian_path(waypoints);
    interface.moveit_execute_plan();
    interface.detach_object();

    waypoints.clear();
    start_pose = interface.moveit_get_ee_pose();
    waypoints.push_back(start_pose);
    target_pose = start_pose;
    target_pose.position.z += 0.08;
    waypoints.push_back(target_pose);
    interface.moveit_plan_cartesian_path(waypoints);
    interface.moveit_execute_plan();

    // Go to home pose
    joint_values = {0, 0, 0, 0, 0};
    interface.moveit_plan_joint_positions(joint_values);
    interface.moveit_execute_plan();

    // Go to sleep pose
    joint_values = {0, -1.85, 1.55, 0.8, 0};
    interface.moveit_plan_joint_positions(joint_values);
    interface.moveit_execute_plan();

    ros::waitForShutdown();
    return 0;
}
