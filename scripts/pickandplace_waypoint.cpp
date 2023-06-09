

//launch commands
//1.
//	$ roslaunch xarm_gazebo xarm7_beside_table.launch
//2.
//	$ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "xarm7";
    
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 home (pose goal) %s", success ? "" : "home FAILED");

    move_group_interface_arm.move();
ros::WallDuration(2.0).sleep();

    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
 /**
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("link7");

    geometry_msgs::Pose target_pose1;
  
    target_pose1.orientation = current_pose.pose.orientation;
//target_pose1.position.x = -0.16;
//    target_pose1.position.y = 0.7700113;
//    target_pose1.position.z = 0.25;//0.138470;
    double roll = 0.0;
double pitch = 1.5;
double yaw = 0.0;
target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

move_group_interface_arm.setPoseTarget(target_pose1);

//    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 placement(pose goal) %s", success ? "" : "ee placement FAILED");

    move_group_interface_arm.move();
    move_group_interface_arm.move();
ros::Duration(2.0).sleep(); // Add a 2-second delay before the next pose goal

**/

std::vector<geometry_msgs::Pose> waypoints;

    // Define the waypoints
    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 0.3;
    waypoint1.position.y = 0.092;
    waypoint1.position.z = 0.120;
    waypoint1.orientation.x =0.709;
    waypoint1.orientation.y = 0.002;
    waypoint1.orientation.z = 0.00;
    waypoint1.orientation.w = 0.706;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 0.3;
    waypoint2.position.y =-0.3;
    waypoint2.position.z = 0.120;
    waypoint2.orientation.x =0.709;
    waypoint2.orientation.y = 0.002;
    waypoint2.orientation.z = 0.0;
    waypoint2.orientation.w = 0.706;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 0.3;
    waypoint3.position.y =-0.100;
    waypoint3.position.z = 0.120;
    waypoint3.orientation.x =0.709;
    waypoint3.orientation.y = 0.002;
    waypoint3.orientation.z = 0.000;
    waypoint3.orientation.w = 0.7060;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 0.3;
    waypoint4.position.y = 0.3;
    waypoint4.position.z = 0.120;
    waypoint4.orientation.x =0.709;
    waypoint4.orientation.y = 0.002;
    waypoint4.orientation.z = 0.000;
    waypoint4.orientation.w = 0.706;
    waypoints.push_back(waypoint4);



    // Set the start state as the current state
    move_group_interface_arm.setStartStateToCurrentState();

    // Plan and execute the trajectory
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_interface_arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    ROS_INFO_STREAM("Waypoint trajectory computed with fraction: " << fraction);

    if (fraction > 0.9)
    {
        my_plan.trajectory_ = trajectory;
        move_group_interface_arm.execute(my_plan);
        ROS_INFO("Trajectory execution complete.");
    }
    else
    {
        ROS_WARN("Unable to compute the complete trajectory. Please check the waypoints.");
    }
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 home (pose goal) %s", success ? "" : "home FAILED");

    move_group_interface_arm.move();
ros::WallDuration(2.0).sleep();
  ros::shutdown();
  return 0;
}
