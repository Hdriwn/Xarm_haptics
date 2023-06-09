

//launch commands
//1.
//	$ roslaunch xarm_gazebo xarm7_beside_table.launch
//2.
//	$ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
//rosrun rosserial_python serial_node.py /dev/ttyUSB0
////permission denied error sudo chmod a+rw /dev/ttyACM0
//sudo chmod a+rw /dev/ttyUSB0

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <vector>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

std_msgs::Float64 pos;
std_msgs::Float64 posprev;
void callback(const std_msgs::Float64::ConstPtr& msg, moveit::planning_interface::MoveGroupInterface& move_group_interface_arm)
{
 // ROS_INFO("Received message: %f", msg->data);

/// 90 - > 0.25
/// x -?  x*0.25/90
//  mapping x from (a,b) to (c,d)
// from (-90,90) to (0.125, 0.340)
//x'=((x-a)(d-c)/(b-a))+c 

//mapped_value = (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
pos.data = (msg->data + 90) * (0.340-0.125) / (180 + 0.125) ;
	//pos.data = 0.002777778*(msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  tf::TransformListener listener;

  ros::AsyncSpinner spinner(2);
  spinner.start();

    static const std::string PLANNING_GROUP_ARM = "xarm7";
    
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 1. Move to home position
    	move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("hold-up"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 hold-up (pose goal) %s", success ? "" : "hold-up FAILED");

    move_group_interface_arm.move();
	//ros::WallDuration(2.0).sleep();


    geometry_msgs::Pose target_pose1;
/**
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.0;
    target_pose1.position.z =0.120;//0.138470;
    target_pose1.orientation.x = 0.709;
    target_pose1.orientation.y = 0.002;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.706;
    **/
    target_pose1.position.x = 0.480;
    target_pose1.position.y = 0.125;
    target_pose1.position.z =0.750;//0.138470;
    target_pose1.orientation.x = 0.707;
    target_pose1.orientation.y = 0.000;
    target_pose1.orientation.z = 0.707;
    target_pose1.orientation.w = 0.000;

	move_group_interface_arm.setPoseTarget(target_pose1);	
	move_group_interface_arm.move();

    success = (move_group_interface_arm.plan(my_plan) == 			moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success){
		ROS_INFO("SUCCESS");
	}

    move_group_interface_arm.move();
	//ros::Duration(2.0).sleep(); // Add a 2-second delay before the next pose goal
  // Set the orientation constraint
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.link_name = move_group_interface_arm.getEndEffectorLink();
  orientation_constraint.header.frame_id = "link_base";
  orientation_constraint.orientation = target_pose1.orientation;
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = 0.1;
  orientation_constraint.weight = 1.0;

  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(orientation_constraint);

  move_group_interface_arm.setPathConstraints(constraints);


ros::Subscriber sub = n.subscribe<std_msgs::Float64>("chatter", 10000000, boost::bind(callback, _1, boost::ref(move_group_interface_arm)));
ros::Rate loop_rate(10000000);

//loop
while(ros::ok()){
 tf::StampedTransform transform;
    try
    {
      // Wait for the transform between the base link and link7
      listener.waitForTransform("link_base", "link7", ros::Time(0), ros::Duration(0.0));

      // Get the transform between the base link and link7
      listener.lookupTransform("link_base", "link7", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.0).sleep();
      continue;
    }
ros::Time start_time = ros::Time::now();
    // Extract the position and orientation from the transform
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "link_base";
    pose.header.stamp =start_time;
    pose.pose.position.y = transform.getOrigin().y();
   // ROS_INFO("End Effector Pose: y=%.3f", pose.pose.position.y);

ROS_INFO("Hapkit : %f , Xarm7 : %f",pos.data, pose.pose.position.y);
target_pose1.position.y = pos.data;
	move_group_interface_arm.setPoseTarget(target_pose1);
	//move_group_interface_arm.move();
    moveit::planning_interface::MoveItErrorCode planning_result = move_group_interface_arm.plan(my_plan);
    ros::Duration planning_duration = ros::Time::now() - start_time;

    if (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        
	if(!(posprev.data == target_pose1.position.y)){
	ROS_INFO("Planning succeeded. Time taken: %.3f seconds", planning_duration.toSec());
        move_group_interface_arm.move();
        ROS_INFO("Execution complete.");
	}
    } else {
        ROS_WARN("Planning failed.");
    }

    posprev.data = target_pose1.position.y ;
    ros::spinOnce();
    loop_rate.sleep();
		
}


/**
 move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));   
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 home (pose goal) %s", success ? "" : "home FAILED");

move_group_interface_arm.move();
ros::WallDuration(2.0).sleep();
 // ros::shutdown();
**/
ros::waitForShutdown();
  return 0;
}



