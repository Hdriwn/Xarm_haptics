

//launch commands
//1.
//	$ roslaunch xarm_gazebo xarm7_beside_table.launch
//2.
//	$ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
//rosrun rosserial_python serial_node.py /dev/ttyUSB0
////permission denied error sudo chmod a+rw /dev/ttyACM0
//sudo chmod a+rw /dev/ttyUSB0
//

// roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.242 report_type:=normal
// roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=192.168.1.242 velocity_control:=false report_type:=normal



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

#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/SetAxis.h>
#include <xarm_msgs/SetString.h>
#include <xarm_msgs/Move.h>
bool x = false;
std_msgs::Float64 pos;
std_msgs::Float64 posprev;
void callback(const std_msgs::Float64::ConstPtr& msg, ros::ServiceClient& client_line)
{
 // ROS_INFO("Received message: %f", msg->data);

/// 90 - > 0.25
/// x -?  x*0.25/90
//  mapping x from (a,b) to (c,d)
// from (-90,90) to (0.125, 0.340)
//x'=((x-a)(d-c)/(b-a))+c 

//mapped_value = (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
//posprev.data = ((msg->data + 90) * (200+200.0) )/ (180 -200) ;
//posprev.data = (msg->data / 90.0) * 200.0;
posprev.data = ((msg->data + 90) * (200.0 / 180.0)) - 100.0;

if(posprev.data > 200){
pos.data=200;
}
else if(posprev.data<-200){
pos.data=-200;
}
else{
pos.data = posprev.data;
//ROS_INFO("else condition entered");
}
	//pos.data = 0.002777778*(msg->data);
xarm_msgs::Move line;
line.request.pose.clear();

line.request.pose.push_back(280);
		line.request.pose.push_back(pos.data);
		line.request.pose.push_back(200);
		line.request.pose.push_back(1.57);
		line.request.pose.push_back(0);
		line.request.pose.push_back(0);
	
		line.request.mvvelo = 100;//50,100
		line.request.mvacc=300;//200,300
//ros::Duration(0.2).sleep();
		if(x){
		client_line.call(line);
		ROS_INFO("count: %f",pos.data);
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(2);
  spinner.start();

    static const std::string PLANNING_GROUP_ARM = "xarm7";
    
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    	

//services
ros::ServiceClient client_state = n.serviceClient<xarm_msgs::SetInt16>("xarm/set_state");
ros::ServiceClient client_mode = n.serviceClient<xarm_msgs::SetInt16>("xarm/set_mode");
ros::ServiceClient client_motion = n.serviceClient<xarm_msgs::SetAxis>("xarm/motion_ctrl");
ros::ServiceClient client_line = n.serviceClient<xarm_msgs::Move>("/xarm/move_line");

xarm_msgs::SetInt16 srv_mode;
xarm_msgs::SetInt16 srv_state;
xarm_msgs::SetAxis srv_enable;
xarm_msgs::Move line;

ros::service::waitForService("xarm/motion_ctrl");
ros::service::waitForService("xarm/set_state");
ros::service::waitForService("xarm/move_line");
ROS_WARN("outof wait");


//ros::Rate loop_rate(10);
//loop

// motion enable:
	srv_enable.request.id = 8;
	srv_enable.request.data = 1;
	client_motion.call(srv_enable);
	ros::Duration(1.0).sleep();
//set thr mode and state
		srv_mode.request.data = 7;
		srv_state.request.data = 0;
		client_mode.call(srv_mode);
		client_state.call(srv_state);


		
		ros::Duration(1.0).sleep();
		line.request.pose.push_back(280);
		line.request.pose.push_back(0);
		line.request.pose.push_back(200);
		line.request.pose.push_back(1.57);
		line.request.pose.push_back(0);
		line.request.pose.push_back(0);
	
		line.request.mvvelo = 50;
		line.request.mvacc=200;
		client_line.call(line);
		ROS_INFO("Setting intial position... ");
ros::Duration(6).sleep();

line.request.pose.clear();
		srv_mode.request.data = 7;
		srv_state.request.data = 0;
		client_mode.call(srv_mode);
		client_state.call(srv_state);
line.request.pose.push_back(280);
		line.request.pose.push_back(50);
		line.request.pose.push_back(200);
		line.request.pose.push_back(1.57);
		line.request.pose.push_back(0);
		line.request.pose.push_back(0);
	
		line.request.mvvelo = 50;
		line.request.mvacc=200;
		client_line.call(line);
		ROS_INFO("Setting 50 position... ");
ros::Duration(2.0).sleep();

line.request.pose.clear();
		srv_mode.request.data = 7;
		srv_state.request.data = 0;
		client_mode.call(srv_mode);
		client_state.call(srv_state);
line.request.pose.push_back(280);
		line.request.pose.push_back(-50);
		line.request.pose.push_back(200);
		line.request.pose.push_back(1.57);
		line.request.pose.push_back(0);
		line.request.pose.push_back(0);
	
		line.request.mvvelo = 50;
		line.request.mvacc=200;
		client_line.call(line);
		ROS_INFO("Setting -50 position... ");

ros::Duration(2.0).sleep();
//ros::Duration(5.0).sleep();
ros::Rate loop_rate(1.0);
int count = 0;
int val=30;
ros::Subscriber sub = n.subscribe<std_msgs::Float64>("chatter", 100000, boost::bind(callback, _1, boost::ref(client_line)));

while(ros::ok()){
		x = false;
		srv_mode.request.data = 7;
		srv_state.request.data = 0;
		client_mode.call(srv_mode);
		client_state.call(srv_state);
		x= true;
//val = val*std::pow(-1,count);
/**
line.request.pose.clear();
		srv_mode.request.data = 7;
		srv_state.request.data = 0;
		client_mode.call(srv_mode);
		client_state.call(srv_state);
line.request.pose.push_back(280);
		line.request.pose.push_back(pos.data);
		line.request.pose.push_back(200);
		line.request.pose.push_back(3.14);
		line.request.pose.push_back(0);
		line.request.pose.push_back(0);
	
		line.request.mvvelo = 100;
		line.request.mvacc=300;
ros::Duration(0.2).sleep();
		client_line.call(line);
		ROS_INFO("count: %f",pos.data);


		//ROS_INFO("Setting line loop... ");
//count +=1;
	**/	
    ros::spinOnce();
    loop_rate.sleep();
		
}


ros::waitForShutdown();
  return 0;
}



