#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "end_effector_pose_listener");
  ros::NodeHandle nh;

  tf::TransformListener listener;

  ros::Rate rate(10.0); // Rate at which to check the transform

  while (ros::ok())
  {
    tf::StampedTransform transform;

    try
    {
      // Wait for the transform between the base link and link7
      listener.waitForTransform("link_base", "link7", ros::Time(0), ros::Duration(1.0));

      // Get the transform between the base link and link7
      listener.lookupTransform("link_base", "link7", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // Extract the position and orientation from the transform
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "link_base";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = transform.getOrigin().x();
    pose.pose.position.y = transform.getOrigin().y();
    pose.pose.position.z = transform.getOrigin().z();
    pose.pose.orientation.x = transform.getRotation().x();
    pose.pose.orientation.y = transform.getRotation().y();
    pose.pose.orientation.z = transform.getRotation().z();
    pose.pose.orientation.w = transform.getRotation().w();

    // Publish the pose
    ROS_INFO("End Effector Pose: x=%.3f, y=%.3f, z=%.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    ROS_INFO("End Effector Orientation: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    rate.sleep();
  }

  return 0;
}

