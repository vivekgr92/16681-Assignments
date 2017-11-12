#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>



ImageConverter* ic;
float current_pose_x,current_pose_y,current_pose_z,quater_curr_orient_x,quater_curr_orient_y,quater_curr_orient_z,quater_curr_orient_w;

//apriltags_ros::AprilTagDetection tagmsg;          // Message of the node apriltags_ros
std::string childFrame="";
std::string parentFrame="";
void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
  ROS_INFO("In subscribtion call back apriltag detections [%x]",msg.detections.size());  // Data should be in the stream

  static tf::TransformBroadcaster br;
  tf::Transform transform;


  for (int i=0; i<msg.detections.size();i++)
  {
    // Position
    current_pose_x=msg.detections[0].pose.pose.position.x;    // See the message structure for extracting the pose info
    current_pose_y=msg.detections[0].pose.pose.position.y;
    current_pose_z=msg.detections[0].pose.pose.position.z;

    // Quaternions

    quater_curr_orient_x=msg.detections[0].pose.pose.orientation.x;
    quater_curr_orient_y=msg.detections[0].pose.pose.orientation.y;
    quater_curr_orient_z=msg.detections[0].pose.pose.orientation.z;
    quater_curr_orient_w=msg.detections[0].pose.pose.orientation.w;

    //Set values to the IC pointer

    ic->setTagLocations(current_pose_x,current_pose_y,current_pose_z);

    //Do the transformation

    transform.setOrigin( tf::Vector3(current_pose_x, current_pose_y, current_pose_z) );
    transform.setRotation( tf::Quaternion(quater_curr_orient_x, quater_curr_orient_y, quater_curr_orient_z, quater_curr_orient_w) );
    br.sendTransform(tf::StampedTransform(transform,msg.detections[0].pose.header.stamp,parentFrame,childFrame));
    //msg.detections[0].pose.header.stamp
    //ros::Time::now()


  }

  //ROS_INFO("Current Z [%f]",current_pose_x);

  //TODO: Parse message and publish transforms as apriltag_tf and camera
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  /*******************************************************************/
  
  ros::NodeHandle n;
  ros::Subscriber apriltag_sub;
  n.getParam("childFrame",childFrame);
  n.getParam("parentFrame",parentFrame);

  //TODO: Add a subscriber to get the AprilTag detections The callback function skeleton is given.
  apriltag_sub = n.subscribe("tag_detections", 1000,apriltag_detection_callback );


  /******************************************************************/
  ImageConverter converter;
  ic = &converter;
  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
