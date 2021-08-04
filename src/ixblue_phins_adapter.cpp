#include <ros/ros.h>

#include "ixblue_ins_msgs/Ins.h"

#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

#include <tf2/utils.h>

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;

std::string frame_id;

void insCallback(const ixblue_ins_msgs::Ins::ConstPtr& msg)
{
  ros::Time timestamp = msg->header.stamp;
  if(timestamp.is_zero()) // simulator has zero timestamp
    timestamp = ros::Time::now();
  sensor_msgs::NavSatFix nsf;
  nsf.header = msg->header;
  nsf.header.frame_id = frame_id;
  nsf.header.stamp = timestamp;
  nsf.latitude = msg->latitude;
  nsf.longitude = msg->longitude;
  nsf.altitude = msg->altitude;
  nsf.position_covariance = msg->position_covariance;
  nsf.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  position_pub.publish(nsf);

  sensor_msgs::Imu imu;
  imu.header = msg->header;
  imu.header.frame_id = frame_id;
  imu.header.stamp = timestamp;
  tf2::Quaternion q;
  double yaw = M_PI*(90-msg->heading)/180.0;
  q.setRPY(M_PI*msg->roll/180.0, M_PI*msg->pitch/180.0, yaw);
  //q.setRPY(0.0, 0.0, yaw);
  tf2::convert(q, imu.orientation);
  orientation_pub.publish(imu);

  geometry_msgs::TwistWithCovarianceStamped twcs;
  twcs.header = msg->header;
  twcs.header.frame_id = frame_id;
  twcs.header.stamp = timestamp;
  tf2::Vector3 speed;
  tf2::convert(msg->speed_vessel_frame, speed);
  auto map_speed = tf2::quatRotate(q, speed);
  tf2::convert(map_speed, twcs.twist.twist.linear);
  velocity_pub.publish(twcs);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ixblue_ins_adapter");
  ros::NodeHandle n;

  frame_id = ros::param::param<std::string>("~frame_id", "base_link");

  position_pub = n.advertise<sensor_msgs::NavSatFix>("project11/nav/phins/position", 1);
  orientation_pub = n.advertise<sensor_msgs::Imu>("project11/nav/phins/orientation",1);
  velocity_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("project11/nav/phins/velocity",1);

  ros::Subscriber ins_sub = n.subscribe("/d_phins/ins", 1, &insCallback);

  ros::spin();
}
