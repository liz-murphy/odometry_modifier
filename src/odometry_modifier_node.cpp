#include "ros/ros.h"
#include "ros/console.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_dijkstra.h"

#include <g2o/core/factory.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "g2o/types/sclam2d/types_sclam2d.h"
using namespace std;
G2O_USE_TYPE_GROUP(sclam);

class OdomModifier
{
  public:
    OdomModifier();
    ~OdomModifier(){};
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

  private:
  //  ros::NodeHandle nh_;
    g2o::SE2 last_pose_;
    ros::Time last_stamp_;
    bool initialized_;
    g2o::SE2 prev_calibrated_pose_;
    double vl_, vr_, b_;
    ros::Publisher calibrated_pub_;
    tf::TransformBroadcaster calibrated_broadcaster_;
};

OdomModifier::OdomModifier()
{
  ros::NodeHandle nh_("~");
  initialized_ = false;

  // Read in odometry calibration parameters
  double vl, vr, b;
  nh_.param("vl", vl_, 1.0); 
  nh_.param("vr", vr_, 1.0); 
  nh_.param("b", b_, 1.0); 

  ROS_INFO("Setting vl= %f, vr= %f, b= %f", vl_, vr_, b_);

  calibrated_pub_ = nh_.advertise<nav_msgs::Odometry>("calibrated_odom", 10);
}

void OdomModifier::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll,pitch,yaw);
  g2o::SE2 current_pose(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, yaw);
   
  if(initialized_)
  {
    double dt = (odom_msg->header.stamp - last_stamp_).toNSec()/1e9;
    g2o::SE2 motion = last_pose_.inverse() * current_pose;

    // convert to velocity measurment
    g2o::MotionMeasurement motionMeasurement(motion.translation().x(), motion.translation().y(), motion.rotation().angle(), dt);
    g2o::VelocityMeasurement velocityMeasurement = g2o::OdomConvert::convertToVelocity(motionMeasurement);

    // apply calibration
    g2o::VelocityMeasurement calibratedVelocityMeasurment = velocityMeasurement;
    calibratedVelocityMeasurment.setVl(vl_ * calibratedVelocityMeasurment.vl());
    calibratedVelocityMeasurment.setVr(vr_ * calibratedVelocityMeasurment.vr());
    g2o::MotionMeasurement mm = g2o::OdomConvert::convertToMotion(calibratedVelocityMeasurment, b_);

    // combine calibrated odometry with the previous pose
    g2o::SE2 remappedOdom;
    remappedOdom.fromVector(mm.measurement());
    prev_calibrated_pose_ = prev_calibrated_pose_* remappedOdom;
    nav_msgs::Odometry calibrated_msg;
    calibrated_msg.header.frame_id = "odom_calibrated";
    calibrated_msg.header.stamp = odom_msg->header.stamp;
    calibrated_msg.pose.pose.position.x = prev_calibrated_pose_[0];
    calibrated_msg.pose.pose.position.y = prev_calibrated_pose_[1];

    // ... orientation
    tf::Quaternion q;
    calibrated_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(prev_calibrated_pose_[2]); 
    calibrated_pub_.publish(calibrated_msg);

    // TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom_msg->header.stamp;
    odom_trans.header.frame_id = "odom_calibrated";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = prev_calibrated_pose_[0];
    odom_trans.transform.translation.y = prev_calibrated_pose_[1];
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(prev_calibrated_pose_[2]); 
    calibrated_broadcaster_.sendTransform(odom_trans);
  }
  else
  {
    prev_calibrated_pose_ = current_pose; // fix initial position 
    initialized_ = true;
  }

  last_pose_ = current_pose;
  last_stamp_ = odom_msg->header.stamp;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odometry_modifier");
  ros::NodeHandle nh; 
  OdomModifier* om = new OdomModifier();
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, &OdomModifier::odomCallback, om);
  ros::spin();
  return 0;
}
