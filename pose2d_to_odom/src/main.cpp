/* pose2d to odometry converter - tank robot */
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

class Pose2DToOdometry
{
public:
  Pose2DToOdometry();
  void onPose2D(const geometry_msgs::Pose2D::ConstPtr &pose2d);

private:
  ros::NodeHandle _n;
  ros::Subscriber _pose2DSub;
  ros::Publisher _odomPub;

  ros::Publisher _stampedPosePub;

  geometry_msgs::Pose2D _lastPose;
  ros::Time _lastPoseReceivedTime;
  uint32_t odomSeq;
  bool ready;
};

Pose2DToOdometry::Pose2DToOdometry()
{
  _pose2DSub = _n.subscribe<geometry_msgs::Pose2D>("/pose2D", 1, &Pose2DToOdometry::onPose2D, this);
  _stampedPosePub = _n.advertise<geometry_msgs::PoseStamped>("/stamped", 1);
  _odomPub = _n.advertise<nav_msgs::Odometry>("/odomCustom", 1);
  ready = false;
  odomSeq = 0;

  // a
}

void Pose2DToOdometry::onPose2D(const geometry_msgs::Pose2D::ConstPtr &pose2d)
{

  ros::Time stamp = ros::Time::now();
  // ROS_INFO(pose2d);
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.header.stamp = stamp;
  pose.pose.position.x = pose2d->x;
  pose.pose.position.y = pose2d->y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose2d->theta);

  if (!ready)
  {
    ready = true; // We've received first scan data [could be simplified]
  }
  else
  {
    ros::Duration dt = stamp - _lastPoseReceivedTime;

    // geometry_msgs::Pose2D pose2dDiff = *pose2d - _lastPose;
    geometry_msgs::Pose2D pose2dDiff;

    pose2dDiff.x = pose2d->x - _lastPose.x;
    pose2dDiff.y = pose2d->y - _lastPose.y;
    pose2dDiff.theta = pose2d->theta - _lastPose.theta;

    double dtSecs = dt.toSec();
    geometry_msgs::Twist twist; // roll and pitch should be 0, laser is supposed to be gyro-stabilized [ :') ]
    twist.linear.x = pose2dDiff.x;
    twist.linear.y = pose2dDiff.y;
    twist.angular.z = pose2dDiff.theta; // yaw

    geometry_msgs::PoseWithCovariance covPose;

    covPose.pose.position.x = pose2d->x;
    covPose.pose.position.y = pose2d->y;
    covPose.pose.orientation = tf::createQuaternionMsgFromYaw(pose2d->theta);

    geometry_msgs::TwistWithCovariance covTwist;
    covTwist.twist = twist;

    nav_msgs::Odometry odom;
    odom.header.seq = odomSeq++;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.pose = covPose;
    // odom.twist = covTwist;
    odom.child_frame_id = "base_link";

    _odomPub.publish(odom);
  }
  _lastPose = *pose2d;
  _lastPoseReceivedTime = stamp;

  _stampedPosePub.publish(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose2d_to_odom");

  Pose2DToOdometry converter;

  ros::Rate loop_rate(480);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
