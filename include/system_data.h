#pragma once

#include <string>
#include "point_type.h"

const std::string out_pcd_dir = "/home/xl/projects/ws_lio_sam/parse/pcd/";
const std::string out_imu_file = "/home/xl/projects/ws_lio_sam/parse/imu/raw_imu.txt";
const std::string out_lidar_pose_file = "/home/xl/projects/ws_lio_sam/parse/odometry/lidar_poses.txt";
const std::string out_imu_pose_file = "/home/xl/projects/ws_lio_sam/parse/odometry/imu_poses.txt";

FILE *imu_file_handle_ = fopen(out_imu_file.c_str(), "a");
FILE *lidar_pose_file_handle = fopen(out_lidar_pose_file.c_str(), "a");
FILE *imu_pose_file_handle = fopen(out_imu_pose_file.c_str(), "a");

void savePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  pcl::PointCloud<PointXYZIRT> Cloud;
  pcl::fromROSMsg(*laserCloudMsg, Cloud);
  double currentLaserTime = laserCloudMsg->header.stamp.toSec();
  std::string out_pcd_file = out_pcd_dir + std::to_string(currentLaserTime) + ".pcd";
  pcl::io::savePCDFile(out_pcd_file, Cloud);
}

void saveRawImu(const sensor_msgs::Imu::ConstPtr& imuMsg) {
  if(imu_file_handle_ == nullptr)
    return;

  double imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  double currentImuTime = imuMsg->header.stamp.toSec();

  fprintf(imu_file_handle_, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
          currentImuTime,
          imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w,
          imuRoll, imuPitch, imuYaw,
          imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z,
          imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
}

// msg_type: lidar / imu
void saveOdometryPose(const nav_msgs::Odometry &odometryMsg,
                      const std::string &msg_type) {
  double timestamp, x, y, z, qx, qy, qz, qw;

  timestamp = odometryMsg.header.stamp.toSec();
  x = odometryMsg.pose.pose.position.x;
  y = odometryMsg.pose.pose.position.y;
  z = odometryMsg.pose.pose.position.z;
  qx = odometryMsg.pose.pose.orientation.x;
  qy = odometryMsg.pose.pose.orientation.y;
  qz = odometryMsg.pose.pose.orientation.z;
  qw = odometryMsg.pose.pose.orientation.w;

  if(msg_type == "lidar") {
    if(lidar_pose_file_handle == nullptr) return;
    fprintf(lidar_pose_file_handle, "%lf %lf %lf %lf %lf %lf %lf %lf\n", timestamp, x, y, z, qx, qy, qz, qw);

  } else if(msg_type == "imu") {
    if(imu_pose_file_handle == nullptr) return;
    fprintf(imu_pose_file_handle, "%lf %lf %lf %lf %lf %lf %lf %lf\n", timestamp, x, y, z, qx, qy, qz, qw);

  } else {}
}
