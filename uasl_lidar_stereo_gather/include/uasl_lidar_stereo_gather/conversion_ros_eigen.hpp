#ifndef UASL_LIDAR_STEREO_GATHER_CONVERSION_ROS_EIGEN_HPP
#define UASL_LIDAR_STEREO_GATHER_CONVERSION_ROS_EIGEN_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace datag
{

void cvt_pose_2_eigen(const geometry_msgs::Pose& odometry, Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& odometry_eigen)
{
	
	const Eigen::Quaterniond rot_quaternion(odometry.orientation.w
											,odometry.orientation.x
											,odometry.orientation.y
											,odometry.orientation.z);
									
	const Eigen::Matrix3d rot_matrix = rot_quaternion.toRotationMatrix();
	
	odometry_eigen = Eigen::Matrix4d::Identity();
    Eigen::Vector3d translation(
            odometry.position.x,
            odometry.position.y,
            odometry.position.z);
    odometry_eigen.translation() = translation;
    odometry_eigen.linear() = rot_matrix;
}

void cvt_pose_2_eigen(const geometry_msgs::PoseStampedConstPtr& odometry, Eigen::Matrix4d& odometry_eigen)
{	
	const Eigen::Quaterniond rot_quaternion(odometry->pose.orientation.w
											,odometry->pose.orientation.x
											,odometry->pose.orientation.y
											,odometry->pose.orientation.z);
									
	const Eigen::Matrix3d rot_matrix = rot_quaternion.toRotationMatrix();
	
	odometry_eigen = Eigen::Matrix4d::Identity();
	odometry_eigen.block(0,0,3,3) = rot_matrix;
	odometry_eigen(0,3) = odometry->pose.position.x;
	odometry_eigen(1,3) = odometry->pose.position.y;
	odometry_eigen(2,3) = odometry->pose.position.z;
}

void cvt_eigen_2_pose(const Eigen::Matrix4d& odometry_eigen, geometry_msgs::Pose& pose)
{
	pose.position.x = odometry_eigen(0,3);
	pose.position.y = odometry_eigen(1,3);
	pose.position.z = odometry_eigen(2,3);
	
	const Eigen::Matrix3d rot_matrix = odometry_eigen.block(0,0,3,3);
	
	const Eigen::Quaterniond rot_quaternion(rot_matrix);
	
	pose.orientation.w = rot_quaternion.w();
	pose.orientation.x = rot_quaternion.x();
	pose.orientation.y = rot_quaternion.y();
	pose.orientation.z = rot_quaternion.z();	
}

void cvt_eigen_2_pose(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& odometry_eigen, geometry_msgs::Pose& pose)
{
	const Eigen::Vector3d translation = odometry_eigen.translation();
	pose.position.x = translation(0,3);
	pose.position.y = translation(1,3);
	pose.position.z = translation(2,3);
	
	const Eigen::Matrix3d rot_matrix = odometry_eigen.linear();
	
	const Eigen::Quaterniond rot_quaternion(rot_matrix);
	
	pose.orientation.w = rot_quaternion.w();
	pose.orientation.x = rot_quaternion.x();
	pose.orientation.y = rot_quaternion.y();
	pose.orientation.z = rot_quaternion.z();	
}

void convert_pc(const sensor_msgs::PointCloud2& pointcloud_msg, pcl::PointCloud<pcl::PointXYZ>& laser_cloud)
{
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pointcloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,laser_cloud);
}

void convert_pc(const sensor_msgs::PointCloud& pointcloud_msg, pcl::PointCloud<pcl::PointXYZ>& laser_cloud)
{
	sensor_msgs::PointCloud2 input_cloud;
	sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg,input_cloud);
	convert_pc(input_cloud,laser_cloud);
}

void convert_pc_pcl_2_ros(const pcl::PointCloud<pcl::PointXYZ>& laser_cloud, sensor_msgs::PointCloud2& pointcloud_msg)
{
	pcl::PCLPointCloud2 pcl_pc2;
	pcl::toPCLPointCloud2(laser_cloud,pcl_pc2);
	pcl_conversions::fromPCL(pcl_pc2,pointcloud_msg);

} 
} //End of datag namespace
#endif
