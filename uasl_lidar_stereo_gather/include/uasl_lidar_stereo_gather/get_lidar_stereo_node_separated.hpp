#ifndef UASL_LIDAR_STEREO_GATHER_GET_LIDAR_STEREO_NODE_SEPARATED_HPP
#define UASL_LIDAR_STEREO_GATHER_GET_LIDAR_STEREO_NODE_SEPARATED_HPP

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#endif

#include <ros/ros.h>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>

#define UASL_USE_CAMERA //If this is defined, we use the cameras, else we don't

#ifdef UASL_USE_CAMERA
#include "uasl_image_acquisition/acquisition.hpp"
#endif



namespace datag {

class Get_lidar_stereo_node_separated
{
	public:
	Get_lidar_stereo_node_separated();
	virtual ~Get_lidar_stereo_node_separated() {}
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
	ros::NodeHandle nh;
	
	ros::Subscriber laser_sub;
	
	ros::Publisher out_pub_laser;
	ros::Publisher out_pub_img_1;
	ros::Publisher out_pub_img_2;
	
	#ifdef UASL_USE_CAMERA
	cam::Acquisition acq;
	#endif
	
	void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
		
};
} //End of datag namespace
#endif
