#ifndef UASL_LIDAR_STEREO_GATHER_GET_DATA_BASE_HPP
#define UASL_LIDAR_STEREO_GATHER_GET_DATA_BASE_HPP

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
#include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/core.hpp>
#endif

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

//#define UASL_USE_CAMERA//If this is defined, we use the cameras, else we don't

#ifdef UASL_USE_CAMERA
#include "uasl_image_acquisition/acquisition.hpp"
#endif

#include <string>

namespace datag {

class Get_data_base
{
	public:
	Get_data_base();
	virtual ~Get_data_base() {}	
	
	protected:
	ros::NodeHandle nh;
	
	ros::Subscriber laser_sub;
	
	#ifdef UASL_USE_CAMERA
	cam::Acquisition acq;
	#endif
	
	bool get_images(sensor_msgs::Image& img_1, sensor_msgs::Image& img_2, const std_msgs::Header& header);//Get the images
	virtual void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg) = 0;//Callback function for lidar
	
};

} //end of datag namespace

#endif
