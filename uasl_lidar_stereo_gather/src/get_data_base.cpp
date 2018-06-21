#include "get_data_base.hpp"

#ifdef UASL_USE_CAMERA
#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
#include <opencv2/core/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/highgui.hpp>
#endif

#include <cv_bridge/cv_bridge.h>
#include "uasl_image_acquisition/camera_mvbluefox.hpp"
#endif

namespace datag {

Get_data_base::Get_data_base()
{
	ros::NodeHandle pnh("~");
	
	std::string laser_topic;
	
	if(!pnh.getParam("laser_topic", laser_topic))
	{
		ROS_ERROR("Could not find the laser_topic parameter.");	
	}
	
	
	ROS_INFO("Starting a node to acquire data. Listening on %s for the lidar.", laser_topic.c_str());
	
	laser_sub = nh.subscribe(laser_topic, 1, &Get_data_base::laser_callback, this);
	
	#ifdef UASL_USE_CAMERA
	std::string cam_1_id, cam_2_id;
	if(!pnh.getParam("cam_1_id", cam_1_id))
	{
		ROS_ERROR("Could not find the cam_1_id parameter.");
	}
	if(!pnh.getParam("cam_2_id", cam_2_id))
	{
		ROS_ERROR("Could not find the cam_2_id parameter.");
	}
	acq.add_camera<cam::bluefox>(cam_1_id);
	acq.add_camera<cam::bluefox>(cam_2_id);
    acq.start_acq();
    #endif		
}

bool Get_data_base::get_images(sensor_msgs::Image& img_1, sensor_msgs::Image& img_2, const std_msgs::Header& header)
{
	#ifdef UASL_USE_CAMERA
	std::vector<cv::Mat> images;
	if(acq.get_images(images) > 0)
	{
		if(images.size() == 2)
		{
			//Copy the images
			cv_bridge::CvImage(header, "bgr8", images[0]).toImageMsg(img_1);
			cv_bridge::CvImage(header, "bgr8", images[1]).toImageMsg(img_2);
			
			//Show the images
			cv::imshow("img_1",images[0]);
			cv::imshow("img_2",images[1]);
			cv::waitKey(1);	
		}
		else return false;	
	}
	else return false;
	#endif
	return true;		
}

} //end of datag namespace
