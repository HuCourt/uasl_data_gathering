#include "get_lidar_stereo_node_separated.hpp"

#include "sensor_msgs/PointCloud2.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef UASL_USE_CAMERA
#include <cv_bridge/cv_bridge.h>
#include "uasl_image_acquisition/camera_mvbluefox.hpp"
#endif

#include <string>

namespace datag {

Get_lidar_stereo_node_separated::Get_lidar_stereo_node_separated()
{
	ros::NodeHandle pnh("~");
	
	std::string laser_topic, output_topic_lidar, output_topic_img_1, output_topic_img_2, cam_1_id, cam_2_id; 
	pnh.getParam("laser_topic", laser_topic);
	pnh.getParam("cam_1_id", cam_1_id);
	pnh.getParam("cam_2_id", cam_2_id);
	pnh.getParam("output_topic_lidar", output_topic_lidar);
	pnh.getParam("output_topic_img_1", output_topic_img_1);
	pnh.getParam("output_topic_img_2", output_topic_img_2);
	
	ROS_INFO("Starting a node to acquire data. Listening on %s for the lidar.", laser_topic.c_str());
	
	laser_sub = nh.subscribe(laser_topic, 1, &Get_lidar_stereo_node_separated::laser_callback, this);
	
	#ifdef UASL_USE_CAMERA
	out_pub_laser = nh.advertise<sensor_msgs::PointCloud2>(output_topic_lidar, 1);
	out_pub_img_1 = nh.advertise<sensor_msgs::Image>(output_topic_img_1, 1);
	out_pub_img_2 = nh.advertise<sensor_msgs::Image>(output_topic_img_2, 1);
	acq.add_camera<cam::bluefox>(cam_1_id);
    	acq.add_camera<cam::bluefox>(cam_2_id);
    	acq.start_acq();
	#else
	out_pub_laser = nh.advertise<sensor_msgs::Pointcloud2>(output_topic_lidar, 1);
	#endif
}

void Get_lidar_stereo_node_separated::laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	//Callback of the LIDAR. The images are acquired here as well.
	#ifdef UASL_USE_CAMERA
	std::vector<cv::Mat> images;
	if(acq.get_images(images) > 0)
	{
		if(images.size() != 2) ROS_ERROR("The number of retrieved images is different from 2.");
		else
		{
			sensor_msgs::PointCloud2 out_msg_lidar = sensor_msgs::PointCloud2(*pointcloud_msg);//Copy the pointcloud
			
			sensor_msgs::Image out_msg_img_1, out_msg_img_2;
			//Copy the images
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[0]).toImageMsg(out_msg_img_1);
			cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[1]).toImageMsg(out_msg_img_2);
			
			//For consistency purpose, the header of the images is defined as the one from the pointcloud
			out_msg_img_1.header = out_msg_lidar.header;
			out_msg_img_2.header = out_msg_lidar.header;
			
			//Publish the message
			out_pub_laser.publish(out_msg_lidar);
			out_pub_img_1.publish(out_msg_img_1);
			out_pub_img_2.publish(out_msg_img_2);
			
			//Show the images
			cv::imshow("img_1",images[0]);
			cv::imshow("img_2",images[1]);
			cv::waitKey(1);
		}
	}
	else
	{
		ROS_WARN("Error during image acquisition, skipping a pointcloud.");
	}
	#else
	sensor_msgs::Pointcloud2 out_msg(*pointcloud_msg);	
	out_pub.publish(out_msg);
	#endif		
}

} //End of datag namespace


int main(int argc, char * argv[])
{
	ros::init(argc, argv, "get_lidar_stereo_separated_node");
	
	datag::Get_lidar_stereo_node_separated node;
	
	ros::spin();
	
	return 0;
}
