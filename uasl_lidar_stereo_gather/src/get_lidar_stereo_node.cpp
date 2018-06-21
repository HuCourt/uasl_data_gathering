#include "get_lidar_stereo_node.hpp"

#include "uasl_msg_gather/Pointcloud2_stereo.h"
#include <string>

namespace datag {

Get_lidar_stereo_node::Get_lidar_stereo_node()
{
	ros::NodeHandle pnh("~");
	
	std::string output_topic; 
	if(!pnh.getParam("output_topic", output_topic))
	{
		ROS_ERROR("Could not find the output_topic parameter.");
	}
	
	out_pub = nh.advertise<uasl_msg_gather::Pointcloud2_stereo>(output_topic,1);
}

void Get_lidar_stereo_node::laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	//Callback of the LIDAR. The images are acquired here as well.	
	uasl_msg_gather::Pointcloud2_stereo out_msg;
	if(get_images(out_msg.img_1,out_msg.img_2,pointcloud_msg->header))
	{
		out_msg.pc = sensor_msgs::PointCloud2(*pointcloud_msg);//Copy the pointcloud
		
		//Publish the message
		out_pub.publish(out_msg);			
	}
	else
	{
		ROS_WARN("Error during image acquisition, skipping a pointcloud.");
	}	
}

} //End of datag namespace


int main(int argc, char * argv[])
{
	ros::init(argc, argv, "get_lidar_stereo_node");
	
	datag::Get_lidar_stereo_node node;
	
	ros::spin();
	
	return 0;
}
