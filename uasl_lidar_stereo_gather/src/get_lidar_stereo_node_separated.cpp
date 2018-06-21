#include "get_lidar_stereo_node_separated.hpp"

#include <string>

namespace datag {

Get_lidar_stereo_node_separated::Get_lidar_stereo_node_separated()
{
	ros::NodeHandle pnh("~");
	
	std::string output_topic_lidar, output_topic_img_1, output_topic_img_2; 
	if(!pnh.getParam("output_topic_lidar", output_topic_lidar))
	{
		ROS_ERROR("Could not find the output_topic_lidar parameter.");
	}
	if(!pnh.getParam("output_topic_img_1", output_topic_img_1))
	{
		ROS_ERROR("Could not find the output_topic_img_1 parameter.");
	}
	if(!pnh.getParam("output_topic_img_2", output_topic_img_2))
	{
		ROS_ERROR("Could not find the output_topic_img_2 parameter.");
	}
	
	out_pub_laser = nh.advertise<sensor_msgs::PointCloud2>(output_topic_lidar, 1);
	out_pub_img_1 = nh.advertise<sensor_msgs::Image>(output_topic_img_1, 1);
	out_pub_img_2 = nh.advertise<sensor_msgs::Image>(output_topic_img_2, 1);
}

void Get_lidar_stereo_node_separated::laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	//Callback of the LIDAR. The images are acquired here as well.	
	sensor_msgs::PointCloud2 out_msg_lidar;
	sensor_msgs::Image out_msg_img_1, out_msg_img_2;
	
	if(get_images(out_msg_img_1,out_msg_img_2,pointcloud_msg->header))
	{
		out_msg_lidar = sensor_msgs::PointCloud2(*pointcloud_msg);//Copy the pointcloud
		
		//Publish the messages
		out_pub_laser.publish(out_msg_lidar);
		out_pub_img_1.publish(out_msg_img_1);
		out_pub_img_2.publish(out_msg_img_2);		
	}
	else
	{
		ROS_WARN("Error during image acquisition, skipping a pointcloud.");
	}	
}

} //End of datag namespace


int main(int argc, char * argv[])
{
	ros::init(argc, argv, "get_lidar_stereo_separated_node");
	
	datag::Get_lidar_stereo_node_separated node;
	
	ros::spin();
	
	return 0;
}
