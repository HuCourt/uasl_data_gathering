#ifndef UASL_LIDAR_STEREO_GATHER_CONVERT_DATA_NODE_HPP
#define UASL_LIDAR_STEREO_GATHER_CONVERT_DATA_NODE_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include "uasl_msg_gather/Pointcloud2_stereo.h"

namespace datag {

class Convert_data
{
	public:
	Convert_data();
	
	private:
	ros::NodeHandle nh;
	
	ros::Publisher pc_pub;
	ros::Publisher img_1_pub;
	ros::Publisher img_2_pub;
	
	ros::Subscriber input_sub;
	
	std::string bag_name_input;
	std::string bag_name_output;
	std::string action_in;
	std::string action_out;
	std::string input_merged_data_topic;
	std::string output_img_1_topic;
	std::string output_img_2_topic;
	std::string output_pc_topic;
	
	rosbag::Bag bag_input;
	rosbag::Bag bag_output;
	
	tf2_ros::TransformBroadcaster tf_br;
	
	void input_callback(const uasl_msg_gather::Pointcloud2_stereo::ConstPtr& pointcloud_msg);
	void input_tf(const tf2_msgs::TFMessage::ConstPtr& tf_msg, const ros::Time& time);
};

} //end of datag namespace

#endif
