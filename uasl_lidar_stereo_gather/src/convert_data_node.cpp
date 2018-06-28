#include "convert_data_node.hpp"

#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


//From a bag or a topic file with an all inclusive message type, separate the different type of messages.
//Action taken for the input depends on the action_in parameter :
//action_in=bag : open a bag file (default choice)
//action_in=topic : listens on several topics
//Action taken for the output depends on the action_out parameter :
//action_out=bag : create a bag file (default choice)
//action_out=topic : publish on several topics

namespace datag {

Convert_data::Convert_data()
{
	ros::NodeHandle pnh("~");
	
	//Get the arguments
	pnh.param<std::string>("bag_input",bag_name_input,"default_bag_input.bag");
	pnh.param<std::string>("bag_output",bag_name_output,"default_bag_output.bag");
	pnh.param<std::string>("action_in",action_in,"bag");
	pnh.param<std::string>("action_out",action_out,"bag");
	pnh.param<std::string>("input_merged_data_topic",input_merged_data_topic,"default_input_merged_data_topic");
	pnh.param<std::string>("output_img_1_topic",output_img_1_topic,"default_output_img_1_topic");
	pnh.param<std::string>("output_img_2_topic",output_img_2_topic,"default_output_img_2_topic");
	pnh.param<std::string>("output_pc_topic",output_pc_topic,"default_output_pc_topic");
	
	if(action_out == "topic")
	{
		pc_pub = nh.advertise<sensor_msgs::PointCloud2>(output_pc_topic,10);
		img_1_pub = nh.advertise<sensor_msgs::Image>(output_img_1_topic,10);
		img_2_pub = nh.advertise<sensor_msgs::Image>(output_img_2_topic,10);
		
		ROS_INFO("Publishing on topics.");
	}
	else
	{
		bag_output.open(bag_name_output, rosbag::bagmode::Write);
		ROS_INFO("Publishing in a bagfile.");
	}
	
	if(action_in == "topic")
	{
		input_sub = nh.subscribe(input_merged_data_topic, 10, &Convert_data::input_callback,this);
		
		
		ROS_INFO("Listening on topics...");
	}
	else 
	{
		//Open the bag
		bag_input.open(bag_name_input, rosbag::bagmode::Read);
		rosbag::View view(bag_input);
		ROS_INFO("Opening bag %s with %d messages.",bag_name_input.c_str(), view.size());
		
		sensor_msgs::Image img_1;
		sensor_msgs::Image img_2;
		sensor_msgs::PointCloud2 pc;
		
		foreach(rosbag::MessageInstance const m, view)
    	{
    		uasl_msg_gather::Pointcloud2_stereo::ConstPtr grouped_data = m.instantiate<uasl_msg_gather::Pointcloud2_stereo>();
    		if(grouped_data != nullptr)
    		{
    			input_callback(grouped_data); 
    		}
    		
    		tf2_msgs::TFMessage::ConstPtr tf_data = m.instantiate<tf2_msgs::TFMessage>();
    		if(tf_data != nullptr)
    		{
    			input_tf(tf_data,m.getTime());
    		}
    	}
    	
    	bag_input.close();
    	ROS_INFO("Finished reading bag file.");
	}
	
}

void Convert_data::input_tf(const tf2_msgs::TFMessage::ConstPtr& tf_msg, const ros::Time& time)
{
	if(action_in == "bag" && action_out == "topic")
	{
		tf_br.sendTransform(tf_msg->transforms);		
	}
	else if(action_out == "bag") bag_output.write("tf",time,*tf_msg);
}

void Convert_data::input_callback(const uasl_msg_gather::Pointcloud2_stereo::ConstPtr& pointcloud_msg)
{
	sensor_msgs::Image img_1(pointcloud_msg->img_1);
	sensor_msgs::Image img_2(pointcloud_msg->img_2);
	sensor_msgs::PointCloud2 pc(pointcloud_msg->pc);
	
	if(action_out == "topic")
	{
		pc_pub.publish(pc);
		img_1_pub.publish(img_1);
		img_2_pub.publish(img_2);
	}
	else if(action_out == "bag")
	{
		bag_output.write(output_pc_topic,pointcloud_msg->pc.header.stamp,pc);
		bag_output.write(output_img_1_topic,pointcloud_msg->img_1.header.stamp,img_1);
		bag_output.write(output_img_2_topic,pointcloud_msg->img_2.header.stamp,img_2);
	}			
}
} //end of datag namespace

int main(int argc, char * argv[])
{
	ros::init(argc,argv,"convert_pc_pose_to_bag");
	
	datag::Convert_data c_data;
	ros::spin();
	
	return 0;
}
