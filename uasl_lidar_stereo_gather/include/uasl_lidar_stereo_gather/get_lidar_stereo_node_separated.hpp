#ifndef UASL_LIDAR_STEREO_GATHER_GET_LIDAR_STEREO_NODE_SEPARATED_HPP
#define UASL_LIDAR_STEREO_GATHER_GET_LIDAR_STEREO_NODE_SEPARATED_HPP

#include "get_data_base.hpp"

namespace datag {

class Get_lidar_stereo_node_separated : Get_data_base
{
	public:
	Get_lidar_stereo_node_separated();
	virtual ~Get_lidar_stereo_node_separated() {}
	
	private:
	ros::Publisher out_pub_laser;
	ros::Publisher out_pub_img_1;
	ros::Publisher out_pub_img_2;
	
	void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg) override;
		
};
} //End of datag namespace
#endif
