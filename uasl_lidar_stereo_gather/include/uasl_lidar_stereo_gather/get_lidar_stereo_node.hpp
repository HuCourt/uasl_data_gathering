#ifndef UASL_LIDAR_STEREO_GATHER_GET_LIDAR_STEREO_NODE_HPP
#define UASL_LIDAR_STEREO_GATHER_GET_LIDAR_STEREO_NODE_HPP

#include "get_data_base.hpp"

namespace datag {

class Get_lidar_stereo_node : Get_data_base
{
	public:
	Get_lidar_stereo_node();
	virtual ~Get_lidar_stereo_node() {}
	
	private:
	ros::Publisher out_pub;
	
	void laser_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg) override;
		
};
} //End of datag namespace
#endif
