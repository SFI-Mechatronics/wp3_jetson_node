/*
 * decompressor.h
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#ifndef DECOMPRESSOR_H_
#define DECOMPRESSOR_H_

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "octree_decompression.h"

#include "defines.h"

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudDecompression Decompressor;

namespace wp3 {

class CloudDecompressor
{
public:
	// Constructor
	CloudDecompressor(std::string sensorName, std::string inputMsgTopic, const bool showStatistics);
	~CloudDecompressor();

	// Callback for PointCloudXYZ subscriber
	void roscallback(const std_msgs::String::ConstPtr& msg);

private:

	// ROS handles
	ros::NodeHandle nh_;

	ros::Subscriber sub_;
	ros::Publisher pub_;

	// Pointers to temporary point clouds
	PointCloudXYZI::Ptr decompressedCloud;
	PointCloudXYZI::Ptr outputCloud;

	Decompressor pointCloudDecoder;

	pcl::PassThrough<PointType_out> ptfilter; // Initializing with true will allow us to extract the removed indices
};

}


#endif /* DECOMPRESSOR_H_ */
