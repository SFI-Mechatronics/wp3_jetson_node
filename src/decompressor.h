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
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include "octree_compression.h"
#include "octree_decompression.h"

#include "octree_compression.h"

#include "defines.h"

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudCompression Compressor;
typedef wp3::PointCloudDecompression Decompressor;

namespace wp3 {

class CloudDecompressor
{
public:
	// Constructor
	CloudDecompressor(std::string sensorName);
	~CloudDecompressor();

	// Callback for PointCloudXYZ subscriber
	void roscallback(const std_msgs::String::ConstPtr& msg);

private:
	std::string rosSensorName;
	std::string rosTransformGlobalFrame;
	std::string rosTransformLocalFrame;

	// ROS handles
	ros::NodeHandle nh_;


	ros::Subscriber sub_;
	ros::Publisher pub_;

	// Pointers to temporary point clouds
	PointCloudXYZI::Ptr compressedCloud;
	PointCloudXYZI::Ptr outputCloud;

	// Compression setup
	static const bool showStatistics = _STATISTICS;
	static const double octreeResolution = _OCTREERESOLUTION;

//	Compressor pointCloudEncoder;
	Decompressor pointCloudDecoder;

	// Filters
//	Eigen::Vector4f minPT, maxPT;
//	pcl::CropBox<PointType> crop;
	pcl::PassThrough<PointType_out> ptfilter; // Initializing with true will allow us to extract the removed indices
};

}


#endif /* DECOMPRESSOR_H_ */
