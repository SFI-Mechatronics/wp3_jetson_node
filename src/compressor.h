/*
 * compressor.h
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#ifndef COMPRESSOR_H_
#define COMPRESSOR_H_

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

#include "defines.h"

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudCompression Compressor;


namespace wp3 {

class CloudCompressor
{
public:
	// Constructor
	CloudCompressor(std::string sensorName, std::string inputCloudTopic, std::string outputMsgTopic, double resolution);
	~CloudCompressor();

	// Callback for PointCloudXYZ subscriber
	void roscallback(const PointCloud::ConstPtr &cloud);

private:
	std::string rosSensorName;
	std::string rosTransformGlobalFrame;
	std::string rosTransformLocalFrame;

	// ROS handles
	ros::NodeHandle nh_;

	tf::TransformListener tfListener;

	ros::Subscriber sub_;
	ros::Publisher pub_;

	// Pointers to temporary point clouds
	PointCloud::Ptr transformedCloud;
	PointCloud::Ptr croppedCloud;


	// Compression setup
	static const bool showStatistics = _STATISTICS;
	double octreeResolution;
	static const unsigned int iFrameRate = _IFRAMERATE;

	Compressor pointCloudEncoder;

	// Filters
	Eigen::Vector4f minPT, maxPT;
	pcl::CropBox<PointType> crop;
};

}


#endif /* COMPRESSOR_H_ */
