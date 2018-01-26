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
	CloudCompressor(std::string outputMsgTopic, std::string inputCloudTopic, std::string sensorName, double resolution, bool showStatistics);

	// Empty deconstrucor
	~CloudCompressor(){}

	// Callback for PointCloudXYZ subscriber
	void roscallback(const PointCloud::ConstPtr &cloud);

private:

	// ROS variables
	std::string rosTFGlobalFrame_;
	std::string rosTFLocalFrame_;

	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	tf::TransformListener tfListener_;

	// Pointers to temporary point clouds
	PointCloud::Ptr transformedCloud_;
	PointCloud::Ptr croppedCloud_;

	// Compression setup
	double octreeResolution_;

	Compressor pointCloudEncoder_;

	// Box crop filter
	Eigen::Vector4f minPT_, maxPT_;
	pcl::CropBox<PointType> crop_;
};

}


#endif /* COMPRESSOR_H_ */
