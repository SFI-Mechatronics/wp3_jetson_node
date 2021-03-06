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
#include <fstream>
#include <cstdlib>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include "octree_compression.h"

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef wp3::PointCloudCompression Compressor;


namespace wp3 {

class CloudCompressor
{
public:
	// Constructor
	CloudCompressor(std::string outputMsgTopic, std::string inputCloudTopic, std::string rosTFLocalFrame, std::string rosTFGlobalFrame,
			double octreeResolution, unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics);

	// Deconstrucor
	~CloudCompressor();

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
	pcl::CropBox<PointType> crop_;

	// Logging
	bool showStatistics_;
	std::string logFile_;
	std::ofstream logStream_;
};

}


#endif /* COMPRESSOR_H_ */
