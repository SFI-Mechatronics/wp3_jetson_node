/*
 * decompressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */


#include "decompressor.h"

namespace wp3 {

CloudDecompressor::CloudDecompressor(std::string sensorName, std::string inputMsgTopic) :
				 decompressedCloud(new PointCloudXYZI ()),
				 outputCloud(new PointCloudXYZI ()),
				 ptfilter(false),
				 pointCloudDecoder(showStatistics, octreeResolution)
{

	sub_ = nh_.subscribe<std_msgs::String>("/" + sensorName + inputMsgTopic, 1, &wp3::CloudDecompressor::roscallback, this);
//	pub_ = nh_.advertise<std_msgs::String>("/" + sensorName + _TOPICOUT, 1);
}

CloudDecompressor::~CloudDecompressor(){

}

// Callback for ROS subscriber
void CloudDecompressor::roscallback(const std_msgs::String::ConstPtr& msg){

	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	compressedData << msg->data;

//	std::cout << compressedData.str() << std::endl;
	pointCloudDecoder.decodePointCloud (compressedData, decompressedCloud);

	ptfilter.setInputCloud (decompressedCloud);
	ptfilter.setFilterFieldName ("intensity");
	ptfilter.setFilterLimits (_MINI, FLT_MAX);
	ptfilter.filter (*outputCloud);

	// Publish the compressed cloud
//	outputCloud->header.frame_id = rosTransformGlobalFrame;
//	pub_.publish(outputCloud);

}

}
