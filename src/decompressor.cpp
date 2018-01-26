/*
 * decompressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "decompressor.h"

namespace wp3 {

CloudDecompressor::CloudDecompressor(std::string outputCloudTopic, std::string inputMsgTopic, std::string sensorName, const float intensityLimit, const bool showStatistics) :
				 decompressedCloud_(new PointCloudXYZI ()),
				 outputCloud_(new PointCloudXYZI ()),
				 ptfilter_(false),
				 intensityLimit_(intensityLimit),
				 pointCloudDecoder_(showStatistics)
{
	sub_ = nh_.subscribe<std_msgs::String>("/" + sensorName + inputMsgTopic, 1, &wp3::CloudDecompressor::roscallback, this);
	pub_ = nh_.advertise<PointCloudXYZI>("/" + sensorName + outputCloudTopic, 1);
}

CloudDecompressor::~CloudDecompressor(){

}

// Callback for ROS subscriber
void CloudDecompressor::roscallback(const std_msgs::String::ConstPtr& msg){

	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	compressedData << msg->data;

//	std::cout << compressedData.str() << std::endl;
	pointCloudDecoder_.decodePointCloud (compressedData, decompressedCloud_);

	ptfilter_.setInputCloud (decompressedCloud_);
	ptfilter_.setFilterFieldName ("intensity");
	ptfilter_.setFilterLimits (intensityLimit_, FLT_MAX);
	ptfilter_.filter (*outputCloud_);

	// Publish the compressed cloud
	outputCloud_->header.frame_id = _GLOBALFRAME;
	pub_.publish(outputCloud_);

}

}
