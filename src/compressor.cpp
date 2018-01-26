/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "compressor.h"

namespace wp3 {

// Constructor
CloudCompressor::CloudCompressor(std::string outputMsgTopic, std::string inputCloudTopic, std::string sensorName, double resolution, bool showStatistics) :
						 transformedCloud_(new PointCloud()),
						 croppedCloud_(new PointCloud ()),
						 octreeResolution_(resolution),
						 pointCloudEncoder_(showStatistics, resolution, _MINX, _MINY, _MINZ, _MAXX, _MAXY, _MAXZ, _IFRAMERATE)
{
	rosTFLocalFrame_ = sensorName + _KINECTFRAME;
	rosTFGlobalFrame_ = _GLOBALFRAME;

	// Setup box crop filter
	minPT_ << _MINX, _MINY, _MINZ, 1;
	maxPT_ << _MAXX, _MAXY, _MAXZ, 1;
	crop_.setMin(minPT_);
	crop_.setMax(maxPT_);

	sub_ = nh_.subscribe<PointCloud>("/" + sensorName + inputCloudTopic, 1, &wp3::CloudCompressor::roscallback, this);
	pub_ = nh_.advertise<std_msgs::String>("/" + sensorName + outputMsgTopic, 1);
}

// Callback for ROS subscriber
void CloudCompressor::roscallback(const PointCloud::ConstPtr &cloud){
	// Get transformation published by master
	tf::StampedTransform transform;
	try{
		tfListener_.lookupTransform(rosTFGlobalFrame_, rosTFLocalFrame_, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}

	// Transform the point cloud
	pcl_ros::transformPointCloud(*cloud, *transformedCloud_, transform);

	// Crop the point cloud
	crop_.setInputCloud(transformedCloud_);
	crop_.filter(*croppedCloud_);

	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	// Encode point cloud to stream
	pointCloudEncoder_.encodePointCloud (croppedCloud_, compressedData);

	// Publish the encoded stream
	std_msgs::String msg;
	msg.data = compressedData.str();
	pub_.publish(msg);

}


}
