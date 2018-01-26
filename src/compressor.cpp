/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "compressor.h"

namespace wp3 {

// Constructor
CloudCompressor::CloudCompressor(std::string outputMsgTopic, std::string inputCloudTopic, std::string rosTFLocalFrame, std::string rosTFGlobalFrame,
		double octreeResolution, unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics) :
						 transformedCloud_(new PointCloud()),
						 croppedCloud_(new PointCloud ()),
						 octreeResolution_(octreeResolution),
						 rosTFLocalFrame_(rosTFLocalFrame),
						 rosTFGlobalFrame_(rosTFGlobalFrame),
						 pointCloudEncoder_(showStatistics, octreeResolution, minPT[0], minPT[1], minPT[2], maxPT[0], maxPT[1], maxPT[2], iFrameRate)
{
	// Setup box crop filter

	crop_.setMin(minPT);
	crop_.setMax(maxPT);

	sub_ = nh_.subscribe<PointCloud>(inputCloudTopic, 1, &wp3::CloudCompressor::roscallback, this);
	pub_ = nh_.advertise<std_msgs::String>(outputMsgTopic, 1);
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
