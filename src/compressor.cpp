/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "compressor.h"

namespace wp3 {

CloudCompressor::CloudCompressor(std::string sensorName) :
				 transformedCloud(new PointCloud()),
				 croppedCloud(new PointCloud ()),
				 pointCloudEncoder(showStatistics, octreeResolution, _MINX, _MINY, _MINZ, _MAXX, _MAXY, _MAXZ, iFrameRate)
{
	rosSensorName = sensorName;
	rosTransformLocalFrame = rosSensorName + _KINECTFRAME;
	rosTransformGlobalFrame = _GLOBALFRAME;

	// Box crop filter
	minPT << _MINX, _MINY, _MINZ, 1;
	maxPT << _MAXX, _MAXY, _MAXZ, 1;
	crop.setMin(minPT);
	crop.setMax(maxPT);

	sub_ = nh_.subscribe<PointCloud>("/" + sensorName + _KINECTPOINTS, 1, &wp3::CloudCompressor::roscallback, this);
	pub_ = nh_.advertise<std_msgs::String>("/" + sensorName + _TOPICOUT, 1);
}

CloudCompressor::~CloudCompressor(){

}

// Callback for ROS subscriber
void CloudCompressor::roscallback(const PointCloud::ConstPtr &cloud){
	// Get transformation published by master
	tf::StampedTransform transform;
	try{
		tfListener.lookupTransform(rosTransformGlobalFrame, rosTransformLocalFrame, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}

	// Transform the point cloud
	pcl_ros::transformPointCloud(*cloud, *transformedCloud, transform);

	// Crop the point cloud
	crop.setInputCloud(transformedCloud);
	crop.filter(*croppedCloud);

	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	pointCloudEncoder.encodePointCloud (croppedCloud, compressedData);

		std_msgs::String msg;
		msg.data = compressedData.str();
		pub_.publish(msg);

	// Publish the compressed cloud
//	outputCloud->header.frame_id = rosTransformGlobalFrame;
//	pub_.publish(outputCloud);

}


}
