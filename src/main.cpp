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

#include "defines.h"
#include "compressor.h"

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudCompression Compressor;
//typedef wp3::PointCloudDecompression Decompressor;


void killHandler(int)
{
	ROS_INFO("%s","Shutdown request received.");
	ros::NodeHandle nh;
	ROS_INFO("%s","Terminating nodehandle.");
	nh.shutdown();
	ROS_INFO("%s","Terminating rosnode.");
	ros::shutdown();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, _DEFAULTNODENAME, ros::init_options::NoSigintHandler);
	ros::NodeHandle nh("~");

	signal(SIGINT, killHandler);
	signal(SIGTERM, killHandler);

	ros::Rate loopRate(_ROSRATE);

	if(!nh.hasParam("sensor_name"))
		ROS_ERROR("%s","Missing _sensor_name:=<name> parameter! Shutting down...");
	else if(!nh.hasParam("resolution"))
		ROS_ERROR("%s","Missing _resolution:=<resolution> parameter! Shutting down...");
	else {

		std::string sensorName;
		nh.getParam("sensor_name", sensorName);

		double resolution;
		nh.getParam("resolution", resolution);

		ros::Duration(1.0).sleep();

		wp3::CloudCompressor compressor(_TOPICOUT, _KINECTPOINTS, sensorName, resolution, _STATISTICS);

//		wp3::CloudDecompressor decompressor(sensorName, _TOPICOUT, false);

		while(ros::ok()){
			ros::spinOnce();
			loopRate.sleep();
		}

	} // end else if has sensorname


	ROS_INFO("%s","Shutdown complete.");
	return 0;
}
