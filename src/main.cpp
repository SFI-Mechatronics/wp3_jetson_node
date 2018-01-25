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
#include "decompressor.h"

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudCompression Compressor;
typedef wp3::PointCloudDecompression Decompressor;

// ROS Subscriber

//ros::Publisher pubSerialized;

bool running = true;



void killHandler(int)
{
	ROS_INFO("%s","Shutdown request received. Terminating node.");
	running = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, _DEFAULTNODENAME, ros::init_options::NoSigintHandler);
	ros::NodeHandle nh("~");

	signal(SIGINT, killHandler);
	signal(SIGTERM, killHandler);

	ros::Rate loopRate(_ROSRATE);

	PointCloudXYZI::Ptr output_clout(new PointCloudXYZI);

	if(!nh.hasParam("sensor_name"))
		ROS_ERROR("%s","Missing _sensor_name:=<name> parameter! Shutting down...");
	else {

		std::string sensorName;
		nh.getParam("sensor_name", sensorName);

		ros::Duration(1.0).sleep();

		wp3::CloudCompressor compressor(sensorName, _KINECTPOINTS, _TOPICOUT);

		wp3::CloudDecompressor decompressor(output_clout, sensorName, _TOPICOUT, false);


		while(running && ros::ok()){
			ros::spinOnce();
			loopRate.sleep();
		}

	} // end else if has sensorname

	nh.shutdown();
	ros::shutdown();

	return 0;
}
