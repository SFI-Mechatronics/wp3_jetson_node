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
#include "octree_decompression.h"

// ROS defines
#define _ROSRATE 60
#define _DEFAULTNODENAME "compress_pc"
#define _GLOBALFRAME "world"
#define _KINECTFRAME "_ir_optical_frame"
#define _KINECTPOINTS "/sd/points_nocolor"
#define _TOPICOUT "/wp3/pc2_compressed"

// Compression defines
#define _STATISTICS false
#define _OCTREERESOLUTION 0.05
#define _IFRAMERATE 10

// Filter defines
#define _MINX 0.2
#define _MINY 0.2
#define _MINZ 0.2
#define _MAXX 10.0
#define _MAXY 15.0
#define _MAXZ 5.0

#define _MINI 2.0

// Typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudCompression Compressor;
typedef wp3::PointCloudDecompression Decompressor;

// ROS Subscriber
ros::Subscriber subKinect;
ros::Publisher pubCloud;
//ros::Publisher pubSerialized;

bool running = true;

class CloudHandler
{
public:
	// Constructor
	CloudHandler(std::string sensorName);
	~CloudHandler();

	// Callback for ROS subscriber
	void callback(const PointCloud::ConstPtr &cloud);

private:
	std::string rosSensorName;
	std::string rosTransformGlobalFrame;
	std::string rosTransformLocalFrame;

	// ROS transform listener
	tf::TransformListener tfListener;
	tf::StampedTransform transform;

	// Pointers to temporary point clouds
	PointCloud::Ptr transformedCloud;
	PointCloud::Ptr croppedCloud;

	PointCloudXYZI::Ptr compressedCloud;
	PointCloudXYZI::Ptr outputCloud;

	// Compression setup
	static const bool showStatistics = _STATISTICS;
	static const double octreeResolution = _OCTREERESOLUTION;
	static const unsigned int iFrameRate = _IFRAMERATE;

	Compressor* pointCloudEncoder;
	Decompressor* pointCloudDecoder;

	// Filters
	Eigen::Vector4f minPT, maxPT;
	pcl::CropBox<PointType> crop;
	pcl::PassThrough<PointType_out> ptfilter; // Initializing with true will allow us to extract the removed indices
};


CloudHandler::CloudHandler(std::string sensorName) :
 transformedCloud(new PointCloud()),
 croppedCloud(new PointCloud ()),
 compressedCloud(new PointCloudXYZI ()),
 outputCloud(new PointCloudXYZI ()),
 ptfilter(false)
{
	rosSensorName = sensorName;
	rosTransformLocalFrame = rosSensorName + _KINECTFRAME;
	rosTransformGlobalFrame = _GLOBALFRAME;

	// Compressor
	pointCloudEncoder = new Compressor(showStatistics, octreeResolution, iFrameRate);

	// Decompressor
	pointCloudDecoder = new Decompressor(showStatistics, octreeResolution);

	// Box crop filter
	minPT << _MINX, _MINY, _MINZ, 1;
	maxPT << _MAXX, _MAXY, _MAXZ, 1;
	crop.setMin(minPT);
	crop.setMax(maxPT);
}

CloudHandler::~CloudHandler(){
    delete (pointCloudEncoder);
    delete (pointCloudDecoder);
}

// Callback for ROS subscriber
void CloudHandler::callback(const PointCloud::ConstPtr &cloud){
	// Get transformation published by master
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

	pointCloudEncoder->encodePointCloud (croppedCloud, compressedData);

	pointCloudDecoder->decodePointCloud (compressedData, compressedCloud);

	ptfilter.setInputCloud (compressedCloud);
	ptfilter.setFilterFieldName ("intensity");
	ptfilter.setFilterLimits (_MINI, FLT_MAX);
	ptfilter.filter (*outputCloud);

//	std_msgs::String msg;
//	msg.data = compressedData.str();
//	pubSerialized.publish(msg);

	// Publish the compressed cloud
	outputCloud->header.frame_id = rosTransformGlobalFrame;
	pubCloud.publish(outputCloud);

}

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

	if(!nh.hasParam("sensor_name")){
		ROS_ERROR("%s","Missing sensor_name parameter!");
		nh.shutdown();
		ros::shutdown();
		return 0;
	}

	std::string sensorName;
	nh.getParam("sensor_name", sensorName);

	ros::Duration(1.0).sleep();

	CloudHandler handler(sensorName);

	subKinect = nh.subscribe<PointCloud>("/" + sensorName + _KINECTPOINTS, 1, &CloudHandler::callback, &handler);
	pubCloud = nh.advertise<PointCloudXYZI>("/" + sensorName + _TOPICOUT, 1);
//	pubSerialized = nh.advertise<std_msgs::String>("/" + sensorName + "/wp3/pc2_coded", 1);

	while(running && ros::ok()){
		ros::spinOnce();
		loopRate.sleep();
	}

	if(ros::ok()){
		nh.shutdown();
		ros::shutdown();
	}

	delete(&handler);

	return 0;
}
