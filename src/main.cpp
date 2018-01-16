#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <signal.h>

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>

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
#include <pcl/compression/octree_pointcloud_compression.h>

//include "custom_octree.h"

typedef pcl::octree::OctreeContainerPointIndex LeafType;
typedef pcl::octree::OctreeContainerEmpty BranchType;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
 typedef pcl::io::OctreePointCloudCompression<pcl::PointXYZ> Compressor;

//typedef pcl::octree::OctreePointCloudDensityContainer LeafType;

//typedef pcl::octree::Octree2BufBase<LeafType, BranchType> OctreeBaseType;
//typedef pcl::octree::OctreePointCloud<PointCloud, LeafType, BranchType, OctreeBaseType> Octree;


// ROS Subscriber
ros::Subscriber subKinect;
ros::Publisher pubCloud;
//ros::Publisher pubSerialized;

bool running = true;

const double resolution = 0.05;

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
	PointCloud::Ptr compressedCloud;


	// Compression setup
	static const bool showStatistics = true;
	static const double pointResolution = 0.01;
	static const double octreeResolution = 0.05;
	static const bool doVoxelGridDownDownSampling = true;
	static const unsigned int iFrameRate = 10;
	static const bool doColorEncoding = false;
	static const unsigned char colorBitResolution = 8;
	static const pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;

	Compressor* pointCloudEncoder;
	Compressor* pointCloudDecoder;

	Eigen::Vector4f minPT, maxPT;
	pcl::CropBox<pcl::PointXYZ> crop;
};


CloudHandler::CloudHandler(std::string sensorName) :
 transformedCloud(new PointCloud()),
 croppedCloud(new PointCloud ()),
 compressedCloud(new PointCloud ())
{
	rosSensorName = sensorName;
	rosTransformLocalFrame = rosSensorName + "_ir_optical_frame";
	rosTransformGlobalFrame = "world";

	// compress point cloud
	pointCloudEncoder = new Compressor(compressionProfile, showStatistics,
			pointResolution, octreeResolution, doVoxelGridDownDownSampling,
			iFrameRate, doColorEncoding, colorBitResolution);

	pointCloudDecoder = new Compressor();

	minPT << 0.2, 0.2, 0.2, 1;
	maxPT << 10, 15, 5, 1;

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

//	CustomOctree<pcl::PointXYZ> octree(resolution);
//	octree.defineBoundingBox(0.2, 0.2, 0.2, 10, 15, 5);
//	octree.setInputCloud (croppedCloud);
//	octree.addPointsFromInputCloud ();
//
	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	pointCloudEncoder->deleteTree();
	pointCloudEncoder->setResolution(0.05);
	pointCloudEncoder->defineBoundingBox(0.2, 0.2, 0.2, 10, 15, 5);
	pointCloudEncoder->encodePointCloud (croppedCloud, compressedData);
	std::cout << compressedData.tellp() << "\n" << endl;
	pointCloudDecoder->decodePointCloud (compressedData, compressedCloud);

//	std_msgs::String msg;
//	msg.data = compressedData.str();
//	pubSerialized.publish(msg);

	// Publish the compressed cloud
	compressedCloud->header.frame_id = rosTransformGlobalFrame;
	pubCloud.publish(compressedCloud);

}

void killHandler(int)
{
	ROS_INFO("%s","Shutdown request received. Terminating node.");
	running = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "compress_pc", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh("~");

	signal(SIGINT, killHandler);
	signal(SIGTERM, killHandler);

	ros::Rate loopRate(20);

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

	subKinect = nh.subscribe<PointCloud>("/" + sensorName + "/sd/points_nocolor", 1, &CloudHandler::callback, &handler);
	pubCloud = nh.advertise<PointCloud>("/" + sensorName + "/wp3/pc2_compressed", 1);
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
