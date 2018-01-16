/*
 * octree_compression.h
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#ifndef OCTREE_COMPRESSION_H_
#define OCTREE_COMPRESSION_H_


#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/impl/octree2buf_base.hpp>

#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/impl/octree_pointcloud.hpp>

#include <pcl/compression/entropy_range_coder.h>
#include "point_coding.h"

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace pcl::octree;

namespace wp3 {

typedef pcl::PointXYZ PointT;
typedef OctreePointCloudDensityContainer LeafT;
typedef OctreeContainerEmpty BranchT;
typedef Octree2BufBase<LeafT, BranchT> OctreeT;

class PointCloudCompression : public OctreePointCloud<PointT, LeafT, BranchT, OctreeT>
{
public:


	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

	// Boost shared pointers
	typedef boost::shared_ptr<PointCloudCompression > Ptr;
	typedef boost::shared_ptr<const PointCloudCompression > ConstPtr;

	typedef typename OctreeT::LeafNode LeafNode;
	typedef typename OctreeT::BranchNode BranchNode;



	/** \brief Constructor
	 *
	 */
	PointCloudCompression (
			bool showStatistics_arg = false,
			const double pointResolution_arg = 0.001,
			const double octreeResolution_arg = 0.01,
			const unsigned int iFrameRate_arg = 30 ):
				OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (octreeResolution_arg),
				point_coder_ (),
				entropy_coder_ (),
				i_frame_rate_ (iFrameRate_arg),
				i_frame_counter_ (0),
				frame_ID_ (0),
				point_count_ (0),
				point_resolution_(pointResolution_arg),
				octree_resolution_(octreeResolution_arg),
				i_frame_ (true),
				b_show_statistics_ (showStatistics_arg){

		frame_header_identifier_ = "<WP3-OCT-COMPRESSED>";
	} // End Constructor

	/** \brief Empty deconstructor. */
	virtual ~PointCloudCompression (){

	}

	/** \brief Initialize globals */
	void initialize() {

			this->setResolution (octree_resolution_);
			point_coder_.setPrecision (static_cast<float> (point_resolution_));

	} // End initialize()


	/** \brief Encode point cloud to output stream
	 * \param cloud_arg:  point cloud to be compressed
	 * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
	 */
	void encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);

	/** \brief Decode point cloud from input stream
	 * \param compressed_tree_data_in_arg: binary input stream containing compressed data
	 * \param cloud_arg: reference to decoded point cloud
	 */
	void decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);

	/** \brief Apply entropy encoding to encoded information and output to binary stream
	 * \param compressed_tree_data_out_arg: binary output stream
	 */
	void entropyEncoding(std::ostream& compressed_tree_data_out_arg);

	virtual void serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);

	/** \brief Decode leaf nodes information during deserialization
	 * \param key_arg octree key of new leaf node
	 */
	// param leaf_arg reference to new leaf node
	//virtual void deserializeTreeCallback (LeafT&, const OctreeKey& key_arg);

private:

	/** \brief Write frame information to output stream
	 * \param compressed_tree_data_out_arg: binary output stream
	 */
	void writeFrameHeader (std::ostream& compressed_tree_data_out_arg);

	/** \brief Vector for storing binary tree structure */
	std::vector<char> binary_tree_data_vector_;

	/** \brief Point coding instance */
	PointCoding<PointT> point_coder_;

	/** \brief Static range coder instance */
	pcl::StaticRangeCoder entropy_coder_;

	uint32_t i_frame_rate_;
	uint32_t i_frame_counter_;
	uint32_t frame_ID_;
	uint64_t point_count_;
	bool i_frame_;

	//bool activating statistics
	bool b_show_statistics_;
	uint64_t compressed_point_data_len_;

	const double point_resolution_;
	const double octree_resolution_;

	//header
	const char* frame_header_identifier_;

};

}

#endif /* OCTREE_COMPRESSION_H_ */
