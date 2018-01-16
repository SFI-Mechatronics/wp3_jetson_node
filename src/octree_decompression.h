/*
 * octree_decompression.h
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#ifndef OCTREE_DECOMPRESSION_H_
#define OCTREE_DECOMPRESSION_H_


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


typedef pcl::PointXYZI PointT_decomp;
typedef OctreePointCloudDensityContainer LeafT_decomp;
typedef OctreeContainerEmpty BranchT_decomp;
typedef Octree2BufBase<LeafT_decomp, LeafT_decomp> OctreeT_decomp;


class PointCloudDecompression : public OctreePointCloud<PointT_decomp, LeafT_decomp, BranchT_decomp, OctreeT_decomp>
{
public:
	typedef PointT_decomp PointT;
	typedef LeafT_decomp LeafT;
	typedef BranchT_decomp BranchT;
	typedef OctreeT_decomp OctreeT;

	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

	// Boost shared pointers
	typedef boost::shared_ptr<PointCloudDecompression > Ptr;
	typedef boost::shared_ptr<const PointCloudDecompression > ConstPtr;

	typedef typename OctreeT::LeafNode LeafNode;
	typedef typename OctreeT::BranchNode BranchNode;


	/** \brief Constructor
	 *
	 */
	PointCloudDecompression (
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
	virtual ~PointCloudDecompression (){

	}


	/** \brief Initialize globals */
	void initialize() {

			this->setResolution (octree_resolution_);
			point_coder_.setPrecision (static_cast<float> (point_resolution_));

	} // End initialize()


    /** \brief Provide a pointer to the output data set.
      * \param cloud_arg: the boost shared pointer to a PointCloud message
      */
    inline void setOutputCloud (const PointCloudPtr &cloud_arg)
    {
      if (output_ != cloud_arg)
      {
        output_ = cloud_arg;
      }
    }


	/** \brief Decode point cloud from input stream
	 * \param compressed_tree_data_in_arg: binary input stream containing compressed data
	 * \param cloud_arg: reference to decoded point cloud
	 */
	void decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);


    /** \brief Get the amount of points within a leaf node voxel which is addressed by a point
      * \param[in] point_arg: a point addressing a voxel
      * \return amount of points that fall within leaf node voxel
      */
	unsigned int getVoxelDensityAtPoint (const PointT& point_arg) const
	{
	  unsigned int point_count = 0;

	  OctreePointCloudDensityContainer* leaf = this->findLeafAtPoint (point_arg);

	  if (leaf)
	    point_count = leaf->getPointCounter ();

	  return (point_count);
	}

private:

	/** \brief Decode leaf nodes information during deserialization
	 * \param key_arg octree key of new leaf node
	 */
	// param leaf_arg reference to new leaf node
	virtual void deserializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);


    /** \brief Read frame information to output stream
      * \param compressed_tree_data_in_arg: binary input stream
      */
    void readFrameHeader (std::istream& compressed_tree_data_in_arg);


    /** \brief Synchronize to frame header
      * \param compressed_tree_data_in_arg: binary input stream
      */
    void syncToHeader (std::istream& compressed_tree_data_in_arg);


    /** \brief Entropy decoding of input binary stream and output to information vectors
      * \param compressed_tree_data_in_arg: binary input stream
      */
    void entropyDecoding (std::istream& compressed_tree_data_in_arg);

    /** \brief Pointer to output point cloud dataset. */
    PointCloudPtr output_;

	/** \brief Vector for storing binary tree structure */
	std::vector<char> binary_tree_data_vector_;

	/** \brief Point coding instance */
	PointCoding<PointT> point_coder_;

	/** \brief Static range coder instance */
	pcl::StaticRangeCoder entropy_coder_;

	// Settings
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

#endif /* OCTREE_DECOMPRESSION_H_ */