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

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace pcl::octree;

namespace wp3 {

typedef pcl::PointXYZ PointT_comp;
typedef OctreePointCloudDensityContainer LeafT_comp;
typedef OctreeContainerEmpty BranchT_comp;
typedef Octree2BufBase<LeafT_comp, BranchT_comp> OctreeT_comp;

class PointCloudCompression : public OctreePointCloud<PointT_comp, LeafT_comp, BranchT_comp, OctreeT_comp>
{
public:
	typedef PointT_comp PointT;
	typedef LeafT_comp LeafT;
	typedef BranchT_comp BranchT;
	typedef OctreeT_comp OctreeT;

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
			const double octreeResolution_arg = 0.01,
			const unsigned int iFrameRate_arg = 30 ):
				OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (octreeResolution_arg),
				//point_coder_ (),
				entropy_coder_ (),
				i_frame_rate_ (iFrameRate_arg),
				i_frame_counter_ (0),
				frame_ID_ (0),
				point_count_ (0),
				octree_resolution_(octreeResolution_arg),
				i_frame_ (true),
				b_show_statistics_ (showStatistics_arg),
				pointIntensityVector_ (){

		frame_header_identifier_ = "<WP3-OCT-COMPRESSED>";
	} // End Constructor


	/** \brief Empty deconstructor. */
	virtual ~PointCloudCompression (){

	}


	/** \brief Initialize globals */
	void initialize() {

			this->setResolution (octree_resolution_);

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


	/** \brief Encode point cloud to output stream
	 * \param cloud_arg:  point cloud to be compressed
	 * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
	 */
	void encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);


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


	virtual void serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);


	/** \brief Write frame information to output stream
	 * \param compressed_tree_data_out_arg: binary output stream
	 */
	void writeFrameHeader (std::ostream& compressed_tree_data_out_arg);

	/** \brief Apply entropy encoding to encoded information and output to binary stream
	 * \param compressed_tree_data_out_arg: binary output stream
	 */
	void entropyEncoding(std::ostream& compressed_tree_data_out_arg);

    /** \brief Pointer to output point cloud dataset. */
    PointCloudPtr output_;

	/** \brief Vector for storing binary tree structure */
	std::vector<char> binary_tree_data_vector_;

	/** \brief Vector for storing point intensity information  */
    std::vector<char> pointIntensityVector_;

//	/** \brief Point coding instance */
//	PointCoding<PointT> point_coder_;

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

	const double octree_resolution_;

	//header
	const char* frame_header_identifier_;


};

}

#endif /* OCTREE_COMPRESSION_H_ */
