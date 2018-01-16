/*
 * custom_octree.h
 *
 *  Created on: Jan 15, 2018
 *      Author: joacim
 */

#ifndef CUSTOM_OCTREE_H_
#define CUSTOM_OCTREE_H_

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
//#include <pcl/octree/octree2buf_base.h>
//#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_density.h>

using namespace pcl::octree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Custom Octree pointcloud class
 *  \note
 *  \note typename: PointT: type of point used in pointcloud
 *  \author Joacim Dybedal
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT,
//typename LeafT = OctreePointCloudDensityContainer,
typename LeafT = OctreeContainerPointIndices,
typename BranchT = OctreeContainerEmpty,
typename OctreeT = Octree2BufBase<LeafT, BranchT> >
class CustomOctree : public OctreePointCloud<PointT, LeafT, BranchT, OctreeT >
{

public:

	/** \brief Constructor.
	 *  \param resolution_arg:  octree resolution at lowest octree level
	 * */
	CustomOctree (const double resolution_arg) :
		OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (resolution_arg)
		{
		}

	/** \brief Empty class constructor. */
	virtual ~CustomOctree ()
	{
	}

	/** \brief Get a indices from all leaf nodes that did not exist in previous buffer.
	 * \param indicesVector_arg: results are written to this vector of int indices
	 * \param minPointsPerLeaf_arg: minimum amount of points required within leaf node to become serialized.
	 * \return number of point indices
	 */
//	std::size_t getPointIndicesFromNewVoxels (std::vector<int> &indicesVector_arg,
//			const int minPointsPerLeaf_arg = 0)
//	{
//
//		std::vector<OctreeContainerPointIndices*> leaf_containers;
//		this->serializeNewLeafs (leaf_containers);
//
//		std::vector<OctreeContainerPointIndices*>::iterator it;
//		std::vector<OctreeContainerPointIndices*>::const_iterator it_end = leaf_containers.end();
//
//		for (it=leaf_containers.begin(); it!=it_end; ++it)
//		{
//			if ((*it)->getSize()>=minPointsPerLeaf_arg)
//				(*it)->getPointIndices(indicesVector_arg);
//		}
//
//		return (indicesVector_arg.size ());
//	}
};

#endif /* CUSTOM_OCTREE_H_ */
