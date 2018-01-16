/*
 * octree_compression.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#include "octree_compression.h"

namespace wp3 {

void PointCloudCompression::encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg){

	unsigned char recent_tree_depth = static_cast<unsigned char> (this->getTreeDepth ());

	// initialize octree
	this->setInputCloud (cloud_arg);

	// add point to octree
	this->addPointsFromInputCloud ();

	// make sure cloud contains points
	if (this->leaf_count_>0) {

		// if octree depth changed, we enforce I-frame encoding
		i_frame_ |= (recent_tree_depth != this->getTreeDepth ());// | !(iFrameCounter%10);

		// enable I-frame rate
		if (i_frame_counter_++==i_frame_rate_)
		{
			i_frame_counter_ =0;
			i_frame_ = true;
		}

		// increase frameID
		frame_ID_++;

		// initialize point encoding
		point_coder_.initializeEncoding();
		point_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size() ));

		// serialize octree
		if (i_frame_)
			// i-frame encoding - encode tree structure without referencing previous buffer
			this->serializeTree (binary_tree_data_vector_, false);
		else
			// p-frame encoding - XOR encoded tree structure
			this->serializeTree (binary_tree_data_vector_, true);

		// write frame header information to stream
		this->writeFrameHeader (compressed_tree_data_out_arg);

		// apply entropy coding to the content of all data vectors and send data to output stream
		this->entropyEncoding (compressed_tree_data_out_arg);

		// prepare for next frame
		this->switchBuffers ();


		if (b_show_statistics_)
		{
			float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);

			PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
			PCL_INFO ("Frame ID: %d\n", frame_ID_);
			if (i_frame_)
				PCL_INFO ("Encoding Frame: Intra frame\n");
			else
				PCL_INFO ("Encoding Frame: Prediction frame\n");
			PCL_INFO ("Number of encoded points: %ld\n", point_count_);
			PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
			PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
			PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
			PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_) / 1024.0f);
			PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ);
			PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
			PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ));
		}

		i_frame_ = false;

	} else {
		if (b_show_statistics_)
			PCL_INFO ("Info: Dropping empty point cloud\n");
		this->deleteTree();
		i_frame_counter_ = 0;
		i_frame_ = true;
	} // End if leaf_count > 0

} // End encodePointCloud()


void PointCloudCompression::writeFrameHeader (std::ostream& compressed_tree_data_out_arg)
{
	// encode header identifier
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier_), strlen (frame_header_identifier_));
	// encode point cloud header id
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&frame_ID_), sizeof (frame_ID_));
	// encode frame type (I/P-frame)
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&i_frame_), sizeof (i_frame_));
	if (i_frame_)
	{
		double min_x, min_y, min_z, max_x, max_y, max_z;
		double octree_resolution;
		double point_resolution;

		// get current configuration
		octree_resolution = this->getResolution ();
		point_resolution= point_coder_.getPrecision ();
		this->getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

		point_count_ = this->leaf_count_;

		// encode coding configuration
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_), sizeof (point_count_));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&octree_resolution), sizeof (octree_resolution));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_resolution), sizeof (point_resolution));

		// encode octree bounding box
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x), sizeof (min_x));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y), sizeof (min_y));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z), sizeof (min_z));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x), sizeof (max_x));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y), sizeof (max_y));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z), sizeof (max_z));
	}
} // End writeFrameHeader()

void PointCloudCompression::entropyEncoding(std::ostream& compressed_tree_data_out_arg)
{
	uint64_t binary_tree_data_vector_size;
	uint64_t point_avg_color_data_vector_size;

	compressed_point_data_len_ = 0;

	// encode binary octree structure
	binary_tree_data_vector_size = binary_tree_data_vector_.size ();
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
	compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream (binary_tree_data_vector_, compressed_tree_data_out_arg);

	// flush output stream
	compressed_tree_data_out_arg.flush ();
} // End entropyEncoding()


void PointCloudCompression::serializeTreeCallback (LeafT &leaf_arg, const OctreeKey & key_arg)
    {
      // reference to point indices vector stored within octree leaf
      //const std::vector<int>& leafIdx = leaf_arg.getPointIndicesVector();


    }

} // End namespace wp3