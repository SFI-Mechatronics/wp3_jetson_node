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


void PointCloudCompression::decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg){

	// synchronize to frame header
	syncToHeader(compressed_tree_data_in_arg);

	// initialize octree
	this->switchBuffers ();
	this->setOutputCloud (cloud_arg);

	// read header from input stream
	this->readFrameHeader (compressed_tree_data_in_arg);

	// decode data vectors from stream
	this->entropyDecoding (compressed_tree_data_in_arg);

	// initialize point decoding
	point_coder_.initializeDecoding ();

	// initialize output cloud
	output_->points.clear ();
	output_->points.reserve (static_cast<std::size_t> (point_count_));

	if (i_frame_)
		// i-frame decoding - decode tree structure without referencing previous buffer
		this->deserializeTree (binary_tree_data_vector_, false);
	else
		// p-frame decoding - decode XOR encoded tree structure
		this->deserializeTree (binary_tree_data_vector_, true);

	// assign point cloud properties
	output_->height = 1;
	output_->width = static_cast<uint32_t> (cloud_arg->points.size ());
	output_->is_dense = false;

	if (b_show_statistics_)
	{
		float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);

		PCL_INFO ("*** POINTCLOUD DECODING ***\n");
		PCL_INFO ("Frame ID: %d\n", frame_ID_);
		if (i_frame_)
			PCL_INFO ("Decoding Frame: Intra frame\n");
		else
			PCL_INFO ("Decoding Frame: Prediction frame\n");
		PCL_INFO ("Number of decoded points: %ld\n", point_count_);
		PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
		PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
		PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
		PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_) / 1024.0f);
		PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ);
		PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
		PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ));
	}
} // End decodePointCloud


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


void PointCloudCompression::syncToHeader (std::istream& compressed_tree_data_in_arg)
{
	// sync to frame header
	unsigned int header_id_pos = 0;
	while (header_id_pos < strlen (frame_header_identifier_))
	{
		char readChar;
		compressed_tree_data_in_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
		if (readChar != frame_header_identifier_[header_id_pos++])
			header_id_pos = (frame_header_identifier_[0]==readChar)?1:0;
	}
} // End syncToHeader


void PointCloudCompression::readFrameHeader ( std::istream& compressed_tree_data_in_arg)
{
	// read header
	compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&frame_ID_), sizeof (frame_ID_));
	compressed_tree_data_in_arg.read (reinterpret_cast<char*>(&i_frame_), sizeof (i_frame_));
	if (i_frame_)
	{
		double min_x, min_y, min_z, max_x, max_y, max_z;
		double octree_resolution;
		double point_resolution;

		// read coder configuration
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_count_), sizeof (point_count_));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&octree_resolution), sizeof (octree_resolution));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_resolution), sizeof (point_resolution));

		// read octree bounding box
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_x), sizeof (min_x));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_y), sizeof (min_y));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_z), sizeof (min_z));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_x), sizeof (max_x));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_y), sizeof (max_y));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_z), sizeof (max_z));

		// reset octree and assign new bounding box & resolution
		this->deleteTree ();
		this->setResolution (octree_resolution);
		this->defineBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

		// configure color & point coding
		point_coder_.setPrecision (static_cast<float> (point_resolution));
	}
} // End readFrameHeader


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


void PointCloudCompression::entropyDecoding (std::istream& compressed_tree_data_in_arg)
{
	uint64_t binary_tree_data_vector_size;

	compressed_point_data_len_ = 0;

	// decode binary octree structure
	compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
	binary_tree_data_vector_.resize (static_cast<std::size_t> (binary_tree_data_vector_size));
	compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg, binary_tree_data_vector_);

} // End entropyDecoding()

int itest2 = 0;
void PointCloudCompression::serializeTreeCallback (LeafT &leaf_arg, const OctreeKey & key_arg)
{
	// reference to point indices vector stored within octree leaf
	const unsigned int density = leaf_arg.getPointCounter();

	itest2++;
	if(itest2 == 10){
		itest2 = 0;
		std::cout << density;
	}

} // End serializeTreeCallback


void PointCloudCompression::deserializeTreeCallback (LeafT&, const OctreeKey& key_arg)
{
	PointT newPoint;

	// calculate center of lower voxel corner
	newPoint.x = static_cast<float> ((static_cast<double> (key_arg.x) + 0.5) * this->resolution_ + this->min_x_);
	newPoint.y = static_cast<float> ((static_cast<double> (key_arg.y) + 0.5) * this->resolution_ + this->min_y_);
	newPoint.z = static_cast<float> ((static_cast<double> (key_arg.z) + 0.5) * this->resolution_ + this->min_z_);

	// add point to point cloud
	output_->points.push_back (newPoint);

} // End deserializeTreeCallback




} // End namespace wp3
