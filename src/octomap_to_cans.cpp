/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "octomap_msgs/Octomap.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h> // deserialize octreemsg withh msgToMap
#include <math.h>
#include "moveit_msgs/PlanningScene.h"
#include <stack> 
#include <unordered_map>
#include <unordered_set>
#include "std_msgs/Float64.h"
#include <assert.h>
#include <iostream>
#include "filter_octomap/table.h"
#include "filter_octomap/can.h"
#include "filter_octomap/cans.h"
#include <assert.h>
#include <Eigen/Dense>

#include <octomap/ColorOcTree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef pcl::PointXYZ PointT;

// PCL stuff


#define THRESHOLD_CAN 1000 //255.5
#define MIN_CAN_SCORE 50000
#define MIN_CAN_RADIUS 0.02
#define MIN_CAN_HEIGHT 0.1
#define MAX_CAN_HEIGHT 0.27

/* Debuging*/
//#define DEBUG_COUT 1
#define DEBUG 1
#ifdef DEBUG
	#include "visualization_msgs/Marker.h"
	#include "geometry_msgs/Point.h"
	#define STEP_SIZE 0.02 // Size of cells on lowest level
	ros::Publisher vis_pub;
	visualization_msgs::Marker table_marker;
	float START_X, START_Y, START_Z;
	
	void resetMarker() {
		table_marker;
		table_marker.points.clear();
		table_marker.header.frame_id = "world";
		table_marker.ns = "my_namespace";
		table_marker.action = visualization_msgs::Marker::ADD;
		table_marker.pose.orientation.x = 0.0;
		table_marker.pose.orientation.y = 0.0;
		table_marker.pose.orientation.z = 0.0;
		table_marker.pose.orientation.w = 1.0; 
		table_marker.pose.position.x = 0.0;
		table_marker.pose.position.y = 0.0;
		table_marker.pose.position.z = 0.0;
		table_marker.scale.x = 0.01;
		table_marker.scale.y = 0.01;
		table_marker.scale.z = 0.01;
		table_marker.color.a = 1.0; // Don't forget to set the alpha!
		table_marker.color.r = 1.0;
		table_marker.color.g = 0.0;
		table_marker.color.b = 0.0;
		table_marker.lifetime  = ros::Duration(10.);
	}
#else
	/*Deploy*/
	#define STEP_SIZE 0.02 // Size of cells on lowest level
#endif	

ros::Publisher canPublisher;

template <typename T> int sgn(T val) { // https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
    return (T(0) < val) - (val < T(0));
}

struct Index3D {
	size_t x; size_t y; size_t z;
	
	Index3D(size_t x, size_t y, size_t z) // Explicit ctor to use emplace
        : x(x), y(y), z(z) 
    {}
    
    friend std::ostream & operator << (std::ostream &out, const Index3D& i) // For printing to commandline
	{
		out << i.x << "," << i.y << "," << i.z;
		return out;
	}
    
    inline bool operator==(const Index3D& other) const { // For usage in unordered_map
        return (this->x == other.x) && (this->y == other.y) && (this->z == other.z);
    }
    
    // Neighbour indices when looking along the x axis wit z to the top
    Index3D right() const{
		return Index3D(this->x, this->y-1, this->z);
	}
	
	Index3D left() const{
		return Index3D(this->x-1, this->y+1, this->z);
	}
	
	Index3D behind() const{
		return Index3D(this->x+1, this->y, this->z);
	}
	
	Index3D before() const{
		return Index3D(this->x-1, this->y, this->z);
	}
	
	Index3D bellow() const{
		return Index3D(this->x, this->y, this->z-1);
	}
	
	Index3D above() const{
		return Index3D(this->x, this->y, this->z+1);
	}
};

namespace std { // Hash function for Index3D
    template<>
    struct hash<Index3D> {
        inline size_t operator()(const Index3D& x) const {
            return x.x ^ x.y ^ x.z;
        }
    };
}

typedef Index3D VoxelIndex;
typedef Index3D Size3D;
typedef std::unordered_map<VoxelIndex, double> VoxelindexToScore;
typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector3d Vector3D;

template <class T> class Array3D { // from: https://stackoverflow.com/questions/2178909/how-to-initialize-3d-array-in-c
    size_t m_width, m_height, m_here, m_size;
    std::vector<T> m_data;
  public:
    Array3D(size_t x, size_t y, size_t z, T init = 0):
      m_width(x), m_height(y), m_data(x*y*z, init), m_here(0), m_size(x*y*z)
    {}
    Array3D(Size3D size, T init = 0):
      Array3D(size.x, size.y, size.z, init)
    {}
    T& operator()(size_t x, size_t y, size_t z) {
		assert(x < m_width);
		assert(y < m_height);
		assert(x*y*z < m_size);
        return m_data.at(x + y * m_width + z * m_width * m_height);
    }
    T& operator ()(Index3D i) {
		return (*this)(i.x, i.y, i.z);
	}
    T& operator()(size_t index) {
		return m_data.at(index);
	}
    T& operator()() {
		return m_data.at(m_here);
	}
	T& operator++() {
		m_here++;
		m_here%=(m_width * m_height);
		return m_data.at(m_here);
	}
};

typedef std::tuple<double, u_int8_t, u_int8_t, u_int8_t> RGBVoxel;
typedef Array3D<RGBVoxel> RGBVoxelgrid; // Each voxel with occupancy probability and RGB color channels
typedef Array3D<float> ScoreVoxelgrid;
typedef std::unordered_set<VoxelIndex> VoxelList;

void filterBoundary(VoxelList& object, VoxelList& boundary, RGBVoxelgrid& map) {
	std::cout << "Object points: " << object.size() << std::endl;
	for(VoxelIndex it: object) {
		//std::cout << "Looking at " << it << "; neighbors: ";
		//std::cout << std::get<0>(map(it.left())) << "," << std::get<0>(map(it.right())) << "," << std::get<0>(map(it.before())) << "," << std::get<0>(map(it.behind())) << ",";
		bool has_left_neighbour = (object.find(it.left()) != object.end());
		bool has_right_neighbour = (object.find(it.right()) != object.end());
		bool has_before_neighbour = (object.find(it.before()) != object.end());
		bool has_behind_neighbour = (object.find(it.behind()) != object.end());
		//std::cout << "; has_neigh: " << has_left_neighbour << "," << has_right_neighbour << "," << has_before_neighbour << "," << has_behind_neighbour << std::endl;
		//if(!has_left_neighbour && std::get<0>(map(it.left())) < 0.5 && std::get<0>(map(it.left())) > -0.5 ) {
			//has_left_neighbour = true;
		//}
		//if(!has_right_neighbour && std::get<0>(map(it.right())) < 0.5 && std::get<0>(map(it.right())) > -0.5 ) {
			//has_right_neighbour = true;
		//}
		//if(!has_before_neighbour && std::get<0>(map(it.before())) < 0.5 && std::get<0>(map(it.before())) > -0.5 ) {
			//has_before_neighbour = true;
		//}
		//if(!has_behind_neighbour && std::get<0>(map(it.behind())) < 0.5 && std::get<0>(map(it.behind())) > -0.5 ) {
			//has_behind_neighbour = true;
		//}
		if(has_left_neighbour && has_right_neighbour && has_before_neighbour && has_behind_neighbour) {
			//std::cout << "skip\n";
			continue;
		}
		else
			boundary.insert(it);
	}
	std::cout << "Boundary points: " << boundary.size() << std::endl;
}

void floodfill(VoxelindexToScore& candidates, const VoxelIndex start, VoxelList& object) {
	// Puts all indices from candidates into object if they are connected to start via the 6 Neighbourhood; removes moved indices from candidates
	#ifdef DEBUG_COUT
	std::cout << "Starting at " << start.x << "," << start.y << "," << start.z << "\n";
	#endif
	std::stack<VoxelIndex> posToLookAt;
	posToLookAt.emplace(start);
	while(!posToLookAt.empty()) {
		VoxelIndex here = posToLookAt.top();
		#ifdef DEBUG_COUT
		std::cout << "\nStack size: " << posToLookAt.size() << "\n";
		std::cout << "Looking at " << here.x << "," << here.y << "," << here.z << "\n";
		#endif
		posToLookAt.pop();
		if(candidates.find(here)!=candidates.end()) { // Still in candidate list, not yet considered
			object.insert(here);
			candidates.erase(here);
			// Test neighbours
			// y
			if(candidates.find(here.right())!=candidates.end())
				posToLookAt.emplace(here.right());
			if(candidates.find(here.left())!=candidates.end())
				posToLookAt.emplace(here.left());
			// x
			if(candidates.find(here.behind())!=candidates.end())
				posToLookAt.emplace(here.behind());
			if(candidates.find(here.before())!=candidates.end())
				posToLookAt.emplace(here.before());
			// z
			if(candidates.find(here.bellow())!=candidates.end())
				posToLookAt.emplace(here.bellow());
			if(candidates.find(here.above())!=candidates.end())
				posToLookAt.emplace(here.above());
		}
		else { // Was already removed from candidates via another route --> ignore
			continue;
		}
	}
}

void floodfill(ScoreVoxelgrid& map, VoxelIndex start, Array3D<u_int8_t>& explored, double threshold, const Size3D size) {
	// Marks all places in explored true which are above the threshold in map and are connected to the first x,y,z via 6-Neighbourhood
	#ifdef DEBUG_COUT
	std::cout << "Starting at " << start.x << "," << start.y << "," << start.z << "\n";
	#endif
	std::stack<VoxelIndex> posToLookAt;
	posToLookAt.emplace(start);
	while(!posToLookAt.empty()) {
		VoxelIndex here = posToLookAt.top();
		#ifdef DEBUG_COUT
		std::cout << "\nStack size: " << posToLookAt.size() << "\n";
		std::cout << "Looking at " << here.x << "," << here.y << "," << here.z << "\n";
		#endif
		posToLookAt.pop();
		if(explored(here)!=0) continue;
		else {
			explored(here) = 1;
			#ifdef DEBUG_COUT
			std::cout << "Is new; set to " << explored(here) << "; local tableness: " << map(here) << "\n";
			#endif
			// Test neighbours and call floodfill if tableness high enough
			if(here.x>0) {
				if(map(here.x-1,here.y,here.z) > threshold) { 
					posToLookAt.emplace(here.x-1,here.y,here.z); 
					#ifdef DEBUG_COUT
					std::cout << "-x inserted ";
					#endif
					}
				}
			if(here.y>0) {
				if(map(here.x,here.y-1,here.z) > threshold) {
					posToLookAt.emplace(here.x,here.y-1,here.z);
					#ifdef DEBUG_COUT
					std::cout << "-y inserted ";
					#endif
				}
			}
			if(here.z>0) {
				if(map(here.x,here.y,here.z-1) > threshold) {
					posToLookAt.emplace(here.x,here.y,here.z-1);
					#ifdef DEBUG_COUT
					std::cout << "-z inserted ";
					#endif
				}
			}
			if(here.x<size.x-1) {
				if(map(here.x+1,here.y,here.z) > threshold) {
					posToLookAt.emplace(here.x+1,here.y,here.z);
					#ifdef DEBUG_COUT
					std::cout << "+x inserted ";
					#endif
				}
			}
			if(here.y<size.y-1) {
				if(map(here.x,here.y+1,here.z) > threshold) {
					posToLookAt.emplace(here.x,here.y+1,here.z);
					#ifdef DEBUG_COUT
					std::cout << "+y inserted ";
					#endif
				}
			}
			if(here.z<size.z-1) {
				if(map(here.x,here.y,here.z+1) > threshold) { 
					posToLookAt.emplace(here.x,here.y,here.z+1);
					#ifdef DEBUG_COUT
					std::cout << "+z inserted ";
					#endif
				}
			}
		}
	}
}

inline double canScore(RGBVoxelgrid& map, VoxelIndex i) {
	return (std::get<0>(map(i))+2.5) * // 0 if free, max if certainly occupied
		(1.*std::get<1>(map(i))+ 		// R as big as possible
		255.-1.*std::get<2>(map(i)));	// G as small as possible
		// B not used and ignored
}

void generateCanCandidates(RGBVoxelgrid& map, VoxelindexToScore& candidates, const Size3D& size, double threshold) {
	// Looks at all positions in map (of size size) and notes those positions where canScore is bigger threshold into the candidate map
	for(size_t i = 0; i < size.x; i++) // over x
    {
		for(size_t j = 0; j < size.y; j++) // over y
        {
            for(size_t k = 1; k < size.z; k++) // over z (0 ignored as full of table artifacts)
            {
				float score = canScore(map, {i,j,k});
				if(score > threshold) {
					//here = VoxelIndex(i,j,k);
					candidates.emplace(VoxelIndex(i,j,k), score);
				}
			}
		}
	}
}

VoxelIndex getMax(VoxelindexToScore& candidates) {
	// returns the key in candidates that has the bigges value (here: score)
	double biggest = -1.;
	VoxelIndex ret(0,0,0); 
	for (auto& it: candidates) {
		if(it.second > biggest) {
			biggest = it.second;
			ret = it.first;
		}
	}
	return ret;
}

/** \brief Fits a cylinder to the voxels specified in points
 * 
 * \param count Number of cylinder for debug plotting
 * \param nearestNeighbourNormal Number of neighbours to consider for the normal estimation -> have only a few 10 to 100!
 * \param normalDistanceWeight trade-off between normal orientation and point position in error
 * \param RANSACmaxIter Limit RANSAC iterations reducing runtime
 * \param CylLowerRad Hard lower limit on the allowed cylinder radius in [m]
 * \param CylUpperRad Hard upper limit on the allowed cylinder radius in [m]
 * \param expectedNormal Prior knowledge of the expected cylinder axis
 * \param allowedDevFromExpNormal Allowed deviation from expected cylinder axis in [°]
 * 
 * */
bool pclCylinderFit(VoxelList& points, double& radius, Vector3D& normal,Point3D& pointOnAxis, u_int32_t count,
				u_int32_t nearestNeighbourNormal = 8, 
				double normalDistanceWeight = 0.1, 
				u_int32_t RANSACmaxIter = 1000,
				double RANSACmaxDistThres = 0.02, 
				double CylLowerRad = 0.01, double CylUpperRad = 0.1,
				Vector3D expectedNormal = Vector3D(0., 0., 1.),  
				double allowedDevFromExpNormal = 10.) { 
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>(1,1));
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	for (auto& it: points) {
		cloud_filtered->push_back(PointT(START_X + it.x * STEP_SIZE, START_Y + it.y * STEP_SIZE, START_Z + it.z * STEP_SIZE));
	}
	
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_filtered);
	ne.setKSearch (nearestNeighbourNormal); 
	ne.compute (*cloud_normals);
	
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (normalDistanceWeight); 
	seg.setMaxIterations (RANSACmaxIter);
	seg.setDistanceThreshold (RANSACmaxDistThres); 
	seg.setRadiusLimits (CylLowerRad, CylUpperRad); 
	seg.setInputCloud (cloud_filtered);
	seg.setInputNormals (cloud_normals);
	seg.setEpsAngle(allowedDevFromExpNormal/180.*3.141); // See: https://answers.ros.org/question/61811/pcl-sacsegmentation-setaxis-and-setmodeltype-has-no-effect-in-output/
	seg.setAxis( expectedNormal.cast<float>() );
	
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
	seg.segment (*inliers_cylinder, *coefficients_cylinder);
	
	if(coefficients_cylinder->values.size() == 7) {
		if(abs(coefficients_cylinder->values[2]) < 0.1) {
			std::cout << "z of Normal to small\n";
			return false;
		}
		while(abs(coefficients_cylinder->values[2]) > 2) {
			coefficients_cylinder->values[0] += -1.*sgn(coefficients_cylinder->values[5])*sgn(coefficients_cylinder->values[2])*coefficients_cylinder->values[3];
			coefficients_cylinder->values[1] += -1.*sgn(coefficients_cylinder->values[5])*sgn(coefficients_cylinder->values[2])*coefficients_cylinder->values[4];
			coefficients_cylinder->values[2] += -1.*sgn(coefficients_cylinder->values[5])*sgn(coefficients_cylinder->values[2])*coefficients_cylinder->values[5];
		}
		
		pointOnAxis(0) = coefficients_cylinder->values[0];
		pointOnAxis(1) = coefficients_cylinder->values[1];
		pointOnAxis(2) = coefficients_cylinder->values[2];
		normal(0) = coefficients_cylinder->values[3];
		normal(1) = coefficients_cylinder->values[4];
		normal(2) = coefficients_cylinder->values[5];
		radius = coefficients_cylinder->values[6];
		
		#ifdef DEBUG		
		std::cout << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
		resetMarker();
		table_marker.type = visualization_msgs::Marker::CYLINDER;
		table_marker.id = count+1000;
		table_marker.scale.x = coefficients_cylinder->values[6]*2;
		table_marker.scale.y = coefficients_cylinder->values[6]*2;
		table_marker.scale.z = 10.;
		table_marker.pose.position.x = coefficients_cylinder->values[0];
		table_marker.pose.position.y = coefficients_cylinder->values[1];
		table_marker.pose.position.z = coefficients_cylinder->values[2];
		//https://math.stackexchange.com/questions/60511/quaternion-for-an-object-that-to-point-in-a-direction
		tf2::Quaternion myQuaternion;
		double theta = std::acos(coefficients_cylinder->values[5]); // = acos(n_z)
		myQuaternion.setRotation(tf2::Vector3(-1.*coefficients_cylinder->values[4], coefficients_cylinder->values[3], 0), theta);
		table_marker.pose.orientation = tf2::toMsg(myQuaternion);
		table_marker.header.stamp = ros::Time();
		table_marker.ns = "CylinderFitPCL";
		vis_pub.publish( table_marker );
		#endif
		return true;
	}	
	return false;
}

/* Concept for finding cans
 * 
 * Where to look? -> above table resulting from visual and tactile exploration --> in area from x/y_min till x/y_max and up to ~40 cm above
 * 				  -> changes boundary for changeToGrid
 * 
 * What to search for? -> contigous parts of can color, so red (high first channel, low second and third) + occupancy --> canScore
 * 
 * What to send? -> radii, heights, centerpoints
 * 
 * */ 
 
void findCans(RGBVoxelgrid& map, const Size3D size) {
	// Finds cans in table and publishes their positions and parameters
	VoxelindexToScore candidates;
	generateCanCandidates(map, candidates, size, THRESHOLD_CAN);
	
	#ifdef DEBUG // Display can points
	resetMarker();
	table_marker.type = visualization_msgs::Marker::POINTS;
	table_marker.id = 0;
	for (auto& it: candidates) {
		geometry_msgs::Point p;
		p.x = START_X + it.first.x * STEP_SIZE;
		p.y = START_Y + it.first.y * STEP_SIZE;
		p.z = START_Z + it.first.z * STEP_SIZE;
		table_marker.points.push_back(p);
	}
	table_marker.ns = "Candidates";
	table_marker.header.stamp = ros::Time();
	vis_pub.publish( table_marker );
	#endif
	
	filter_octomap::cans cans_msg;
	u_int32_t count=0;
	while(!candidates.empty()) {
		std::cout << "Best node at: " << getMax(candidates) << " with score " << candidates[getMax(candidates)] << std::endl;
		
		VoxelList can;
		floodfill(candidates, getMax(candidates), can);
		VoxelList boundary;
		filterBoundary(can, boundary, map);
		
		#ifdef DEBUG // Display Boundary
		resetMarker();
		table_marker.type = visualization_msgs::Marker::POINTS;
		table_marker.color.r = 0.0;
		table_marker.color.g = 0.0;
		table_marker.color.b = 1.0;
		table_marker.id = count+6;
		for (auto& it: boundary) {
			geometry_msgs::Point p;
			p.x = START_X + it.x * STEP_SIZE;
			p.y = START_Y + it.y * STEP_SIZE;
			p.z = START_Z + it.z * STEP_SIZE;
			table_marker.points.push_back(p);
		}
		table_marker.ns = "Boundary";
		table_marker.header.stamp = ros::Time();
		vis_pub.publish( table_marker );
		#endif
		
		double radius = 0.;
		Vector3D normal; normal << 0,0,0;
		Point3D pointOnAxis; pointOnAxis << 0,0,0;
		if(!pclCylinderFit(boundary, radius, normal, pointOnAxis, count)) {
			std::cout << "No PCL solution\n";
			continue;
		}
		else {
			if(Eigen::isnan(normal.array()).any() || Eigen::isnan(pointOnAxis.array()).any() || std::isnan(radius)) {
				std::cout << "Unusable solution (NaN)\n";
				continue;
			} else {
				std::cout << "Valid PCL fit\n";
			}
		}
		
		u_int32_t N = 0;
		double max_z=-1000;
		double score = 0;
		for (auto& it: can) { // Even better to only do over inliers ...
			double z = START_Z+STEP_SIZE*(it.z-0.5);
			if(z > max_z) max_z = z;
			score += canScore(map, it);
		}
		double height = max_z-START_Z; // distance between highest can point and table
		
		if(score > MIN_CAN_SCORE && radius > MIN_CAN_RADIUS && height > MIN_CAN_HEIGHT && height < MAX_CAN_HEIGHT) { // Filter for real cans
			count++;
			double center_z = START_Z + height / 2;
			double wantedOffset = pointOnAxis(2) - center_z;
			double normalSteps = -1. * wantedOffset/normal(2);
			Point3D centroid; centroid = pointOnAxis + normalSteps*normal;
			
			filter_octomap::can can_msg;
			can_msg.centroid_position.x = centroid(0); can_msg.centroid_position.y = centroid(1); can_msg.centroid_position.z = centroid(2);
			can_msg.height = height;
			can_msg.radius = radius;
			can_msg.score = score;
			cans_msg.cans.push_back(can_msg);
			
			#ifdef DEBUG
			std::cout << "Found " << N << " can points\n";
			std::cout << "Can middle at: " << centroid << "\n";
			std::cout << "Can score: " << score << "\n";
			
			resetMarker();
			table_marker.type = visualization_msgs::Marker::CYLINDER;
			table_marker.id = count+1;
			table_marker.scale.x = radius*2;
			table_marker.scale.y = radius*2;
			table_marker.scale.z = height;
			table_marker.pose.position.x = centroid.x();
			table_marker.pose.position.y = centroid.y();
			table_marker.pose.position.z = centroid.z();
			table_marker.header.stamp = ros::Time();
			table_marker.ns = "CylinderFit";
			vis_pub.publish( table_marker );
			#endif
		}
	}
		
	cans_msg.count = count;
	cans_msg.header.stamp = ros::Time::now();
	cans_msg.header.frame_id = "world";
	canPublisher.publish(cans_msg);
}

void changeToGrid(octomap::ColorOcTree* octomap, RGBVoxelgrid& grid, 
				octomath::Vector3 min, octomath::Vector3 max) {
	size_t i; double x;
	for(i=0, x=min.x(); x <= max.x(); i++, x += STEP_SIZE) // over x
    {
		#ifdef DEBUG_COUT
		std::cout << "\ny     | x: ";
		std::cout << x;
		std::cout << "\n";
		#endif
		size_t j; double y;
        for(j=0, y=min.y(); y <= max.y(); j++, y+= STEP_SIZE) // over y
        {		
			#ifdef DEBUG_COUT	
			std::cout << std::setw(6) << y;
			std::cout << ": ";
			#endif
			size_t k; double z;
            for(k=0, z=min.z(); z <= max.z(); k++, z+= STEP_SIZE) // over z
            {
				if(!octomap->search(x, y, z)) { // if place never initilized
					grid(i,j,k) = std::make_tuple(0., 0, 0, 0); // don't know anything
				} else { // already known -> just remember the log odds
					grid(i,j,k) = std::make_tuple(octomap->search(x, y, z)->getLogOdds(), 
												octomap->search(x, y, z)->getColor().r,
												octomap->search(x, y, z)->getColor().g,
												octomap->search(x, y, z)->getColor().b);
				}
				#ifdef DEBUG_COUT
				if(canScore(grid, {i,j,k}) > THRESHOLD_CAN)
					std::cout << "+";
				else 
					std::cout << "o";
				std::cout << " ";
				#endif
            }
            #ifdef DEBUG_COUT
            std::cout << "\n";
            #endif
        }
    }
}

/**
 * A subscriber to octomap msg, following https://github.com/OctoMap/octomap_rviz_plugins/blob/kinetic-devel/src/occupancy_grid_display.cpp
 */
// %Tag(CALLBACK)%
void chatterCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	// Test if right message type
	ROS_INFO("Received message number %d", msg->header.seq);
	if(!((msg->id == "OcTree")||(msg->id == "ColorOcTree"))) {
		ROS_INFO("Non supported octree type");
		return;
	}

	// creating octree
	octomap::ColorOcTree* octomap = NULL;
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

	if (tree){
		octomap = dynamic_cast<octomap::ColorOcTree*>(tree);
		if(!octomap){
			ROS_INFO("Wrong octomap type. Use a different display type.");
			return;
		}
	} else {
		ROS_INFO("Failed to deserialize octree message.");
		return;
	}

	ros::NodeHandle n;
	ROS_INFO("Waiting for touched table message");
	auto table_msg = ros::topic::waitForMessage<filter_octomap::table>("/octomap_new/table_touched");
	size_t size_x = std::ceil((table_msg->max.x - table_msg->min.x) / STEP_SIZE ) + 1;
	size_t size_y = std::ceil((table_msg->max.y - table_msg->min.y) / STEP_SIZE ) + 1;
	size_t size_z = std::ceil(0.4 / STEP_SIZE ) + 1; // search volume 40 cm above table
	const Size3D size = {size_x, size_y, size_z};
	
	
	#ifdef DEBUG
	START_X = table_msg->min.x;
	START_Y = table_msg->min.y;
	START_Z = table_msg->max.z;
	#endif

	RGBVoxelgrid grid(size, std::make_tuple(-2., 0, 0, 0)); // Default to known occupied
	octomath::Vector3 lower_vertex(table_msg->min.x,table_msg->min.y,table_msg->max.z); // Define boundary of cube to search in
	octomath::Vector3 upper_vertex(table_msg->max.x,table_msg->max.y,table_msg->max.z+0.4);
	
	changeToGrid(octomap, grid, lower_vertex, upper_vertex);
	ROS_INFO("Octomap converted to voxel-grid");
    findCans(grid, size);
    ROS_INFO("Callback done");

	// free memory
	delete tree;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
	ros::init(argc, argv, "findCan");
	ros::NodeHandle n;
	
	ROS_INFO("Node searching for can init");
	
	#ifdef DEBUG
	// see http://wiki.ros.org/rviz/DisplayTypes/Marker#Example_Usage_.28C.2B-.2B-.2BAC8-roscpp.29
	vis_pub = n.advertise<visualization_msgs::Marker>( "debug/can_marker", 0 );
	resetMarker();
	#endif

	canPublisher = n.advertise<filter_octomap::cans>("octomap_new/cans", 10);
	ROS_INFO("Node searching for can publisher done");
	ros::Subscriber sub = n.subscribe("/octomap_full", 10, chatterCallback);
	ROS_INFO("Node searching for can sub done");
	
	ros::spin();
	return 0;
}
