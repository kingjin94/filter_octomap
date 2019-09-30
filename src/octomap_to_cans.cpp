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
#include "std_msgs/Float64.h"
#include <assert.h>
#include <iostream>
#include "filter_octomap/table.h"
#include <assert.h>

#include <octomap/ColorOcTree.h>

#define THRESHOLD_BELONG 0.2
/* Debuging*/
#define DEBUG_COUT 1
#define DEBUG 1
#ifdef DEBUG
	#include "visualization_msgs/Marker.h"
	#include "geometry_msgs/Point.h"
	#define STEPS_X 75 // Right now we have 50 Steps / m
	#define STEPS_Y 100
	#define STEPS_Z 50
	#define START_X 0. // in m
	#define START_Y -1. // in m
	#define START_Z 0 // in m
	#define STEP_SIZE 0.02 // Size of cells on lowest level
	ros::Publisher vis_pub;
	visualization_msgs::Marker table_marker;
#else
	/*Deploy*/
	#define STEPS_X 75 // Right now we have 50 Steps / m
	#define STEPS_Y 150
	#define STEPS_Z 100
	#define START_X 0. // in m
	#define START_Y -1.5 // in m
	#define START_Z 0 // in m
	#define STEP_SIZE 0.02 // Size of cells on lowest level
#endif	

ros::Publisher tablePublisher;

template <class T> class Array3D { // from: https://stackoverflow.com/questions/2178909/how-to-initialize-3d-array-in-c
    size_t m_width, m_height, m_here, m_size;
    std::vector<T> m_data;
  public:
    Array3D(size_t x, size_t y, size_t z, T init = 0):
      m_width(x), m_height(y), m_data(x*y*z, init), m_here(0), m_size(x*y*z)
    {}
    T& operator()(size_t x, size_t y, size_t z) {
		assert(x < m_width);
		assert(y < m_height);
		assert(x*y*z < m_size);
        return m_data.at(x + y * m_width + z * m_width * m_height);
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

typedef Array3D<std::tuple<double, u_int8_t, u_int8_t, u_int8_t>> RGBVoxelgrid; // Each voxel with occupancy probability and RGB color channels
typedef Array3D<float> ScoreVoxelgrid;

void floodfill(Array3D<float>& map, int x, int y, int z, Array3D<int>& explored, double threshold) {
	// Marks all places in explored true which are above the threshold in map and are connected to the first x,y,z via 6-Neighbourhood
	#ifdef DEBUG_COUT
	std::cout << "Starting at " << x << "," << y << "," << z << "\n";
	#endif
	std::stack<std::tuple<int,int,int>> posToLookAt;
	posToLookAt.emplace(x,y,z);
	while(!posToLookAt.empty()) {
		std::tuple<int,int,int> here = posToLookAt.top();
		double x = std::get<0>(here);
		double y = std::get<1>(here);
		double z = std::get<2>(here);
		#ifdef DEBUG_COUT
		std::cout << "\nStack size: " << posToLookAt.size() << "\n";
		std::cout << "Looking at " << x << "," << y << "," << z << "\n";
		#endif
		posToLookAt.pop();
		if(explored(x,y,z)!=0) continue;
		else {
			explored(x,y,z) = 1;
			#ifdef DEBUG_COUT
			std::cout << "Is new; set to " << explored(x,y,z) << "; local tableness: " << map(x,y,z) << "\n";
			#endif
			// Test neighbours and call floodfill if tableness high enough
			if(x>0) {
				if(map(x-1,y,z) > threshold) { 
					posToLookAt.emplace(x-1,y,z); 
					#ifdef DEBUG_COUT
					std::cout << "-x inserted ";
					#endif
					}
				}
			if(y>0) {
				if(map(x,y-1,z) > threshold) {
					posToLookAt.emplace(x,y-1,z);
					#ifdef DEBUG_COUT
					std::cout << "-y inserted ";
					#endif
				}
			}
			if(z>0) {
				if(map(x,y,z-1) > threshold) {
					posToLookAt.emplace(x,y,z-1);
					#ifdef DEBUG_COUT
					std::cout << "-z inserted ";
					#endif
				}
			}
			if(x<STEPS_X-4-1) {
				if(map(x+1,y,z) > threshold) {
					posToLookAt.emplace(x+1,y,z);
					#ifdef DEBUG_COUT
					std::cout << "+x inserted ";
					#endif
				}
			}
			if(y<STEPS_Y-4-1) {
				if(map(x,y+1,z) > threshold) {
					posToLookAt.emplace(x,y+1,z);
					#ifdef DEBUG_COUT
					std::cout << "+y inserted ";
					#endif
				}
			}
			if(z<STEPS_Z-4-1) {
				if(map(x,y,z+1) > threshold) { 
					posToLookAt.emplace(x,y,z+1);
					#ifdef DEBUG_COUT
					std::cout << "+z inserted ";
					#endif
				}
			}
		}
	}
}

inline double canScore(RGBVoxelgrid& map, size_t x, size_t y, size_t z) {
	return (std::get<0>(map(x,y,z))+2.5) * // 0 if free, max if certainly occupied
		(1.*std::get<1>(map(x,y,z))+ 		// R as big as possible
		255.-1.*std::get<2>(map(x,y,z)));	// G as small as possible
		// B not used and ignored
}

void generateCanMap(RGBVoxelgrid& map, ScoreVoxelgrid& canness, size_t size_x, size_t size_y, size_t size_z) {
	for(int i = 0; i < size_x; i++) // over x
    {
		#ifdef DEBUG_COUT
		std::cout << "\nj     | i: ";
		std::cout << i;
		std::cout << "\n";
		#endif
		for(int j = 0; j < size_y; j++) // over y
        {
			#ifdef DEBUG_COUT
			std::cout << std::setw(3) << j;
			std::cout << ": ";
			#endif
            for(int k = 0; k < size_z; k++) // over z
            {
				canness(i,j,k) = canScore(map, i, j, k);
				#ifdef DEBUG_COUT
				std::cout << std::setw( 6 ) << std::setprecision( 4 ) << canness(i,j,k);
				std::cout << " ";
				#endif
			}
			#ifdef DEBUG_COUT
			std::cout << "\n";
			#endif
		}
	}
}

/* Concept for finding cans
 * 
 * Where to look? -> above table resulting from visual and tactile exploration --> in area from x/y_min till x/y_max and up to ~40 cm above
 * 				  -> changes boundary for changeToGrid
 * 
 * What to search for? -> contigous parts of can color, so red (high first channel, low second and third) + occupancy
 * 
 * What to send? -> radii, heights, centerpoints?
 * 
 * */ 
 
void findCans(RGBVoxelgrid& map, size_t size_x, size_t size_y, size_t size_z) {
	// Where in map could tables be?
	//double tableness[STEPS_X-4][STEPS_Y-4][STEPS_Z-4];
	ScoreVoxelgrid canness(size_x, size_y, size_z);
	//int max_i = 0, max_j = 0, max_k = 0;
	generateCanMap(map, canness, size_x, size_y, size_z);
	
	// Floodfill from best tableness to find all points making up the table
	//bool explored[STEPS_X-4][STEPS_Y-4][STEPS_Z-4] = {}; // Pads with zero --> all false
	//RGBVoxelgrid belongsToTable(STEPS_X-4, STEPS_Y-4, STEPS_Z-4, 0);
	//floodfill(tableness, max_i, max_j, max_k, belongsToTable, THRESHOLD_BELONG*maxTableness);
	//// Debug floodfill
	//#ifdef DEBUG
	//int msg_index = 0;
	//table_marker.points.clear();
	//for(int i = 2; i < STEPS_X-2; i++) // over x
    //{
		//#ifdef DEBUG_COUT
		//std::cout << "\nj     | i: ";
		//std::cout << i;
		//std::cout << "\n";
		//#endif
		//for(int j = 2; j < STEPS_Y-2; j++) // over y
        //{
			//#ifdef DEBUG_COUT
			//std::cout << std::setw(3) << j;
			//std::cout << ": ";
			//#endif
            //for(int k = 2; k < STEPS_Z-2; k++) // over z
            //{
				//if(belongsToTable(i-2,j-2,k-2)!=0) {
					//#ifdef DEBUG_COUT
					//std::cout << "+";
					//#endif
					//// See http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
					//geometry_msgs::Point p;
					//p.x = START_X + i * STEP_SIZE;
					//p.y = START_Y + j * STEP_SIZE;
					//p.z = START_Z + k * STEP_SIZE;
					//table_marker.points.push_back(p);
				//}
				//else {
					//#ifdef DEBUG_COUT
					//std::cout << "-";
					//#endif
				//}
			//}
			//#ifdef DEBUG_COUT
			//std::cout << "\n";
			//#endif
		//}
	//}
	//table_marker.header.stamp = ros::Time();
	//vis_pub.publish( table_marker );
	//#endif
	
	//// Extract data about the table --> min / max in x,y,z; centroid; plane (normal + offset)
	//int N=0;
	//octomath::Vector3 centroid(0.,0.,0.);
	//double min_x=1000, max_x=-1000, min_y=1000, max_y=-1000, min_z=1000, max_z=-1000;
	//std::vector<octomath::Vector3>* pointsOnTable = new std::vector<octomath::Vector3>;
	//double score = 0;

	//for(int i = 2; i < STEPS_X-2; i++) // over x
    //{
		//double x = START_X+STEP_SIZE*i;
		//for(int j = 2; j < STEPS_Y-2; j++) // over y
        //{
			//double y = START_Y+STEP_SIZE*j;
            //for(int k = 2; k < STEPS_Z-2; k++) // over z
            //{
				//double z = START_Z+STEP_SIZE*k;
				//if(belongsToTable(i-2,j-2,k-2)!=0) {
					//octomath::Vector3 here(x,y,z);
					//N++;
					//centroid+=here;
					//if(x < min_x) min_x = x;
					//if(y < min_y) min_y = y;
					//if(z < min_z) min_z = z;
					//if(x > max_x) max_x = x;
					//if(y > max_y) max_y = y;
					//if(z > max_z) max_z = z;
					//pointsOnTable->push_back(octomath::Vector3(x,y,z));
					//score += tableness(i-2,j-2,k-2);
				//}
			//}
		//}
	//}
	//centroid/=N;
	//std::cout << "Found " << N << " table points\n";
	//std::cout << "Table middle at: " << centroid << "\n";
	//std::cout << "Minima: (" << min_x << "," << min_y << "," << min_z << "); maxima: ("
				//<< max_x << "," << max_y << "," << max_z << ")\n"; 
	//std::cout << "Table score: " << score << "\n";
				
	//// plane fitting, assuming n_z = 1 (should be the case for a table), see: https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
	//double xx=0, xy=0, xz=0, yy=0, yz=0, zz=0;
	//for(std::size_t i=0; i<pointsOnTable->size(); ++i) {
		//octomath::Vector3 r = pointsOnTable->at(i) - centroid;
		//xx += r.x() * r.x();
		//xy += r.x() * r.y();
		//xz += r.x() * r.z();
		//yy += r.y() * r.y();
		//yz += r.y() * r.z();
		//zz += r.z() * r.z();
	//}
	//double det_z = xx*yy - xy*xy;
	//octomath::Vector3 normal = octomath::Vector3(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
	//normal.normalize();
	//std::cout << "Table normal: " << normal << "\n";
	
	//filter_octomap::table table_msg;
	//table_msg.header.stamp = ros::Time::now();
	//table_msg.header.frame_id = "world";
	//table_msg.centroid_position.x = centroid.x(); table_msg.centroid_position.y = centroid.y(); table_msg.centroid_position.z = centroid.z();
	//table_msg.normal.x = normal.x(); table_msg.normal.y = normal.y(); table_msg.normal.z = normal.z(); 
	//table_msg.max.x = max_x; table_msg.max.y = max_y; table_msg.max.z = max_z; 
	//table_msg.min.x = min_x; table_msg.min.y = min_y; table_msg.min.z = min_z; 
	//table_msg.score = score;
	//tablePublisher.publish(table_msg);
}

void changeToGrid(octomap::ColorOcTree* octomap, RGBVoxelgrid& grid, 
				float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) {
	int i; double x;
	for(i=0, x=x_min; x < x_max; i++, x += STEP_SIZE) // over x
    {
		//std::cout << "\ny     | x: ";
		//std::cout << x;
		//std::cout << "\n";
		int j; double y;
        for(j=0, y=y_min; y < y_max; j++, y+= STEP_SIZE) // over y
        {			
			//std::cout << std::setw(6) << y;
			//std::cout << ": ";
			int k; double z;
            for(k=0, z=z_min; z < z_max; k++, z+= STEP_SIZE) // over z
            {
				if(!octomap->search(x, y, z)) { // if place never initilized
					grid(i,j,k) = std::make_tuple(0., 0, 0, 0); // don't know anything
				} else { // already known -> just remember the log odds
					grid(i,j,k) = std::make_tuple(octomap->search(x, y, z)->getLogOdds(), 
												octomap->search(x, y, z)->getColor().r,
												octomap->search(x, y, z)->getColor().g,
												octomap->search(x, y, z)->getColor().b);
				}
				//if(std::get<0>(grid(i,j,k)) < 1.)
					//std::cout << "o";
				//else if(std::get<1>(grid(i,j,k)) > 128 && std::get<2>(grid(i,j,k)) < 20)
					//std::cout << "r";
				//else if(std::get<1>(grid(i,j,k)) < 20 && std::get<2>(grid(i,j,k)) > 128)
					//std::cout << "g";
				//else if(std::get<1>(grid(i,j,k)) >> 20 && std::get<2>(grid(i,j,k)) > 20)
					//std::cout << "y";
				//else 
					//std::cout << "o";
				//std::cout << " ";
            }
            //std::cout << "\n";
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
	auto table_msg = ros::topic::waitForMessage<filter_octomap::table>("/octomap_new/table_touched");
	size_t size_x = std::ceil((table_msg->max.x - table_msg->min.x) / STEP_SIZE ) + 1;
	size_t size_y = std::ceil((table_msg->max.y - table_msg->min.y) / STEP_SIZE ) + 1;
	size_t size_z = std::ceil(0.4 / STEP_SIZE ) + 1; // search volume 40 cm above table

	RGBVoxelgrid grid(size_x,size_y,size_z,std::make_tuple(0., 0, 0, 0));
	changeToGrid(octomap, grid, table_msg->min.x, table_msg->max.x, table_msg->min.y, table_msg->max.y, table_msg->max.z, table_msg->max.z+0.4);
    findCans(grid, size_x, size_y, size_z);

	// free memory
	delete tree;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
	ros::init(argc, argv, "findTable");
	ros::NodeHandle n;
	
	ROS_INFO("Node searching for table init");
	
	#ifdef DEBUG
	// see http://wiki.ros.org/rviz/DisplayTypes/Marker#Example_Usage_.28C.2B-.2B-.2BAC8-roscpp.29
	vis_pub = n.advertise<visualization_msgs::Marker>( "debug/table_marker", 0 );
	table_marker;
	table_marker.header.frame_id = "world";
    table_marker.ns = "my_namespace";
    table_marker.id = 0;
    table_marker.type = visualization_msgs::Marker::POINTS;
    table_marker.action = visualization_msgs::Marker::ADD;
	table_marker.pose.orientation.x = 0.0;
	table_marker.pose.orientation.y = 0.0;
	table_marker.pose.orientation.z = 0.0;
	table_marker.pose.orientation.w = 1.0; 
	table_marker.scale.x = 0.01;
	table_marker.scale.y = 0.01;
	table_marker.scale.z = 0.01;
	table_marker.color.a = 1.0; // Don't forget to set the alpha!
	table_marker.color.r = 0.0;
	table_marker.color.g = 1.0;
	table_marker.color.b = 0.0;
	table_marker.lifetime  = ros::Duration(10.);
	#endif

	tablePublisher = n.advertise<filter_octomap::table>("octomap_new/table", 10);
	ROS_INFO("Node searching for table publisher done");
	ros::Subscriber sub = n.subscribe("/octomap_full", 10, chatterCallback);
	ROS_INFO("Node searching for table sub done");
	
	ros::spin();
	return 0;
}
