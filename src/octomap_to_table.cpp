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

#define THRESHOLD_BELONG 0.2
/* Debuging*/
//#define DEBUG_COUT 1
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

inline double convolveOneTable(Array3D<double>& map, int x, int y, int z) {
	// Returns tableness at position (x,y,z) in map
	// Tableness is determined in a 5x5 neighbourhood
	// The top is assumed in positive z direction
	// A table is free above the surface (-2. for z+1 and z+2
	// 			  occupied on the surface (3.5 for z)
	//			  unknown bellow the surface (0 for z-1)
	//			  and occupied on the lower side (3.5 for z-2)
	double result = 0;
	
	for(int i = x-2; i < x+2; i++) // over x
    {
		for(int j = y-2; j < y+2; j++) // over y
        {
			result += 3.5*map(i,j,z-2);
			result += 3.5*map(i,j,z);
			result += -2.*map(i,j,z+1);
			result += -2.*map(i,j,z+2);
		}
	}		
	return result;
}

void floodfill(Array3D<double>& map, int x, int y, int z, Array3D<int>& explored, double threshold) {
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

double generateTableMap(Array3D<double>& map, Array3D<double>& tableness, int& max_i, int& max_j, int& max_k) {
	double maxTableness = -100000;
	for(int i = 0; i < STEPS_X-4; i++) // over x
    {
		#ifdef DEBUG_COUT
		std::cout << "\nj     | i: ";
		std::cout << i;
		std::cout << "\n";
		#endif
		for(int j = 0; j < STEPS_Y-4; j++) // over y
        {
			#ifdef DEBUG_COUT
			std::cout << std::setw(3) << j;
			std::cout << ": ";
			#endif
            for(int k = 0; k < STEPS_Z-4; k++) // over z
            {
				tableness(i,j,k) = convolveOneTable(map, i+2, j+2, k+2);
				if(tableness(i,j,k) > maxTableness) {
					maxTableness = tableness(i,j,k);
					max_i = i;
					max_j = j;
					max_k = k;
				}
				#ifdef DEBUG_COUT
				std::cout << std::setw( 6 ) << std::setprecision( 4 ) << tableness(i,j,k);
				std::cout << " ";
				#endif
			}
			#ifdef DEBUG_COUT
			std::cout << "\n";
			#endif
		}
	}
	std::cout << "Best tableness (" << maxTableness << ") found at (" << max_i << "," << max_j << "," << max_k << ")\n";
	return maxTableness;
}

void findTable(Array3D<double>& map) {
	// Where in map could tables be?
	//double tableness[STEPS_X-4][STEPS_Y-4][STEPS_Z-4];
	Array3D<double> tableness(STEPS_X-4, STEPS_Y-4, STEPS_Z-4);
	int max_i = 0, max_j = 0, max_k = 0;
	double maxTableness = generateTableMap(map, tableness, max_i, max_j, max_k);
	
	// Floodfill from best tableness to find all points making up the table
	//bool explored[STEPS_X-4][STEPS_Y-4][STEPS_Z-4] = {}; // Pads with zero --> all false
	Array3D<int> belongsToTable(STEPS_X-4, STEPS_Y-4, STEPS_Z-4, 0);
	floodfill(tableness, max_i, max_j, max_k, belongsToTable, THRESHOLD_BELONG*maxTableness);
	// Debug floodfill
	#ifdef DEBUG
	int msg_index = 0;
	table_marker.points.clear();
	for(int i = 2; i < STEPS_X-2; i++) // over x
    {
		#ifdef DEBUG_COUT
		std::cout << "\nj     | i: ";
		std::cout << i;
		std::cout << "\n";
		#endif
		for(int j = 2; j < STEPS_Y-2; j++) // over y
        {
			#ifdef DEBUG_COUT
			std::cout << std::setw(3) << j;
			std::cout << ": ";
			#endif
            for(int k = 2; k < STEPS_Z-2; k++) // over z
            {
				if(belongsToTable(i-2,j-2,k-2)!=0) {
					#ifdef DEBUG_COUT
					std::cout << "+";
					#endif
					// See http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
					geometry_msgs::Point p;
					p.x = START_X + i * STEP_SIZE;
					p.y = START_Y + j * STEP_SIZE;
					p.z = START_Z + k * STEP_SIZE;
					table_marker.points.push_back(p);
				}
				else {
					#ifdef DEBUG_COUT
					std::cout << "-";
					#endif
				}
			}
			#ifdef DEBUG_COUT
			std::cout << "\n";
			#endif
		}
	}
	table_marker.header.stamp = ros::Time();
	vis_pub.publish( table_marker );
	#endif
	
	// Extract data about the table --> min / max in x,y,z; centroid; plane (normal + offset)
	int N=0;
	octomath::Vector3 centroid(0.,0.,0.);
	double min_x=1000, max_x=-1000, min_y=1000, max_y=-1000, min_z=1000, max_z=-1000;
	std::vector<octomath::Vector3>* pointsOnTable = new std::vector<octomath::Vector3>;
	double score = 0;

	for(int i = 2; i < STEPS_X-2; i++) // over x
    {
		double x = START_X+STEP_SIZE*i;
		for(int j = 2; j < STEPS_Y-2; j++) // over y
        {
			double y = START_Y+STEP_SIZE*j;
            for(int k = 2; k < STEPS_Z-2; k++) // over z
            {
				double z = START_Z+STEP_SIZE*k;
				if(belongsToTable(i-2,j-2,k-2)!=0) {
					octomath::Vector3 here(x,y,z);
					N++;
					centroid+=here;
					if(x < min_x) min_x = x;
					if(y < min_y) min_y = y;
					if(z < min_z) min_z = z;
					if(x > max_x) max_x = x;
					if(y > max_y) max_y = y;
					if(z > max_z) max_z = z;
					pointsOnTable->push_back(octomath::Vector3(x,y,z));
					score += tableness(i-2,j-2,k-2);
				}
			}
		}
	}
	centroid/=N;
	std::cout << "Found " << N << " table points\n";
	std::cout << "Table middle at: " << centroid << "\n";
	std::cout << "Minima: (" << min_x << "," << min_y << "," << min_z << "); maxima: ("
				<< max_x << "," << max_y << "," << max_z << ")\n"; 
	std::cout << "Table score: " << score << "\n";
				
	// plane fitting, assuming n_z = 1 (should be the case for a table), see: https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
	double xx=0, xy=0, xz=0, yy=0, yz=0, zz=0;
	for(std::size_t i=0; i<pointsOnTable->size(); ++i) {
		octomath::Vector3 r = pointsOnTable->at(i) - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}
	double det_z = xx*yy - xy*xy;
	octomath::Vector3 normal = octomath::Vector3(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
	normal.normalize();
	std::cout << "Table normal: " << normal << "\n";
	
	filter_octomap::table table_msg;
	table_msg.header.stamp = ros::Time::now();
	table_msg.header.frame_id = "world";
	table_msg.centroid_position.x = centroid.x(); table_msg.centroid_position.y = centroid.y(); table_msg.centroid_position.z = centroid.z();
	table_msg.normal.x = normal.x(); table_msg.normal.y = normal.y(); table_msg.normal.z = normal.z(); 
	table_msg.max.x = max_x; table_msg.max.y = max_y; table_msg.max.z = max_z; 
	table_msg.min.x = min_x; table_msg.min.y = min_y; table_msg.min.z = min_z; 
	table_msg.score = score;
	tablePublisher.publish(table_msg);
}

void changeToGrid(octomap::OcTree* octomap, Array3D<double>& grid) {
	int i; double x;
	for(i=0, x=START_X; i < STEPS_X; i++, x += STEP_SIZE) // over x
    {
		//std::cout << "\ny     | x: ";
		//std::cout << x;
		//std::cout << "\n";
		int j; double y;
        for(j=0, y=START_Y; j < STEPS_Y; j++, y+= STEP_SIZE) // over y
        {			
			//std::cout << std::setw(6) << y;
			//std::cout << ": ";
			int k; double z;
            for(k=0, z=START_Z; k < STEPS_Z; k++, z+= STEP_SIZE) // over z
            {
				if(!octomap->search(x, y, z)) { // if place never initilized
					grid(i,j,k) = 0.; // don't know anything
				} else { // already known -> just remember the log odds
					grid(i,j,k) = octomap->search(x, y, z)->getLogOdds();
				}
				//if(grid(i,j,k) < -0.5)
					//std::cout << "-";
				//else if(grid(i,j,k) < 0.5)
					//std::cout << "o";
				//else
					//std::cout << "+";
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
	if(!(msg->id == "OcTree")) {
		ROS_INFO("Non supported octree type");
		return;
	}

	// creating octree
	octomap::OcTree* octomap = NULL;
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

	if (tree){
		octomap = dynamic_cast<octomap::OcTree*>(tree);
		if(!octomap){
			ROS_INFO("Wrong octomap type. Use a different display type.");
			return;
		}
	} else {
		ROS_INFO("Failed to deserialize octree message.");
		return;
	}


	Array3D<double> grid(STEPS_X, STEPS_Y, STEPS_Z);
	changeToGrid(octomap, grid);
    findTable(grid);

	// free memory
	delete tree;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
	//// Load old map
	//octomap::OcTree* octomap = NULL;
	//octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read("/home/catkin_ws/test.ot");
	//if (tree){
		//octomap = dynamic_cast<octomap::OcTree*>(tree);
		//if(!octomap){
			//ROS_INFO("Wrong octomap type. Use a different display type.");
			//return -1;
		//}
	//} else {
		//ROS_INFO("Failed to deserialize octree message.");
		//return -1;
	//}

	//// call processor
	//Array3D<double> grid(STEPS_X, STEPS_Y, STEPS_Z);
	//changeToGrid(octomap, grid);
	//findTable(grid);
	
	//// terminate
	//return 0;

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
