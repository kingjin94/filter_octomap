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
#include <algorithm>

#include <octomap/ColorOcTree.h>

#define COLOR_OCTOMAP_SERVER 1

#define TABLE_THRESHOLD 250
/* Debuging*/
//#define DEBUG_COUT 1
#define DEBUG 1
#ifdef DEBUG
	#include "visualization_msgs/Marker.h"
	#include "geometry_msgs/Point.h"
	#define STEPS_X 75 // Right now we have 50 Steps / m
	#define STEPS_Y 150
	#define STEPS_Z 100
	#define START_X 0. // in m
	#define START_Y -1.5 // in m
	#define START_Z -0.1 // in m
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
	#define START_Z -0.1 // in m
	#define STEP_SIZE 0.02 // Size of cells on lowest level
#endif	

ros::Publisher tablePublisher;

#include "octomap_to_x_helper.hxx"

inline double tableScore(RGBVoxelgrid& map, VoxelIndex index, const Size3D size) {
	// Returns tableness at position (x,y,z) in map
	// Tableness is determined in a 5x5 neighbourhood
	// The top is assumed in positive z direction
	// A table is free above the surface (-2. for z+1 and z+2
	// 			  occupied on the surface (3.5 for z)
	//			  unknown bellow the surface (0 for z-1)
	//			  and occupied on the lower side (3.5 for z-2)
	double result = 0;
	//std::cout << "Am at " << index << " with map of size " << size << std::endl;
	size_t neg_x = std::min(index.x, (size_t) 2);
	size_t pos_x = std::min(size.x-index.x, (size_t) 2);
	size_t neg_y = std::min(index.y, (size_t) 2);
	size_t pos_y = std::min(size.y-index.y, (size_t) 2);
	//std::cout << "Wanted offsets: " << neg_x << "," << pos_x << "," << neg_y << "," << pos_y << std::endl;
	
	for(int i = index.x-neg_x; i < index.x+pos_x; i++) // over x
    {
		for(int j = index.y-neg_y; j < index.y+pos_y; j++) // over y
        {
			//std::cout << "Looking at " << i << "," << j << std::endl;
			if(index.z>=2)
				result += 3.5*std::get<0>(map(i,j,index.z-2));
			result += 3.5*std::get<0>(map(i,j,index.z));
			if(index.z<size.z-1)
				result += -2.*std::get<0>(map(i,j,index.z+1));
			if(index.z<size.z-2)
				result += -2.*std::get<0>(map(i,j,index.z+2));
		}
	}		
	return result;
}

void floodfill(Array3D<double>& map, int x, int y, int z, Array3D<int>& explored, double threshold) {
	//// Marks all places in explored true which are above the threshold in map and are connected to the first x,y,z via 6-Neighbourhood
	//#ifdef DEBUG_COUT
	//std::cout << "Starting at " << x << "," << y << "," << z << "\n";
	//#endif
	//std::stack<std::tuple<int,int,int>> posToLookAt;
	//posToLookAt.emplace(x,y,z);
	//while(!posToLookAt.empty()) {
		//std::tuple<int,int,int> here = posToLookAt.top();
		//double x = std::get<0>(here);
		//double y = std::get<1>(here);
		//double z = std::get<2>(here);
		//#ifdef DEBUG_COUT
		//std::cout << "\nStack size: " << posToLookAt.size() << "\n";
		//std::cout << "Looking at " << x << "," << y << "," << z << "\n";
		//#endif
		//posToLookAt.pop();
		//if(explored(x,y,z)!=0) continue;
		//else {
			//explored(x,y,z) = 1;
			//#ifdef DEBUG_COUT
			//std::cout << "Is new; set to " << explored(x,y,z) << "; local tableness: " << map(x,y,z) << "\n";
			//#endif
			//// Test neighbours and call floodfill if tableness high enough
			//if(x>0) {
				//if(map(x-1,y,z) > threshold) { 
					//posToLookAt.emplace(x-1,y,z); 
					//#ifdef DEBUG_COUT
					//std::cout << "-x inserted ";
					//#endif
					//}
				//}
			//if(y>0) {
				//if(map(x,y-1,z) > threshold) {
					//posToLookAt.emplace(x,y-1,z);
					//#ifdef DEBUG_COUT
					//std::cout << "-y inserted ";
					//#endif
				//}
			//}
			//if(z>0) {
				//if(map(x,y,z-1) > threshold) {
					//posToLookAt.emplace(x,y,z-1);
					//#ifdef DEBUG_COUT
					//std::cout << "-z inserted ";
					//#endif
				//}
			//}
			//if(x<STEPS_X-4-1) {
				//if(map(x+1,y,z) > threshold) {
					//posToLookAt.emplace(x+1,y,z);
					//#ifdef DEBUG_COUT
					//std::cout << "+x inserted ";
					//#endif
				//}
			//}
			//if(y<STEPS_Y-4-1) {
				//if(map(x,y+1,z) > threshold) {
					//posToLookAt.emplace(x,y+1,z);
					//#ifdef DEBUG_COUT
					//std::cout << "+y inserted ";
					//#endif
				//}
			//}
			//if(z<STEPS_Z-4-1) {
				//if(map(x,y,z+1) > threshold) { 
					//posToLookAt.emplace(x,y,z+1);
					//#ifdef DEBUG_COUT
					//std::cout << "+z inserted ";
					//#endif
				//}
			//}
		//}
	//}
}

double generateTableMap(Array3D<double>& map, Array3D<double>& tableness, int& max_i, int& max_j, int& max_k) {
	//double maxTableness = -100000;
	//for(int i = 0; i < STEPS_X-4; i++) // over x
    //{
		//#ifdef DEBUG_COUT
		//std::cout << "\nj     | i: ";
		//std::cout << i;
		//std::cout << "\n";
		//#endif
		//for(int j = 0; j < STEPS_Y-4; j++) // over y
        //{
			//#ifdef DEBUG_COUT
			//std::cout << std::setw(3) << j;
			//std::cout << ": ";
			//#endif
            //for(int k = 0; k < STEPS_Z-4; k++) // over z
            //{
				//tableness(i,j,k) = convolveOneTable(map, i+2, j+2, k+2);
				//if(tableness(i,j,k) > maxTableness) {
					//maxTableness = tableness(i,j,k);
					//max_i = i;
					//max_j = j;
					//max_k = k;
				//}
				//#ifdef DEBUG_COUT
				//std::cout << std::setw( 6 ) << std::setprecision( 4 ) << tableness(i,j,k);
				//std::cout << " ";
				//#endif
			//}
			//#ifdef DEBUG_COUT
			//std::cout << "\n";
			//#endif
		//}
	//}
	//std::cout << "Best tableness (" << maxTableness << ") found at (" << max_i << "," << max_j << "," << max_k << ")\n";
	//return maxTableness;
}

void findTable(RGBVoxelgrid& map, const Size3D& size) {
	// Where in map could tables be?
	VoxelindexToScore candidates;
	generateCandidates(map, candidates, size, tableScore, TABLE_THRESHOLD);
	
	#ifdef DEBUG // Display table candidates
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
	std::cout << "Table markers ...";
	#endif
	
	double best_table_score = -1.;
	VoxelList best_table;
	octomath::Vector3 best_centroid(0.,0.,0.);
	octomath::Vector3 best_min(1000.,1000.,1000.);
	octomath::Vector3 best_max(-1000.,-1000.,-1000.);
	
	while(!candidates.empty()) {
		std::cout << "Best node at: " << getMax(candidates) << " with score " << candidates[getMax(candidates)] << std::endl;
		
		VoxelList table;
		floodfill(candidates, getMax(candidates), table);
		
		//double min_x=1000, max_x=-1000, min_y=1000, max_y=-1000, min_z=1000, max_z=-1000;
		octomath::Vector3 centroid(0.,0.,0.);
		octomath::Vector3 min(1000.,1000.,1000.);
		octomath::Vector3 max(-1000.,-1000.,-1000.);
		double score = 0;
		u_int32_t N = 0;
		for (auto& it: table) { // Go over this table candidate
			double x = START_X+STEP_SIZE*(it.x-0.5);
			double y = START_Y+STEP_SIZE*(it.y-0.5);
			double z = START_Z+STEP_SIZE*(it.z-0.5);
			octomath::Vector3 here(x,y,z);
			N++;
			centroid+=here;
			if(x < min.x()) min.x() = x;
			if(y < min.y()) min.y() = y;
			if(z < min.z()) min.z() = z;
			if(x > max.x()) max.x() = x;
			if(y > max.y()) max.y() = y;
			if(z > max.z()) max.z() = z;
			score += tableScore(map, it, size);
		}
		centroid/=N;
		#ifdef DEBUG
		std::cout << "Found " << N << " table points\n";
		std::cout << "Table middle at: " << centroid << "\n";
		std::cout << "Minima: (" << min.x() << "," << min.y() << "," << min.z() << "); maxima: ("
					<< max.x() << "," << max.y() << "," << max.z() << ")\n"; 
		std::cout << "Table score: " << score << "\n";
		#endif
		
		if(score > best_table_score) {
			best_table_score = score;
			best_table = table;
			best_centroid = centroid;
			best_min = min;
			best_max = max;
		}
	}
	
	#ifdef DEBUG
	std::cout << "============\n" << "Best Table: \n";
	std::cout << "Table middle at: " << best_centroid << "\n";
	std::cout << "Minima: " << best_min << "; maxima: "<< best_max << "\n"; 
	std::cout << "Table score: " << best_table_score << "\n";
	
	// Display best table points
	resetMarker();
	table_marker.type = visualization_msgs::Marker::POINTS;
	table_marker.id = 1;
	for (auto& it: best_table) {
		geometry_msgs::Point p;
		p.x = START_X + it.x * STEP_SIZE;
		p.y = START_Y + it.y * STEP_SIZE;
		p.z = START_Z + it.z * STEP_SIZE;
		table_marker.points.push_back(p);
	}
	table_marker.ns = "Best Table points";
	table_marker.header.stamp = ros::Time();
	vis_pub.publish( table_marker );	
			
	resetMarker();
	table_marker.type = visualization_msgs::Marker::CUBE;
	table_marker.id = 2;
	table_marker.scale.x = best_max.x() - best_min.x();
	table_marker.scale.y = best_max.y() - best_min.y();
	table_marker.scale.z = best_max.z() - best_min.z();
	table_marker.pose.position.x = (best_max.x() + best_min.x())/2;
	table_marker.pose.position.y = (best_max.y() + best_min.y())/2;
	table_marker.pose.position.z = (best_max.z() + best_min.z())/2;
	table_marker.header.stamp = ros::Time();
	table_marker.ns = "SurfaceCube";
	vis_pub.publish( table_marker );
	#endif
	
	// plane fitting, assuming n_z = 1 (should be the case for a table), see: https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
	double xx=0, xy=0, xz=0, yy=0, yz=0, zz=0;
	for (auto& it: best_table) {
		octomath::Vector3 r(START_X + it.x * STEP_SIZE, START_Y + it.y * STEP_SIZE, START_Z + it.z * STEP_SIZE);
		r = r - best_centroid;
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
	table_msg.centroid_position.x = best_centroid.x(); table_msg.centroid_position.y = best_centroid.y(); table_msg.centroid_position.z = best_centroid.z();
	table_msg.normal.x = normal.x(); table_msg.normal.y = normal.y(); table_msg.normal.z = normal.z(); 
	table_msg.max.x = best_max.x(); table_msg.max.y = best_max.y(); table_msg.max.z = best_max.z(); 
	table_msg.min.x = best_min.x(); table_msg.min.y = best_min.y(); table_msg.min.z = best_min.z(); 
	table_msg.score = best_table_score;
	tablePublisher.publish(table_msg);
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
		#ifdef COLOR_OCTOMAP_SERVER
		octomap = dynamic_cast<octomap::ColorOcTree*>(tree);
		#else
		octomap = dynamic_cast<octomap::OcTree*>(tree);
		#endif
		if(!octomap){
			ROS_INFO("Wrong octomap type. Use a different display type.");
			return;
		}
	} else {
		ROS_INFO("Failed to deserialize octree message.");
		return;
	}
	
	const Size3D size = {STEPS_X+1, STEPS_Y+1, STEPS_Z+1};

	RGBVoxelgrid grid(size, std::make_tuple(-2., 0, 0, 0)); // Default to known occupied
	octomath::Vector3 lower_vertex(START_X, START_Y, START_Z); // Define boundary of cube to search in
	octomath::Vector3 upper_vertex(START_X+STEPS_X*STEP_SIZE, START_Y+STEPS_Y*STEP_SIZE, START_Z+STEPS_Z*STEP_SIZE); 
	
	changeToGrid(octomap, grid, lower_vertex, upper_vertex);
    findTable(grid, size);

	// free memory
	delete tree;
}

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
	vis_pub = n.advertise<visualization_msgs::Marker>( "debug/table_marker", 0, true);
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

	tablePublisher = n.advertise<filter_octomap::table>("octomap_new/table", 10, true);
	ROS_INFO("Node searching for table publisher done");
	ros::Subscriber sub = n.subscribe("/octomap_full", 10, chatterCallback);
	ROS_INFO("Node searching for table sub done");
	
	ros::spin();
	return 0;
}
