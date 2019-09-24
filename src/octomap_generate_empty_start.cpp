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
#include <bits/stdc++.h>  // sting to float

#include <octomap/ColorOcTree.h>

#define LFREE -0.0872 // old -2.
#define LOCCUPIED 0.0872 // old 3.5
#define COLOR_OCTOMAP_SERVER 1

ros::Publisher update_publisher;
ros::Publisher octomap_publisher;
ros::Publisher entropy_publisher;
float resolution;

#ifdef COLOR_OCTOMAP_SERVER
inline void add_free_and_occupied(octomap::ColorOcTree* octomap) {
#else
inline void add_free_and_occupied(octomap::OcTree* octomap) {	
#endif
	double total_weighted_H = 0;
	for(float z = -0.1+resolution/2; z <= 1.5-resolution/2; z += resolution) // increment by resolution to hit all possible leafs
    {
		std::cout << "\r" << z << "                                          \n";
        for(float y = -1.5+resolution/2; y <= 1.5-resolution/2; y += resolution)
        {
			std::cout << "\ry: " << y;
            for(float x = -1.5+resolution/2; x <= 1.5-resolution/2; x += resolution)
            {
                if(-0.99 <= x && x <= 0.19 &&
                   -.99 <= y && y <= .99 && 
                   0.01 <= z && z <= 2.) {
					octomap->setNodeValue(x, y, z, LFREE); // Space behind robot assumed free
					total_weighted_H += 0.; // known free space -> no entropy
				}
				else {
					if(!octomap->search(x, y, z)) { // if place never initilized
						octomap->setNodeValue(x, y, z, LOCCUPIED); 	// set occupied
						#ifdef COLOR_OCTOMAP_SERVER
						octomap->setNodeColor(x, y, z, 255, 255, 255);
						#endif
						total_weighted_H += -2*.5*std::log(.5); // unknown space
					} else if(octomap->search(x, y, z)->getLogOdds() > -1.9) { // possible that occupied
						double log_odd = octomap->search(x, y, z)->getLogOdds();
						double p = std::exp(log_odd) / (1 + std::exp(log_odd));
						total_weighted_H += -p*std::log(p) - (1-p)*std::log(1-p);
						octomap->setNodeValue(x, y, z, LOCCUPIED); 	// set occupied
						#ifdef COLOR_OCTOMAP_SERVER
						octomap->setNodeColor(x, y, z, 255, 255, 255);
						#endif
					} else { // certainly free
						total_weighted_H += 0.;
					}
				}
            }
        }
    }
    ROS_INFO("Avg. local entropy in surrounding: %f", total_weighted_H / 158/298/298); // normalize to volume of tripple for loop
    octomap->updateInnerOccupancy();
	octomap->prune(); // Compress after major changes have been applied
}

int main(int argc, char **argv)
{
	ROS_INFO("You gave %d arguments", argc);
	if(argc == 2) {
		ROS_INFO("First (and only allowed) is resolution: %f", std::stof(argv[1])); 
		resolution=std::stof(argv[1]);
	} else {
		ROS_INFO("Invalid call, give resolution");
		return -1;
	}
	
	#ifdef COLOR_OCTOMAP_SERVER
	octomap::ColorOcTree* octomap = new octomap::ColorOcTree(resolution);
	#else
	octomap::OcTree* octomap = new octomap::OcTree(resolution);
	#endif
	add_free_and_occupied(octomap);
	std::ostringstream outName;
	outName << "prefilled" << resolution << ".ot";
	octomap->write(outName.str());
	
	delete octomap;
	
	// terminate
	return 0;
}
// %EndTag(FULLTEXT)%

