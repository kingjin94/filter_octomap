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

ros::Publisher entropy_publisher;

void entropyCalcCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  ROS_INFO("Received message number %d", msg->header.seq);
  //ROS_INFO("Got type: %s", msg->id.c_str());
  if(!(msg->id == "OcTree")) {
	  ROS_INFO("Non supported octree type");
	  return;
  }
  //ROS_INFO("Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());
  
  // May have to get transform later
  
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
	//ROS_INFO("Retrieved octree from message.");
	//ROS_INFO("Tree os layers %d deep.", octomap->getTreeDepth());
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);
  //ROS_INFO("Tree min: (%f, %f, %f); max: (%f, %f, %f)", minX, minY, minZ, maxX, maxY, maxZ);
  ROS_INFO("Probability for occupied: %f", octomap->getOccupancyThres());

	/* generate copy of octree, where:
	 *   - space in the back is known free space --> set with itterator begin_leafs_bbx and setNodeValue
	 *   - everything else is forced non-free if its occupancy probability is higher than l_low
	 *   - publish this new map to the planning scene as in planningSceneUpdater.py
	*/

	double total_v = 0;
	double total_weighted_H = 0;
	// Go over all leafs
	for(octomap::OcTree::leaf_iterator it = octomap->begin_leafs(),
       end=octomap->end_leafs(); it!= end; ++it)
	{
		//manipulate node, e.g.:
		//std::cout << "Node center: " << it.getCoordinate() << std::endl;
		//std::cout << "Node size: " << it.getSize() << std::endl;
		//std::cout << "Node value: " << it->getValue() << std::endl;
		double log_odd = it->getLogOdds();
		//std::cout << "Node value: " << it->getLogOdds() << std::endl;
		double p = std::exp(log_odd) / (1 + std::exp(log_odd));
		double H = -p*std::log(p) - (1-p)*std::log(1-p);
		//std::cout << "Node entropy: " << H << std::endl;
		double v = (pow(it.getSize(),3));
		
		total_v+=v;
		total_weighted_H += v*H;
	}
	ROS_INFO("Total volume: %f", total_v);
	double entropy = (total_weighted_H - (3*3*1.5 - total_v)*2.*.5*std::log(.5)) / (3*3*1.5);
	ROS_INFO("Average entropy: %f", entropy); 
	
	std_msgs::Float64 newEntropyMsg;
	newEntropyMsg.data = entropy;
	entropy_publisher.publish(newEntropyMsg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_entropy_publisher");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/octomap_full", 10, entropyCalcCallback);
	entropy_publisher = n.advertise<std_msgs::Float64>("octomap_new/entropy", 100);

	ros::spin();
	return 0;
}
