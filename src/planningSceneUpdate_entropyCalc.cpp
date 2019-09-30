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

#include <octomap/ColorOcTree.h>

#define COLOR_OCTOMAP_SERVER 1

ros::Publisher update_publisher;
ros::Publisher octomap_publisher;
ros::Publisher entropy_publisher;


#ifdef COLOR_OCTOMAP_SERVER
namespace octomap {
class CopyOctree: public OcTree {
	//private:
	//void expandAndSetNode(point3d place, unsigned int depth, float value) {
		//if(depth == this->getTreeDepth()) {
			//this->setNodeValue(place, value);
		//} else {
			//float offset = this->getResolution()/2*pow(2, this->getTreeDepth()-depth-1);
			//for(int i=-1; i<2; i+=2) {
				//for(int j=-1; j<2; j+=2) {
					//for(int k=-1; k<2;k+=2) {
						//float x = place.x()+offset*i;
						//float y = place.y()+offset*j;
						//float z = place.z()+offset*k;
						//expandAndSetNode(point3d(x,y,z), depth+1, value);
					//}
				//}
			//}
		//}
	//}
	
	public:
	CopyOctree(ColorOcTree* colorTree)
	: OcTree(colorTree->getResolution()) {
		ocTreeMemberInit.ensureLinking();
		this->setClampingThresMax(colorTree->getClampingThresMax());
		this->setClampingThresMin(colorTree->getClampingThresMin());
		this->setOccupancyThres(colorTree->getOccupancyThres());
		this->setProbHit(colorTree->getProbHit());
		this->setProbMiss(colorTree->getProbMiss());
		
		OcTreeNode* oldRoot = (OcTreeNode*) colorTree->getRoot();  // Allowed cast as color node derived from octree node
		this->root = new OcTreeNode(*oldRoot);
		
		//for(ColorOcTree::leaf_iterator it = colorTree->begin_leafs(),
			 //end=colorTree->end_leafs(); it!= end; ++it) {
			//if(it.getDepth() == colorTree->getTreeDepth()) // Simple leafs
				//this->setNodeValue(it.getKey(), it->getLogOdds());
			//else {
				//expandAndSetNode(it.getCoordinate(), it.getDepth(), it->getLogOdds());
			//}
		//}
	}
	
	~CopyOctree() {
		//delete this->root; // already done by parent dtor
	}
};
}

void updatePlanningScene(octomap::ColorOcTree* octomap) {
	// Copy color map to normal one
	octomap::OcTree* std_octomap = new octomap::CopyOctree(octomap);
#else
void updatePlanningScene(octomap::OcTree* std_octomap) {
#endif
	// Publish modified map
	octomap_msgs::Octomap newMapMsg;
	newMapMsg.header.frame_id = "world";
	octomap_msgs::binaryMapToMsg(*std_octomap, newMapMsg);
	octomap_publisher.publish(newMapMsg);
	
	// Publish modified map to planning scene
	moveit_msgs::PlanningScene updateMsg;
	updateMsg.is_diff = true;
	updateMsg.world.octomap.octomap = newMapMsg;
	updateMsg.world.octomap.header.stamp = ros::Time::now();
	updateMsg.world.octomap.header.frame_id = "world";
	update_publisher.publish(updateMsg);
	ROS_INFO("Sent OctomapBinary message (size: %d bytes)", (int)newMapMsg.data.size());
	
	delete std_octomap;
}

#ifdef COLOR_OCTOMAP_SERVER
void publishEntropy(const octomap::ColorOcTree* octomap) {
#else
void publishEntropy(const octomap::OcTree* octomap) {
#endif
	double total_v = 0;
	double total_weighted_H = 0;
	// Go over all leafs
	#ifdef COLOR_OCTOMAP_SERVER
	for(octomap::ColorOcTree::leaf_iterator it = octomap->begin_leafs(),
	   end=octomap->end_leafs(); it!= end; ++it)
	#else
	for(octomap::OcTree::leaf_iterator it = octomap->begin_leafs(),
	   end=octomap->end_leafs(); it!= end; ++it)
	#endif
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
		double v = pow(it.getSize(),3);
		
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

/**
 * A subscriber to octomap msg, following https://github.com/OctoMap/octomap_rviz_plugins/blob/kinetic-devel/src/occupancy_grid_display.cpp
 */
// %Tag(CALLBACK)%
void chatterCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	// Test if right message type
	ROS_INFO("Received message number %d", msg->header.seq);
	#ifdef COLOR_OCTOMAP_SERVER
	if(!((msg->id == "OcTree")||(msg->id == "ColorOcTree"))) {
	#else
	if(!(msg->id == "OcTree")) {
	#endif
		ROS_INFO("Non supported octree type; found type:");
		return;
	}

	// create octree from msg
	
	#ifdef COLOR_OCTOMAP_SERVER
	octomap::ColorOcTree* octomap = NULL;
	#else
	octomap::OcTree* octomap = NULL;
	#endif
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
	
	publishEntropy(octomap);
	updatePlanningScene(octomap);
	
	// free memory
	delete tree;
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "planningSceneUpdate_entropyCalc");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
	ros::Subscriber sub = n.subscribe("/octomap_full", 10, chatterCallback);
// %EndTag(SUBSCRIBER)%
	update_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
	octomap_publisher = n.advertise<octomap_msgs::Octomap>("octomap_new", 10, true); // latch topic
	entropy_publisher = n.advertise<std_msgs::Float64>("octomap_new/entropy", 100, true); // latch topic
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
	ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

