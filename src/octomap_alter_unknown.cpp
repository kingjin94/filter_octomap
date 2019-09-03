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

ros::Publisher update_publisher;
ros::Publisher octomap_publisher;
ros::Publisher entropy_publisher;

//void recurse(int depth, octomap::OcTreeNode* node, octomap::OcTree* octomap) {
	//// Basecase
	//if(depth == 16) node->setLogOdds(-2.);
	//// Recurse
	//else {
		//// Split 
		//node->expand();
		//// Follow children that are within wanted area
		//for(int i=0; i<8; ++i) {
			//octomap::OcTreeNode* child = node->children[i];
			//if
		//}
	//}
//}

/**
 * A subscriber to octomap msg, following https://github.com/OctoMap/octomap_rviz_plugins/blob/kinetic-devel/src/occupancy_grid_display.cpp
 */
// %Tag(CALLBACK)%
void chatterCallback(const octomap_msgs::Octomap::ConstPtr& msg)
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

	//double total_v = 0;
	//double total_weighted_H = 0;
	//// Go over all leafs
	//for(octomap::OcTree::leaf_iterator it = octomap->begin_leafs(),
       //end=octomap->end_leafs(); it!= end; ++it)
	//{
		////manipulate node, e.g.:
		////std::cout << "Node center: " << it.getCoordinate() << std::endl;
		////std::cout << "Node size: " << it.getSize() << std::endl;
		////std::cout << "Node value: " << it->getValue() << std::endl;
		//double log_odd = it->getLogOdds();
		////std::cout << "Node value: " << it->getLogOdds() << std::endl;
		//double p = std::exp(log_odd) / (1 + std::exp(log_odd));
		//double H = -p*std::log(p) - (1-p)*std::log(1-p);
		////std::cout << "Node entropy: " << H << std::endl;
		//double v = (pow(it.getSize(),3));
		
		//total_v+=v;
		//total_weighted_H += v*H;
	//}
	//ROS_INFO("Total volume: %f", total_v);
	//double entropy = (total_weighted_H - (3*3*1.5 - total_v)*2.*.5*std::log(.5)) / (3*3*1.5);
	//ROS_INFO("Average entropy: %f", entropy); 
	
	//std_msgs::Float64 newEntropyMsg;
	//newEntropyMsg.data = entropy;
	//entropy_publisher.publish(newEntropyMsg);
	
	// Only expands already used nodes, does not produce new
	//ROS_INFO("Num nodes: %d", octomap->calcNumNodes());
	//octomap->expand();
	//ROS_INFO("Num nodes (after expand): %d", octomap->calcNumNodes());
	//for(octomap::OcTree::leaf_iterator it = octomap->begin_leafs(),
       //end=octomap->end_leafs(); it!= end; ++it)
	//{
		//if(-1. <= it.getX()-it.getSize()/2 && it.getX()+it.getSize()/2 <= 0.1 && 
		//-1. <= it.getY()-it.getSize()/2 && it.getY()+it.getSize()/2 <= 1. && 
		//0. <= it.getZ()-it.getSize()/2 && it.getZ()+it.getSize()/2 <= 1.5) {
			//it->setLogOdds(-1.);
			//ROS_INFO("Change Log Odds");
		//}
	//}
	
	//setNodeValue (const OcTreeKey &key, float log_odds_value, bool lazy_eval=false)
	
	// WORKING VERSION TO TURN BACKSIDE OF ROBOT FREE
	double total_weighted_H = 0;
	for(float z = -0.09; z <= 1.49; z += 0.02) // increment by resolution to hit all possible leafs
    {
        for(float y = -1.49; y <= 1.49; y += 0.02)
        {
            for(float x = -1.49; x <= 1.49; x += 0.02)
            {
                if(-0.99 <= x && x <= 0.19 &&
                   -.99 <= y && y <= .99 && 
                   0.01 <= z && z <= 2.) {
					octomap->setNodeValue(x, y, z, -2.0); // Space behind robot assumed free
					total_weighted_H += 0.; // known free space -> no entropy
				}
				else {
					if(!octomap->search(x, y, z)) { // if place never initilized
						octomap->setNodeValue(x, y, z, 3.5); 	// set occupied
						total_weighted_H += -2*.5*std::log(.5); // unknown space
					} else if(octomap->search(x, y, z)->getLogOdds() > -1.9) { // possible that occupied
						double log_odd = octomap->search(x, y, z)->getLogOdds();
						double p = std::exp(log_odd) / (1 + std::exp(log_odd));
						total_weighted_H += -p*std::log(p) - (1-p)*std::log(1-p);
						octomap->setNodeValue(x, y, z, 3.5); 	// set occupied
					} else { // certainly free
						total_weighted_H += 0.;
					}
				}
            }
        }
    }
    ROS_INFO("Avg. local entropy in surrounding: %f", total_weighted_H / 158/298/298); // normalize to volume of tripple for loop
    
    // Own tree iteration to set part of the map occupied
    /* Pseudo code
     * Stack<node> s
     * s.push(octree->getRoot())
     * 
     * while !s.isEmpty():
     * 		here = s.pop
     * 		for k in here.children:
     *			if k.completelyInsideGoalArea: --> node does not know its position nor size! (only iterators do), can be fixed by enhancing the stack element
     * 				k.setFree()
     * 			else if k.completelyOutsideGoalArea:
     * 				k.setOccupied()
     * 			else if k.partiallyInsideGoalArea:
     * 				if k.onMaxDepth():
     * 					k.setOccupied() // overapproximate occupied area
     * 				else:
     * 					s.push(k)
     * */
    
    // Inspired by the iterator: https://github.com/OctoMap/octomap/blob/ebff3f0a53551ad6ccee73e6a09d1edda1916784/octomap/include/octomap/OcTreeIterator.hxx 
    //struct StackElement{
        //octomap::OcTreeNode* node;
        //octomap::OcTreeKey key;
        //uint8_t depth;
    ////};
	//std::stack<StackElement,std::vector<StackElement> > s;
	//StackElement root;
	//root.node = octomap->getRoot();
	//root.depth = 0;
	//root.key[0] = root.key[1] = root.key[2] = 32768; //octomap->tree_max_val; // 2^15 --> perfect in the middle, I guess
	//s.push(root);
	
	//octomap->write("preModify.bt");
	
	//while(!s.empty()) {
		//StackElement here = s.top();
		//s.pop();
		//ROS_INFO("Entering node with key %d, %d, %d", here.key.k[0], here.key.k[1], here.key.k[2]);
		//StackElement tmp;
		//tmp.depth = here.depth+1;
		//float child_size = octomap->getNodeSize(tmp.depth);
		
		//if(here.depth<15) { // highest: 15
			//octomap::key_type center_offset_key = 32768 >> tmp.depth;//octomap->tree_max_val >> tmp.depth;
			//for(uint8_t i=0; i<8; ++i) {
			//// Create all children if not deep enough
				//if(!octomap->nodeChildExists(here.node, i)) {
					//octomap->createNodeChild (here.node, i);
				//}
				//tmp.node = octomap->getNodeChild(here.node, i);
				//octomap::computeChildKey(i, center_offset_key, here.key, tmp.key);
				//// select children action
				//octomap::point3d child_pos = octomap->keyToCoord(tmp.key, tmp.depth);
				
				//ROS_INFO("Looking at (%f, %f, %f) with size %f in depth %d", child_pos.x(), child_pos.y(), child_pos.z(), child_size, tmp.depth); 
				
				//if(-1. <= child_pos.x()-child_size/2 && child_pos.x()+child_size/2 <= 0.1 && 
					//-1. <= child_pos.y()-child_size/2 && child_pos.y()+child_size/2 <= 1. && 
					//0. <= child_pos.z()-child_size/2 && child_pos.z()+child_size/2 <= 1.5) {
					//ROS_INFO("Is inside");
					//octomap->setNodeValue(tmp.key, -2.0); // Node completly within goal area -> set free
					//for(uint8_t j=0; j<8; ++j)
						//if(octomap->nodeChildExists(tmp.node, j)) {
							//ROS_INFO("Killed child %d", j);
							//octomap->deleteNodeChild(tmp.node, j);
						//}
				//}
					
				//else if(-1. > child_pos.x()+child_size/2 || child_pos.x()-child_size/2 > 0.1 ||
					//-1. > child_pos.y()+child_size/2 || child_pos.y()-child_size/2 > 1. ||
					//0. > child_pos.z()+child_size/2 || child_pos.z()-child_size/2 > 1.5) {
					//ROS_INFO("Is outside");
					//continue; // ignore node complete outside goal area
				//}
				
				//else {
					//ROS_INFO("Is split");
					//s.push(tmp); // neither completely in nor out --> must be further subdivided
				//}
					
 			//}
		//} else { // is lowest possible
			//child_size*=2;
			//octomap::point3d child_pos = octomap->keyToCoord(here.key, here.depth);
			//ROS_INFO("Looking at Leaf (%f, %f, %f) with size %f in depth %d", child_pos.x(), child_pos.y(), child_pos.z(), child_size, here.depth); 
			//if(-1. <= child_pos.x()-child_size/2 && child_pos.x()+child_size/2 <= 0.1 && 
				//-1. <= child_pos.y()-child_size/2 && child_pos.y()+child_size/2 <= 1. && 
				//0. <= child_pos.z()-child_size/2 && child_pos.z()+child_size/2 <= 1.5) {
					//octomap->setNodeValue(here.key, -2.0); // Node completly within goal area -> set free
					//ROS_INFO("Is inside");
				//}
		//}
	//}
	
	//octomap->write("postModify.bt");
	octomap->updateInnerOccupancy();
	octomap->prune(); // Compress after major changes have been applied
	
	//octomap->write("postPrune.bt");

	/*
	 * 	scene_update = PlanningScene()
			scene_update.is_diff = True
			scene_update.world.octomap.octomap = msg
			scene_update.world.octomap.header.stamp = rospy.Time.now()
			scene_update.world.octomap.header.frame_id = "world"
			self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=5)
			self.scene_pub.publish(scene_update)
	 * */
	 moveit_msgs::PlanningScene updateMsg;
	 octomap_msgs::Octomap newMapMsg;
	 newMapMsg.header.frame_id = "world";
	 //octomap->setOccupancyThres(0.15);
	 //ROS_INFO("New threshold: %f", octomap->getOccupancyThres());
	 //octomap_msgs::fullMapToMsg(*octomap, newMapMsg);
	 octomap_msgs::binaryMapToMsg(*octomap, newMapMsg);
	 octomap_publisher.publish(newMapMsg);
	 //ROS_INFO("Generated update: %s", updateMsg);
	 updateMsg.is_diff = true;
	 updateMsg.world.octomap.octomap = newMapMsg;
	 updateMsg.world.octomap.header.stamp = ros::Time::now();
	 updateMsg.world.octomap.header.frame_id = "world";
	 update_publisher.publish(updateMsg);
	 ROS_INFO("Sent OctomapBinary message (size: %d bytes)", (int)newMapMsg.data.size());
	 
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
  ros::init(argc, argv, "listener");

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
	octomap_publisher = n.advertise<octomap_msgs::Octomap>("octomap_new", 10);
	entropy_publisher = n.advertise<std_msgs::Float64>("octomap_new/entropy", 100);
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

