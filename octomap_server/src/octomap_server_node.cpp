/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009 - 2012.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2009-2012, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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


#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>
#include <octomap/AbstractOcTree.h>
#include "std_msgs/String.h"
#include <octomap_msgs/Octomap.h>

#define USAGE "\nUSAGE: octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

void updateMapCallback(const octomap_msgs::Octomap&);
void updateTrajectoriesCallback(const visualization_msgs::Marker&);

using namespace octomap_server;

//define global pointer to later point to octomap server declaration
//(declaring the octomap server object as global jumps the init function causing an error)
OctomapServer* serverPtr;
int* counter;
ros::Publisher * pb;

void updateMapCallback(const octomap_msgs::Octomap& map_msg)
{
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(map_msg);
	octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
	
	if(!octree)
	{
		ROS_ERROR("error receiving OcTree update");
	}
	
	if(!serverPtr->copyMap(octree))
	{
		ROS_ERROR("Copying Failed");
	}
	else
	{
		//ROS_INFO("Received packet %d",++(*counter));
	}
}

void updateTrajectoriesCallback(const visualization_msgs::Marker& trajectory_msg)
{
	pb->publish(trajectory_msg);
	//ROS_INFO("Received packet %d",++(*counter));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server_node");
  //initialise(argc,argv);
  OctomapServer server;
  int count = 0;
  counter = &count;
  serverPtr = &server;
  ros::NodeHandle private_nh;
  ros::Publisher trajectoryPublisher = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pb = &trajectoryPublisher;
  ros::Subscriber mapSub = private_nh.subscribe("updateMap",1000,updateMapCallback);
  ros::Subscriber trajectorySub = private_nh.subscribe("updateTrajectories",1000,updateTrajectoriesCallback);
  
  ros::spin();
  
  return 0;
}
