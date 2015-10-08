/**
  node_info.h
   Created on: Apr 19, 2015
       Author: mfiore
 
 Contains information about a map. The spencer project uses several small maps, representing different nodes in the 
 environment.
 	
 */

#ifndef SOURCE_DIRECTORY__SPENCER_SPENCER_MAP_SERVER_SRC_NodeInfo_H_
#define SOURCE_DIRECTORY__SPENCER_SPENCER_MAP_SERVER_SRC_NodeInfo_H_

#include <string>
#include <vector>
 #include <geometry_msgs/Point.h>
using namespace std;

class NodeInfo {
public:
	NodeInfo();
	NodeInfo(const NodeInfo& other);
	virtual ~NodeInfo();

	string name;   //name of the node
	//coordinates of the origin
	geometry_msgs::Point origin;

	//vertexs of the node
	vector<geometry_msgs::Point> vertexs;

	//centroid of the node
	geometry_msgs::Point center;


};

#endif /* SOURCE_DIRECTORY__SPENCER_SPENCER_MAP_SERVER_SRC_NodeInfo_H_ */
