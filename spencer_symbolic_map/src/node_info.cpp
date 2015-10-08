 //  node_info.cpp
 
 //   Created on: Apr 19, 2015
 //       Author: mfiore
 // /

#include "spencer_symbolic_map/node_info.h"

NodeInfo::NodeInfo() {
	// TODO Auto-generated constructor stub

}
NodeInfo::NodeInfo(const NodeInfo& other) {
	this->center=other.center;
	this->name=other.name;
	this->origin=other.origin;
	this->vertexs=other.vertexs;
}

NodeInfo::~NodeInfo() {
	// TODO Auto-generated destructor stub
}

