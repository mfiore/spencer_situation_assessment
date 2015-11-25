/* 
 * File:   spencer_symbolic_map
 * Author: Michelangelo Fiore
 *
 * Created on May 27, 2013, 4:48 PM
 * 
 * This file represents a symbolic map of the spencer project, which reads the .
 * map data from an xml file.
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <utility>                   // for std::pair

#include <vector>
#include <string>
#include <map>
#include <queue>

#include "situation_assessment_msgs/Graph.h"
#include "situation_assessment_msgs/Property.h"
#include "situation_assessment_msgs/Edge.h"
#include "situation_assessment_msgs/Node.h"
#include <situation_assessment_msgs/GetLocations.h>
#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/NamedPoseList.h>

#include <spencer_symbolic_map/node_info.h>
#include <spencer_symbolic_map/spencer_map.h>

#include <situation_assessment_msgs/GetMap.h>

SpencerMap* spencer_map;

using namespace std;

bool getMap(situation_assessment_msgs::GetMap::Request &req, situation_assessment_msgs::GetMap::Response &res) {
	ROS_INFO("SPENCER_SYMBOLIC_MAP  Got request");

	vector<string> map_nodes=spencer_map->getNodes();
	map<string,vector<string> > map_edges=spencer_map->getEdges();

	situation_assessment_msgs::Graph g;

	vector<situation_assessment_msgs::Node> msg_nodes;
	vector<situation_assessment_msgs::Edge> msg_edges;
	ROS_INFO("SPENCER_SYMBOLIC_MAP  There are %ld nodes:",map_nodes.size());
	for (int i=0; i<map_nodes.size();i++) {
		situation_assessment_msgs::Node n;
		n.label=map_nodes[i];
		NodeInfo node_info=spencer_map->getNodeInfo(n.label);
		n.center=node_info.center;
		n.vertexs=node_info.vertexs;

		ROS_INFO("SPENCER_SYMBOLIC_MAP  - %s",n.label.c_str());

		msg_nodes.push_back(n);
	}
	ROS_INFO("SPENCER_SYMBOLIC_MAP  With edges:");
	for (map<string,vector<string> >::iterator i=map_edges.begin();i!=map_edges.end();i++) {
		vector<string> destinations=i->second;
		for (int j=0; j<destinations.size(); j++) { 
			situation_assessment_msgs::Edge e;
			e.source=i->first;
			e.destination=destinations[j];
			msg_edges.push_back(e);
			ROS_INFO("SPENCER_SYMBOLIC_MAP  - %s %s",e.source.c_str(),e.destination.c_str());
		}
	}
	g.nodes=msg_nodes;
	g.edges=msg_edges;
	res.graph=g;
	return true;

}

bool getLocations(situation_assessment_msgs::GetLocations::Request &req, situation_assessment_msgs::GetLocations::Response &res) {
	vector<string> node_names=spencer_map->getNodes();
	vector<geometry_msgs::Point> points;
	vector<geometry_msgs::Polygon> areas;

	for (int i=0; i<node_names.size(); i++) {
		NodeInfo node_info=spencer_map->getNodeInfo(node_names[i]);
		points.push_back(node_info.center);
		vector<geometry_msgs::Point> vertexs=node_info.vertexs;

		geometry_msgs::Polygon polygon;
		for (int j=0; j<vertexs.size();j++) {
			geometry_msgs::Point32 vertex_point;
			vertex_point.x=vertexs[j].x;
			vertex_point.y=vertexs[j].y;
			vertex_point.z=vertexs[j].z;
			polygon.points.push_back(vertex_point);
		}
		areas.push_back(polygon);
	}
	res.locations=node_names;
	res.centers=points;
	res.areas=areas;

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"spencer_symbolic_map");
	ros::NodeHandle node_handle;

	string doc_path,doc_name;
	node_handle.getParam("/situation_assessment/doc_path",doc_path);
	node_handle.getParam("/situation_assessment/doc_name",doc_name);

	ROS_INFO("SPENCER_SYMBOLIC_MAP  Parameters are:");
	ROS_INFO("SPENCER_SYMBOLIC_MAP  Doc path %s",doc_path.c_str());
	ROS_INFO("SPENCER_SYMBOLIC_MAP  Doc name %s",doc_name.c_str());

	spencer_map=new SpencerMap(node_handle,doc_path,doc_name);
	if (!spencer_map->calculateMapInfos()) return 0;

	ros::ServiceServer get_map_service=node_handle.advertiseService("situation_assessment/get_symbolic_map",getMap);
	ros::ServiceServer get_locations_service=node_handle.advertiseService("situation_assessment/get_locations",getLocations);
	ROS_INFO("SPENCER_SYMBOLIC_MAP  Starting symbolic map");
	ros::spin();

}
