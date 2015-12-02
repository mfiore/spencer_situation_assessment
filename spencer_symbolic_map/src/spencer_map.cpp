/* 
spencer_map.cpp
author Michelangelo Fiore

*/

#include <spencer_symbolic_map/spencer_map.h>

SpencerMap::SpencerMap(ros::NodeHandle node_handle, string doc_path, string doc_name):node_handle_(node_handle),doc_path_(doc_path),doc_name_(doc_name) {

}

//gets map informations from yaml files
bool SpencerMap::calculateMapInfos() {

	// ROS_INFO("SPENCER_SYMBOLIC_MAP  Connecting to Add Area in spencer map");
	// add_area_client_=node_handle_.serviceClient<situation_assessment_msgs::AddArea>("situation_assessment/add_area");
	// add_area_client_.waitForExistence();
	// ROS_INFO("SPENCER_SYMBOLIC_MAP  Connected to add area in spencer map");

	
	ROS_INFO("SPENCER_SYMBOLIC_MAP  Calculating map information");

	tinyxml2::XMLDocument mainDoc,partsDoc;
	string main_doc_path,parts_doc_path;
	main_doc_path=doc_path_+doc_name_+".xml";
	parts_doc_path=doc_path_+doc_name_+"_parts.xml";
	int error1=mainDoc.LoadFile( main_doc_path.c_str() );
	int error2=partsDoc.LoadFile(parts_doc_path.c_str());

	if (error1!=0) {
		ROS_ERROR("Cannot find main map file");
		return false;
	}
	if (error2!=0) {
		ROS_INFO("SPENCER_SYMBOLIC_MAP  Cannot find parts map file");
		return false;
	}
	else {
		ROS_INFO("SPENCER_SYMBOLIC_MAP  Found maps. Starting parsing");
	}
	string resolutionString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("resolution")->GetText();
	boost::trim(resolutionString);
	double resolution=boost::lexical_cast<double>(resolutionString);
	cout<<"resolution is "<<resolution<<"\n";

	string xString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("origin")->FirstChildElement("x")->GetText();
	string yString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("origin")->FirstChildElement("x")->GetText();

	boost::trim(xString);
	boost::trim(yString);

	double map_origin_x=boost::lexical_cast<double>(xString);
	double map_origin_y=boost::lexical_cast<double>(yString);

	tinyxml2::XMLNode *partNode=partsDoc.FirstChildElement("parts")->FirstChild();
	while (partNode!=NULL) {
		NodeInfo aMap;

		string name=partNode->FirstChildElement("name")->GetText();

		string cx=partNode->FirstChildElement("center")->FirstChildElement("x")->GetText();
		string cy=partNode->FirstChildElement("center")->FirstChildElement("y")->GetText();
		boost::trim(cx);
		boost::trim(cy);

		double centerX=boost::lexical_cast<double>(cx);
		double centerY=boost::lexical_cast<double>(cy);

		aMap.name=name;
		geometry_msgs::Point center;
		center.x=map_origin_x+(centerX*resolution);
		center.y=map_origin_y+(centerY*resolution);
		aMap.center=center;

		tinyxml2::XMLNode *vertexNode=partNode->FirstChildElement("annotation")->FirstChildElement("vertex");
		vector<geometry_msgs::Point> vertexs;
		while (vertexNode!=NULL) {
			string svx=vertexNode->FirstChildElement("x")->GetText();
			string svy=vertexNode->FirstChildElement("y")->GetText();
			boost::trim(svx);
		    boost::trim(svy);
			double vx=boost::lexical_cast<double>(svx);
			double vy=boost::lexical_cast<double>(svy);
			geometry_msgs::Point v;
			v.x=map_origin_x+(vx*resolution);
			v.y=(map_origin_y+(vy*resolution));
			vertexs.push_back(v);
			vertexNode=vertexNode->NextSibling();
		}
		aMap.vertexs=vertexs;

		tinyxml2::XMLDocument partInfoNode;
		string partInfoName=doc_path_+doc_name_+"_"+name+".xml";
		ROS_INFO("SPENCER_SYMBOLIC_MAP  Loading map %s",partInfoName.c_str());
		if (partInfoNode.LoadFile(partInfoName.c_str())!=0) {
			ROS_ERROR("Part document not found");
			return false;
		}
		tinyxml2::XMLNode *originNode=partInfoNode.FirstChildElement("map")->FirstChildElement("origin");

		string ox=originNode->FirstChildElement("x")->GetText();
		string oy=originNode->FirstChildElement("y")->GetText();
		boost::trim(ox);
		boost::trim(oy);
		double originX=boost::lexical_cast<double>(ox);
		double originY=boost::lexical_cast<double>(oy);
		
		geometry_msgs::Point origin;
		origin.x=originX+map_origin_x;
		origin.y=(originY+map_origin_y);

		node_info_[name]=aMap;

		nodes_.push_back(name);

		// vector<geometry_msgs::Point32> area_vertexs;
		// for (int i=0; i<vertexs.size();i++) {
		// 	geometry_msgs::Point32 v;
		// 	v.x=map_origin_x+(vertexs[i].x*resolution);
		// 	v.y=-map_origin_y+(vertexs[i].y*resolution);
		// 	v.z=0;
		// 	area_vertexs.push_back(v);
		// }
		// geometry_msgs::Point32 v;
		// v.x=vertexs[0].x;
		// v.y=vertexs[0].y;
		// area_vertexs.push_back(v);

		// geometry_msgs::Polygon polygon;
		// polygon.points=area_vertexs;

		//monitors the area in situation assessment 


		// situation_assessment_msgs::AddArea addAreaRq;
		// addAreaRq.request.name=name;
		// addAreaRq.request.area=polygon;

		// ROS_INFO("SPENCER_SYMBOLIC_MAP about to call add area");
		// if (add_area_client_.call(addAreaRq)) {
		// ROS_INFO("SPENCER_SYMBOLIC_MAP  Called addArea");
		// }
		// else {
		// 	ROS_WARN("Couldn't add area");
		// }

		partNode=partNode->NextSibling();
	}

	ROS_INFO("SPENCER_SYMBOLIC_MAP  Creating Edges");

	tinyxml2::XMLDocument graph_doc;
	string graph_doc_path;
	graph_doc_path=doc_path_+doc_name_+"_graph.xml";
	int error=graph_doc.LoadFile( graph_doc_path.c_str() );
	if (error==0) {
		tinyxml2::XMLNode *edge_node=graph_doc.FirstChildElement("graph")->FirstChild();
		while (edge_node!=NULL) {
			// tinyxml2::XMLNode *edge_element=edge_node->FirstChildElement();
			string n1=edge_node->FirstChildElement()->GetText();
		ROS_INFO("SPENCER_SYMBOLIC_MAP  Found document");

			// edge_element=edge_element->NextSibling();
			string n2=edge_node->LastChildElement()->GetText();

			vector<string> n1_edges=edges_[n1];
			n1_edges.push_back(n2);
			edges_[n1]=n1_edges;

			ROS_INFO("SPENCER_SYMBOLIC_MAP  Edge %s %s",n1.c_str(),n2.c_str());
			edge_node=edge_node->NextSibling();
		}
	}
	else {
		ROS_ERROR("Can't find graph doc");
		return false;
	}
	ROS_INFO("SPENCER_SYMBOLIC_MAP  Parsed maps");
	return true;

}

vector<string> SpencerMap::getNodes() {
	return nodes_;
}
map<string,vector<string> > SpencerMap::getEdges() {
	return edges_;
}

NodeInfo SpencerMap::getNodeInfo(string node) {
	return node_info_[node];
}
