//=========================================================================================//
 
// 										           //				
 
//                   	Point cloud saving for SR300	v1.1			           //
 
//			     Final study project INSA 2018				   //
 
//				VICTOR TALBOT MIQ5					   //
 
//    3D object description, detection and segmentation for autonomous robot grasping      //
    	
//=========================================================================================//
//This code take a pointcloud from a ros topic exemple for a sr300:
// rosrun "your package" printer_pointcloud_ros input:=/camera/depth_registered/points
// for any question mail vpj.talbot@gmail.com
//=========================================================================================//
// 
// Dependencies
// * ROS kinetic
// 
//
//=========================================================================================//

//=========================================================================================//
//					Include library
//=========================================================================================//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <librealsense/rs.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <cstdio>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <iostream>
#include <fstream>
#include <string>
#include <limits>

//===========================================================================================
// 					Global data
//===========================================================================================

//file number saving
int k=1;  //nits digit
int m=0;  //thousands digit
int c=0;  //hundred digit
int d=0;  //ten digit

struct MyPointType{

	union{
		float data[5];
		struct{
			float x;
			float y;
			float z;
			float rgb;
  			float index;
		};
	};

};

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
				   (float, index, index)

)



//names files system
const std::string prefix ="pcd";//files header
const std::string suffix = ".txt";//files extension


// loop boolean
bool _loop=false;



ros::Publisher pub;

//pointcloud declaration
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


//fonction taking pointcloud from ROS topic



void cloud_save (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){


	pcl::PointCloud<pcl::PointXYZRGB> in_cloud;
  	pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
  	pcl::PointCloud<pcl::PointXYZRGBL> cloud_neural_network;
  	pcl::PointCloud<pcl::PointXYZRGBL> out_cloud_2;
  	std::vector<int> indicies;
  	pcl::fromROSMsg(*cloud_msg, in_cloud);
  	pcl::fromROSMsg(*cloud_msg, cloud_neural_network);
  	pcl::fromROSMsg(*cloud_msg, out_cloud);
	
	pcl::PointCloud<MyPointType> cloud_neural_network_2;
	pcl::PointCloud<MyPointType> cloud_neural_network_3;
	int cpt=in_cloud.points.size ();
	ROS_INFO ("points %d", cpt);
	
	cloud_neural_network_2.points.resize (cpt);
	cloud_neural_network_2.width = cpt;
	cloud_neural_network_2.height = 1;


	for (size_t i = 0; i < in_cloud.points.size (); ++i){
	
	//in_cloud[i].x=in_cloud.points[i].x*1000;
	//in_cloud[i].y=in_cloud.points[i].y*1000;
	//in_cloud[i].z=in_cloud.points[i].z*1000;
	if (in_cloud.points[i].x < 0) in_cloud.points[i].x = -in_cloud.points[i].x ;

	//if (in_cloud.points[i].y < 0) in_cloud.points[i].y = -in_cloud.points[i].y ;
	//if (in_cloud.points[i].rgb <0) in_cloud.points[i].rgb=-in_cloud.points[i].rgb;
	if (in_cloud.points[i].rgb != in_cloud.points[i].rgb) in_cloud.points[i].rgb=0;
	if (cloud_neural_network_2.points[i].x != cloud_neural_network_2.points[i].x) cloud_neural_network_2.points[i].x=0;
	if (cloud_neural_network_2.points[i].y != cloud_neural_network_2.points[i].y) cloud_neural_network_2.points[i].y=0;
	if (cloud_neural_network_2.points[i].z != cloud_neural_network_2.points[i].z) cloud_neural_network_2.points[i].z=0;

	cloud_neural_network_2[i].x=in_cloud.points[i].x;
	cloud_neural_network_2[i].y=in_cloud.points[i].y;
	cloud_neural_network_2[i].z=in_cloud.points[i].z;
	cloud_neural_network_2[i].rgb=in_cloud.points[i].rgb;
	cloud_neural_network_2[i].index=i;

	
	}

  	pcl::removeNaNFromPointCloud(in_cloud, out_cloud, indicies);
  	//pcl::removeNaNFromPointCloud(cloud_neural_network, out_cloud_2, indicies);


	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	//pcl::PCLPointCloud2* cloud_without_NaN = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl_conversions::toPCL(*cloud_msg, *cloud);


	//pcl::removeNaNFromPointCloud(*cloud, *cloud_without_NaN, index); 	
	//name saving

	for (size_t i = 0; i < out_cloud.points.size (); ++i){
	
		//cloud_neural_network[i].x=out_cloud.points[i].x;
		//cloud_neural_network[i].y=out_cloud.points[i].y;
		//cloud_neural_network[i].z=out_cloud.points[i].z;
		//cloud_neural_network[i].rgb=out_cloud.points[i].rgb;
		//cloud_neural_network[i].label=i;

	}

	cpt=cloud_neural_network.points.size ();
	ROS_INFO ("points %d", cpt);
	std::stringstream ss;
	ss << prefix << m << c<<d<<k << ".txt";
	ROS_INFO ("Data saved to %s", ss.str ().c_str ());
	pcl::io::savePCDFile (ss.str (), cloud_neural_network_2);
	pcl::PCDWriter writer;
	//writer.writeASCII (ss.str (), *cloud);
	//writer.writeASCII (ss.str (), out_cloud_2);
	//writer.writeASCII ("1", out_cloud);
	//Saving in 4digits format need to get a amelioration exemple name: pcd0001.txt

	k++;
	if(k>9){

		k=0;
		d++;
		if(d>9){
			d=0;
			c++;
		}if(c>9){
			c=0;
			m++;
		}
	}
//waiting for user command
// to do amelioration with a second listener ros 
	std::cout << "Taking an other pointcloud 1: yes 2: no ?" <<  std::endl;
	int continu(0); 
	std::cin >> continu; 
	if(continu == 2) _loop =false;

	
}


int main(int argc, char **argv){
  
	ros::init(argc, argv, "printer_ros");
	ros::NodeHandle nh;	
 	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_save);
	ros::spin();
	return 0;
}
