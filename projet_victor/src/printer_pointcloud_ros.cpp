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

#include <limits>
//===========================================================================================
// 					Global data
//===========================================================================================

//file number saving
int k=1;  //nits digit
int m=0;  //thousands digit
int c=0;  //hundred digit
int d=0;  //ten digit

struct MyPointType
{
  float x;
  float y;
  float z;
	float rgb;
	float index;
};

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
  	std::vector<int> indicies;
  	pcl::fromROSMsg(*cloud_msg, in_cloud);
  	pcl::fromROSMsg(*cloud_msg, out_cloud);
	
	pcl::PointCloud<MyPointType> cloud_neural_network;
	int index=in_cloud.points.size ();
	ROS_INFO ("points %d", index);

	for (size_t i = 0; i < in_cloud.points.size (); ++i){
	
	in_cloud[i].x=in_cloud.points[i].x*1000;
	in_cloud[i].y=in_cloud.points[i].y*1000;
	in_cloud[i].z=in_cloud.points[i].z*1000;
	if (in_cloud.points[i].x < 0) in_cloud.points[i].x = -in_cloud.points[i].x ;

	if (in_cloud.points[i].y < 0) in_cloud.points[i].y = -in_cloud.points[i].y ;
	if (in_cloud.points[i].rgb <0) in_cloud.points[i].rgb=-in_cloud.points[i].rgb;
	if (in_cloud.points[i].rgb != in_cloud.points[i].rgb) in_cloud.points[i].rgb=0;

	}

  	pcl::removeNaNFromPointCloud(in_cloud, out_cloud, indicies);
/*
	for (size_t i = 0; i < out_cloud.points.size (); ++i){
	
	cloud_neural_network[i].x=out_cloud.points[i].x;
	cloud_neural_network[i].y=out_cloud.points[i].y;
	cloud_neural_network[i].z=out_cloud.points[i].z;
	cloud_neural_network[i].rgb=out_cloud.points[i].rgb;
	cloud_neural_network[i].index=i;

	}
*/
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	//pcl::PCLPointCloud2* cloud_without_NaN = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl_conversions::toPCL(*cloud_msg, *cloud);


	//pcl::removeNaNFromPointCloud(*cloud, *cloud_without_NaN, index); 	
	//name saving

	std::stringstream ss;
	ss << prefix << m << c<<d<<k << ".txt";
	ROS_INFO ("Data saved to %s", ss.str ().c_str ());
	
	pcl::PCDWriter writer;
	//writer.writeASCII (ss.str (), *cloud);
	writer.writeASCII (ss.str (), out_cloud);

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
