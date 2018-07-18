//=========================================================================================//
 
// 										           //				
 
//                   	RGB image saving for SR300	v1.1			           //
 
//			     Final study project INSA 2018				   //
 
//				VICTOR TALBOT MIQ5					   //
 
// 3D object description, detection and segmentation for autonomous robot grasping         //
    
//=========================================================================================//
//This code take a rgb-image from a ros topic exemple for a sr300:
// input:=/camera/depth/image_rect_raw
//for any question mail vpj.talbot@gmail.com
//=========================================================================================//
// 
// Dependencies
// * ROS kinetic
// * OpenCV
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
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <cstdio>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>


//===========================================================================================
// 					Global data
//===========================================================================================

//file number saving
int k=1;  //nits digit
int m=0;  //thousands digit
int c=0;  //hundred digit
int d=0;  //ten digit

//names files system
const std::string prefix ="pcd";//files header
const std::string suffix = "d.png";//files extension for png data

//loop boolean

bool         _loop = true;

cv::Mat product;

//Space interuption

#define SPACE_KEY (32)

    template<typename T> 
    inline std::string toString( const T& ao_Obj )
    {
      std::stringstream lo_stream;

      lo_stream << ao_Obj;

      return lo_stream.str();
    }


//fonction taking image from ROS topic 
void depth_callback (const sensor_msgs::ImageConstPtr& msg){


	cv_bridge::CvImagePtr cv_ptr;

	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1); //Convert image from ros message to open cv data
    	}
    	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//Visualition for focus help
	product = cv_ptr->image.mul(32.5);
	cv::imshow("OpenCV viewer uEye RGB", product);
	int key = cv::waitKey( 50 );

	//if space press take a picture
	if ( key == SPACE_KEY ){
	static int count = 0;
	//Save a corect name for neural network processing

	std::string path_rgb = prefix + std::to_string(m)+ std::to_string(c)+ std::to_string(d)+  std::to_string(k)+suffix;
       	cv::imwrite(path_rgb.c_str() , product);
	std::cout << "frame captured as:" << path_rgb.c_str()<< std::endl;
	k++;
	//Saving in 4digits format need to get a amelioration exemple name: pcd0001r.png
	if(k>9){

		k=0;
		d++;
		if(d>9){
			d=0;
			c++;
		}
		if(c>9){
			c=0;
			m++;
		}
	}
    }
	
}

//Main loop wait for frame for input topic

int main(int argc, char **argv){
  
	ros::init(argc, argv, "printer_depth_ros");
	ros::NodeHandle nh;

	//Uncoment to use SR300 by default
	//image_topic = "/camera/depth/image_rect_raw";
	//ros::Subscriber sub = nh.subscribe ("image_topic", 1, image_callback);

  	ros::Subscriber sub = nh.subscribe ("input", 1, depth_callback); //go to the processing part leave when you use ctrl+c


	ros::spin();
	return 0;
}



