//=========================================================================================//
 
// 										           //				
 
//                   	       Pilotage du robot V 0.1					   //
 
//			     Final study project INSA 2018				   //
 
//				VICTOR TALBOT MIQ5					   //
 
// 3D object description, detection and segmentation for autonomous robot grasping         //
    
//=========================================================================================//

//=========================================================================================//
//			      Declaration for robot control
//                             Works for pilotage_kuka.cpp
//		     You can send my an email at : vpj.talbot@gmail.com
//=========================================================================================//


#include <iiwa_ros.h>
#include <cmath>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <string.h>
	
//===========================================================================================
// 					Global data
//===========================================================================================

bool new_pose = false, motion_done = false;
int input(0);
double thickness(0.0);
double add_thickness(0.0);
geometry_msgs::PoseStamped command_cartesian_position;
iiwa_msgs::JointPosition command_joint_position;

//Fonction designed by Raphael Schruoffeneger

void sleepForMotion(iiwa_ros::iiwaRos& iiwa, const double maxSleepTime) {
	double ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
	ros::Time start_wait = ros::Time::now();
	while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime)) {
		ros::Duration(0.5).sleep();
		ttd = iiwa.getTimeToDestinationService().getTimeToDestination();
	}

  	if (ttd > 0.0) {
		ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
		ros::Duration(ttd).sleep();
	} 
}


//Move the robot with articular command take the robot name and all seven joint data
//example : Moveto(iiwa_1,0.0,3.14/6,0.0,1.57,0.0,0.0,0.0);
	
void Moveto(iiwa_ros::iiwaRos& iiwa, double a1,double a2,double a3,double a4,double a5,double a6,double a7){

	while (!iiwa.getJointPosition(command_joint_position)) {}
	
	command_joint_position.position.a1 = a1;
	command_joint_position.position.a2 = a2;
	command_joint_position.position.a3 = a3;
	command_joint_position.position.a4 = a4;
	command_joint_position.position.a5 = a5;
	command_joint_position.position.a6 = a6;
	command_joint_position.position.a7 = a7;
	iiwa.setJointPosition(command_joint_position);

}








