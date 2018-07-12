//=========================================================================================//
 
// 										           //				
 
//                   		Pilotage du robot V 0.1					   //
 
//			     Final study project INSA 2018				   //
 
//				VICTOR TALBOT MIQ5					   //
 
// 3D object description, detection and segmentation for autonomous robot grasping         //
    
//=========================================================================================//
// 
// Dependencies
// * IIWA
// 

//===========================================================================================
// This code take control of the kuka
//===========================================================================================

#include "Pilotage.h" //contain all the declaration

using namespace std;

int main (int argc, char **argv) {


	// Initialize ROS
	ros::init(argc, argv, "CommandRobot");
	ros::NodeHandle nh("~");
	
	// ROS spinner.
	ros::AsyncSpinner spinner(1);
	spinner.start();
  
	//Robot connection
	iiwa_ros::iiwaRos iiwa_1;
	iiwa_1.init("iiwa_2");
	
	// Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
	bool use_cartesian_command;
	nh.param("use_cartesian_command", use_cartesian_command, false);
	
	// Dynamic parameter to choose the rate at wich this node should run
	double ros_rate;
	nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
	ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  
	geometry_msgs::PoseStamped command_cartesian_position;
	iiwa_msgs::JointPosition command_joint_position;
	//tf2::Quaternion q;
	

	while (ros::ok()) {

		if (iiwa_1.getRobotIsConnected()) {
	
			// Ask the user what he wants to do
			cout << "Wait for instruction (1: take picture, 2:garage, 3:stand by, 4: exit)" << endl;
			cin >> input ;

			switch (input){	
				case 1: {//picture position
					Moveto(iiwa_1,0.0,-3.14/2,0.0,0.0,0.0,-1.57,0.0);
					break;
				}
			
				case 2:	{//Parking
					Moveto(iiwa_1,0.0,0.0,0.0,0.0,0.0,0.0,0.0);		
					break;
				}

				case 3:	{//Wait postion
					Moveto(iiwa_1,0.0,3.14/6,0.0,1.57,0.0,0.0,0.0);	
					break;
				}

				case 4:	{//exit
					Moveto(iiwa_1,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        				return(0);
					break;
				}

				default:{
					cout << "error" << endl;
					break;
				}
			}
			sleepForMotion(iiwa_1, 2.0);
			loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.

		} else {
			ROS_WARN_STREAM("Robot is not connected...");
			ros::Duration(5.0).sleep(); // 5 seconds
		}
	}

}




