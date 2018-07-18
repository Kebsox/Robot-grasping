//=========================================================================================//
 
// 										           //				
 
//                   	Point cloud saving for R200	v1.1			           //
 
//			     Final study project INSA 2018				   //
 
//				VICTOR TALBOT MIQ5					   //
 
// 3D object description, detection and segmentation for autonomous robot grasping         //
    
//=========================================================================================//
// 
// Dependencies
// * LibRealSense
// * OpenCV
//


//===========================================================================================
//            This code take a picture from the sr300 when you use the Mbouton
// 	  Save a point cloud and a a rgb image with format name for further processing
//		     You can send my an email at : vpj.talbot@gmail.com
//===========================================================================================
 


#include <librealsense/rs.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <cstdio>


using namespace std;
using namespace rs;
 
//===========================================================================================
// 					Global data
//===========================================================================================


// Window size and frame rate
int const INPUT_WIDTH      = 640;
int const INPUT_HEIGHT     = 480;
int const FRAMERATE        = 30;
 

// Named windows

char* const WINDOW_RGB     = "RGB Image";
char* const WINDOW_DEPTH   = "Depth Image";
 
// Camera déclaration

context      _rs_ctx;
device&      _rs_camera = *_rs_ctx.get_device( 0 );
intrinsics   _depth_intrin;
intrinsics   _color_intrin;
extrinsics   _depth_to_color;
 
//constant for files name
const std::string prefix ="pcd";//files header
const std::string suffix = ".txt";//files extension
const std::string suffix_rgb = "r.png";//files extension for png data

int k=0;
int m=0;
int c=0;
int d=0;

//loop boolean

bool         _loop = true;
bool	     _save = false;

//===========================================================================================
//Initialize the application state. Upon success will return the static app_state vars address
//===========================================================================================

 
bool initialize_streaming( ){

	bool success = false;

	if( _rs_ctx.get_device_count( ) > 0 ){

		_rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
		_rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );

		_rs_camera.start( );
		success = true;
	}
	return success;
}
 
 
 
 
//===========================================================================================
// If the left mouse button was clicked on either image, stop streaming and close windows.
//===========================================================================================

static void onMouse( int event, int x, int y, int, void* window_name ){

	if( event == cv::EVENT_LBUTTONDOWN ){
		_loop = false;
	}
}
 
//===========================================================================================
// If the central mouse button was clicked on either image, stop streaming and take a picture.
//===========================================================================================
static void onKeybord( int event, int x, int y, int, void* window_name ){

	if( event == cv::EVENT_MBUTTONDOWN ){
		_save = true;
	}
} 



//===========================================================================================
// Create the depth and RGB windows, set their mouse callbackss.
// Required if we want to create a window and have the ability to use it in
// different functions
//===========================================================================================

void setup_windows( ){

	cv::namedWindow( WINDOW_RGB, 0 );
	cv::namedWindow( WINDOW_DEPTH, 0 );
	cv::setMouseCallback( WINDOW_DEPTH, onKeybord, WINDOW_DEPTH );
	cv::setMouseCallback( WINDOW_RGB, onKeybord, WINDOW_RGB );
}
 
 
//===========================================================================================
// Called every frame gets the data from streams and displays them using OpenCV.
//===========================================================================================

bool display_next_frame( ){


 
	_depth_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::depth );

        
 
     
	// Create color image
	cv::Mat rgb( _color_intrin.height,
                            _color_intrin.width,
                            CV_8UC3,
                            (uchar *)_rs_camera.get_frame_data( rs::stream::color ) );
 

	// Create depth image
	cv::Mat depth16( _depth_intrin.height,
                                  _depth_intrin.width,
                                  CV_16U,
                                  (uchar *)_rs_camera.get_frame_data( rs::stream::depth ) );
 	// < 800
 
	cv::Mat depth8u = depth16;
	depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );

	imshow( WINDOW_DEPTH, depth8u );
	cvWaitKey( 1 );
	//cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	imshow( WINDOW_RGB, rgb );
 	cvWaitKey( 1 );
 
	return true;
}


 
//===========================================================================================
// Main loop
//===========================================================================================
int main( ) try{

	

	rs::log_to_console( rs::log_severity::warn );
 
	if( !initialize_streaming( ) ){

		std::cout << "Unable to locate a camera" << std::endl;
		rs::log_to_console( rs::log_severity::fatal );
		return EXIT_FAILURE;
	}
	printf("\nUsing device 0, an %s\n", _rs_camera.get_name());
	printf("    Serial number: %s\n", _rs_camera.get_serial());
	printf("    Firmware version: %s\n", _rs_camera.get_firmware_version());
 	setup_windows( );
 

	_color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );


	printf("\n height %d\n", _color_intrin.height);

       	printf("\n fx %d\n", _color_intrin.fx);
       	printf("\n fy %d\n", _color_intrin.fx);



       _rs_camera.stop( );
       cv::destroyAllWindows( );
        
 
       return EXIT_SUCCESS;
 
}
catch( const rs::error & e ){

	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch( const std::exception & e ){

	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
