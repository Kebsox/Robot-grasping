//=========================================================================================//
 
// 										           //				
 
//                   		Point cloud saving	v1.0				   //
 
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
// This code take a picture from the sr300 when you use the Mbouton
// Save a point cloud and a a rgb image with format name for further processing
//===========================================================================================
 


#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 



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
	_color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );
        
 
     
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
	cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	imshow( WINDOW_RGB, rgb );
 	cvWaitKey( 1 );
 
	return true;
}

//===========================================================================================
// Save a png picture from the camera
//===========================================================================================

bool save_picture(){
	
	_depth_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::depth );
	_color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );
        
 
     
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


	cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );

	std::string path_rgb = prefix + std::to_string(m)+ std::to_string(c)+ std::to_string(d)+  std::to_string(k)+suffix_rgb;
	
	cv::imwrite(path_rgb.c_str(), rgb);
	cvWaitKey( 1 );

}

//===========================================================================================
// Create and save pointcloud from the camera
//===========================================================================================

bool create_point_cloud(){

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	cloud->width    = INPUT_WIDTH ;
	cloud->height   = INPUT_HEIGHT ;
	
	_depth_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::depth );
 	_depth_to_color     = _rs_camera.get_extrinsics(rs::stream::depth, rs::stream::color);
	_color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );
	float scale 	    = _rs_camera.get_depth_scale();

	const uint8_t * color_image = (const uint8_t *)_rs_camera.get_frame_data(rs::stream::color);
	const uint16_t one_meter = static_cast<uint16_t>(1.0f / _rs_camera.get_depth_scale());

	const uint16_t * depth_image = reinterpret_cast<const uint16_t *>(_rs_camera.get_frame_data(rs::stream::depth));
      

	for(int dy=0; dy<_depth_intrin.height; ++dy){


    		for(int dx=0; dx<_depth_intrin.width; ++dx){

			uint16_t _depth_value = depth_image[dy * _depth_intrin.width + dx];
			float _depth_in_meters = _depth_value * scale;

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
        		rs::float2 _depth_pixel = {(float)dx, (float)dy};
        		rs::float3 _depth_point = _depth_intrin.deproject(_depth_pixel, _depth_in_meters);
        		rs::float3 _color_point = _depth_to_color.transform(_depth_point);
        		rs::float2 _color_pixel = _color_intrin.project(_color_point);

			pcl::PointXYZRGB p;
			p.x = _depth_point.x;
			p.y = _depth_point.y;
			p.z = _depth_point.z;

			
			// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
       			const int cx = (int)std::round(_color_pixel.x), cy = (int)std::round(_color_pixel.y);
        		if(cx < 0 || cy < 0 || cx >= _color_intrin.width || cy >= _color_intrin.height){
            			p.r = 255;
            			p.g = 255;
            			p.b = 255;
        		} else{
           			
				// indexing into the colour stream can be somewhat convoluted
           			p.r = * (color_image + (cy * _color_intrin.width + cx) * 3);
           			p.g = * (color_image + (cy * _color_intrin.width + cx) * 3+1);            
           			p.b = * (color_image + (cy * _color_intrin.width + cx) * 3+2);
        		}

				
			// Push this new point into the Point Cloud
        		cloud->push_back(p);
		}

	}
	cloud->width    = INPUT_WIDTH ;
	cloud->height   = INPUT_HEIGHT ;
	std::string path = prefix + std::to_string(m)+ std::to_string(c)+ std::to_string(d)+  std::to_string(k)+suffix;
	pcl::io::savePCDFileASCII (path.c_str(), *cloud);

}
 
//===========================================================================================
// Main loop
//===========================================================================================
int main( ) try{

	//take the numerotation from the last time

	ifstream index("/home/kebsox/catkin_ws/index.txt");

	if(index){

		index >> k;
		cout << "k " << k << endl;

	}else{

    		cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << endl;

	}

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
 

	
       // Loop until someone left clicks on either of the images in either window.

       while( _loop ){

		if( _rs_camera.is_streaming( ) )
		_rs_camera.wait_for_frames( );
 
		display_next_frame( );

		if(_save){
			_save=false;
			create_point_cloud();
			std::cout << "Tout roule" << std::endl;	
			save_picture();
			k++;
			cout << "Continue 1: yes 2: no ?" <<  endl; 
			int continu(0); //On prepare une case mémoire pour stocker un entier
   			cin >> continu; //On fait entrer un nombre dans cette case
			if(continu == 2) _loop =false;


		}	
       }
 	//save the numbers of file create

	ofstream indexf("/home/kebsox/catkin_ws/index.txt");

	if(indexf){

		indexf << k << endl;

	}else{

    		cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << endl;

	}

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
