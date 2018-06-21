/////////////////////////////////////////////////////////////////////////////////////////////
 
// 										           //				
 
//                   		Point cloud saving	v0.1				   //
 
//			Projet de fin d'étude INSA 2018					   //
 
//				VICTOR TALBOT MIQ5					   //
 
// 3D object description, detection and segmentation for autonomous robot grasping         //
    
/////////////////////////////////////////////////////////////////////////////////////////////
// 
// Dependencies
// * LibRealSense
// * OpenCV
//
/////////////////////////////////////////////////////////////////////////////
// This code take a picture from the sr300 when you use the Mbouton
// Save a point cloud and a a rgb image
/////////////////////////////////////////////////////////////////////////////
 
#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 



using namespace std;
using namespace rs;
 
 
// Window size and frame rate
int const INPUT_WIDTH      = 640;
int const INPUT_HEIGHT     = 480;
int const FRAMERATE        = 60;
 

// Compteur de points

int cpt_global(0);
int cpt_local(0);




// Named windows

char* const WINDOW_RGB     = "RGB Image";
 
 
context      _rs_ctx;
device&      _rs_camera = *_rs_ctx.get_device( 0 );
intrinsics   _depth_intrin;
intrinsics   _color_intrin;
bool         _loop = true;
bool	     _save = false;
 
 
// Initialize the application state. Upon success will return the static app_state vars address
 
bool initialize_streaming( ){

	bool success = false;

	if( _rs_ctx.get_device_count( ) > 0 ){

//		_rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
//		_rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
		_rs_camera.enable_stream(rs::stream::color, rs::preset::best_quality);
		_rs_camera.enable_stream(rs::stream::depth, rs::preset::best_quality);
		_rs_camera.start( );
		success = true;
	}
	return success;
}
 
 
 
 
/////////////////////////////////////////////////////////////////////////////
// If the left mouse button was clicked on either image, stop streaming and close windows.
/////////////////////////////////////////////////////////////////////////////
static void onMouse( int event, int x, int y, int, void* window_name ){

	if( event == cv::EVENT_LBUTTONDOWN ){
		_loop = false;
	}
}
 

static void onKeybord( int event, int x, int y, int, void* window_name ){

	if( event == cv::EVENT_MBUTTONDOWN ){
		_save = true;
	}
} 



/////////////////////////////////////////////////////////////////////////////
// Create the depth and RGB windows, set their mouse callbackss.
// Required if we want to create a window and have the ability to use it in
// different functions
/////////////////////////////////////////////////////////////////////////////
void setup_windows( ){

	cv::namedWindow( WINDOW_RGB, 0 );
	cv::setMouseCallback( WINDOW_RGB, onMouse, WINDOW_RGB );
	cv::setMouseCallback( WINDOW_RGB, onKeybord, WINDOW_RGB );
}
 
 
/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using OpenCV.
/////////////////////////////////////////////////////////////////////////////
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


	cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	imshow( WINDOW_RGB, rgb );
 	cvWaitKey( 1 );
 
	return true;
}


bool create_point_cloud(){
	
	cpt_local=0;
	// Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)_rs_camera.get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)_rs_camera.get_frame_data(rs::stream::color);


//std::cerr  <<  depth_intrin.height << " depth" << std::endl;


	// Retrieve camera parameters for mapping between depth and color

        rs::intrinsics depth_intrin = _rs_camera.get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = _rs_camera.get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = _rs_camera.get_stream_intrinsics(rs::stream::color);
        float scale = _rs_camera.get_depth_scale();

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.is_dense = true;


	
//std::cerr << "test 1.5"  << std::endl;
	for(int dy=0; dy < depth_intrin.height; ++dy){


	//std::cerr << "test 2.1"  << std::endl;

		for(int dx=0; dx < depth_intrin.width; ++dx){


			//std::cerr << "test 2.2"  << std::endl;
			cpt_local=cpt_local+1;
			//cout << "a vaut : " << cpt_local << endl;
			// Retrieve the 16-bit depth value and map it into a depth in meters
                	uint16_t depth_value = dy * depth_intrin.width + dx;

                	float depth_in_meters = depth_value * scale;

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			//cout << "b vaut : " << depth_value << endl;
			

			//cout << "dx vaut : " << dx << endl;
			//cout << "dy vaut : " << dy << endl;
			//cout << "depth vaut : " << depth_intrin.width << endl;
			//cout << "res vaut : " << depth_value << endl;




                	if(depth_value == 0)
                   		 continue;

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
			rs::float3 color_point = depth_to_color.transform(depth_point);
			rs::float2 color_pixel = color_intrin.project(color_point);

			cout << "x vaut : " << depth_point.x << endl;
			cout << "y vaut : " << depth_point.y << endl;
			cout << "z : " << depth_point.z << endl;

			// Use the color from the nearest color pixel
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
				
				if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
                   			 continue;

			uint8_t r, g, b;
                	r = *(color_image + (cy * color_intrin.width + cx) * 3);
                	g = *(color_image + (cy * color_intrin.width + cx) * 3 + 1);
                	b = *(color_image + (cy * color_intrin.width + cx) * 3 + 2);
                	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

			//std::cerr << "test 2"  << std::endl;

			cloud.points[cpt_local].x = 1;
			cloud.points[cpt_local].y = depth_point.y;
			cloud.points[cpt_local].z = depth_point.z;
			cloud.points[cpt_local].rgb = *reinterpret_cast<float*>(&rgb);

			//cout << "r vaut : " << r << endl;
			//cout << "g vaut : " << g << endl;
			//cout << "b vaut : " << b << endl;
			//cout << "cpt_local vaut : " << cpt_local << endl;
			



			
		}
	}

	//pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	//std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
	//std::cerr << "compteur " << cpt_local  << std::endl;
}
 
/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
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
 

	
       // Loop until someone left clicks on either of the images in either window.

       while( _loop ){

		if( _rs_camera.is_streaming( ) )
		_rs_camera.wait_for_frames( );
 
		display_next_frame( );

		if(_save){
			_save=false;
			create_point_cloud();
			std::cout << "Tout roule" << std::endl;	

		}
       }
 
 	std::cout << "Tout roule mais bof" << std::endl;
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
