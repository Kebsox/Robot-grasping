/////////////////////////////////////////////////////////////////////////////////////////////
 
// 										                   //				
 
//                   		Point cloud saving	v1.1				   //
 
//			Projet de fin d'étude INSA 2018					   //
 
//				VICTOR TALBOT MIQ5					   //
 
// 3D object description, detection and segmentation for autonomous robot grasping         //
    
/////////////////////////////////////////////////////////////////////////////////////////////
// 


#include <librealsense/rs.hpp>

#include <ros/ros.h>
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace rs;


int i(1);

int main(int argc, char **argv) try{
 
	rs::context ctx;
	printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
	if(ctx.get_device_count() == 0) return EXIT_FAILURE;

	// This tutorial will access only a single device, but it is trivial to extend to multiple devices
	rs::device * dev = ctx.get_device(0);
	printf("\nUsing device 0, an %s\n", dev->get_name());
	printf("    Serial number: %s\n", dev->get_serial());
	printf("    Firmware version: %s\n", dev->get_firmware_version());

	dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
	dev->enable_stream(rs::stream::color, rs::preset::best_quality);
	dev->start();




	// First we create a new point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Wait until the device is ready
	dev->wait_for_frames();

	// Retrieve our images
	const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
	const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

	rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
	rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
	rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
	float scale = dev->get_depth_scale();



	for(int dy=0; dy<depth_intrin.height; ++dy){

		for(int dx=0; dx<depth_intrin.width; ++dx){

			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
			float depth_in_meters = depth_value * scale;

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			if(depth_value == 0) continue;

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
			rs::float3 color_point = depth_to_color.transform(depth_point);
			rs::float2 color_pixel = color_intrin.project(color_point);

			// Create a new point
			pcl::PointXYZRGB p;
			p.x = depth_point.x;
			p.y = depth_point.y;
			p.z = depth_point.z;

			// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
			std::cout << "Test" << std::endl;

cout << "cx vaut : " << cx << endl;
cout << "cy vaut : " << cy << endl;

			if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height){

				p.r = 255;
 				p.g = 255;
				p.b = 255;
cout << "r vaut : " << p.r << endl;
			std::cout << "Test1.1" << std::endl;
			}else{
				// indexing into the colour stream can be somewhat convoluted
				p.r = * (color_image + (cy * color_intrin.width + cx) * 3);
				p.g = * (color_image + (cy * color_intrin.width + cx) * 3+1);            
				p.b = * (color_image + (cy * color_intrin.width + cx) * 3+2);
			std::cout << "Test1.2" << std::endl;
			}

cout << "x vaut : " << p.x << endl;
cout << "y vaut : " << p.y << endl;
cout << "z vaut : " << p.z << endl;
cout << "r vaut : " << p.r << endl;
cout << "g vaut : " << p.g << endl;
cout << "b vaut : " << p.b << endl;
cout << "color_image vaut : " << color_image << endl;
cout << "color_intr vaut : " << color_intrin.width << endl;


				// Push this new point into the Point Cloud

cloud->push_back(p);
/*
				cloud.points[i].x = p.x;
				cloud.points[i].y = p.y;
				cloud.points[i].z = p.z;
				cloud.points[i].r = p.r;
				cloud.points[i].g = p.g;
				cloud.points[i].b = p.b;
				i++;
*/
		}
	}


	//pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
 	//std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;


}



catch(const rs::error & e){

	// Method calls against librealsense objects may throw exceptions of type rs::error
	printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
	printf("    %s\n", e.what());
	return EXIT_FAILURE;
} 
