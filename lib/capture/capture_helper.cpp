#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <boost/make_shared.hpp>
#include "capture_helper.h"


bool convert_k4a_image_to_cv_mat(k4a_image_t k4a_image, cv::Mat& cv_image, int cv_type)
{
	uint8_t* buffer = k4a_image_get_buffer(k4a_image);
	
	if (cv_type == CV_8UC4)
	{
		cv::Vec4b* color_buffer = reinterpret_cast<cv::Vec4b*> (buffer);
		cv_image = cv::Mat(k4a_image_get_height_pixels(k4a_image), k4a_image_get_width_pixels(k4a_image), CV_8UC4, color_buffer).clone();
		return true;
	}
	else if (cv_type == CV_16UC1)
	{
		uint16_t* depth_buffer = reinterpret_cast<uint16_t*>(buffer);
		cv_image = cv::Mat(k4a_image_get_height_pixels(k4a_image), k4a_image_get_width_pixels(k4a_image), CV_16UC1, depth_buffer).clone();
		return true;
	}
	
	return false;
}



bool build_write_point_cloud(const k4a_image_t point_cloud_image,
	const k4a_image_t color_image,
	const char* file_name, cv::Mat color_mat, cv::Mat depth_mat,
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud)
{

	int width = k4a_image_get_width_pixels(point_cloud_image);
	int height = k4a_image_get_height_pixels(color_image);

	
	cloud->width = width;
	cloud->height = height;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
	uint8_t* color_image_data = k4a_image_get_buffer(color_image);

	int count = 0;
	bool dense = true;
	pcl::PointXYZRGB* pt = &cloud->points[0];
	for (int i = 0; i < width * height; i++, pt++)
	{
		pcl::PointXYZRGB point;
		point.x = point_cloud_image_data[3 * i + 0];
		point.y = point_cloud_image_data[3 * i + 1];
		point.z = point_cloud_image_data[3 * i + 2];
		
		if (point.z == 0) count++;

		point.b = color_image_data[4 * i + 0];
		point.g = color_image_data[4 * i + 1];
		point.r = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		*pt = point;
	}

	std::cout << "Number of nan values: " << count << std::endl;


	if (file_name[0] != '\0')
	{
		#define PLY_START_HEADER "ply"
		#define PLY_END_HEADER "end_header"
		#define PLY_ASCII "format ascii 1.0"
		#define PLY_ELEMENT_VERTEX "element vertex"

		// save to the ply file
		std::ofstream ofs(file_name); // text mode first
		ofs << PLY_START_HEADER << std::endl;
		ofs << PLY_ASCII << std::endl;
		ofs << PLY_ELEMENT_VERTEX << " " << cloud->points.size() << std::endl;
		ofs << "property float x" << std::endl;
		ofs << "property float y" << std::endl;
		ofs << "property float z" << std::endl;
		ofs << "property uchar red" << std::endl;
		ofs << "property uchar green" << std::endl;
		ofs << "property uchar blue" << std::endl;
		ofs << PLY_END_HEADER << std::endl;
		ofs.close();

		std::stringstream ss;
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			// image data is BGR
			ss << (float)cloud->points[i].x << " " << (float)cloud->points[i].y << " " << (float)cloud->points[i].z;
			ss << " " << (float)cloud->points[i].r << " " << (float)cloud->points[i].g << " " << (float)cloud->points[i].b;
			ss << std::endl;
		}
		std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
		ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
	}

	return true;
}