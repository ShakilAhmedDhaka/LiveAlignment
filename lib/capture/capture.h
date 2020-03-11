#pragma once

#ifndef _CAPTURE_H_
#define _CAPTURE_H_

#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "k4a/k4a.h"
#include "global_helpers.h"



class Capture
{
public:
	cv::Mat color_mat, depth_mat;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

private:
	k4a_device_t device;
	const int32_t TIMEOUT_IN_MS = 2000;
	k4a_transformation_t transformation;
	k4a_capture_t capture;
	std::string file_name;
	uint32_t device_count;
	k4a_device_configuration_t config;
	k4a_calibration_t calibration;
	k4a_image_t depth_image;
	k4a_image_t color_image;

	cv::Mat transformed_depth_mat;
	int color_image_width_pixels;
	int color_image_height_pixels;

public:

	Capture(uint8_t deviceId = K4A_DEVICE_DEFAULT);
	~Capture();

	bool point_cloud_color_to_depth(std::string file_name = "\0");

	bool point_cloud_depth_to_color(k4a_image_t& transformed_depth_image, std::string file_name = "\0");

	bool capture_frame();

	// project a cloud pixel to a color frame pixel
	bool project_depthcam_to_colorcam(int depthcam_index, cv::Point& color_point);
	bool project_colorcam_to_depthcam(cv::Point colorcam_pixel, int& depthcam_index);
	//bool project_colorcam_to_depthcam(cv::Point colorcam_pixel, int& depthcam_index, cv::Mat& depth_im, int color_w, int color_h);


private:
	k4a_memory_destroy_cb_t* cb;

};

#endif