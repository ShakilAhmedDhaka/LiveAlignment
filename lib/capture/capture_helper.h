#pragma once

#ifndef _CAPTURE_HELPER_H_
#define _CAPTURE_HELPER_H_

#include <opencv2/opencv.hpp>
#include <k4a/k4a.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef NDEBUG
#   define M_Assert(Expr, Msg) \
    __M_Assert(#Expr, Expr, __FILE__, __LINE__, Msg)
#else
#   define M_Assert(Expr, Msg) ;
#endif


bool build_write_point_cloud
(
	const k4a_image_t point_cloud_image,
	const k4a_image_t color_image,
	const char* file_name = "\0",
	cv::Mat color_mat = cv::Mat(),
	cv::Mat depth_mat = cv::Mat(),
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = NULL
);

bool convert_k4a_image_to_cv_mat(k4a_image_t k4a_image, cv::Mat& cv_image, int cv_type);

#endif