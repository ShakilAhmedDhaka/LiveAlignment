#pragma once

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

#include "global_helpers.h"


namespace VISH
{
	void mark_color_mat(cv::Mat& color_mat, cv::Point point, cv::Vec4b color, int radius);
	void mark_cloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, int point_index, cv::Vec3b color, int radius);
	

	class Visualizer
	{

	public:

		static int cloud_picked_index_static;
		static pcl::PointXYZRGB cloud_picked_point_static;
		static float clicked_point_depth_static;

		pcl::PointCloud<pcl::PointXYZRGB> cloud_val;
		boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer;

		Visualizer(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud);
		~Visualizer();
		
		// member function cannot be passed as callback function because every member function has implicit 'this'
		// object passed as parameter. but callback functions usually do not have 'this' as a parameter. therefore we 
		// have to use static function to pass as a callback function which does not have 'this' as a parameter as static
		// members does not belong to any particular object.
		static void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* cookie);
		
		void update_visualizer(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud);


	private:
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbv;



	};
}

#endif