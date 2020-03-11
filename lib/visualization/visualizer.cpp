#include "visualizer.h"



// static member vairables have to be defined outside of the class and some other main function
int VISH::Visualizer::cloud_picked_index_static = 0;
pcl::PointXYZRGB VISH::Visualizer::cloud_picked_point_static = pcl::PointXYZRGB(0, 0, 0);
float VISH::Visualizer::clicked_point_depth_static = 0;

namespace VISH
{
	void mark_color_mat(cv::Mat& color_mat, cv::Point point, cv::Vec4b color, int radius)
	{
		if (!GLOBAL_HELPERS::check_out_scope(color_mat, point)) return;

		int height = color_mat.size().height;
		int width = color_mat.size().width;
		int radius_h = std::min(radius, std::min(point.y, height - point.y));
		int radius_w = std::min(radius, std::min(point.x, width - point.x));
		
		radius = std::min(radius_h, radius_w);

		for (int i = point.y - radius; i <= point.y + radius; i++)
		{
			for (int j = point.x - radius; j <= point.x + radius; j++)
			{
				color_mat.at<cv::Vec4b>(i, j) = cv::Vec4b(color[0], color[1], color[2]);
			}
		}
	}

	void mark_cloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, int point_index, cv::Vec3b color, int radius)
	{
		std::cout << "point_index: " << point_index << std::endl;
		for (int i = point_index - radius * cloud->width; i <= point_index + radius * cloud->width; i += cloud->width)
		{
			for (int j = -radius; j <= radius; j++)
			{
				cloud->points[i + j].r = color[2];
				cloud->points[i + j].g = color[1];
				cloud->points[i + j].b = color[0];
			}
		}
	}



	void Visualizer::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* cookie)
	{
		int idx = event.getPointIndex();
		if (idx == -1)  return;

		pcl::PointXYZ picked_pt;
		event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);

		pcl::PointCloud<pcl::PointXYZRGB> cloud_;
		cloud_ = *reinterpret_cast<pcl::PointCloud<pcl::PointXYZRGB>*> (cookie);
		int actual_index;
		GLOBAL_HELPERS::get_actual_cloud_index(cloud_, picked_pt, actual_index);
		PCL_INFO("Point index picked: %d (real: %d) - [%f, %f, %f]\n", idx, actual_index, picked_pt.x, picked_pt.y, picked_pt.z);

		//actual_index = idx;
		cloud_picked_index_static = actual_index;
		cloud_picked_point_static = cloud_.points[actual_index];
		clicked_point_depth_static = picked_pt.z;
	}


	/// CLASS DEFINITIONS




	Visualizer::Visualizer(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud)
	{	
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
		viewer->setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, -1.0, 0);
		viewer->setBackgroundColor(0, 0, 0);
		rgbv = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud);

		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgbv, "rgbv");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			3, "rgbv");

		cloud_picked_index_static = 0;
		clicked_point_depth_static = 0;
		cloud_val = *cloud;
		viewer->registerPointPickingCallback(&pointPickingEventOccurred, static_cast<void*> (&cloud_val));
	}


	Visualizer::~Visualizer()
	{

	}




	void Visualizer::update_visualizer(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud)
	{
		rgbv = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud);
		viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgbv, "rgbv");
	}



}