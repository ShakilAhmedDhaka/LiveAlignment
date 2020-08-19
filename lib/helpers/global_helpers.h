#pragma once

#ifndef _GLOBAL_HELPERS_H_
#define _GLOBAL_HELPERS_H_

#include <iostream>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/surface/organized_fast_mesh.h>


#include <opencv2/opencv.hpp>


namespace GLOBAL_HELPERS
{



struct path_leaf_string
{
	std::string operator()(const boost::filesystem::directory_entry& entry) const
	{
		return entry.path().leaf().string();
	}
};

// check if a point if out of the boundaries of the given image matrix
bool check_out_scope(cv::Mat color_mat, cv::Point point);

// callback functon for cv onmouseclick. pass a cv::point pointer casted as void.
void cvonMouseClick(int event, int x, int y, int, void* point);


void get_actual_cloud_index(pcl::PointCloud<pcl::PointXYZRGB>& cloud_, pcl::PointXYZ& picked_pt,int& actual_index);

char* concat(char* ch1, const char* ch2);

bool is_file_exist(std::string fileName);


// tokenizes the string and returns the last token
std::vector<std::string> get_tokens(std::string str, char* delimeter, int& sz);

std::string get_pair_save_name(char** argv, int no_1, int no_2);

// converts a cloud from pcl format to eigen format
void pclToEigenVector(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
	std::vector<Eigen::Vector3d>& vertices);
// converts the color values of a cloud from pcl format to eigen format
void pclToEigenVector_color(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
	std::vector<Eigen::Vector3d>& vertices);


// converts a cloud from eigen format to pcl format
void eigenVector_to_pclCloud(std::vector<Eigen::Vector3d>& transformed,
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud);


// converts a cloud from eigen format to pcl format
void eigenVector_color_to_pclCloud(std::vector<Eigen::Vector3d>& transformed,
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud);



void copyCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_in, 
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_out);


class Global_helpers
{
public:

	struct CapturedData
	{
		cv::Mat color_mat, depth_mat;
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
	};


	struct TagPoints
	{
		cv::Point point;
		Eigen::Vector3d point3D;
		Eigen::Vector3d centerPoint3D;
		int tag; //id
		int pos;
		int cloud_index;
		int center_cloud_index;
		int frame_serial;
		double centerx;
		double centery;

		bool operator < (const TagPoints& a)const
		{
			if (tag == a.tag)	return pos < a.pos;
			return tag < a.tag;
		};
	};


	Global_helpers();
	~Global_helpers();


	void set_path(std::string directory);
	// reads all the filename in given directory and stores the names in vector
	static void read_directory(const std::string& name, std::vector<std::string>& v);
	// returns the maximum among the number of pointclouds or color or depth images
	static int get_serial(std::string directory);
	// generates a filename suitable to save next bunch of images and pointclouds and meshes
	static std::string get_new_filename(std::string directory);
	
	static void write_pcd(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string full_path);
	static void write_pcd(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string filepath, std::string filename);


	static void write_ply(pcl::PointCloud<pcl::PointXYZRGB> cloud, bool binary, bool use_camera, std::string full_path);
	static void write_ply(pcl::PointCloud<pcl::PointXYZRGB> cloud, bool binary, bool use_camera, std::string filepath, std::string filename);
	
	static void write_cvImage(cv::Mat image, std::string full_path, std::string ext);
	static void write_cvImage(cv::Mat image, std::string filepath , std::string filename,  std::string ext);
	
	static void pointcloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string full_path);
	static void pointcloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string filepath, std::string filename);


private:
	std::string path;
	std::chrono::high_resolution_clock::time_point cur_time;
	std::string now;
	

};

// convert a 3d point from pcl format to eigen format
void pclToEigen(pcl::PointXYZ& picked_pt, Global_helpers::TagPoints& tags);


void insert_virtual_points(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags);


struct PlySaver
{

	PlySaver(pcl::PointCloud<pcl::PointXYZRGB> cloud, cv::Mat color, cv::Mat depth, bool binary, bool use_camera, std::string directory) :
		cloud_(cloud), color_(color), depth_(depth), binary_(binary), use_camera_(use_camera), directory_(directory)
	{
	}

	pcl::PointCloud<pcl::PointXYZRGB> cloud_;
	cv::Mat color_;
	cv::Mat depth_;
	bool binary_;
	bool use_camera_;
	std::string directory_;

	std::vector<Global_helpers::TagPoints> feature_points;
};


// takes input files from a directory specified in argv
int take_input(int argc, char** argv, int pc_serial, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, cv::Mat& color, cv::Mat& depth,
	std::vector<Global_helpers::TagPoints>& tags);

// takes input of a set of files: one element of the set =  one point cloud, rgb file, one feature points file
int take_input(int argc, char** argv, std::vector< boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> >& clouds, std::vector<std::string>& filenames,
	std::vector<std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>>& tags_list);


// save tag points from a color frame
void save_tags(std::vector <Global_helpers::TagPoints > feature_points, std::string filename);



}






#endif