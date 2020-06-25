#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <QMainWindow>
#include <QTimer>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkCleanPolyData.h>

#include "global_helpers.h"
#include "capture.h"
#include "apriltags_container.h"
#include "alignment.h"


#define NUMBER_OF_TAGS 100
#define TAGS_CORNERS 4
#define TAG_CORNER_BUFFER 4
#define TAG_BUFFER 16

namespace Ui {
class pclviewer;
}

class pclviewer : public QMainWindow
{
	Q_OBJECT

public:
	explicit pclviewer(int argc, char** argv, QWidget *parent = nullptr);
	~pclviewer();

public slots:
	void processFrameAndUpdateGUI();
	void add_cloud_buttonPressed();
	void reset_buttonPressed();
	void remove_cloud_buttonPressed();
	void save_data_buttonPressed();


private:
	virtual void closeEvent(QCloseEvent* event);

	// gets 3d points from corressponding apriltags. return a map whose (key, value) = (tag_id, std::vector<ispresentindepth, cv::point tagCenter),
	//  if for one tag, some of its corners is not present in the depth then it will have false. If all the corners are present then,
	// it will have true as a value.
	std::map<int, std::vector<double> > getFeaturePoints(Capture& capture, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags);


	// Input: current frame
	// Goal: Store the aligned points in "points" an array for faster access of tags( order 1 ) later on
	// Return: None
	void storeMarkersAligned(int current_serial);
	void unStoreMarkersAligned(int current_serial);

	void cloudToPolygonMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PolygonMesh::Ptr& mesh);
	
protected:
	
	Capture capture;
	Apriltags apriltags;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_aligned;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_aligned;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloudFiltered;
	std::vector< boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> > clouds;
	std::vector< pcl::PolygonMesh::Ptr > meshes;

	cv::Mat* captured_color;

	std::vector<GLOBAL_HELPERS::Global_helpers::CapturedData> all_captured_data;
	GLOBAL_HELPERS::Global_helpers::CapturedData captured_data;
	std::vector<std::string> filenames;
	std::vector< std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> > tags;
	std::vector<TBasic::RSAlign> aligns;
	TBasic::RSAlign transform_align;



	// For each TAG: Find first index by Tag ID (Tag.tag) * 14
	// First Index: 0 / [1...n]. Denotes for which pose the tags were registered.
	// for example: if during the 2nd pose the tags were discovered first, then 
	// first index will be 2. If it was for 3rd pose first index will be 3. If
	// this tag has not been discovered yet then first index is 0.
	// Second index: How many corners were found
	// The next twelve index contains 4 3D points for 4 corners
	short points[NUMBER_OF_TAGS * TAGS_CORNERS * TAG_CORNER_BUFFER + 5];

	int different_tags;
	QImage video_image;
	cv::Mat color_show;

private:

	Ui::pclviewer* ui;
	QTimer* qtimer;
	QTimer* qtimer2;
	int cur_ser;
	int mesh_id_serial;
	ofstream dbug;
	int argc;
	char** argv;



	std::string meshids[100] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10",
						"11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21",
						"22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32",
						"33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43",
						"44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54",
						"55", "56", "57", "58", "59", "60", "61", "62", "63", "64", "65",
						"66", "67", "68", "69", "70", "71", "72", "73", "74", "75", "76",
						"77", "78", "79", "80", "81", "82", "83", "84", "85", "86", "87",
						"88", "89", "90", "91", "92", "93", "94", "95", "96", "97", "98" };
};

#endif // PCLVIEWER_H
