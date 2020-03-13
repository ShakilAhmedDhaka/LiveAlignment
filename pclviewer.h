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

#include "global_helpers.h"
#include "capture.h"
#include "apriltags_container.h"
#include "alignment.h"


#define NUMBER_OF_TAGS 100

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
	
protected:
	
	Capture capture;
	Apriltags apriltags;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_aligned;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_aligned;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
	std::vector< boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> > clouds;

	cv::Mat* captured_color;

	std::vector<GLOBAL_HELPERS::Global_helpers::CapturedData> all_captured_data;
	GLOBAL_HELPERS::Global_helpers::CapturedData captured_data;
	std::vector<std::string> filenames;
	std::vector< std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> > tags;
	std::vector<TBasic::RSAlign> aligns;
	TBasic::RSAlign transform_align;

	short points[NUMBER_OF_TAGS*14+5];

	int different_tags;
	QImage video_image;
	cv::Mat color_show;

private:

	Ui::pclviewer* ui;
	QTimer* qtimer;
	QTimer* qtimer2;
	int cur_ser;
	int argc;
	char** argv;
};

#endif // PCLVIEWER_H
