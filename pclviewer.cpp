#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "visualizer.h"



#define MINIMUM_CORNERS_IN_A_TAG 3


pclviewer::pclviewer(int argc, char** argv, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::pclviewer), cur_ser(0), argc(argc), argv(argv)
{
	ui->setupUi(this);

	this->setWindowTitle ("PCL viewer");

	memset(points, 0, sizeof(points));

	transform_align.m_s = 1;
	transform_align.m_R = Eigen::Matrix<double, 3, 3>::Identity();
	transform_align.m_t.resize(3);
	transform_align.m_t[0] = 0.0, transform_align.m_t[1] = 0.0, transform_align.m_t[2] = 0.0;
	aligns.push_back(transform_align);
	
	//capture.capture_frame();
	// Setup the cloud pointer
	cloud_aligned = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>();
	cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>();
	// The number of points in the cloud
	capture;
	capture.capture_frame();
	//cloud = capture.cloud;
	cloud_aligned = capture.cloud;
	captured_color = &capture.color_mat;


	// Set up the QVTK window
	viewer_aligned.reset(new pcl::visualization::PCLVisualizer("viewer_aligned", false));
	viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
	ui->qvtkWidget_2->SetRenderWindow(viewer_aligned->getRenderWindow());
	ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
	viewer_aligned->setupInteractor(ui->qvtkWidget_2->GetInteractor(), ui->qvtkWidget_2->GetRenderWindow());
	viewer->setupInteractor (ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow ());
	ui->qvtkWidget_2->update();
	ui->qvtkWidget->update ();


	viewer_aligned->addPointCloud (cloud_aligned, "cloud_aligned");
	//viewer->addPointCloud (cloud, "cloud");

	qtimer = new QTimer(this);
	connect(qtimer, SIGNAL(timeout()), this, SLOT(processFrameAndUpdateGUI()));
	qtimer->start(1);
 
	// Connect "random" button and the function
	connect (ui->pushButton_add_cloud,  SIGNAL (clicked ()), this, SLOT (add_cloud_buttonPressed ()));
	connect (ui->pushButton_reset,  SIGNAL (clicked ()), this, SLOT (reset_buttonPressed ()));
	connect (ui->pushButton_remove_cloud,  SIGNAL (clicked ()), this, SLOT (remove_cloud_buttonPressed ()));
	connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (save_data_buttonPressed ()));
	//connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
	//connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
 
	//cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(clouds[0]);
  
	viewer_aligned->setCameraPosition(0.0, 0.0, -5000.0, 0.0, -1.0, -1.0, 0);
	viewer->setCameraPosition(0.0, 0.0, -5000.0, 0.0, -1.0, -1.0, 0);
	//viewer_aligned->resetCamera();
	//viewer->resetCamera();
	//viewer->spinOnce();
	//viewer_aligned->spinOnce();
	ui->qvtkWidget_2->update();
	ui->qvtkWidget->update ();
	std::cout << "debug6" << std::endl;
}

pclviewer::~pclviewer()
{
	delete ui;
}


void pclviewer::closeEvent(QCloseEvent* event)
{
	qtimer->stop();
	cloud.reset();
	cloud_aligned.reset();
	for (int i = 0; i < clouds.size(); i++)
	{
		clouds[i].reset();
	}
	viewer_aligned.reset();
	viewer.reset();
	capture.~Capture();
}


void pclviewer::processFrameAndUpdateGUI()
{
	capture.capture_frame();

	viewer_aligned->updatePointCloud(cloud_aligned, "cloud_aligned");
	ui->qvtkWidget_2->update();
	
	// we know the size of captured color frame and its not odd
	int width_offset =  2;
	//if (captured_color->size().width % 2) width_offset++;
	int height_offset =  2;
	//if (captured_color->size().height % 2) height_offset++;
	

	cv::resize(*captured_color, color_show, cv::Size(captured_color->size().width/2, captured_color->size().height/2) );

	std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> atags;
	apriltags.get_tags(color_show, atags, width_offset, height_offset);
	//apriltags.vis_apriltags(color_show, atags, false, false, false, true);

	std::map<int, std::vector<double> > tagsPresent = getFeaturePoints(capture, atags);
	
	cv::Vec4b red = cv::Vec4b(0, 0, 255, 255);
	for (std::map<int, std::vector<double> >::iterator it = tagsPresent.begin(); it != tagsPresent.end(); it++)
	{
		if (it->second[0] - double(MINIMUM_CORNERS_IN_A_TAG) < -.1)
		{
			apriltags.vis_apriltags(color_show, atags, width_offset, height_offset, true, false, false, true);
		}
		else
		{
			apriltags.vis_apriltags(color_show, atags, width_offset, height_offset, false, false, false, true);
		}
	}
	

	cv::resize(color_show, color_show, cv::Size(ui->color_video_label->width(), ui->color_video_label->height()));
	video_image = QImage((uchar*)color_show.data, color_show.cols,
	color_show.rows, color_show.step, QImage::Format_RGB32);
	ui->color_video_label->setPixmap(QPixmap::fromImage(video_image));
	ui->color_video_label->show();
}


void apply_scale(float scale, std::vector<Eigen::Vector3d>& m_points)
{
	for (int i = 0; i < m_points.size(); i++)
	{
		m_points[i][0] = m_points[i][0] * scale;
		m_points[i][1] = m_points[i][1] * scale;
		m_points[i][2] = m_points[i][2] * scale;
	}
}




void loadfeaturepoints(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags,
	std::vector<Eigen::Vector3d>& m_points)
{
	for (int i = 0; i < tags.size(); i++)
	{
		m_points.push_back(tags[i].point3D);
	}
}

// get the 3d points of the apriltags corners
std::map<int, std::vector<double> > pclviewer::getFeaturePoints(Capture& capture, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags)
{
	std::map<int, std::vector<double> > tagsPresent;
	
	for (int i = 0; i < tags.size(); i+=4)
	{
		int corners_count = 0;
		int indx_corners_count = tags[i].tag * 14;
		for (int j = 0; j < 4; j++)
		{
			if (tagsPresent.find(tags[i+j].tag) == tagsPresent.end())
			{
				tagsPresent[tags[i+j].tag].push_back(0.0);
				tagsPresent[tags[i+j].tag].push_back(tags[i+j].centerx);
				tagsPresent[tags[i+j].tag].push_back(tags[i+j].centery);
			}
			if (capture.project_colorcam_to_depthcam(tags[i+j].point, tags[i+j].cloud_index))
			{
				if (cloud_aligned->points.at(tags[i+j].cloud_index).z == 0)
				{
					std::cout << "z==0" << std::endl;
					tags[i+j].cloud_index = -1;
					tags[i+j].point3D[0] = -1;
					tags[i+j].point3D[1] = -1;
					tags[i+j].point3D[2] = -1;


					//tagsPresent[tags[i].tag][0] = 0.0;
				}
				else
				{
					pcl::PointXYZRGB cloud_point = cloud_aligned->points.at(tags[i+j].cloud_index);
					tags[i+j].point3D[0] = cloud_point.x;
					tags[i+j].point3D[1] = cloud_point.y;
					tags[i+j].point3D[2] = cloud_point.z;

					tagsPresent[tags[i+j].tag][0] = tagsPresent[tags[i+j].tag][0] + 1.0;
					corners_count++;
				}
			}
			else
			{
				std::cout << "picked an nan depth point" << std::endl;
				tags[i+j].cloud_index = -1;
				tags[i+j].point3D[0] = -1;
				tags[i+j].point3D[1] = -1;
				tags[i+j].point3D[2] = -1;

				//tagsPresent[tags[i].tag][0] = 0.0;
			}
		}
		
	}

	return tagsPresent;
}




void pclviewer::storeMarkersAligned(int current_frame)
{
	std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> tag = tags[current_frame];
	std::vector<Eigen::Vector3d> points_3d, fPoints;
	loadfeaturepoints(tag, points_3d);
	
	if (current_frame == 0)
	{
		loadfeaturepoints(tag, fPoints);
	}
	
	
	apply_scale(aligns[current_frame].m_s, points_3d);
	aligns[current_frame].apply(points_3d, fPoints);
	


	for (int i = 0; i < tag.size(); i ++)
	{
		if (tag[i].cloud_index != -1)
		{
			int startIndex = tag[i].tag * TAG_BUFFER + tag[i].pos * TAG_CORNER_BUFFER;
			
			try
			{
				if (points[startIndex] == 0)
				{
					for (int j = 1; j < 4; j++)
					{
						points[startIndex + j] = fPoints[i][j - 1];
					}

					points[startIndex] = 1;
				}
			}
			catch (int e)
			{
				throw("Tag ID is out of bound. Try increasing the size of the array 'points'");
			}
		}
	}
}



void pclviewer::add_cloud_buttonPressed()
{
	if (cur_ser == 0)
	{
		cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(cloud_aligned)));
		clouds.resize(1);
		clouds[0].reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(cloud)));
		viewer->addPointCloud(cloud, "cloud");

		tags.resize(1);
		apriltags.get_tags(capture.color_mat, tags[0], 1, 1,0);
		getFeaturePoints(capture, tags[0]);
		//apriltags.filter_tag(tags[0]);
		sort(tags[0].begin(), tags[0].end());
		storeMarkersAligned(0);
		cur_ser++;
	}
	else
	{
		std::cout << "current serial: " << cur_ser << std::endl;

		clouds.resize(cur_ser + 1);
		clouds[cur_ser].reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(cloud_aligned)));
		aligns.push_back(transform_align);
		
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> transforming;
		std::vector<Eigen::Vector3d> transformed;
		std::vector<Eigen::Vector3d> vertices_3d2;
		std::vector<Eigen::Vector3d> color_vertices;
		GLOBAL_HELPERS::pclToEigenVector(cloud_aligned, vertices_3d2);
		GLOBAL_HELPERS::pclToEigenVector_color(cloud_aligned, color_vertices);
		transforming = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_aligned);


		tags.resize(cur_ser + 1);
		apriltags.get_tags(*captured_color, tags[cur_ser], 1, 1, cur_ser);
		getFeaturePoints(capture, tags[cur_ser]);
		//apriltags.filter_tag(tags[cur_ser]);
		sort(tags[cur_ser].begin(), tags[cur_ser].end());
		different_tags = 0;
		for (int i = 0; i < tags[cur_ser].size(); i++)
		{
			if (tags[cur_ser][i].cloud_index != -1)
			{
				int startIndex = tags[cur_ser][i].tag * TAG_BUFFER + tags[cur_ser][i].pos * TAG_CORNER_BUFFER;
				try
				{
					if (points[startIndex] == 1)
					{
						Eigen::Vector3d pts1 = Eigen::Vector3d(points[startIndex + 1], points[startIndex + 2], points[startIndex + 3]);
						aligns[cur_ser].m_points2.push_back(pts1);

						Eigen::Vector3d pts2 = tags[cur_ser][i].point3D;
						aligns[cur_ser].m_points1.push_back(pts2);

						different_tags++;
					}
				}
				catch (int e)
				{
					throw("Tag ID is out of bound. Try increasing the size of the array 'points'");
				}
				
			}
		}
		//apriltags.common_tags(tags[cur_ser-1], tags[cur_ser]);
		//apriltags.rearrange_tags(tags[cur_ser-1], tags[cur_ser], different_tags);

		std::cout << "number of common tags: " << tags[cur_ser].size() << std::endl;

		
		//loadfeaturepoints(tags[cur_ser-1], aligns[cur_ser].m_points2);
		//loadfeaturepoints(tags[cur_ser], aligns[cur_ser].m_points1);
		
		aligns[cur_ser].compute_scale();

		apply_scale(aligns[cur_ser].m_s, aligns[cur_ser].m_points1);
		apply_scale(aligns[cur_ser].m_s, vertices_3d2);
		std::cout << "scale factor: " << aligns[cur_ser].m_s << std::endl;
		// computing the transformation matrix
		std::cout << "Applying transformation" << std::endl;
		aligns[cur_ser].compute_trans();
		//aligns[cur_ser].save("../outputs/mat.txt");
		//aligns[i+1].prune(different_tags, different_tags);
		//aligns[cur_ser + 1].save(tm_name.c_str());
		apply_scale(1.0f / aligns[cur_ser].m_s, vertices_3d2);

		
		apply_scale(aligns[cur_ser].m_s, vertices_3d2);
		aligns[cur_ser].apply(vertices_3d2, transformed);
		//vertices_3d2 = transformed;
		

		GLOBAL_HELPERS::eigenVector_to_pclCloud(transformed, transforming);
		GLOBAL_HELPERS::eigenVector_color_to_pclCloud(color_vertices, transforming);

		*cloud += *transforming;

		storeMarkersAligned(cur_ser);
		cur_ser++;
	}


	// for debugging purpose. Comment if debugging is not needed.
	for (int i = 0; i < tags[cur_ser-1].size(); i++)
	{
		if (tags[cur_ser-1][i].cloud_index != -1)
		{
			VISH::mark_cloud(cloud, tags[cur_ser-1][i].cloud_index, cv::Vec3b(0, 0, 255), 3);
		}
	}

	viewer->updatePointCloud(cloud, "cloud");
	ui->qvtkWidget->update();

	ui->comboBox_clouds_serial->addItem( QString::number(aligns.size()-1) );
	ui->comboBox_clouds_serial->setCurrentIndex(aligns.size());
}


void pclviewer::reset_buttonPressed()
{
	//if (clouds.size() == 0) return;

	std::cout << "reset button pressed" << std::endl;
	cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(new pcl::PointCloud<pcl::PointXYZRGB>);
	//GLOBAL_HELPERS::copyCloud(cloud_aligned, cloud);
	clouds.clear();
	aligns.clear();
	aligns.push_back(transform_align);
	tags.clear();
	cur_ser = 0;
	
	viewer->updatePointCloud(cloud, "cloud");
	ui->qvtkWidget->update();

	ui->comboBox_clouds_serial->clear();
	ui->comboBox_clouds_serial->addItem("NONE");
}


void pclviewer::remove_cloud_buttonPressed()
{
	if (clouds.size() == 0) return;

	int remove_ind = ui->comboBox_clouds_serial->currentIndex()-1;
	clouds.erase(clouds.begin() + remove_ind);
	aligns.erase(aligns.begin() + remove_ind, aligns.end());
	if (remove_ind == 0)	aligns.push_back(transform_align);
	//aligns.clear();
	//aligns.push_back(transform_align);
	tags.erase(tags.begin() + remove_ind);
	std::cout << "aligns size: " << aligns.size() << std::endl;
	std::cout << "tags size: " << tags.size() << std::endl;
	std::cout << "clouds size: " << clouds.size() << std::endl;


	if (clouds.size() == 0)
	{
		reset_buttonPressed();
		return;
	}

	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(clouds[0])));
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> transforming;

	for (int i = 1; i < remove_ind; i++)
	{
		std::cout << "realigning clouds" << std::endl;
		std::vector<Eigen::Vector3d> transformed;
		std::vector<Eigen::Vector3d> vertices_3d2;
		std::vector<Eigen::Vector3d> color_vertices;
		GLOBAL_HELPERS::pclToEigenVector(clouds[i], vertices_3d2);
		GLOBAL_HELPERS::pclToEigenVector_color(clouds[i], color_vertices);
		transforming.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(clouds[i])));

		for (int j = i; j > 0; j--)
		{
			apply_scale(aligns[j].m_s, vertices_3d2);
			aligns[j].apply(vertices_3d2, transformed);
			vertices_3d2 = transformed;
		}

		GLOBAL_HELPERS::eigenVector_to_pclCloud(transformed, transforming);
		GLOBAL_HELPERS::eigenVector_color_to_pclCloud(color_vertices, transforming);

		*cloud += *transforming;

	}


	int run_from = std::max(remove_ind, 1);

	for (int i = run_from; i < clouds.size(); i++)
	{
		std::cout << "realigning clouds" << std::endl;
		std::vector<Eigen::Vector3d> transformed;
		std::vector<Eigen::Vector3d> vertices_3d2;
		std::vector<Eigen::Vector3d> color_vertices;
		GLOBAL_HELPERS::pclToEigenVector(clouds[i], vertices_3d2);
		GLOBAL_HELPERS::pclToEigenVector_color(clouds[i], color_vertices);
		transforming.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(clouds[i])));

		apriltags.common_tags(tags[i - 1], tags[i]);
		apriltags.rearrange_tags(tags[i - 1], tags[i], different_tags);
		std::cout << "number of common tags: " << tags[i].size() << std::endl;

		aligns.push_back(transform_align);
		loadfeaturepoints(tags[i - 1], aligns[i].m_points2);
		loadfeaturepoints(tags[i], aligns[i].m_points1);

		aligns[i].compute_scale();

		apply_scale(aligns[i].m_s, aligns[i].m_points1);
		apply_scale(aligns[i].m_s, vertices_3d2);
		std::cout << "scale factor: " << aligns[i].m_s << std::endl;
		// computing the transformation matrix
		std::cout << "Applying transformation" << std::endl;
		aligns[i].compute_trans();
		//aligns[i].save("../outputs/mat.txt");
		//aligns[i+1].prune(different_tags, different_tags);
		//aligns[cur_ser + 1].save(tm_name.c_str());
		apply_scale(1.0f / aligns[i].m_s, vertices_3d2);

		for (int j = i; j > 0; j--)
		{
			apply_scale(aligns[j].m_s, vertices_3d2);
			aligns[j].apply(vertices_3d2, transformed);
			vertices_3d2 = transformed;
		}

		GLOBAL_HELPERS::eigenVector_to_pclCloud(transformed, transforming);
		GLOBAL_HELPERS::eigenVector_color_to_pclCloud(color_vertices, transforming);

		*cloud += *transforming;
	}


	cur_ser = clouds.size();
	viewer->updatePointCloud(cloud, "cloud");
	ui->qvtkWidget->update();

	ui->comboBox_clouds_serial->clear();
	ui->comboBox_clouds_serial->addItem("NONE");
	for (int i = 0; i < clouds.size(); i++)
	{
		ui->comboBox_clouds_serial->addItem(QString::number(i));
	}
	ui->comboBox_clouds_serial->setCurrentIndex(clouds.size());
}


void pclviewer::save_data_buttonPressed()
{
	for (int i = 0; i < clouds.size(); i++)
	{
		std::ostringstream str_out, obj_file;
		str_out << "../outputs/office/" << "capture_" << i << ".pcd";
		obj_file << "../outputs/office/" << "capture_" << i << ".obj";
		pcl::io::savePCDFileASCII(str_out.str().c_str(), *(clouds[i]));

		std::ofstream outfile(obj_file.str().c_str());


		pcl::PolygonMesh triangles;
		pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;

		ofm.setInputCloud(clouds[i]);
		ofm.setMaxEdgeLength(1.5);
		ofm.setTrianglePixelSize(1);
		ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);

		// Reconstruct
		ofm.reconstruct(triangles);

		outfile << "# Vertices: " << clouds[i]->size() << endl;
		outfile << "# Faces: " << triangles.polygons.size() << endl;

		for (int j = 0; j < (*clouds[i]).size(); j++)
		{
			std::string r = std::to_string(clouds[i]->at(j).r) + "";
			std::string g = std::to_string(clouds[i]->at(j).g) + "";
			std::string b = std::to_string(clouds[i]->at(j).b) + "";

			//std::cout << "v " << clouds[i]->at(j).x << " " << clouds[i]->at(j).y << " " << clouds[i]->at(j).z << " " <<
			//	r << " " << g << " " << b << endl;
			outfile << "v " << clouds[i]->at(j).x << " " << clouds[i]->at(j).y << " " << clouds[i]->at(j).z << " " << 
				r.c_str() << " " << g.c_str() << " " << b.c_str() <<endl;
		}


		

		for (int i = 0; i < triangles.polygons.size(); i++)
		{
			pcl::Vertices face = triangles.polygons.at(i);
			outfile << "f " << face.vertices[0] << " " << face.vertices[1] << " " << face.vertices[2] << endl;

		}

		outfile << "# End of File" << endl;

		outfile.close();
		//pcl::io::saveOBJFile(str_out.str() + ".obj", triangles);
		std::cout << "saved " << str_out.str() + ".obj" << std::endl;
	}

	
}