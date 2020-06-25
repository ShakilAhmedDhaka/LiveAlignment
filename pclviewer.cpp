#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "visualizer.h"

#include <pcl/filters/voxel_grid.h>



#define MINIMUM_CORNERS_IN_A_TAG 3
#define PI_VAL 3.14159265


pclviewer::pclviewer(int argc, char** argv, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::pclviewer), cur_ser(0), argc(argc), argv(argv)
{
	ui->setupUi(this);

	this->setWindowTitle ("PCL viewer");
	dbug.open("log.txt");
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
	if (!dbug.is_open())
	{
		std::cout << "could not open log file" << std::endl;
		QCoreApplication::quit();
	}
}

pclviewer::~pclviewer()
{
	delete ui;
}


void pclviewer::closeEvent(QCloseEvent* event)
{
	reset_buttonPressed();
	qtimer->stop();
	cloud.reset();
	cloud_aligned.reset();
	for (int i = meshes.size()-1; i >= 0; i--)
	{
		meshes[i].reset();
	}
	std::vector<pcl::PolygonMesh::Ptr>().swap(meshes);
	std::vector<TBasic::RSAlign>().swap(aligns);
	std::vector< std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> >().swap(tags);


	viewer_aligned->removeAllPointClouds();
	viewer_aligned.reset();
	viewer->removeAllPointClouds();
	viewer.reset();
	dbug << "viewers closed" << std::endl;
	dbug.close();
	//capture.~Capture();
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
	

	cv::resize(*captured_color, color_show, cv::Size(captured_color->size().width/width_offset, captured_color->size().height/height_offset) );

	std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> atags;
	apriltags.get_tags(color_show, atags, width_offset, height_offset);
	//apriltags.vis_apriltags(color_show, atags, false, false, false, true);

	std::map<int, std::vector<double> > tagsPresent = getFeaturePoints(capture, atags);
	
	cv::Vec4b red = cv::Vec4b(0, 0, 255, 255);


	//std::cout << "*********Begin Coloring**********" << std::endl;
	for (int i = 0; i+3 < atags.size(); i += 4)
	{
		bool isCorrectAngle = false;

		if (tagsPresent.find(atags[i].tag) == tagsPresent.end())	continue;
		
		if (tagsPresent[atags[i].tag][0] < MINIMUM_CORNERS_IN_A_TAG)
		{
			//std::cout << "********* Not Enough Tags **********" << std::endl;
			apriltags.vis_apriltags(color_show, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> (atags.begin()+i, atags.begin()+i+4), 
				width_offset, height_offset, true, false, false, true, isCorrectAngle);
		}
		else
		{
			//std::cout << "********* See If Align **********" << std::endl;
			if (MINIMUM_CORNERS_IN_A_TAG >= 3)
			{
				std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> threePoints;
				if (atags[i].cloud_index != -1)	threePoints.push_back(atags[i]);
				if (atags[i + 1].cloud_index != -1)	threePoints.push_back(atags[i + 1]);
				if (atags[i + 2].cloud_index != -1)	threePoints.push_back(atags[i + 2]);
				if (atags[i + 3].cloud_index != -1)	threePoints.push_back(atags[i + 3]);

				if (threePoints.size() > 3)	threePoints.pop_back();

				//std::cout << "**************** Points Loaded ******************" << std::endl;
				//std::cout << "**************** threePoints size: "<< threePoints.size() << std::endl;
				if (threePoints.size() == 3)
				{
					Eigen::Vector3d vec1 = threePoints[1].point3D - threePoints[0].point3D;
					Eigen::Vector3d vec2 = threePoints[2].point3D - threePoints[0].point3D;

					//std::cout << "**************** Two Vectors ******************" << std::endl;
					Eigen::Vector3d sNormal = vec1.cross(vec2);
					sNormal = sNormal.normalized();

					//std::cout << "**************** Normalized ******************" << std::endl;

					Eigen::Vector3d zAxis = Eigen::Vector3d(0, 0, -1);
					double angle = (acos(zAxis.dot(sNormal)) * 180.0) /  PI_VAL;

					if (angle <= 30) isCorrectAngle = true;
					//std::cout << "****************Angle: " << angle << std::endl;
				}

				threePoints.clear();
			}
			
			apriltags.vis_apriltags(color_show, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>(atags.begin() + i, atags.begin() + i + 4), 
				width_offset, height_offset, false, false, false, true, isCorrectAngle);
		}
	}

	//std::cout << "*********End Coloring**********" << std::endl;
	/*for (std::map<int, std::vector<double> >::iterator it = tagsPresent.begin(); it != tagsPresent.end(); it++)
	{
		if (it->second[0] - double(MINIMUM_CORNERS_IN_A_TAG) < -.1)
		{
			apriltags.vis_apriltags(color_show, atags, width_offset, height_offset, true, false, false, true);
		}
		else
		{
			apriltags.vis_apriltags(color_show, atags, width_offset, height_offset, false, false, false, true);
		}
	}*/
	

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


void pclviewer::unStoreMarkersAligned(int current_frame)
{
	std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> tag = tags[current_frame];
	
	for (int i = 0; i < tag.size(); i++)
	{
		if (tag[i].cloud_index != -1)
		{
			int startIndex = tag[i].tag * TAG_BUFFER + tag[i].pos * TAG_CORNER_BUFFER;
			if (points[startIndex] == current_frame + 1)	points[startIndex] = 0;
		}
	}
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

					points[startIndex] = current_frame+1;
				}
			}
			catch (int e)
			{
				throw("Tag ID is out of bound. Try increasing the size of the array 'points'");
			}
		}
	}
}




void pclviewer::cloudToPolygonMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PolygonMesh::Ptr& mesh)
{
	pcl::PolygonMesh::Ptr old_mesh(new pcl::PolygonMesh);
	pcl::OrganizedFastMesh<pcl::PointXYZRGB> fast_mesh;
	fast_mesh.setInputCloud(cloud);
	fast_mesh.reconstruct(*old_mesh);


	mesh->polygons.clear();
	mesh->cloud.data.clear();
	mesh->cloud.height = 1;
	mesh->cloud.is_dense = false;
	mesh->cloud.data.push_back(1);


	// Mapping new face and vertices
	std::vector<Eigen::Vector3d> newVerticesList;
	std::vector<Eigen::Vector3d> newVerticesListColor;
	std::vector<unsigned int> indicesMap;
	indicesMap.resize(cloud->width * cloud->height);

	int discarded_points = 0, average_point_z = 0, average_point_z_discarded = 0, non_zero_points = 0, points_depth_zero = 0;
	for (int i = 0; i < cloud->height; i++)
	{
		for (int j = 0; j < cloud->width; j++)
		{
			if (cloud->at(j, i).z < 10.0f)
			{
				cloud->at(j, i).z = 0.0f;
				indicesMap[i * cloud->width + j] = -1;
				if (cloud->at(j, i).z > 0.0f)
				{
					discarded_points++;
					average_point_z_discarded += cloud->at(j, i).z;
				}
				else
				{
					points_depth_zero++;
				}
			}
			else
			{
				indicesMap[i * cloud->width + j] = newVerticesList.size();
				newVerticesList.push_back(Eigen::Vector3d(cloud->at(j, i).x, cloud->at(j, i).y, cloud->at(j, i).z));
				newVerticesListColor.push_back(Eigen::Vector3d(cloud->at(j, i).r, cloud->at(j, i).g, cloud->at(j, i).b));
			}

			if (cloud->at(j, i).z > 0.0f)
			{
				average_point_z += cloud->at(j, i).z;
				non_zero_points++;
			}
			
		}
	}

	dbug << "total number of candidate points: " << cloud->width * cloud->height << std::endl;
	if (non_zero_points > 0)	dbug << "average z value of all points: " << average_point_z / non_zero_points << std::endl;
	else dbug << "All points depths are zero!!!!!!" << std::endl;
	dbug << "discarded points: " << discarded_points + points_depth_zero << std::endl;
	if (discarded_points > 0)	dbug << "average z value of discarded points: " << average_point_z_discarded / discarded_points << std::endl;
	else dbug << "no non zero depth value was discarded" << std::endl;

	// transfering cloud data to new mesh
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_temp->width = newVerticesList.size();
	cloud_temp->height = 1;
	cloud_temp->resize(newVerticesList.size());
	cloud_temp->is_dense = true;

	for (int i = 0; i < newVerticesList.size(); i++)
	{
		Eigen::Vector3d pt(newVerticesList[i][0], newVerticesList[i][1], newVerticesList[i][2]);
		std::vector<Eigen::Vector3d> src; src.push_back(pt);
		std::vector<Eigen::Vector3d> dst;
		
		aligns[cur_ser].apply(src, dst);
		cloud_temp->points[i].x = dst[0][0];
		cloud_temp->points[i].y = dst[0][1];
		cloud_temp->points[i].z = dst[0][2];
		cloud_temp->points[i].r = newVerticesListColor[i][0];
		cloud_temp->points[i].g = newVerticesListColor[i][1];
		cloud_temp->points[i].b = newVerticesListColor[i][2];
	}

	pcl::toPCLPointCloud2(*cloud_temp, mesh->cloud);


	// creating new faces
	int discarded_faces = 0, common_vertices = 0, average_depth_face = 0, discard_for_difference = 0;
	mesh->polygons.resize(old_mesh->polygons.size());
	std::vector<pcl::Vertices, std::allocator<pcl::Vertices>>::iterator face;
	for (face = old_mesh->polygons.begin(); face != old_mesh->polygons.end(); face++)
	{
		unsigned int v1 = face->vertices[0];
		unsigned int v2 = face->vertices[1];
		unsigned int v3 = face->vertices[2];
		unsigned int v4 = face->vertices[3];


		pcl::PointXYZRGB p1 = cloud->points.at(v1);
		pcl::PointXYZRGB p2 = cloud->points.at(v2);
		pcl::PointXYZRGB p3 = cloud->points.at(v3);
		pcl::PointXYZRGB p4 = cloud->points.at(v4);



		float max_z = std::max(std::max(std::max(p1.z, p2.z), p3.z), p4.z);
		float min_z = std::min(std::min(std::min(p1.z, p2.z), p3.z), p4.z);
		average_depth_face += (max_z - min_z);

		if (p1.z == 0 || p2.z == 0 || p3.z == 0 || p4.z == 0)
		{
			discarded_faces++;
		}
		else if ( (max_z - min_z) > 50 )
		{
			discarded_faces++;
			discard_for_difference++;
		}
		else
		{
			if (indicesMap[v1] != -1 && indicesMap[v2] != -1 && indicesMap[v3] != -1 && indicesMap[v4] != -1)
			{
				if (v1 == v2 || v2 == v3 || v3 == v4 || v4 == v1)
				{
					common_vertices++;
				}
				else
				{
					
				}
				pcl::Vertices newFace;
				newFace.vertices = { indicesMap[v1], indicesMap[v2], indicesMap[v3], indicesMap[v4] };
				mesh->polygons.push_back(newFace);
			}
		}
	}

	dbug << "total number of candidate faces: " << old_mesh->polygons.size() << std::endl;
	dbug << "discarded faces: " << discarded_faces << std::endl;
	dbug << "dicarded for differences in z values: " << discard_for_difference << std::endl;
	dbug << "incident of common vertices in the same candidate face: " << common_vertices << std::endl;
	if (old_mesh->polygons.size() > 0)	dbug << "average depth difference between candidate face vertices: " << average_depth_face / old_mesh->polygons.size() << std::endl;
	else dbug << "number of candidate faces are zero!!!" << std::endl;
	dbug << "number of faces accepted: " << mesh->polygons.size() << std::endl;
}


void pclviewer::add_cloud_buttonPressed()
{
	if (cur_ser == 0)
	{
		dbug << "adding pose: 0" << std::endl;
		meshes.resize(1);
		meshes[0] = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
		cloudToPolygonMesh(cloud_aligned, meshes[0]);

		/*std::string prefix = "polygon";
		std::vector<char> chars(prefix.begin(), prefix.end() + prefix.length() + 1);
		const std::string meshid = strcat(&chars[0], "" + cur_ser);*/
		if (!viewer->addPolygonMesh(*meshes[cur_ser], meshids[cur_ser]))
		{
			dbug << "duplicate id for mesh: " << cur_ser << ". chose another id" << std::endl;
		}

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
		dbug << "adding pose: " << cur_ser << std::endl;
		meshes.resize(cur_ser + 1);
		meshes[cur_ser] = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
		//cloudToPolygonMesh(cloud_aligned, meshes[cur_ser]);
		aligns.push_back(transform_align);
		

		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> transforming;
		std::vector<Eigen::Vector3d> transformed;
		std::vector<Eigen::Vector3d> vertices_3d2;
		std::vector<Eigen::Vector3d> color_vertices;
		GLOBAL_HELPERS::pclToEigenVector(cloud_aligned, vertices_3d2);
		GLOBAL_HELPERS::pclToEigenVector_color(cloud_aligned, color_vertices);
		transforming.reset(new pcl::PointCloud<pcl::PointXYZRGB> (*cloud_aligned));// boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_aligned);


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
					if (points[startIndex] != 0)
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
		//apply_scale(aligns[cur_ser].m_s, vertices_3d2);
		std::cout << "scale factor: " << aligns[cur_ser].m_s << std::endl;
		// computing the transformation matrix
		std::cout << "Applying transformation" << std::endl;
		aligns[cur_ser].compute_trans();
		//aligns[cur_ser].save("../outputs/mat.txt");
		//aligns[i+1].prune(different_tags, different_tags);
		//aligns[cur_ser + 1].save(tm_name.c_str());
		
		/*apply_scale(1.0f / aligns[cur_ser].m_s, vertices_3d2);

		
		apply_scale(aligns[cur_ser].m_s, vertices_3d2);
		aligns[cur_ser].apply(vertices_3d2, transformed);*/
		//vertices_3d2 = transformed;
		

		//GLOBAL_HELPERS::eigenVector_to_pclCloud(transformed, transforming);
		//GLOBAL_HELPERS::eigenVector_color_to_pclCloud(color_vertices, transforming);

		
		cloudToPolygonMesh(transforming, meshes[cur_ser]);

		//addMeshToViewer(meshes[cur_ser], cur_ser);
		if (!viewer->addPolygonMesh(*meshes[cur_ser], meshids[cur_ser]))
		{
			dbug << "duplicate id for mesh: " << cur_ser << ". chose another id" << std::endl;
		}


		storeMarkersAligned(cur_ser);
		cur_ser++;
	}


	// for debugging purpose. Comment if debugging is not needed.
	//for (int i = 0; i < tags[cur_ser-1].size(); i++)
	//{
	//	if (tags[cur_ser-1][i].cloud_index != -1)
	//	{
	//		VISH::mark_cloud(cloud, tags[cur_ser-1][i].cloud_index, cv::Vec3b(0, 0, 255), 3);
	//	}
	//}
	
	ui->qvtkWidget->update();

	ui->comboBox_clouds_serial->addItem( QString::number(aligns.size()-1) );
	ui->comboBox_clouds_serial->setCurrentIndex(aligns.size());
}


void pclviewer::reset_buttonPressed()
{
	dbug << "reset button pressed" << std::endl;

	// removing meshes from viewer
	for (int i = 0; i < cur_ser; i++)
	{
		std::string prefix = "polygon";
		std::vector<char> chars(prefix.begin(), prefix.end() + prefix.length() + 1);
		const std::string meshid = strcat(&chars[0], "" + i);
		std::cout << "removing mesh id: " << meshid << " from viewer" << std::endl;
		if (!viewer->removePolygonMesh(meshids[i]))
		{
			dbug << "removal of mesh: " << i << " from visualizer was unsuccessfull" << std::endl;
		}
	}

	// resetting mesh pointers
	for (int i = meshes.size() - 1; i >= 0; i--)
	{
		meshes[i].reset();
	}

	// removing elements and freeing up the memory
	std::vector<pcl::PolygonMesh::Ptr>().swap(meshes);
	std::vector<TBasic::RSAlign>().swap(aligns);
	std::vector< std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> >().swap(tags);
	
	// reinitialize some values
	memset(points, 0, sizeof(points) / sizeof(points[0]));
	aligns.push_back(transform_align);
	cur_ser = 0;
	ui->qvtkWidget->update();
	ui->comboBox_clouds_serial->clear();
	ui->comboBox_clouds_serial->addItem("NONE");

	dbug << "resetting was done succesfully" << std::endl;
}


void pclviewer::remove_cloud_buttonPressed()
{
	int remove_ind = ui->comboBox_clouds_serial->currentIndex()-1;
	
	dbug << "removing pose: " << remove_ind << std::endl;
	
	if (meshes.size() == 0) return;
	std::vector<int> remove_indices;
	remove_indices.push_back(remove_ind);
	if (remove_ind == 0)
	{
		reset_buttonPressed();
		return;
	}
	
	// deleting features points correspondece for this pose
	unStoreMarkersAligned(remove_ind);

	// removing all polygon meshes from viewer
	// from the pose to be removed
	for (int i = remove_ind; i < cur_ser; i++)
	{
		std::string prefix = "polygon";
		std::vector<char> chars(prefix.begin(), prefix.end() + prefix.length() + 1);
		const std::string meshid = strcat(&chars[0], "" + i);
		if (!viewer->removePolygonMesh(meshids[i]))
		{
			dbug << "removal of mesh: " << i << " from visualizer was unsuccessfull" << std::endl;
		}
	}

	
	// escaping the pose to be removed. starting after that.
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> transforming;
	int viewerId = remove_ind;
	//for (int i = remove_ind + 1; i < cur_ser; i++)
	//{
	//	std::cout << "realigning pose: " << i << std::endl;
	//	std::vector<Eigen::Vector3d> transformed;
	//	std::vector<Eigen::Vector3d> vertices_3d2;
	//	std::vector<Eigen::Vector3d> color_vertices;
	//	GLOBAL_HELPERS::pclToEigenVector(clouds[i], vertices_3d2);
	//	GLOBAL_HELPERS::pclToEigenVector_color(clouds[i], color_vertices);
	//	transforming.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(clouds[i])));

	//	// clearing previous feature points for alignment
	//	aligns[i].m_points1.clear();
	//	aligns[i].m_points2.clear();
	//	int different_tags = 0;	// different_tags = number of features that matches with some other previously found features
	//	for (int j = 0; j < tags[i].size(); j++)
	//	{
	//		if (tags[i][j].cloud_index != -1)
	//		{
	//			int startIndex = tags[i][j].tag * TAG_BUFFER + tags[i][j].pos * TAG_CORNER_BUFFER;
	//			try
	//			{
	//				if (points[startIndex] != 0)
	//				{
	//					Eigen::Vector3d pts1 = Eigen::Vector3d(points[startIndex + 1], points[startIndex + 2], points[startIndex + 3]);
	//					aligns[i].m_points2.push_back(pts1);

	//					Eigen::Vector3d pts2 = tags[i][j].point3D;
	//					aligns[i].m_points1.push_back(pts2);

	//					different_tags++;
	//				}
	//			}
	//			catch (int e)
	//			{
	//				throw("Tag ID is out of bound. Try increasing the size of the array 'points'");
	//			}

	//		}
	//	}

	//	// if different_tags is 0 then this pose has no corresponding features with previous poses
	//	// and to be removed
	//	if (different_tags == 0)
	//	{
	//		unStoreMarkersAligned(i); // this is necessary to avoid misguiding the following poses
	//		remove_indices.push_back(i);
	//		continue;
	//	}

	//	aligns[i].compute_scale();

	//	apply_scale(aligns[i].m_s, aligns[i].m_points1);
	//	apply_scale(aligns[i].m_s, vertices_3d2);
	//	std::cout << "scale factor: " << aligns[i].m_s << std::endl;
	//	// computing the transformation matrix
	//	std::cout << "Applying transformation" << std::endl;
	//	aligns[i].compute_trans();
	//	
	//	aligns[i].apply(vertices_3d2, transformed);
	//	vertices_3d2 = transformed;

	//	GLOBAL_HELPERS::eigenVector_to_pclCloud(transformed, transforming);
	//	GLOBAL_HELPERS::eigenVector_color_to_pclCloud(color_vertices, transforming);

	//	cloudToPolygonMesh(transforming, meshes[i]);
	//	std::string prefix = "polygon";
	//	std::vector<char> chars(prefix.begin(), prefix.end() + prefix.length() + viewerId);
	//	const std::string meshid = strcat(&chars[0], "" + i);
	//	viewer->addPolygonMesh(*meshes[i], meshid);
	//	viewerId++;
	//}


	// removing all corresponding data of the poses that could not align
	for (int i = 0; i < remove_indices.size(); i++)
	{
		meshes[remove_indices[i]].reset();
		meshes.erase(meshes.begin() + remove_indices[i]);
		aligns.erase(aligns.begin() + remove_indices[i]);
		tags.erase(tags.begin() + remove_indices[i]);
	}

	std::cout << "aligns size: " << aligns.size() << std::endl;
	std::cout << "tags size: " << tags.size() << std::endl;
	std::cout << "poses size: " << meshes.size() << std::endl;


	cur_ser = meshes.size();
	ui->qvtkWidget->update();

	ui->comboBox_clouds_serial->clear();
	ui->comboBox_clouds_serial->addItem("NONE");
	for (int i = 0; i < meshes.size(); i++)
	{
		ui->comboBox_clouds_serial->addItem(QString::number(i));
	}
	ui->comboBox_clouds_serial->setCurrentIndex(meshes.size());
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


void pclviewer::addMeshToViewer(pcl::PolygonMesh::Ptr polymesh, int id)
{
	switch (id)
	{

	case 0:
		if (!viewer->addPolygonMesh(*polymesh, "0"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 1:
		if (!viewer->addPolygonMesh(*polymesh, "1"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 2:
		if (!viewer->addPolygonMesh(*polymesh, "2"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 3:
		if (!viewer->addPolygonMesh(*polymesh, "3"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 4:
		if (!viewer->addPolygonMesh(*polymesh, "4"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 5:
		if (!viewer->addPolygonMesh(*polymesh, "5"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 6:
		if (!viewer->addPolygonMesh(*polymesh, "6"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	case 7:
		if (!viewer->addPolygonMesh(*polymesh, "7"))
		{
			dbug << "duplicate id for mesh: " << id << ". chose another id" << std::endl;
		}
		break;
	default:
		break;
	}
	
}