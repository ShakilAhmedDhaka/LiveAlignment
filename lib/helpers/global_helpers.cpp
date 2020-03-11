#include <fstream>


#include "global_helpers.h"


namespace GLOBAL_HELPERS
{
	bool check_out_scope(cv::Mat mat, cv::Point point)
	{
		int height = mat.size().height;
		int width = mat.size().width;

		if (point.x < 0)
		{
			std::cout << "point column position is negetive" << std::endl;
			return false;
		}
		if (point.x >= width)
		{
			std::cout << "point column position is larger than image width" << std::endl;
			return false;
		}
		if (point.y < 0)
		{
			std::cout << "point row position is negetive" << std::endl;
			return false;
		}
		if (point.y >= height)
		{
			std::cout << "point row position is larger than image height" << std::endl;
			return false;
		}

		return true;
	}



	void cvonMouseClick(int event, int x, int y, int, void* point)
	{
		if (event != cv::EVENT_LBUTTONDOWN) return;

		cv::Point* p = (cv::Point*) point;
		p->x = x;
		p->y = y;
	}


	void get_actual_cloud_index(pcl::PointCloud<pcl::PointXYZRGB>& cloud_, pcl::PointXYZ& picked_pt, int& actual_index)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud;
		pcl::search::KdTree<pcl::PointXYZ> search;

		pcl::PCLPointCloud2::Ptr cloud;

		if (!cloud)
		{

			cloud.reset(new pcl::PCLPointCloud2());
			pcl::toPCLPointCloud2(cloud_, *cloud);
			xyzcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(cloud_, *xyzcloud);
			search.setInputCloud(xyzcloud);
		}

		std::vector<int> indices(1);
		std::vector<float> distances(1);

		search.nearestKSearch(picked_pt, 1, indices, distances);
		
		int corr_ind = 0;
		/*float close = 1000000000;
		for (int i = 0; i < 100; i++)
		{
			if (std::max(picked_pt.z - cloud_.at(indices[i]).z, cloud_.at(indices[i]).z - picked_pt.z) < close)
			{
				close = std::max(picked_pt.z - cloud_.at(indices[i]).z, cloud_.at(indices[i]).z - picked_pt.z);
				corr_ind = i;
			}
		}*/
		actual_index = indices[corr_ind];
	}

	// concate char arrays
	char* concat(char* ch1, const char* ch2)
	{
		char* ch3 = (char*)malloc(1 + strlen(ch1) + strlen(ch2));;
		strcpy(ch3, ch1);
		strcat(ch3, ch2);
		return ch3;
	}


	// prints the error in case user gives wrong input
	void print_error()
	{
		std::cout << "input just the name of the file without extension.\nname of pointcloud and jpg file must be same" << std::endl;
		std::cout << "example: ./tags_3d ../outputs/0_604" << std::endl;
		std::cout << "given pointcloud file does not exist" << std::endl;
	}




	// take input files: one point cloud and rgb file
	int take_input(int argc, char** argv, int pc_serial, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud, cv::Mat& color, cv::Mat& depth,
		std::vector<Global_helpers::TagPoints>& tags)
	{
		// Taking cloud input either as pcd or ply

		if (boost::filesystem::exists(concat(argv[pc_serial], ".pcd")))
		{
			pcl::io::loadPCDFile(concat(argv[pc_serial], ".pcd"), *cloud);
			//cout<<"cloud->width "<<cloud->width<<" cloud->height "<<cloud->height<<endl;
		}

		if (boost::filesystem::exists(concat(argv[pc_serial], ".jpg")))
		{
			color = cv::imread(concat(argv[pc_serial], ".jpg"), 1);
			cvtColor(color, color, cv::COLOR_BGR2BGRA);
			std::cout << color.type() << std::endl;
		}
		else
		{
			print_error();
			return -1;
		}
		if (boost::filesystem::exists(concat(argv[pc_serial], ".png")))
		{
			depth = cv::imread(concat(argv[pc_serial], ".png"), CV_16UC1);
		}
		else
		{
			print_error();
			return -1;
		}
		if (boost::filesystem::exists(concat(argv[pc_serial], "_feature_points.txt")))
		{
			int tags_size = 0;
			std::ifstream ifile(concat(argv[pc_serial], "_feature_points.txt"));
			ifile >> tags_size;
			std::cout << tags_size << std::endl;
			for (int i = 0; i < tags_size; i++)
			{
				Global_helpers::TagPoints tagPoint;
				ifile >> tagPoint.tag;
				std::cout << "tag id: " << tagPoint.tag << std::endl;
				ifile >> tagPoint.pos;
				std::cout << "tag pos: " << tagPoint.pos << std::endl;
				ifile >> tagPoint.cloud_index;
				std::cout << "tag cloud index: " << tagPoint.cloud_index << std::endl;
				ifile >> tagPoint.point.x >> tagPoint.point.y;
				std::cout << "tag 2d point; x: " << tagPoint.point.x << " y: " << tagPoint.point.y << std::endl;

				ifile >> tagPoint.point3D[0] >> tagPoint.point3D[1] >> tagPoint.point3D[2];
				std::cout << "tag 3d point; x: " << tagPoint.point3D[0] << " y: " << tagPoint.point3D[1] << " z: " << tagPoint.point3D[2] << std::endl;
			
				tags.push_back(tagPoint);
			} 
		}
		else
		{
			print_error();
			return -1;
		}
		return 0;
	}



	// takes input of a set of files: one element of the set =  one point cloud, rgb file, one feature points file
	int take_input(int argc, char** argv, std::vector< boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> >& clouds, std::vector<std::string>& filenames,
		std::vector<std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>>& tags_list)
	{
		std::cout << "Number of files: " << argc - 1 << std::endl;
		clouds.resize(argc - 1);
		filenames.resize(argc - 1);
		int sz = 0;
		for (int i = 1; i < argc; i++)
		{
			// Taking cloud input either as pcd or ply
			std::string fname_str = std::string(argv[i]);
			char* fname = new char[fname_str.length() + 1];
			strcpy(fname, fname_str.c_str());
			filenames[i - 1] = std::string(get_tokens(fname, "/", sz)[sz]);
			if (boost::filesystem::exists(concat(argv[i], ".pcd")))
			{
				clouds[i - 1] = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
				pcl::io::loadPCDFile(concat(argv[i], ".pcd"), *clouds[i - 1]);
				//cout<<"cloud->width "<<cloud->width<<" cloud->height "<<cloud->height<<endl;
			}
			else if (boost::filesystem::exists(concat(argv[i], ".ply"))) pcl::io::loadPLYFile(concat(argv[i], ".ply"), *clouds[i - 1]);
			else
			{
				print_error();
				return -1;
			}


			if (boost::filesystem::exists(concat(argv[i], "_feature_points.txt")))
			{
				int tags_size = 0;
				std::ifstream ifile(concat(argv[i], "_feature_points.txt"));
				ifile >> tags_size;
				std::cout << tags_size << std::endl;
				std::vector<Global_helpers::TagPoints> tags;
				for (int i = 0; i < tags_size; i++)
				{
					Global_helpers::TagPoints tagPoint;
					ifile >> tagPoint.tag;
					std::cout << "tag id: " << tagPoint.tag << std::endl;
					ifile >> tagPoint.pos;
					std::cout << "tag pos: " << tagPoint.pos << std::endl;
					ifile >> tagPoint.cloud_index;
					std::cout << "tag cloud index: " << tagPoint.cloud_index << std::endl;
					ifile >> tagPoint.point.x >> tagPoint.point.y;
					std::cout << "tag 2d point; x: " << tagPoint.point.x << " y: " << tagPoint.point.y << std::endl;

					ifile >> tagPoint.point3D[0] >> tagPoint.point3D[1] >> tagPoint.point3D[2];
					std::cout << "tag 3d point; x: " << tagPoint.point3D[0] << " y: " << tagPoint.point3D[1] << " z: " << tagPoint.point3D[2] << std::endl;

					tags.push_back(tagPoint);
				}

				tags_list.push_back(tags);
			}
			else
			{
				print_error();
				return -1;
			}
		}

		return 1;
	}



	void pclToEigen(pcl::PointXYZ& picked_pt, Global_helpers::TagPoints& tags)
	{
		tags.point3D[0] = picked_pt.x;
		tags.point3D[1] = picked_pt.y;
		tags.point3D[2] = picked_pt.z;
	}



	std::vector<std::string> get_tokens(std::string str, char* delimeter, int& sz)
	{
		int i = 0;
		std::vector<std::string> tokens;

		// stringstream class check1 
		std::stringstream str_stream(str);

		std::string intermediate;
		while (getline(str_stream, intermediate, '/'))
		{
			tokens.push_back(intermediate);
		}

		sz = tokens.size() - 1;
		
		return tokens;
	}

	std::string get_pair_save_name(char** argv, int no_1, int no_2)
	{
		int sz = 0;
		std::string first_file = argv[no_1];
		std::string second_file = argv[no_2];
		std::vector<std::string> tokens = get_tokens(first_file, "/", sz);
		std::string save_name = "";
		for (int i = 0; i < tokens.size() - 1; i++) save_name += tokens[i] + "/";
		std::string first_token = get_tokens(first_file, "/", sz)[sz];
		std::string second_token = get_tokens(second_file, "/", sz)[sz];
		std::cout << "first token: " << first_token << std::endl;
		std::cout << "second token: " << second_token << std::endl;
		save_name += first_token;
		save_name += "_";
		save_name += second_token;
		//save_name + "_feature_points.txt";
		//std::cout << "save name: " << save_name << std::endl;
		return save_name;
	}

	bool is_file_exist(std::string fileName)
	{
		std::ifstream infile(fileName.c_str());
		return infile.good();
	}



	void pclToEigenVector(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
		std::vector<Eigen::Vector3d>& vertices)
	{

		vertices.resize(cloud->points.size());
		for (int i = 0; i < cloud->points.size(); i++)
		{
			vertices[i][0] = cloud->points[i].x;
			vertices[i][1] = cloud->points[i].y;
			vertices[i][2] = cloud->points[i].z;
		}
	}

	void pclToEigenVector_color(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud,
		std::vector<Eigen::Vector3d>& vertices)
	{

		vertices.resize(cloud->points.size());
		for (int i = 0; i < cloud->points.size(); i++)
		{
			vertices[i][0] = cloud->points[i].r;
			vertices[i][1] = cloud->points[i].g;
			vertices[i][2] = cloud->points[i].b;
		}
	}


	void eigenVector_to_pclCloud(std::vector<Eigen::Vector3d>& transformed,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud)
	{
		//cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>();
		//cloud->resize(transformed.size());
		for (int i = 0; i < cloud->points.size(); i++)
		{
			cloud->points[i].x = transformed[i][0];
			cloud->points[i].y = transformed[i][1];
			cloud->points[i].z = transformed[i][2];
		}

	}


	void eigenVector_color_to_pclCloud(std::vector<Eigen::Vector3d>& transformed,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud)
	{
		//cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>();
		//cloud->resize(transformed.size());
		for (int i = 0; i < cloud->points.size(); i++)
		{
			cloud->points[i].r = transformed[i][0];
			cloud->points[i].g = transformed[i][1];
			cloud->points[i].b = transformed[i][2];
		}

	}


	void save_tags(std::vector<Global_helpers::TagPoints> feature_points, std::string filename)
	{
		std::string fullpath = filename + "_feature_points.txt";

		std::ofstream ofile(fullpath);

		ofile << feature_points.size() << std::endl;
		for (int i = 0; i < feature_points.size(); i++)
		{
			Global_helpers::TagPoints tp = feature_points[i];
			ofile << tp.tag << std::endl;
			ofile << tp.pos << std::endl;
			ofile << tp.cloud_index << std::endl;
			ofile << tp.point.x << " " << tp.point.y << std::endl;
			ofile << tp.point3D[0] << " " << tp.point3D[1] << " " << tp.point3D[2] << std::endl;
		}
	}

	void insert_virtual_points(std::vector<Global_helpers::TagPoints>& tags)
	{
		// CrossProduct((v2-v1), (v3-v1))
		Eigen::Vector3d normal, avg_normal = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
		Global_helpers::TagPoints tg;
		tg.point3D = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < 4; i++)
		{			
			normal = ((tags[(i+1)%4].point3D - tags[i].point3D).cross(tags[(i+3)%4].point3D - tags[i].point3D));
			normal.normalize();
			avg_normal += normal/4.0f;
			tg.point3D += tags[i].point3D / 4.0f;
		}


		tg.point3D +=  avg_normal;
		tg.tag = tags[0].tag;
		tg.pos = 4;
		tags.push_back(tg);
	}
		


	void copyCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_in, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_out)
	{
		int width = cloud_in->width;
		int height = cloud_in->height;
		std::cout << width << std::endl;
		std::cout << height << std::endl;
		cloud_out->width = width;
		cloud_out->height = height;
		cloud_out->is_dense = cloud_in->is_dense;
		cloud_out->points.resize(cloud_in->width * cloud_in->height);

		pcl::PointXYZRGB* pt_out = &cloud_out->points[0];
		pcl::PointXYZRGB* pt_in = &cloud_in->points[0];
		for (int i = 0; i < cloud_in->width * cloud_in->height; i++, pt_out++, pt_in++)
		{
			pt_out->x = pt_in->x;
			pt_out->y = pt_in->y;
			pt_out->z = pt_in->z;

			pt_out->r = pt_out->r;
			pt_out->g = pt_out->g;
			pt_out->b = pt_out->b;
			//pt_out->a = pt_out->a;
		}
	}


	/// GLOBAL HELPERS CLASS ///


	Global_helpers::Global_helpers()
	{

	}

	Global_helpers::~Global_helpers()
	{

	}

	
	void Global_helpers::read_directory(const std::string& name, std::vector<std::string>& v)
	{
		boost::filesystem::path p(name);
		boost::filesystem::directory_iterator start(p);
		boost::filesystem::directory_iterator end;
		std::transform(start, end, std::back_inserter(v), path_leaf_string());
	}
 


	int Global_helpers::get_serial(std::string directory)
	{
		std::vector<std::string> v;
		read_directory(directory, v);

		int c = 0, p = 0, r = 0;
		for (int i = 0; i < v.size(); i++)
		{
			std::vector <std::string> tokens;
			std::stringstream check1(v[i]);
			std::string intermediate;
			while (getline(check1, intermediate, '.'))
			{
				tokens.push_back(intermediate);
			}

			std::string ext = tokens[tokens.size() - 1];
			if (!ext.compare("pcd")) c++;
			else if (!ext.compare("ply")) p++;
			else if (!ext.compare("jpg")) r++;

		}
		return std::max(c, std::max(p, r));
	}



	std::string Global_helpers::get_new_filename(std::string directory)
	{
		int serial_number = get_serial(directory);
		std::cout << "current serial: " << serial_number << std::endl;
		std::stringstream ss;
		ss << serial_number;
		std::string serial = ss.str();
		std::chrono::high_resolution_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
		std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(cur_time.time_since_epoch()).count() % 1000);
		std::string new_filename = directory + "/" + serial + "_" + now;
		return new_filename;
	}





	void Global_helpers::write_pcd(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string full_path)
	{
		pcl::PCDWriter pcd_writer;
		pcl::io::savePCDFileASCII(full_path + ".pcd", cloud);
		std::cout << "saved: " << full_path + ".pcd" << std::endl;
	}

	void Global_helpers::write_pcd(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string filepath, std::string filename)
	{
		write_pcd(cloud, filepath + "/" + filename);
	}

	void Global_helpers::write_ply(pcl::PointCloud<pcl::PointXYZRGB> cloud, bool binary, bool use_camera, std::string full_path)
	{
		pcl::PLYWriter ply_writer;
		ply_writer.write(full_path + ".ply", cloud, binary, use_camera);
		std::cout << "saved: " << full_path + ".ply" << std::endl;
	}


	void Global_helpers::write_ply(pcl::PointCloud<pcl::PointXYZRGB> cloud, bool binary, bool use_camera, std::string filepath, std::string filename)
	{
		write_ply(cloud, binary, use_camera, filepath + "/" + filename);
	}


	void Global_helpers::write_cvImage(cv::Mat image, std::string full_path, std::string ext)
	{
		cv::imwrite(full_path + "." + ext, image);
		std::cout << "saved: " << full_path + "." + ext << std::endl;
	}

	void Global_helpers::write_cvImage(cv::Mat image, std::string filepath, std::string filename, std::string ext)
	{
		write_cvImage(image, filepath + "/" + filename, ext);
	}



	void Global_helpers::pointcloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string full_path)
	{
		pcl::PolygonMesh triangles;
		pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;

		// Set parameters
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		*cloud_ptr = cloud;
		ofm.setInputCloud(cloud_ptr);
		ofm.setMaxEdgeLength(1.5);
		ofm.setTrianglePixelSize(1);
		ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);

		// Reconstruct
		ofm.reconstruct(triangles);
		pcl::io::saveOBJFile(full_path + ".obj", triangles);
		std::cout << "saved: " << full_path + ".obj" << std::endl;
	}


	void Global_helpers::pointcloud_to_mesh(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::string filepath, std::string filename)
	{
		pointcloud_to_mesh(cloud, filepath + "/" + filename);
	}


	void Global_helpers::set_path(std::string directory)
	{
		path = directory;
	}


}