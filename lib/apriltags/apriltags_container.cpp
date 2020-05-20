#include "apriltags_container.h"
#include "visualizer.h"


Apriltags::Apriltags()
{
	// setting apriltag input parameters for user
	getopt = getopt_create();
	tf = NULL;
	td = apriltag_detector_create();
	famname = set_tag_param();
}



Apriltags::Apriltags(const char* family)
{
	// setting apriltag input parameters for user
	getopt = getopt_create();
	tf = NULL;
	td = apriltag_detector_create();
	famname = set_tag_param(family);
}




Apriltags::~Apriltags()
{
	apriltag_detector_destroy(td);
	if (!strcmp(famname, "tag36h11"))
		tag36h11_destroy(tf);
	else if (!strcmp(famname, "tag36h10"))
		tag36h10_destroy(tf);
	else if (!strcmp(famname, "tag36artoolkit"))
		tag36artoolkit_destroy(tf);
	else if (!strcmp(famname, "tag25h9"))
		tag25h9_destroy(tf);
	else if (!strcmp(famname, "tag25h7"))
		tag25h7_destroy(tf);
	else if (!strcmp(famname, "tag16h5"))
	{
		tag16h5_destroy(tf);
	}
		
	getopt_destroy(getopt);
}


void Apriltags::rearrange_tags(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags1, 
	std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags2, int& different_tags)
{
	std::queue<GLOBAL_HELPERS::Global_helpers::TagPoints> left_q_tag1, right_q_tag1, left_q_tag2, right_q_tag2;
	different_tags = 0;
	int diff_tag = -1;
	for (int i = 0; i < tags1.size(); i++)
	{
		left_q_tag1.push(tags1[i]);
		left_q_tag2.push(tags2[i]);

		if (tags1[i].tag != diff_tag)
		{
			diff_tag = tags1[i].tag;
			different_tags++;
		}
	}

	tags1.clear();
	tags2.clear();

	while (!left_q_tag1.empty())
	{
		int tag = -1;
		GLOBAL_HELPERS::Global_helpers::TagPoints tg1, tg2;
		while (!left_q_tag1.empty())
		{
			tg1 = left_q_tag1.front();
			tg2 = left_q_tag2.front();
			left_q_tag1.pop();
			left_q_tag2.pop();

			if (tg1.tag != tag)
			{
				tag = tg1.tag;
				//std::cout << "inserting points " << tg1.point3D[0] << " " << tg1.point3D[1] << " " << tg1.point3D[2] << std::endl;
				//std::cout << "inserting points " << tg2.point3D[0] << " " << tg2.point3D[1] << " " << tg2.point3D[2] << std::endl;
				tags1.push_back(tg1);
				tags2.push_back(tg2);
			}
			else
			{
				right_q_tag1.push(tg1);
				right_q_tag2.push(tg2);
			}

		}

		while (!right_q_tag1.empty())
		{
			left_q_tag1.push(right_q_tag1.front());
			right_q_tag1.pop();
			left_q_tag2.push(right_q_tag2.front());
			right_q_tag2.pop();
		}
	}
}




const char* Apriltags::set_tag_param()
{
	return set_tag_param("tag36h11");
}


// setting parameters for apriltags during input
const char* Apriltags::set_tag_param(const char* family)
{
	getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
	getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
	getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
	getopt_add_string(getopt, 'f', "family", "tag16h5", "Tag family to use");
	getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
	getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
	getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
	getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
	getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
	getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
	getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");


	// Initialize tag detector with options
	//tf = tag25h7_create();
	tf = tag36h11_create();
	const char* famname = getopt_get_string(getopt, "family");
	//if (!strcmp(famname, "tag36h11"))
	//	tf = tag36h11_create();
	//else if (!strcmp(famname, "tag36h10"))
	//	tf = tag36h10_create();
	//else if (!strcmp(famname, "tag36artoolkit"))
	//	tf = tag36artoolkit_create();
	//else if (!strcmp(famname, "tag25h9"))
	//	tf = tag25h9_create();
	//else if (!strcmp(famname, "tag25h7"))
	//	tf = tag25h7_create();
	//else if (!strcmp(famname, "tag16h5"))
	//	tf = tag16h5_create();
	//else if (!strcmp(family, "tag16h5"))
	//{
	//	std::cout << "************ in tag16h5 condition ***************" << std::endl;
	//	tf = tag16h5_create();
	//}
	//else
	//{
	//	printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
	//	exit(-1);
	//}
	tf->black_border = getopt_get_int(getopt, "border");


	apriltag_detector_add_family(td, tf);
	td->quad_decimate = getopt_get_double(getopt, "decimate");
	td->quad_sigma = getopt_get_double(getopt, "blur");
	td->nthreads = getopt_get_int(getopt, "threads");
	td->debug = getopt_get_bool(getopt, "debug");
	td->refine_edges = getopt_get_bool(getopt, "refine-edges");
	td->refine_decode = getopt_get_bool(getopt, "refine-decode");
	td->refine_pose = getopt_get_bool(getopt, "refine-pose");

	return famname;
}



void Apriltags::vis_apriltags(cv::Mat& frame, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& aprl_tags,
	int width_offset, int height_offset,
	bool drawMiddleSquare, bool showTagID, bool drawBorder, bool drawCornerBlocks,
	bool correctAngle)
{
	
	for (int i = 0; i+3 < aprl_tags.size(); i+=4)
	{
		cv::Point bottomLeft = cv::Point(aprl_tags[i].point.x / width_offset, aprl_tags[i].point.y / height_offset);
		cv::Point bottomRight = cv::Point(aprl_tags[i + 1].point.x / width_offset, aprl_tags[i + 1].point.y / height_offset);
		cv::Point topRight = cv::Point(aprl_tags[i + 2].point.x / width_offset, aprl_tags[i + 2].point.y / height_offset);
		cv::Point topLeft = cv::Point(aprl_tags[i + 3].point.x / width_offset, aprl_tags[i + 3].point.y / height_offset);
		//cv::Point center = cv::Point(aprl_tags[i].centerx / width_offset, aprl_tags[i].centery / height_offset);

		cv::Vec4b red = cv::Vec4b(0, 0, 255, 255);
		cv::Vec4b green = cv::Vec4b(0, 255, 0, 255);
		cv::Vec4b blue = cv::Vec4b(255, 0, 0, 255);
		cv::Vec4b yellow = cv::Vec4b(0, 255, 255, 255);

		// draw a square in the center of each tag
		if (drawMiddleSquare)
		{
			cv::Point center = cv::Point(aprl_tags[i].centerx / width_offset, aprl_tags[i].centery / height_offset);
			cv::Vec4b red = cv::Vec4b(0, 0, 255, 255);
			int blockRadiusColor = 5;

			VISH::mark_color_mat(frame, center, red, blockRadiusColor);

			return;
		}


		// Draw detection outlines
		if (drawBorder)
		{
			int thickness = 2;

			line(frame, bottomLeft, bottomRight, cv::Scalar(0, 0xff, 0), thickness);
			line(frame, bottomLeft, topLeft, cv::Scalar(0, 0, 0xff), thickness);
			line(frame, bottomRight, topRight, cv::Scalar(0xff, 0, 0), thickness);
			line(frame, topRight, topLeft, cv::Scalar(0xff, 0, 0), thickness);
		}
	

		// draw small blocks in each corner of each tag
		if (drawCornerBlocks)
		{
			int blockRadiusColor = 2;

			VISH::mark_color_mat(frame, bottomLeft, red, blockRadiusColor);
			VISH::mark_color_mat(frame, bottomRight, green, blockRadiusColor);
			VISH::mark_color_mat(frame, topRight, blue, blockRadiusColor);
			VISH::mark_color_mat(frame, topLeft, yellow, blockRadiusColor);
		}
		
		// show the id of each tag
		if (showTagID)
		{
			std::stringstream ss;
			ss << aprl_tags[i].tag;
			cv::String text = ss.str();
			int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
			double fontscale = 1.0;


			int baseline;
			cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
				&baseline);
			putText(frame, text, cv::Point( (aprl_tags[i].centerx / width_offset) - textsize.width / 2,
				(aprl_tags[i].centery / height_offset) + textsize.height / 2),
			fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
		}
		
	}


	cv::Point center = cv::Point(aprl_tags[0].centerx / width_offset, aprl_tags[0].centery / height_offset);
	int blockRadiusColor = 5;
	if (correctAngle)
	{
		cv::Vec4b green = cv::Vec4b(0, 255, 0, 255);
		VISH::mark_color_mat(frame, center, green, blockRadiusColor);
	}
	else if(drawMiddleSquare == false)
	{
		cv::Vec4b yellow = cv::Vec4b(0, 255, 255, 255);
		VISH::mark_color_mat(frame, center, yellow, blockRadiusColor);
	}
	
}

void Apriltags::get_tags(cv::Mat& frame, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& aprl_tags,
	int width_offset, int height_offset, int f_serial)
{
	width_offset = std::max(width_offset, 1);
	height_offset = std::max(height_offset, 1);
	cv::Mat gray;
	cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
	image_u8_t im{ gray.cols, gray.rows, gray.cols, gray.data };

	zarray_t* detections = apriltag_detector_detect(td, &im);
	
	// Draw detection outlines
	GLOBAL_HELPERS::Global_helpers::TagPoints tag_point;
	for (int i = 0; i < zarray_size(detections); i++)
	{
		apriltag_detection_t* det;
		zarray_get(detections, i, &det);
		
		// saving the tags
		for (int j = 0; j < 4; j++)
		{
			int px = (int)det->p[j][0];
			int py = (int)det->p[j][1];

			tag_point.point.x = px * width_offset;
			tag_point.point.y = py * height_offset;
			tag_point.tag = det->id;
			tag_point.pos = j;
			tag_point.frame_serial = f_serial;
			tag_point.centerx = det->c[0] * width_offset;
			tag_point.centery = det->c[1] * height_offset;
			aprl_tags.push_back(tag_point);
		}
	}

	//sort(aprl_tags.begin(), aprl_tags.end());

	zarray_destroy(detections);
}


void Apriltags::filter_tag(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags)
{
	for (int i = 0; i < tags.size(); i++)
	{
		if (tags[i].cloud_index == -1) tags.erase(tags.begin() + i);
	}
}

void Apriltags::common_tags(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags1, 
	std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags2)
{
	for (int i = 0; i < tags1.size(); i++)
	{
		bool rem = true;
		for (int j = 0; j < tags2.size(); j++)
		{
			if (tags1[i].tag == tags2[j].tag && tags1[i].pos == tags2[j].pos
				//&& (tags1[i].tag == 4 || tags1[i].tag == 3)
				)
			{
				rem = false;
			}
		}

		if (rem)
		{
			tags1.erase(tags1.begin() + i);
			i--;
		}
	}

	for (int i = 0; i < tags2.size(); i++)
	{
		bool rem = true;
		for (int j = 0; j < tags1.size(); j++)
		{
			if (tags1[j].tag == tags2[i].tag && tags1[j].pos == tags2[i].pos
				//&& (tags2[i].tag == 4 || tags2[i].tag == 3) 

				)
			{
				rem = false;
			}
		}

		if (rem)
		{
			tags2.erase(tags2.begin() + i);
			i--;
		}
	}


	std::cout << "after filtering" << std::endl;
	std::cout << "tags1 size: " << tags1.size() << std::endl;
	std::cout << "tags2 size: " << tags2.size() << std::endl;

}
