#pragma once

#ifndef _APRILTAGS_H_
#define _APRILTAGS_H_


#include <opencv2/opencv.hpp>
#include <Eigen/SparseCore>
#include "global_helpers.h"

// apriltags headers shloud be included at the end
// as they did not give header guards in the
// header files
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"
#include "common/getopt.h"




class Apriltags
{
	public:
		getopt_t* getopt;
		apriltag_family_t* tf;
		apriltag_detector_t* td;
		const char* famname;

		Apriltags();
		Apriltags(const char* family);
		~Apriltags();
	
		// drawBorder = draws border on the tags
		// drawBlock = draws blocks on the corners of the tags
		void vis_apriltags(cv::Mat& frame, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& aprl_tags,
			int width_offset=0, int height_offset=0,
			bool drawMiddleSquare = false, bool showTagID = true, bool drawborder = true, bool drawCornerBlocks = false,
			bool correctAngle  = false);
		
		// returns sorted tags from a color image based on the tag id
		// and then the order of tags in their frame
		void get_tags(cv::Mat& frame, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& aprl_tags, 
			int width_offset=0, int height_offset=0, int f_serial = -1);

		
		// remove any tag that does not have correspondence with 2d (not present in color image)
		void filter_tag(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags);
		
		// keep the common tags from both scenes. remove the rest
		void common_tags(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags1, std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags2);

		// rearrange the tags such that a point from the same tag appears after the number of different tags.
		// it is rearranged so that when we limit the number of corresponding points while aligning, we can have different points
		// from different tags as much as possible
		void rearrange_tags(std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags1, 
			std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints>& tags2, int& different_tags);
	private:
		std::vector<GLOBAL_HELPERS::Global_helpers::TagPoints> tags;
		const char* set_tag_param();
		const char* set_tag_param(const char* family);
};




#endif