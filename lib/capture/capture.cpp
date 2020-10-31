#include <boost/make_shared.hpp>

#include "capture.h"
#include "capture_helper.h"

void __M_Assert(const char *expr_str, bool expr, const char *file, int line,
                const char *msg) {
  if (!expr) {
    std::cerr << "Assert failed:\t" << msg << "\n"
              << "Expected:\t" << expr_str << "\n"
              << "Source:\t\t" << file << ", line " << line << "\n";
    abort();
  }
}

void cb(void *buffer, void *context) {}

Capture::Capture(uint8_t deviceId) {

  device = NULL;
  const int32_t TIMEOUT_IN_MS = 1000;
  transformation = NULL;
  capture = NULL;
  file_name = "";
  device_count = 0;
  config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  depth_image = NULL;
  color_image = NULL;
  xy_table = NULL;
  transformed_depth_image = NULL;
  cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  device_count = k4a_device_get_installed_count();

  bool device_close = false;
  // M_Assert(device_count > 0, "No K4A devices found");
  if (device_count == 0) {
    printf("No K4A devices found\n");
    device_close = true;
    goto Exit;
  }

  // M_Assert(K4A_RESULT_SUCCEEDED == k4a_device_open(K4A_DEVICE_DEFAULT,
  // &device), "Failed to open device");
  if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
    printf("Failed to open device\n");
    device_close = true;
    goto Exit;
  }

  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  config.synchronized_images_only = true; // ensures that depth and color images
                                          // are both available in the capture

  // M_Assert(K4A_RESULT_SUCCEEDED ==
  //		k4a_device_get_calibration(device, config.depth_mode,
  //config.color_resolution, &calibration), "Failed to get calibration");
  if (K4A_RESULT_SUCCEEDED !=
      k4a_device_get_calibration(device, config.depth_mode,
                                 config.color_resolution, &calibration)) {
    printf("Failed to get calibration\n");
    device_close = true;
    goto Exit;
  }

  color_width = calibration.color_camera_calibration.resolution_width;
  color_height = calibration.color_camera_calibration.resolution_height;

  transformation = k4a_transformation_create(&calibration);

  k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_width, color_height,
                   color_width * (int)sizeof(k4a_float2_t), &xy_table);

  create_xy_table();
  cloud->width = color_width;
  cloud->height = color_height;
  cloud->is_dense = false;
  cloud->resize(int(color_width) * int(color_height));

  if (K4A_RESULT_SUCCEEDED !=
      k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height,
                       color_width * (int)sizeof(uint16_t),
                       &transformed_depth_image)) {
    printf("Failed to create transformed depth image\n");
  }
  std::cout << "transformed_depth_image created" << std::endl;

  // M_Assert(K4A_RESULT_SUCCEEDED == k4a_device_start_cameras(device, &config),
  // "Failed to start cameras");
  if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config)) {
    printf("Failes to start device\n");
    device_close = true;
    goto Exit;
  }

Exit:

  if (device_close) {
    if (device != NULL) {
      k4a_device_close(device);
    }
  }
}

Capture::~Capture() {
  if (depth_image != NULL) {
    k4a_image_release(depth_image);
  }
  if (color_image != NULL) {
    k4a_image_release(color_image);
  }
  if (capture != NULL) {
    k4a_capture_release(capture);
  }
  if (transformation != NULL) {
    k4a_transformation_destroy(transformation);
  }
  if (device != NULL) {
    k4a_device_close(device);
  }
}

bool Capture::point_cloud_color_to_depth(std::string file_name) {
  int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
  int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);

  k4a_image_t transformed_color_image = NULL;
  if (K4A_RESULT_SUCCEEDED !=
      k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, depth_image_width_pixels,
                       depth_image_height_pixels,
                       depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                       &transformed_color_image)) {
    printf("Failed to create transformed color image\n");
    return false;
  }

  k4a_image_t point_cloud_image = NULL;
  if (K4A_RESULT_SUCCEEDED !=
      k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depth_image_width_pixels,
                       depth_image_height_pixels,
                       depth_image_width_pixels * 3 * (int)sizeof(int16_t),
                       &point_cloud_image)) {
    printf("Failed to create point cloud image\n");
    return false;
  }

  if (K4A_RESULT_SUCCEEDED !=
      k4a_transformation_color_image_to_depth_camera(
          transformation, depth_image, color_image, transformed_color_image)) {
    printf("Failed to compute transformed color image\n");
    return false;
  }

  if (K4A_RESULT_SUCCEEDED !=
      k4a_transformation_depth_image_to_point_cloud(transformation, depth_image,
                                                    K4A_CALIBRATION_TYPE_DEPTH,
                                                    point_cloud_image)) {
    printf("Failed to compute point cloud\n");
    return false;
  }

  if (!build_write_point_cloud(point_cloud_image, transformed_color_image,
                               file_name.c_str(), color_mat, depth_mat,
                               cloud)) {
    return false;
  }

  k4a_image_release(transformed_color_image);
  k4a_image_release(point_cloud_image);

  return true;
}

bool Capture::point_cloud_depth_to_color(k4a_image_t &transformed_depth_image,
                                         std::string file_name) {
  // setting the correct version of the depth_mat
  // depth_mat = transformed_depth_mat;

  k4a_image_t point_cloud_image = NULL;
  if (K4A_RESULT_SUCCEEDED !=
      k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels,
                       color_image_height_pixels,
                       color_image_width_pixels * 3 * (int)sizeof(int16_t),
                       &point_cloud_image)) {
    printf("Failed to create point cloud image\n");
    return false;
  }

  if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(
                                  transformation, transformed_depth_image,
                                  K4A_CALIBRATION_TYPE_COLOR,
                                  point_cloud_image)) {
    printf("Failed to compute point cloud\n");
    return false;
  }

  if (!build_write_point_cloud(point_cloud_image, color_image,
                               file_name.c_str(), color_mat, depth_mat,
                               cloud)) {
    return false;
  }

  k4a_image_release(point_cloud_image);
  return true;
}

bool Capture::capture_frame() {
  // Get a capture
  int try_capture = 0;
  bool capture_success = false;
  while (try_capture < 3) {
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS)) {
    case K4A_WAIT_RESULT_SUCCEEDED:
      capture_success = true;
      break;
    case K4A_WAIT_RESULT_TIMEOUT:
      printf("Timed out waiting for a capture\n");
    case K4A_WAIT_RESULT_FAILED:
      printf("Failed to read a capture\n");
    }

    try_capture++;
    if (capture_success)
      break;
  }

  if (!capture_success)
    return false;

  // Get a depth image
  depth_image = k4a_capture_get_depth_image(capture);
  if (depth_image == 0) {
    printf("Failed to get depth image from capture\n");
    return false;
  }
  // convert_k4a_image_to_cv_mat(depth_image, depth_mat, CV_16UC1);

  // Get a color image
  color_image = k4a_capture_get_color_image(capture);
  if (color_image == 0) {
    printf("Failed to get color image from capture\n");
    return false;
  }

  convert_k4a_image_to_cv_mat(color_image, color_mat, CV_8UC4);

  // getting transformed depth image ( depth image in the geometry of the color
  // image) useful when getting colol pixel correspondence to depth
  color_image_width_pixels = k4a_image_get_width_pixels(color_image);
  color_image_height_pixels = k4a_image_get_height_pixels(color_image);
  std::cout << "Depth image: width: " << k4a_image_get_width_pixels(depth_image)
            << " height: " << k4a_image_get_height_pixels(depth_image)
            << std::endl;

  if (K4A_RESULT_SUCCEEDED !=
      k4a_transformation_depth_image_to_color_camera(
          transformation, depth_image, transformed_depth_image)) {
    printf("Failed to compute transformed depth image\n");
    return false;
  }
  std::cout << "transformed_depth_image to color camera" << std::endl;

  /*if (!convert_k4a_image_to_cv_mat(transformed_depth_image,
  transformed_depth_mat, CV_16UC1))
  {
          std::cout << "Failed to convert from k4 Image type to cv Image type"
  << std::endl; return false;
  }
  std::cout << "convert transformed_depth_image to cv image" << std::endl;*/

  // to view point cloud either we can use the geometry of the color image or
  // the geometry of the depth image. If we chose geometry of the color image
  // then, we have to transform depth image into the geometry of the color image
  // and after that we can combine them. If we chose geometry of the depth image
  // then, we have to transform the color image into the geometry of the depth
  // image and after that we can combine them. But, because we have already
  // transformed depth image into the geometry of the color image for
  // correspondence purpose, building the point cloud into the geometry of color
  // image will save us time. But, in the end whichever option we chose, we have
  // to set depth_mat to correct version.

  // geometry of the color image
  /*if (!point_cloud_depth_to_color(transformed_depth_image))
  {
          return false;
  }*/

  generate_point_cloud(transformed_depth_image);

  // geometry of the depth image
  // if (!point_cloud_color_to_depth())
  //{
  //	return false;
  //}

  return true;
}

bool Capture::project_depthcam_to_colorcam(int depthcam_index,
                                           cv::Point &color_point) {
  float pixel_depth = cloud->points.at(depthcam_index).z;
  std::cout << "cloud pixel index: " << depthcam_index
            << "; pixel_depth: " << pixel_depth << std::endl;

  k4a_float2_t depth_pixel, color_pixel;
  depth_pixel.xy.x = static_cast<float>(depthcam_index % 640);
  depth_pixel.xy.y = static_cast<float>(depthcam_index / 640);

  int valid;
  if (K4A_RESULT_SUCCEEDED !=
      k4a_calibration_2d_to_2d(
          &calibration, &depth_pixel, pixel_depth, K4A_CALIBRATION_TYPE_DEPTH,
          K4A_CALIBRATION_TYPE_COLOR, &color_pixel, &valid)) {
    std::cout << "projection was not successfull" << std::endl;
    return false;
  }

  if (!valid) {
    std::cout
        << "No corresponding color pixel for the given depth pixel was found"
        << std::endl;
    return false;
  }

  color_point.x = color_pixel.xy.x;
  color_point.y = color_pixel.xy.y;

  return true;
}

bool Capture::project_colorcam_to_depthcam(cv::Point colorcam_pixel,
                                           int &depthcam_index) {
  // sanity check: if geven color pixel values are out of bound
  // std::cout << "colorcam_pixel size: " << color_image_width_pixels << " " <<
  // color_image_height_pixels << std::endl;
  std::cout << "colorcam_pixel x: " << colorcam_pixel.x
            << " colorcam_pixel y: " << colorcam_pixel.y << std::endl;
  if (colorcam_pixel.x >= color_image_width_pixels ||
      colorcam_pixel.y >= color_image_height_pixels || colorcam_pixel.x < 0 ||
      colorcam_pixel.y < 0) {
    std::cout << "colorcam_pixel out of bound" << std::endl;
    return false;
  }

  uint16_t *depth_data =
      (uint16_t *)(void *)k4a_image_get_buffer(transformed_depth_image);
  // float pixel_depth = transformed_depth_mat.at<ushort>(colorcam_pixel);
  float pixel_depth =
      (float)depth_data[colorcam_pixel.y * color_width + colorcam_pixel.x];

  std::cout << "color pixel: " << colorcam_pixel.x << " " << colorcam_pixel.y
            << "; pixel_depth: " << pixel_depth << std::endl;

  k4a_float2_t color_pixel, depth_pixel;
  color_pixel.xy.x = colorcam_pixel.x;
  color_pixel.xy.y = colorcam_pixel.y;

  int valid;
  if (K4A_RESULT_SUCCEEDED !=
      k4a_calibration_2d_to_2d(
          &calibration, &color_pixel, pixel_depth, K4A_CALIBRATION_TYPE_COLOR,
          K4A_CALIBRATION_TYPE_COLOR, &depth_pixel, &valid)) {
    std::cout << "projection was not successfull" << std::endl;
    // depthcam_index = -1;
    return false;
  }

  if (!valid) {
    std::cout
        << "No corresponding color pixel for the given depth pixel was found"
        << std::endl;
    // depthcam_index = -1;
    return false;
  }

  std::cout << "depth pixel; x = " << (int)depth_pixel.xy.x
            << " , y = " << (int)depth_pixel.xy.y << std::endl;
  depthcam_index = (int)depth_pixel.xy.y * color_width + (int)depth_pixel.xy.x;

  return true;
}

void Capture::create_xy_table() {
  k4a_float2_t *table_data =
      (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

  int width = color_width;
  int height = color_height;

  k4a_float2_t p;
  k4a_float3_t ray;
  int valid;

  for (int y = 0, idx = 0; y < height; y++) {
    p.xy.y = (float)y;
    for (int x = 0; x < width; x++, idx++) {
      p.xy.x = (float)x;

      k4a_calibration_2d_to_3d(&calibration, &p, 1.f,
                               K4A_CALIBRATION_TYPE_COLOR,
                               K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);

      if (valid) {
        table_data[idx].xy.x = ray.xyz.x;
        table_data[idx].xy.y = ray.xyz.y;
      } else {
        table_data[idx].xy.x = nanf("");
        table_data[idx].xy.y = nanf("");
      }
    }
  }
}

void Capture::generate_point_cloud(const k4a_image_t depth_image) {
  int width = color_width;
  int height = color_height;

  uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
  uint8_t *color_data = k4a_image_get_buffer(color_image);
  k4a_float2_t *xy_table_data =
      (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

  int valid_point = 0;
  for (int i = 0; i < width * height; i++) {
    if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) &&
        !isnan(xy_table_data[i].xy.y)) {
      cloud->at(i).x = xy_table_data[i].xy.x * (float)depth_data[i];
      cloud->at(i).y = xy_table_data[i].xy.y * (float)depth_data[i];
      cloud->at(i).z = (float)depth_data[i];

      cloud->at(i).r = color_data[4 * i + 0];
      cloud->at(i).g = color_data[4 * i + 1];
      cloud->at(i).b = color_data[4 * i + 2];

      valid_point++;
    } else {
      cloud->at(i).x = nanf("");
      cloud->at(i).y = nanf("");
      cloud->at(i).z = nanf("");

      cloud->at(i).b = 0;
      cloud->at(i).g = 0;
      cloud->at(i).r = 0;
    }
  }

  std::cout << "valid points: " << valid_point << std::endl;
}

// bool Capture::project_colorcam_to_depthcam(cv::Point colorcam_pixel, int&
// depthcam_index, cv::Mat& depth_im, int color_w, int color_h)
//{
//	//std::cout << "width: " << k4a_image_get_width_pixels(depth_image) << "
//height: " << k4a_image_get_height_pixels(depth_image) << std::endl;
//	k4a_image_t transformed_depth_image = NULL;
//	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
//color_w, 		color_h, color_w * (int)sizeof(uint16_t), &transformed_depth_image))
//	{
//		printf("Failed to create transformed depth image\n");
//		return false;
//	}
//
//
//	std::vector<uchar> array;
//	if (depth_im.isContinuous())
//	{
//		array.assign(depth_im.data, depth_im.data + depth_im.total());
//	}
//	else
//	{
//		for (int i = 0; i < depth_im.rows; ++i)
//		{
//			array.insert(array.end(), depth_im.ptr<uint16_t>(i),
//depth_im.ptr<uint16_t>(i) + depth_im.cols);
//		}
//	}
//
//	uint8_t* b_data = reinterpret_cast<uint8_t*>(&array[0]);
//	k4a_image_t new_depth_im = NULL;
//	void* context = NULL;
//
//	if (K4A_RESULT_SUCCEEDED !=
//k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16, 		depth_im.cols,
//depth_im.rows, depth_im.cols * (int)sizeof(uint16_t), 			b_data, sizeof b_data,
//cb, context, &new_depth_im))
//	{
//		printf("Failed to create image from buffer\n");
//		return false;
//	}
//
//
//	if (K4A_RESULT_SUCCEEDED !=
//		k4a_transformation_depth_image_to_color_camera(transformation,
//new_depth_im, transformed_depth_image))
//	{
//		printf("Failed to compute transformed depth image\n");
//		return false;
//	}
//
//	cv::Mat transformed_depth_mat;
//	if (!convert_k4a_image_to_cv_mat(transformed_depth_image,
//transformed_depth_mat, CV_16UC1))
//	{
//		std::cout << "Failed to convert from k4 Image type to cv Image
//type" << std::endl; 		return false;
//	}
//
//	float pixel_depth = transformed_depth_mat.at<unsigned
//short>(colorcam_pixel);
//	//std::cout << "color pixel: " << colorcam_pixel.x << " " <<
//colorcam_pixel.y << "; pixel_depth: " << pixel_depth << std::endl;
//
//	k4a_float2_t color_pixel, depth_pixel;
//	color_pixel.xy.x = colorcam_pixel.x;
//	color_pixel.xy.y = colorcam_pixel.y;
//
//	int valid;
//	if (K4A_RESULT_SUCCEEDED != k4a_calibration_2d_to_2d(&calibration,
//&color_pixel, 		pixel_depth, K4A_CALIBRATION_TYPE_COLOR,
//K4A_CALIBRATION_TYPE_DEPTH, &depth_pixel, &valid))
//	{
//		std::cout << "projection was not successfull" << std::endl;
//		return false;
//	}
//
//	if (!valid)
//	{
//		std::cout << "No corresponding color pixel for the given depth
//pixel was found" << std::endl; 		return false;
//	}
//
//	std::cout << "depth pixel; x = " << (int)depth_pixel.xy.x << " , y = "
//<< (int)depth_pixel.xy.y << std::endl; 	depthcam_index = (int)depth_pixel.xy.y
//* 640 + (int)depth_pixel.xy.x;
//
//	return true;
//}