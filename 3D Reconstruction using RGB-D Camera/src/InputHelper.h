#pragma once
#include <pxcsensemanager.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

typedef pcl::PointXYZRGB PointT;

class InputHelper
{
private:
	cv::Size frameSize;
	float frameRate;
public:
	InputHelper(int width, int height, int frameRate);
	void getCameraInput();
	int visualizePCD(std::string filename);
private:
	void imageToPCD(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<PointT>::Ptr cloud, PXCPointF32 CameraC, PXCPointF32 CameraF);
	cv::Mat PXCImage2CVMAt(PXCImage* pxcImage, PXCImage::PixelFormat format);
};

