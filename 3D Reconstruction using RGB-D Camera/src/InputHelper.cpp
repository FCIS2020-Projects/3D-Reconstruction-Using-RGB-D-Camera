#include "InputHelper.h"

InputHelper::InputHelper(int width, int height, int frameRate)
{
	//Define some parameter for the camera
	this->frameSize = cv::Size(width, height);
	this->frameRate = frameRate;
}
void InputHelper::getCameraInput()
{
	// Create the Open CV windows and images
	cv::namedWindow("IR", cv::WINDOW_NORMAL);
	cv::namedWindow("Color", cv::WINDOW_NORMAL);
	cv::namedWindow("Depth", cv::WINDOW_NORMAL);

	cv::Mat frameIR = cv::Mat::zeros(frameSize, CV_8UC1);
	cv::Mat frameColor = cv::Mat::zeros(frameSize, CV_8UC3);
	cv::Mat frameDepth = cv::Mat::zeros(frameSize, CV_32FC1);
	cv::Mat frameDepthToShow = cv::Mat::zeros(frameSize, CV_32FC1);

	// Initialize the realsense manager
	PXCSenseManager* pxcSenseManager = PXCSenseManager::CreateInstance();


	// Enable the streams to be used
	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_IR, frameSize.width, frameSize.height, frameRate);
	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, frameSize.width, frameSize.height, frameRate);
	pxcSenseManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, frameSize.width, frameSize.height, frameRate);

	// Initialize the pipeline
	pxcSenseManager->Init();

	PXCCaptureManager* pxcCaptureManager = pxcSenseManager->QueryCaptureManager();
	PXCCapture::Device* device = pxcCaptureManager->QueryDevice();
	PXCPointF32 CameraC = device->QueryColorPrincipalPoint();
	PXCPointF32 CameraF = device->QueryColorFocalLength();

	bool keepRunning = true;
	int i = 1;
	while (keepRunning) {
		// Acquire all the frames from the camera
		if (pxcSenseManager->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;

		//pxcSenseManager->AcquireFrame();
		PXCCapture::Sample* sample = pxcSenseManager->QuerySample();

		// Convert each frame to OpenCV image
		frameIR = PXCImage2CVMAt(sample->ir, PXCImage::PIXEL_FORMAT_Y8);
		frameColor = PXCImage2CVMAt(sample->color, PXCImage::PIXEL_FORMAT_RGB24);
		frameDepth = PXCImage2CVMAt(sample->depth, PXCImage::PIXEL_FORMAT_DEPTH_F32);
		frameDepth.convertTo(frameDepthToShow, CV_8UC1);

		// Display the image
		imshow("IR", frameIR);
		imshow("Color", frameColor);
		imshow("Depth", frameDepthToShow);

		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		imageToPCD(frameColor, frameDepth, cloud, CameraC, CameraF);
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addPointCloud(cloud, "cloud");
		viewer.spin();
		// Check for user input
		int key = cv::waitKey(1);
		if (key == 27)
			keepRunning = false;
		if (key == 32)
		{
			std::stringstream ss;
			ss << std::setw(2) << std::setfill('0') << i;
			pcl::io::savePCDFileASCII("arsany" + ss.str() + ".pcd", *cloud);
			i++;
		}

		// Release the memory from the frames
		pxcSenseManager->ReleaseFrame();

	}

	pxcSenseManager->Release();
}
int InputHelper::visualizePCD(std::string filename)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
	        << cloud->width * cloud->height
	        << " data points from test_pcd.pcd with the following fields: "
	        << std::endl;
	/*for (std::size_t i = 0; i < cloud->points.size (); ++i)
	std::cout << "    " << cloud->points[i].x
	            << " "    << cloud->points[i].y
	            << " "    << cloud->points[i].z << std::endl;*/
	/*pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped());*/
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.setBackgroundColor(1, 1, 1, 0);
	viewer.addPointCloud(cloud, "cloud");
	viewer.spin();
	return (0);
}

void InputHelper::imageToPCD(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<PointT>::Ptr cloud, PXCPointF32 CameraC, PXCPointF32 CameraF)
{
	const double CAMERA_FACTOR = 100;
	const double CAMERA_CX = CameraC.x;
	const double CAMERA_CY = CameraC.y;
	const double CAMERA_FX = CameraF.x;
	const double CAMERA_FY = CameraF.y;
	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			float d = depth.ptr<float>(m)[n];
			if (d == 0)
				continue;
			PointT p;
			p.z = double(d) / CAMERA_FACTOR;
			p.x = (n - CAMERA_CX) * p.z / CAMERA_FX;
			p.y = (m - CAMERA_CY) * p.z / CAMERA_FY;

			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			cloud->points.push_back(p);
		}
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
}

cv::Mat InputHelper::PXCImage2CVMAt(PXCImage* pxcImage, PXCImage::PixelFormat format) {

	PXCImage::ImageData data;
	pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);

	int width = pxcImage->QueryInfo().width;
	int height = pxcImage->QueryInfo().height;

	if (!format)
		format = pxcImage->QueryInfo().format;

	int type;
	if (format == PXCImage::PIXEL_FORMAT_Y8)
		type = CV_8UC1;
	else if (format == PXCImage::PIXEL_FORMAT_RGB24)
		type = CV_8UC3;
	else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32)
		type = CV_32FC1;

	cv::Mat ocvImage = cv::Mat(cv::Size(width, height), type, data.planes[0]);

	pxcImage->ReleaseAccess(&data);

	return ocvImage;
}