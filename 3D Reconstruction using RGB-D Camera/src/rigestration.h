#pragma once
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;



class rigestration
{
private:
	float leaf_size = 0.005;
	float ne_radius = 0.02;
	float fpfh_radius = 0.05;
	float max_distance = 0.005;
	float max_iterations = 250;
	float trans_epsilon = 1e-10;
	float ef_epsilon = 1e-10;

	float nr_k = 20;
	float stddev_mult = 1;
public:
	int rigestration::runRigestration(std::string folderName, std::string fileName, int num_of_pcdFiles);
private:
	void pre_process(pcl::PointCloud< PointNT >::Ptr, pcl::PointCloud< PointNT >::Ptr, pcl::PointCloud< pcl::FPFHSignature33 >::Ptr);
	void sac_ia(pcl::PointCloud< PointNT >::Ptr, pcl::PointCloud< PointNT >::Ptr,pcl::PointCloud< PointNT >::Ptr, Eigen::Matrix4f&);
	void iterative_closest_point(pcl::PointCloud< PointNT >::Ptr, pcl::PointCloud< PointNT >::Ptr,Eigen::Matrix4f&, Eigen::Matrix4f&);
};

