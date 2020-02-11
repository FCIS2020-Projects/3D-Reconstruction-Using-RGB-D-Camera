#include "rigestration.h"

void rigestration::pre_process(pcl::PointCloud< PointNT >::Ptr input, pcl::PointCloud< PointNT >::Ptr output, pcl::PointCloud< pcl::FPFHSignature33 >::Ptr fpfhs)
{

    std::vector<int> indieces;
    pcl::removeNaNFromPointCloud(*input, *input, indieces);
    //Downsampling
    pcl::VoxelGrid<PointNT> grid;
    grid.setInputCloud(input);
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.filter(*output);

    pcl::console::print_highlight("Estimating normals...\n");
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointNT>::Ptr tree_normals(new pcl::search::KdTree<PointNT>);
    tree_normals->setInputCloud(output);
    pcl::NormalEstimation<PointNT, pcl::Normal> ne;
    ne.setInputCloud(output);
    ne.setRadiusSearch(ne_radius);
    ne.setSearchMethod(tree_normals);
    ne.compute(*cloud_normals);

    pcl::console::print_highlight("Estimating features...\n");
    pcl::search::KdTree<PointNT>::Ptr tree_fpfh(new pcl::search::KdTree<PointNT>);
    tree_fpfh->setInputCloud(output);
    pcl::FPFHEstimationOMP<PointNT, pcl::Normal, pcl::FPFHSignature33> fpfh;

    fpfh.setNumberOfThreads(4);
    fpfh.setInputCloud(output);
    fpfh.setInputNormals(cloud_normals);
    fpfh.setSearchMethod(tree_fpfh);
    fpfh.setRadiusSearch(fpfh_radius);
    fpfh.compute(*fpfhs);
}

void rigestration::sac_ia(pcl::PointCloud< PointNT >::Ptr source, pcl::PointCloud< PointNT >::Ptr target,
    pcl::PointCloud< PointNT >::Ptr source_out, Eigen::Matrix4f& sac_trans)
{
    pcl::PointCloud< PointNT >::Ptr target_o(new pcl::PointCloud< PointNT >);
    pcl::PointCloud< pcl::FPFHSignature33 >::Ptr source_fpfhs(new pcl::PointCloud< pcl::FPFHSignature33 >);
    pcl::PointCloud< pcl::FPFHSignature33 >::Ptr target_fpfhs(new pcl::PointCloud< pcl::FPFHSignature33 >);

    pre_process(source, source_out, source_fpfhs);
    pre_process(target, target_o, target_fpfhs);

    pcl::console::print_highlight("Starting alignment ((RANSAC_IA))...\n");
    pcl::PointCloud<PointNT>::Ptr sac_align(new pcl::PointCloud<PointNT>);
    pcl::SampleConsensusInitialAlignment<PointNT, PointNT, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_out);
    sac_ia.setSourceFeatures(source_fpfhs);
    sac_ia.setInputTarget(target_o);
    sac_ia.setTargetFeatures(target_fpfhs);
    sac_ia.align(*sac_align);
    sac_trans = sac_ia.getFinalTransformation();
}

void rigestration::iterative_closest_point(pcl::PointCloud< PointNT >::Ptr source, pcl::PointCloud< PointNT >::Ptr target,
    Eigen::Matrix4f& sac_trans, Eigen::Matrix4f& icp_trans)
{
    pcl::console::print_highlight("Starting ICP...\n");
    pcl::PointCloud<PointNT>::Ptr icp_cloud(new pcl::PointCloud<PointNT>);
    pcl::IterativeClosestPoint<PointNT, PointNT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(max_distance);
    icp.setMaximumIterations(max_iterations);
    icp.setTransformationEpsilon(trans_epsilon);
    icp.setEuclideanFitnessEpsilon(ef_epsilon);
    icp.align(*icp_cloud, sac_trans);

    icp_trans = icp.getFinalTransformation();
}


int
rigestration::runRigestration(std::string folderName, std::string fileName,int num_of_pcdFiles)
{

    pcl::PointCloud<PointNT>::Ptr icp_cloud(new pcl::PointCloud<PointNT>);
    pcl::PointCloud<PointNT>::Ptr source(new pcl::PointCloud<PointNT>);
    pcl::PointCloud<PointNT>::Ptr target(new pcl::PointCloud<PointNT>);
    pcl::PointCloud<PointNT>::Ptr global_alian_cloud(new pcl::PointCloud<PointNT>);

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f sac_trans, icp_trans;

    //Main Registration loop
    for (int i = 2; i <= num_of_pcdFiles; i++) {
        std::string filenamesource = folderName + "/" + fileName + std::to_string(i) + ".pcd";
        std::string filenametarget = folderName + "/" + fileName + std::to_string(i - 1) + ".pcd";

        if (pcl::io::loadPCDFile<PointNT>(filenamesource, *source) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file source .pcd \n");
            return (-1);
        }

        if (pcl::io::loadPCDFile<PointNT>(filenametarget, *target) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file target .pcd \n");
            return (-1);
        }

        std::cout << "File ->" << i << std::endl;

        pcl::PointCloud<PointNT>::Ptr source_process(new pcl::PointCloud<PointNT>);
        sac_ia(source, target, source_process, sac_trans);

        std::cout << sac_trans << std::endl;

        iterative_closest_point(source_process, target, sac_trans, icp_trans);
        std::cout << "------icp transformation: " << std::endl;
        std::cout << icp_trans << std::endl;


        pcl::transformPointCloud(*source, *icp_cloud, icp_trans);

        pcl::transformPointCloud(*icp_cloud, *icp_cloud, GlobalTransform);
        *global_alian_cloud += *icp_cloud;

        GlobalTransform = GlobalTransform * icp_trans;

    }

    pcl::PointCloud<PointNT>::Ptr Downsaple_global(new pcl::PointCloud<PointNT>);
    std::vector<int> indieces;
    pcl::removeNaNFromPointCloud(*global_alian_cloud, *global_alian_cloud, indieces);

    //Downsampling
    pcl::VoxelGrid<PointNT> grid;
    grid.setInputCloud(global_alian_cloud);
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.filter(*Downsaple_global);

    //Statistical Outliar Removal
    pcl::PointCloud<PointNT>::Ptr cloud_sor(new pcl::PointCloud<PointNT>);
    pcl::StatisticalOutlierRemoval<PointNT> sor;
    sor.setInputCloud(Downsaple_global);
    sor.setMeanK(nr_k);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(*Downsaple_global);

    //Visualization
    pcl::visualization::PCLVisualizer viewer("Alignment");
    viewer.addPointCloud(global_alian_cloud, ColorHandlerT(global_alian_cloud, 0.0, 255.0, 0.0), "global_alian_cloud");
    viewer.spin();
  


    return 0; 
}