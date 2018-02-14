#include "CloudHandler.h"


CloudHandler::CloudHandler():_cloud{new pcl::PointCloud<pcl::PointXYZRGBA>}
{}

CloudHandler::~CloudHandler(){}

void CloudHandler::GraspCloud(const std::string pathToCloud)
{
    std::cout << "READING FROM FILE: " << pathToCloud;
    pcl::io::loadPCDFile (pathToCloud, *_cloud);
}

void CloudHandler::FilterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter(*cloud_filtered);
    //std::cerr << "PointCloud after filtering has: " << _cloud_filtered->points.size () << " data points." << std::endl;
}

void CloudHandler::EstimatePointNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals )
{
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
}

void CloudHandler::CylinderSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,
                                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder)
{
     pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_CYLINDER);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setNormalDistanceWeight (0.1);
     seg.setMaxIterations (10000);
     seg.setDistanceThreshold (0.05);
     seg.setRadiusLimits (0, 0.1);
     seg.setInputCloud (cloud_filtered);
     seg.setInputNormals (cloud_normals);

     pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
     pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

     seg.segment (*inliers_cylinder, *coefficients_cylinder);
     std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers_cylinder);
     extract.setNegative (false);

     extract.filter (*cloud_cylinder);
}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  CloudHandler::StairsAndPapesDetection()
{
 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals {new pcl::PointCloud<pcl::Normal>};
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered {new pcl::PointCloud<pcl::PointXYZRGBA>};
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGBA> ());
 FilterCloud(cloud_filtered);
 EstimatePointNormal(cloud_filtered,cloud_normals);
 CylinderSegmentation(cloud_filtered,cloud_normals,cloud_cylinder);




 return cloud_cylinder;










}




pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudHandler::GiveCloudPointer() const
 {
     return _cloud;
 }

void CloudHandler::PrintData()
{
    for (size_t i = 0; i < 5 ; ++i)
        std::cout << "    " << _cloud->points[i].x
                  << " "    << _cloud->points[i].y
                  << " "    << _cloud->points[i].z
                  << " "    << _cloud->points[i].rgba<<std::endl;
}
