#include "CloudHandler.h"


CloudHandler::CloudHandler():_cloud{new pcl::PointCloud<pcl::PointXYZRGBA>}
{}

CloudHandler::~CloudHandler(){}

void CloudHandler::GraspCloud(const std::string pathToCloud)
{
    pcl::io::loadPCDFile (pathToCloud, *_cloud);
    std::cerr << "READING FROM FILE: " << pathToCloud << " " << _cloud->points.size () << std::endl ;
}

void CloudHandler::FilterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud (_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 5);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
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

void CloudHandler::PlaneSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,
                                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane)
{
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f {new pcl::PointCloud<pcl::Normal>};



    cloud_f = cloud_filtered;
    cloud_normals_f=cloud_normals;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);

    int i=0;
    int nr_points = (int) cloud_filtered->points.size ();
std::cerr<< "MIN POINTS TO ITERATE " << 0.3 * nr_points<<std::endl;
//    while (cloud_f->points.size() > 0.3 * nr_points)
//    {
    seg.setInputCloud (cloud_f);
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers_plane, *coefficients_plane);

//    if (inliers_plane->indices.size () == 0)
//        {
//          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//          break;
//        }
    std::cerr << " CLOUD F BEFORE CUTTING " <<  cloud_f->points.size() << std::endl;
    extract.setInputCloud (cloud_f);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    std::cerr << i << ". PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
    extract.setNegative (true);
    extract.filter (*cloud_f);

     extract.setInputCloud(cloud_normals);
     extract.filter(*cloud_normals_f);
     std::cerr << "CLOUD F AFTER CUTTING " << cloud_f->points.size() << "NORMALS "<< cloud_normals_f->points.size() << std::endl;

//    i++;
//    }
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
     extract.setNegative (true);

     extract.filter (*cloud_cylinder);
}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  CloudHandler::StairsAndPapesDetection()
{
 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals {new pcl::PointCloud<pcl::Normal>};
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered {new pcl::PointCloud<pcl::PointXYZRGBA>};
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGBA> ());
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ());

 FilterCloud(cloud_filtered);
 EstimatePointNormal(cloud_filtered,cloud_normals);
 PlaneSegmentation(cloud_filtered,cloud_normals,cloud_plane);
 //CylinderSegmentation(cloud_filtered,cloud_normals,cloud_cylinder);




 return cloud_plane;










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
