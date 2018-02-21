#include "CloudHandler.h"


CloudHandler::CloudHandler():_cloud{new pcl::PointCloud<CloudHandler::PointT>},
                                    cloud_normals {new pcl::PointCloud<pcl::Normal>},
                                    cloud_filtered {new pcl::PointCloud<CloudHandler::PointT>},
                                    cloud_cylinder {new pcl::PointCloud<CloudHandler::PointT>},
                                  _planes {},
                                 _centroid_planes {}
{}

CloudHandler::~CloudHandler(){}

void CloudHandler::GraspCloud(const std::string pathToCloud)
{
   // pcl::io::loadPCDFile (pathToCloud, *_cloud);

    pcl::PCDReader reader;
    reader.read (pathToCloud, *_cloud);
    cloud_filtered=_cloud;
    std::cerr << "READING FROM FILE: " << pathToCloud << " " << _cloud->points.size () << std::endl <<std::endl;
}

void CloudHandler::FilterCloud()
{
    pcl::PassThrough<CloudHandler::PointT> pass;
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*_cloud,*cloud_blob);
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
    //pass.setInputCloud (_cloud);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits (0, 1.5);
    //pass.filter(*cloud_filtered);
    //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);
     pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

}

void CloudHandler::EstimatePointNormal()
{
    pcl::search::KdTree<CloudHandler::PointT>::Ptr tree (new pcl::search::KdTree<CloudHandler::PointT> ());
    pcl::NormalEstimation<CloudHandler::PointT, pcl::Normal> ne;

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
}

void CloudHandler::PlaneSegmentation()
{


    pcl::SACSegmentationFromNormals<CloudHandler::PointT, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::ExtractIndices<CloudHandler::PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<CloudHandler::PointT>::Ptr cloud_f {cloud_filtered};
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f {cloud_normals};



    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);

    int i=0;
    int nr_points = (int) cloud_filtered->points.size ();
//    std::cerr << std::endl;
//    std::cerr<< "MIN POINTS TO ITERATE " << 0.3 * nr_points<<std::endl;
//    std::cerr << std::endl;
    while (cloud_f->points.size() > 0.2 * nr_points)
    {
    seg.setInputCloud (cloud_f);
    seg.setInputNormals (cloud_normals_f);
    seg.segment (*inliers_plane, *coefficients_plane);
    if (inliers_plane->indices.size () == 0)
        {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }
    //std::cerr << " CLOUD F BEFORE CUTTING " <<  cloud_f->points.size() << std::endl;
    extract.setInputCloud (cloud_f);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    pcl::PointCloud<CloudHandler::PointT>::Ptr cloud_plane (new pcl::PointCloud<CloudHandler::PointT> ());
    extract.filter (*cloud_plane);
    //std::cout << "CLOUD PLANE" << cloud_plane << std::endl;



    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud_plane, centroid);

     _planes.push_back(cloud_plane);
     _centroid_planes.push_back(centroid);


//    planes.push_back(std::move(cloud_plane));
   // std::cout << "PLANES " << planes.size() << std::endl;
//    std::cerr << std::endl;
 // std::cerr << i << ". PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
//    std::cerr << std::endl;
    extract.setNegative (true);
    extract.filter (*cloud_f);





    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals_f);



//std::cerr << std::endl;
//    std::cerr << " CLOUD F AFTER CUTTING " << cloud_f->points.size() << std::endl;
//std::cerr<< std::endl;
    i++;
    }


}


void CloudHandler::CylinderSegmentation()
{
     pcl::SACSegmentationFromNormals<CloudHandler::PointT, pcl::Normal> seg;
     pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_CYLINDER);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setNormalDistanceWeight (0.1);
     seg.setMaxIterations (10000);
     seg.setDistanceThreshold (0.05);
     seg.setRadiusLimits (0, 0.1);
     seg.setInputCloud (cloud_filtered);
     seg.setInputNormals (cloud_normals);

     pcl::ExtractIndices<CloudHandler::PointT> extract;

     seg.segment (*inliers_cylinder, *coefficients_cylinder);
     std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers_cylinder);
     extract.setNegative (false);
     extract.filter (*cloud_cylinder);


     //pcl::PointCloud<CloudHandler::PointT> cloud_xyz;
     // pcl::fromPCLPointCloud2 (cloud_cylinder, cloud_xyz);
     //Eigen::Vector4f centroid;
     //pcl::compute3DCentroid (*cloud_cylinder, centroid);

    // std::cerr << " CENTER" << centroid << std::endl;
     //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
     //pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
       // Add to renderer*
     //p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);





}

//void CloudHandler::RegionGrowingMethod()
//{





//}


pcl::PointCloud <pcl::PointXYZRGB>::Ptr CloudHandler::RegionGrowingMethod()
{



     pcl::search::Search<CloudHandler::PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<CloudHandler::PointT> > (new pcl::search::KdTree<CloudHandler::PointT>);
     EstimatePointNormal();
     pcl::RegionGrowing<CloudHandler::PointT, pcl::Normal> reg;
     reg.setMinClusterSize (50);
     reg.setMaxClusterSize (1000000);
     reg.setSearchMethod (tree);
     reg.setNumberOfNeighbours (30);
     reg.setInputCloud (cloud_filtered);
     //reg.setIndices (indices);
     reg.setInputNormals (cloud_normals);
     reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
     reg.setCurvatureThreshold (1.0);

     std::vector <pcl::PointIndices> clusters;
     reg.extract (clusters);

     std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
     std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
return colored_cloud;


}



void CloudHandler::StairsAndPapesDetection()
{

 //pcl::PointCloud<CloudHandler::PointT>::Ptr cloud_plane (new pcl::PointCloud<CloudHandler::PointT> ());
 FilterCloud();
 EstimatePointNormal();
 PlaneSegmentation();

 std::cout << "NUMBER OF PLANES" << _planes.size() << std::endl;
 std::cout << "NUMBER OF POINTS" << _centroid_planes.size() << std::endl;
 //CylinderSegmentation(cloud_filtered,cloud_normals,cloud_cylinder);
 //std::cout << "DRUGI " << planes.size() << std::endl;

}




pcl::PointCloud<CloudHandler::PointT>::Ptr CloudHandler::GiveCloudPointer() const
 {
     return _cloud;
 }

 std::vector<pcl::PointCloud<CloudHandler::PointT>::Ptr> CloudHandler::GivePlanes() const
 {
     return _planes;
 }

 std::vector<Eigen::Vector4f> CloudHandler::GiveCentroidPlanes() const
 {
     return _centroid_planes;
 }







void CloudHandler::PrintData()
{
    for (size_t i = 0; i < 5 ; ++i)
        std::cout << "    " << _cloud->points[i].x
                  << " "    << _cloud->points[i].y
                  << " "    << _cloud->points[i].z
                  << " "    << _cloud->points[i].rgba<<std::endl;
}
