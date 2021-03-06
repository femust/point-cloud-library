#include "CloudHandler.h"


CloudHandler::CloudHandler(){}

CloudHandler::~CloudHandler(){}

void CloudHandler::GraspCloud(const std::string pathToCloud)
{
    pcl::PCDReader reader;
    _cloud = CloudHandler::PointCloud::Ptr (new CloudHandler::PointCloud);
    cloud_filtered = CloudHandler::PointCloud::Ptr (new CloudHandler::PointCloud);
    reader.read (pathToCloud, *_cloud);
    cloud_filtered=_cloud;

    std::cerr << "READING FROM FILE: " << pathToCloud << " " << _cloud->points.size () << std::endl <<std::endl;
}

void CloudHandler::GraspCloud(PointCloud::Ptr xyz)
    {
    _cloud = CloudHandler::PointCloud::Ptr (new CloudHandler::PointCloud);
    cloud_filtered = CloudHandler::PointCloud::Ptr (new CloudHandler::PointCloud);
     cloud_filtered=xyz;
    _cloud=xyz;
    }

void CloudHandler::FilterCloud()
{
    pcl::PassThrough<CloudHandler::PointT> pass;
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*_cloud,*cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_filtered->points.size() << " data points." << std::endl;

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
    std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points." << std::endl;
}

void CloudHandler::EstimatePointNormal()
{
    pcl::search::KdTree<CloudHandler::PointT>::Ptr tree (new pcl::search::KdTree<CloudHandler::PointT> ());
    pcl::NormalEstimation<CloudHandler::PointT, pcl::Normal> ne;
    cloud_normals = SurfaceNormals::Ptr (new SurfaceNormals);

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (25);
    ne.compute (*cloud_normals);

    //std::cout << CloudHandler::_horizontalLimit <<" " << CloudHandler::_verticalLimit;
}

//void CloudHandler::PlaneSegmentation()
//{


//    pcl::SACSegmentationFromNormals<CloudHandler::PointT, pcl::Normal> seg;
//    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//    pcl::ExtractIndices<CloudHandler::PointT> extract;
//    pcl::ExtractIndices<pcl::Normal> extract_normals;
//    pcl::PointCloud<CloudHandler::PointT>::Ptr cloud_f {cloud_filtered};
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_f {cloud_normals};



//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//    seg.setNormalDistanceWeight (0.1);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.03);

//    int i=0;
//    int nr_points = (int) cloud_filtered->points.size ();
////    std::cerr << std::endl;
////    std::cerr<< "MIN POINTS TO ITERATE " << 0.3 * nr_points<<std::endl;
////    std::cerr << std::endl;
//    while (cloud_f->points.size() > 0.2 * nr_points)
//    {
//    seg.setInputCloud (cloud_f);
//    seg.setInputNormals (cloud_normals_f);
//    seg.segment (*inliers_plane, *coefficients_plane);
//    if (inliers_plane->indices.size () == 0)
//        {
//          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//          break;
//        }
//    //std::cerr << " CLOUD F BEFORE CUTTING " <<  cloud_f->points.size() << std::endl;
//    extract.setInputCloud (cloud_f);
//    extract.setIndices (inliers_plane);
//    extract.setNegative (false);
//    pcl::PointCloud<CloudHandler::PointT>::Ptr cloud_plane (new pcl::PointCloud<CloudHandler::PointT> ());
//    extract.filter (*cloud_plane);
//    //std::cout << "CLOUD PLANE" << cloud_plane << std::endl;



//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid (*cloud_plane, centroid);

//     _planes.push_back(cloud_plane);
//     _centroid_planes.push_back(centroid);


////    planes.push_back(std::move(cloud_plane));
//   // std::cout << "PLANES " << planes.size() << std::endl;
////    std::cerr << std::endl;
// // std::cerr << i << ". PointCloud representing the planar component: " << cloud_plane->width * cloud_plane->height << " data points." << std::endl;
////    std::cerr << std::endl;
//    extract.setNegative (true);
//    extract.filter (*cloud_f);





//    extract_normals.setNegative (true);
//    extract_normals.setInputCloud (cloud_normals);
//    extract_normals.setIndices (inliers_plane);
//    extract_normals.filter (*cloud_normals_f);



////std::cerr << std::endl;
////    std::cerr << " CLOUD F AFTER CUTTING " << cloud_f->points.size() << std::endl;
////std::cerr<< std::endl;
//    i++;
//    }


//}


//void CloudHandler::CylinderSegmentation()
//{
//     pcl::SACSegmentationFromNormals<CloudHandler::PointT, pcl::Normal> seg;
//     pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
//     pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
//     cloud_cylinder = CloudHandler::PointCloud::Ptr (new CloudHandler::PointCloud);



//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_CYLINDER);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setNormalDistanceWeight (0.1);
//     seg.setMaxIterations (10000);
//     seg.setDistanceThreshold (0.05);
//     seg.setRadiusLimits (0, 0.1);
//     seg.setInputCloud (cloud_filtered);
//     seg.setInputNormals (cloud_normals);

//     pcl::ExtractIndices<CloudHandler::PointT> extract;

//     seg.segment (*inliers_cylinder, *coefficients_cylinder);
//     std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

//     extract.setInputCloud (cloud_filtered);
//     extract.setIndices (inliers_cylinder);
//     extract.setNegative (false);
//     extract.filter (*cloud_cylinder);


//     //pcl::PointCloud<CloudHandler::PointT> cloud_xyz;
//     // pcl::fromPCLPointCloud2 (cloud_cylinder, cloud_xyz);
//     //Eigen::Vector4f centroid;
//     //pcl::compute3DCentroid (*cloud_cylinder, centroid);

//    // std::cerr << " CENTER" << centroid << std::endl;
//     //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
//     //pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
//       // Add to renderer*
//     //p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);





//}


void CloudHandler::RegionGrowingMethod()
{
     pcl::search::Search<CloudHandler::PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<CloudHandler::PointT> > (new pcl::search::KdTree<CloudHandler::PointT>);
     pcl::RegionGrowing<CloudHandler::PointT, pcl::Normal> reg;
     pcl::PCA<CloudHandler::PointT> principal;//(&cloud_filtered);

     reg.setMinClusterSize (100); //was 1000
     reg.setMaxClusterSize (1000000);
     reg.setSearchMethod (tree);
     reg.setNumberOfNeighbours (50); //was50
     reg.setInputCloud (cloud_filtered);
     reg.setInputNormals (cloud_normals);
     reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
     reg.setCurvatureThreshold (1.0);

     //boost::shared_ptr<std::vector <pcl::PointIndices>> clusters {new <std::vector<pcl::PointIndices()>>};

//     std::vector<pcl::PointIndices> clusters;
//     std::vector<pcl::PointIndices::Ptr> eigen_inliers;
//     std::vector <Eigen::Matrix3f> EigenVectors;
//     std::vector <Eigen::Vector3f> EigenValues;

    int maxEigen;
    int minEigen;
    int midEigen;
    Eigen::Matrix3f checkEigenVector;
    Eigen::Vector3f checkEigenValue;
    float max=0,min=0;
    Eigen::Vector4f centroid;

    reg.extract (clusters);
    std::cout << "number of clusters found " << clusters.size() << std::endl;
  int numberRejectedClusters=0;
    for (auto it=clusters.begin(); it!=clusters.end() ; it++)
    {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices (*it));
      PointCloud projectedToEigenSpace;

      principal.setIndices(inliers);
      principal.setInputCloud(cloud_filtered);




      ///END OF TEST

//      Eigen::Vector3f A, B;
//      A<< Eigen::Vector3f::UnitZ();
//      B<<1,1,1.41;

//      std::cout << " NORM " << A.norm()<< " HEHE "  << B.norm() <<std::endl;

//      std::cout << " A " << A << " B " << B << std::endl;
//       Eigen::Matrix3f R;
//       R = Eigen::Quaternionf().setFromTwoVectors(A,B);
//std::cout << "MATRX R" << R << std::endl;



      checkEigenVector=principal.getEigenVectors();
      checkEigenValue=principal.getEigenValues();

//      max = checkEigenValue.maxCoeff(&maxEigen);
//      min = checkEigenValue.minCoeff(&minEigen);

//      for(pcl::PointCloud<pcl::Normal>::iterator it = cloud_normals->begin(); it!= cloud_normals->end(); it++)
//      {
//      std::cout << "EIGENVALUE"<< checkEigenValue << std::endl;
//      std::cout << "VECTOR" << checkEigenVector << std::endl <<std::endl;
//          std::cout << "EIGENVECTOR 0: " << checkEigenVector.col(0)<<std::endl;
//          std::cout << "EIGENVECTOR 1: " << checkEigenVector.col(1)<<std::endl;
//          std::cout << "EIGENVECTOR 2: " << checkEigenVector.col(2)<<std::endl;
//          std::cout << "EIGENVALUES 0: " << checkEigenValue.row(0)<<std::endl;
//          std::cout << "EIGENVALUES 1: " << checkEigenValue.row(1)<<std::endl;
//          std::cout << "EIGENVALUES 2: " << checkEigenValue.row(2)<<std::endl;

          float projectionOnZaxis=checkEigenVector.col(0)(2)+checkEigenVector.col(1)(2);


               if ((fabs(projectionOnZaxis) <= CloudHandler::_horizontalLimit) || (fabs(projectionOnZaxis)>=CloudHandler::_verticalLimit))
               {

                   //TEST
                   pcl::ExtractIndices<CloudHandler::PointT> extract;
                   pcl::PointCloud<CloudHandler::PointT>::Ptr cloud_f (new pcl::PointCloud<CloudHandler::PointT>);
                   extract.setInputCloud (cloud_filtered);
                   extract.setIndices (inliers);
                   extract.setNegative (false);
                   extract.filter (*cloud_f);

                   principal.project(*cloud_f,projectedToEigenSpace);

                   CloudHandler::PointT minPt, maxPt;

                   float depth;
                   float width;


                   pcl::getMinMax3D (projectedToEigenSpace, minPt, maxPt);

//                   std::cout << " MIN " << minPt << std::endl;
//                   std::cout << " MAX " << maxPt << std::endl;

                   width=maxPt.x+fabs(minPt.x);
                   depth=maxPt.y+fabs(minPt.y);


//                   std::cout << "DEPTH OF THE STEP " << depth << std::endl;
//                   std::cout << "WIDTH OF THE STEP " <<width << std::endl;




                 depths.push_back(depth);
                 widths.push_back(width);
                 EigenValues.push_back(checkEigenValue);
                 EigenVectors.push_back(checkEigenVector);
                 eigen_inliers.push_back(inliers);

                 pcl::compute3DCentroid (*cloud_filtered,*it, centroid);
                 _centroid_planes.push_back(centroid);
               }
               else
               {
                   numberRejectedClusters++;
//                   std::cout << std::endl;
//                   std::cout << "POINT CLOUD REJECTED"<<std::endl;
//                   std::cout << std::endl;
               }
    }
    std::cout << "number of clusters rejected: " << numberRejectedClusters << std::endl;
    std::cout << "number of clusters found with vertical or horizontal orientation" << _centroid_planes.size()<< std::endl;

    _colored_cloud =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud <pcl::PointXYZRGB>);

    _colored_cloud = reg.getColoredCloud ();
}

void CloudHandler::StairInformation()
{
    for (std::size_t index=0; index < EigenVectors.size() ;index++)
    {

std::cout << "\n\n" << index << " PLANE"<<std::endl;
std::cout << "CENTER " << std::endl;
std::cout << _centroid_planes[index] << "\nDEPTH: \n" << depths[index] << "\n WIDTH \n" << widths[index] <<std::endl;
std::cout << "EIGENVECTOR \n" << EigenVectors[index] << std::endl;
    }





}

void CloudHandler::StairsAndPapesDetection()
{
    switch(mode_)
    {
        case GRAPH:
            FilterCloud();
            EstimatePointNormal();
            RegionGrowingMethod();
            break;

//        case SEPARATE:
//            FilterCloud();
//            EstimatePointNormal();
//            PlaneSegmentation();

        break;
    }
}



CloudHandler::PointCloud::Ptr CloudHandler::GiveCloudPointer() const
 {
     return _cloud;
 }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudHandler::GiveColoredCloud() const
 {
     return _colored_cloud;
 }

 std::vector<pcl::PointCloud<CloudHandler::PointT>::Ptr> CloudHandler::GivePlanes() const
 {
     return _planes;
 }

CloudHandler::SurfaceNormals::Ptr CloudHandler::GiveNormals() const
 {
     return cloud_normals;
 }

std::vector<Eigen::Vector4f> CloudHandler::GiveCentroidPlanes() const
 {
     return _centroid_planes;
 }

void CloudHandler::PrintData()
{
    for (size_t i = 0; i < _cloud->size() ; ++i)
        std::cout << "    " << _cloud->points[i].x
                  << " "    << _cloud->points[i].y
                  << " "    << _cloud->points[i].z << std::endl;
                  //<< " "    << _cloud->points[i].rgba<<std::endl;
}
