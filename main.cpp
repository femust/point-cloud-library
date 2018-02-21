#include <string>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include<vector>

#include "CloudHandler.h"

typedef pcl::PointXYZRGBA PointT;



int main(int argc, char* argv[])
{
    if (argc<2)
    {
        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
        return 0;
    }

pcl::visualization::CloudViewer viewer ("Cluster viewer");



CloudHandler stairCloud;
pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;



stairCloud.GraspCloud(std::string(argv[1]));
colored_cloud=stairCloud.RegionGrowingMethod();

viewer.showCloud(colored_cloud);
while (!viewer.wasStopped ())
{
}



//int v1 = 0;
//viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
//pcl::PointCloud<PointT>::ConstPtr main_cloud (stairCloud.GiveCloudPointer());
//viewer->addPointCloud<PointT> (main_cloud,"view", v1);
//viewer->addCoordinateSystem (0.5);


//std::vector<pcl::PointCloud<PointT>::Ptr> SegmentedPlanes;


//stairCloud.StairsAndPapesDetection();
//SegmentedPlanes=stairCloud.GivePlanes();
//std::vector<Eigen::Vector4f> centers =stairCloud.GiveCentroidPlanes();



//int v2=0;
//viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//viewer->setBackgroundColor (1, 1, 1, v2);
//viewer->addCoordinateSystem (0.5);

//for (auto it=SegmentedPlanes.begin(); it != SegmentedPlanes.end();++it)
//{
//    float index;
//    index = SegmentedPlanes.begin() - it;
//    std::cout << index;
//   // pcl::PointCloud<PointT>::ConstPtr plane_segments (*it);
//   // viewer->addPointCloud<PointT> (plane_segments,std::to_string(index), v2);

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_cylinder (*it, 10*index, 250*index , 20*index);
//    viewer->addPointCloud<PointT>(*it,color_cylinder,std::to_string(index));
//}
////std::cout << "PRINTING IN MAIN" << centers[0]<< std::endl;

////std:: cout << "SRODECZEK " <<centers[0][0] << std::endl;

//for (auto it=centers.begin(); it != centers.end();++it)
//{
//    float index;
//    index = centers.begin() - it;
//    std::cout << std::endl;
//    std::cout << " HEJH EJ" << std::endl;
//    std::cout << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;

//    PointT o;
//    o.x = (*it)[0];
//    o.y = (*it)[1];
//    o.z = (*it)[2];
//    std::cout << o.x << " " << o.y << " " << o.z << std::endl;
////    pcl::PointCloud<PointT>::ConstPtr plane_segments (*it);
////   viewer->addPointCloud<PointT> (plane_segments,std::to_string(index), v2);

//    //viewer->addPointCloud<PointT>(*it,color_cylinder,std::to_string(index));
//    viewer->addSphere (o, 0.02, 250, 0, 0,std::to_string(index-100),v2);
//}







//while (!viewer->wasStopped ())
//{
//    viewer->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//}
return 0;

}

