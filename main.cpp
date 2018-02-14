#include <pcl/visualization/cloud_viewer.h>

#include "CloudHandler.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>


int main(int argc, char* argv[])
{
    if (argc<2)
    {
        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
        return 0;
    }



CloudHandler stairCloud;
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr checking (new pcl::PointCloud<pcl::PointXYZRGBA> ());


stairCloud.GraspCloud(std::string(argv[1]));
checking=stairCloud.StairsAndPapesDetection();


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//int v1 = 0;
//viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//viewer->setBackgroundColor (0.5, 0, 0, v1);
//pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr main_cloud (stairCloud.GiveCloudPointer());
//viewer->addPointCloud<pcl::PointXYZRGBA> (main_cloud,"view", v1);
//viewer->addCoordinateSystem (0.5,v1);

//int v2=0;
//viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//viewer->setBackgroundColor (1, 1, 1, v2);
//pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr main_cloud2 (checking);
//viewer->addPointCloud<pcl::PointXYZRGBA> (main_cloud2,"view2", v2);
//viewer->addCoordinateSystem (0.5,v2);


viewer->setBackgroundColor(1,1,1);
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_color (checking);
viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_color,"view", 0);

////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorRGB (stairCloud.GiveCloudPointer(), 250, 0 , 0);
////





while (!viewer->wasStopped ())
{
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
return 0;

}

