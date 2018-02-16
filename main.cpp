#include <string>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include<vector>

#include "CloudHandler.h"

int main(int argc, char* argv[])
{
    if (argc<2)
    {
        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
        return 0;
    }

CloudHandler stairCloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

stairCloud.GraspCloud(std::string(argv[1]));


int v1 = 0;
viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr main_cloud (stairCloud.GiveCloudPointer());
viewer->addPointCloud<pcl::PointXYZRGBA> (main_cloud,"view", v1);
viewer->addCoordinateSystem (0.5);


std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> SegmentedPlanes;
SegmentedPlanes=stairCloud.StairsAndPapesDetection();



int v2=0;
viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
viewer->setBackgroundColor (1, 1, 1, v2);
viewer->addCoordinateSystem (0.5);

for (std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>::iterator it=SegmentedPlanes.begin(); it != SegmentedPlanes.end();++it)
{
    float index;
    index = SegmentedPlanes.begin() - it;
    std::cout << index;
   // pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr plane_segments (*it);
   // viewer->addPointCloud<pcl::PointXYZRGBA> (plane_segments,std::to_string(index), v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_cylinder (*it, 250, 0 , 0);
    viewer->addPointCloud<pcl::PointXYZRGBA>(*it,color_cylinder,std::to_string(index));
}



while (!viewer->wasStopped ())
{
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
return 0;

}

