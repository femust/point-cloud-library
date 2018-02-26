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


    //    using namespace std;
    //    vector<int>numbers = {4,5,3,2,5,42};



    //    for (vector<int>::iterator it = numbers.begin(); it!=numbers.end(); it++){
    //        cout << *it << endl;
    //        cout << &it << endl;
    //        cout << &(*it->indices) << endl;
    //    }

//    if (argc<2)
//    {
//        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
//        return 0;
//    }

    pcl::PointCloud<PointT>::Ptr cloud {new pcl::PointCloud<PointT>};
    cloud->width    = 10;
    cloud->height   = 10;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

  std::cout << "SIZE"  << cloud->size() << " HEIGHT " << cloud->height << " WIDTH " << cloud->width <<std::endl;

    for (size_t i = 0; i < cloud->size() ; ++i)
     {

       cloud->points[i].x = 0;
       cloud->points[i].y = (i%10)+1;

       cloud->points[i].z = int(1+i/10);

    }



    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CloudHandler stairCloud;
    float index;
    PointT o;
    std::vector<Eigen::Vector4f> centers;

   // stairCloud.GraspCloud(std::string(argv[1]));
    stairCloud.GraspCloud(cloud);
    stairCloud.PrintData();

   stairCloud.StairsAndPapesDetection();

    int v1 = 0;

    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
    viewer->addCoordinateSystem(1,0,0,0,"global",v1);
    viewer->addPointCloud<PointT> (stairCloud.GiveCloudPointer(),"view", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "view");


    int v2=0;
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (1, 1, 1, v2);
    viewer->addCoordinateSystem (1);
    viewer->addPointCloud(stairCloud.GiveColoredCloud(),"sample",v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample");
////    viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(stairCloud.GiveColoredCloud(),stairCloud.GiveNormals(),100,0.2,"normal",v2);




centers =stairCloud.GiveCentroidPlanes();

for (auto it=centers.begin(); it != centers.end();++it)
{
    index = centers.begin() - it;
    o.x = (*it)[0];
    o.y = (*it)[1];
    o.z = (*it)[2];
    viewer->addSphere (o, 0.2, 250, 0, 0,std::to_string(index),v2);
    std::cout << "CENTER "<< index<< " " << o.x << " " << o.y << " " << o.z <<std::endl;
}

while (!viewer->wasStopped ())
{
    viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}

return 0;

}

