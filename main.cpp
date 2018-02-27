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




//    if (argc<2)
//    {
//        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
//        return 0;
//    }

    pcl::PointCloud<PointT>::Ptr cloud_a {new pcl::PointCloud<PointT>};
    cloud_a->width    = 1000;
    cloud_a->height   = 1;
    cloud_a->is_dense = false;
    cloud_a->points.resize (cloud_a->width * cloud_a->height);

  std::cout << "cloud_a"  << cloud_a->size() << " HEIGHT " << cloud_a->height << " WIDTH " << cloud_a->width <<std::endl;

    for (size_t i = 0; i < cloud_a->size() ; ++i)
     {
        static int a=0;

       cloud_a->points[i].x = -20;

       if (i%200==0)
       {
    a=a+1;
       }
       cloud_a->points[i].y = i%200;
       cloud_a->points[i].z =  a;
    }

    pcl::PointCloud<PointT>::Ptr cloud_b {new pcl::PointCloud<PointT>};
    cloud_b->width    = 1000;
    cloud_b->height   = 1;
    cloud_b->is_dense = false;
    cloud_b->points.resize (cloud_b->width * cloud_b->height);
//  std::cout << "cloud_b"  << cloud_b->size() << " HEIGHT " << cloud_b->height << " WIDTH " << cloud_b->width <<std::endl;
    for (size_t i = 0; i < cloud_b->size() ; ++i)
     {
       static int a=0;
       if (i%200==0)
       {
            a=a+1;
       }
       cloud_b->points[i].x = a;
       cloud_b->points[i].y = i%200 ;
       cloud_b->points[i].z =  -20;
    }
pcl::PointCloud<PointT>::Ptr cloud_c {new pcl::PointCloud<PointT>};
    cloud_c->width    = 1000;
    cloud_c->height   = 1;
    cloud_c->is_dense = false;
    cloud_c->points.resize (cloud_c->width * cloud_c->height);

    for (size_t i = 0; i < cloud_c->size() ; ++i)
     {
        static int a=0;
       if (i%200==0)
       {
    a=a+1;
       }
       cloud_c->points[i].x = a;
       cloud_c->points[i].y = i%200 ;
       cloud_c->points[i].z =  a;
    }





    pcl::PointCloud<PointT>::Ptr cloud {new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud2 {new pcl::PointCloud<PointT>};
    *cloud2=*cloud_a+*cloud_b;
    *cloud=*cloud2+*cloud_c;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CloudHandler stairCloud;
    float index;
    PointT o;
    std::vector<Eigen::Vector4f> centers;

   // stairCloud.GraspCloud(std::string(argv[1]));
    stairCloud.GraspCloud(cloud);
    stairCloud.PrintData();

   stairCloud.StairsAndPapesDetection();
   stairCloud.StairInformation();

//    int v1 = 0;

//    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//    viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
//    viewer->addCoordinateSystem(1,0,0,0,"global",v1);
//    viewer->addPointCloud<PointT> (stairCloud.GiveCloudPointer(),"view", v1);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "view");


    int v2=0;
//    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (1, 1, 1, v2);
    viewer->addCoordinateSystem (5);
    viewer->addPointCloud(stairCloud.GiveColoredCloud(),"sample",v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample");
   viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(stairCloud.GiveColoredCloud(),stairCloud.GiveNormals(),10,5,"normal",v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0,0 , "normal");
    viewer->setCameraPosition(20,0,0,0,0,0,0,0,1);
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 100, "normal");



centers =stairCloud.GiveCentroidPlanes();

for (auto it=centers.begin(); it != centers.end();++it)
{
    index = it-centers.begin();
    o.x = (*it)[0];
    o.y = (*it)[1];
    o.z = (*it)[2];
    viewer->addSphere (o, 1, 250, 0, 0,std::to_string(index),v2);
//    std::cout <<index <<  "CENTER "<< " " << o.x << " " << o.y << " " << o.z <<std::endl;
}

while (!viewer->wasStopped ())
{
    viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}

return 0;

}

