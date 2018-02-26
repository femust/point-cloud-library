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

    pcl::PointCloud<PointT>::Ptr cloud_a {new pcl::PointCloud<PointT>};
    cloud_a->width    = 10;
    cloud_a->height   = 5;
    cloud_a->is_dense = false;
    cloud_a->points.resize (cloud_a->width * cloud_a->height);

  std::cout << "cloud_a"  << cloud_a->size() << " HEIGHT " << cloud_a->height << " WIDTH " << cloud_a->width <<std::endl;

    for (size_t i = 0; i < cloud_a->size() ; ++i)
     {

       cloud_a->points[i].x = int(i/10);

       cloud_a->points[i].y = 0;

       cloud_a->points[i].z = (i%5);

    }

    pcl::PointCloud<PointT>::Ptr cloud_b {new pcl::PointCloud<PointT>};
    cloud_b->width    = 10;
    cloud_b->height   = 5;
    cloud_b->is_dense = false;
    cloud_b->points.resize (cloud_b->width * cloud_b->height);

  std::cout << "cloud_b"  << cloud_b->size() << " HEIGHT " << cloud_b->height << " WIDTH " << cloud_b->width <<std::endl;

    for (size_t i = 0; i < cloud_b->size() ; ++i)
     {

       cloud_b->points[i].x = int(i/10);
       cloud_b->points[i].y = (i%5);

       cloud_b->points[i].z = -10;

    }


      pcl::PointCloud<PointT>::Ptr cloud {new pcl::PointCloud<PointT>};

      //pcl::concatenatePointCloud(*cloud_a,*cloud_b,*cloud);
      cloud->width    = 20;
      cloud->height   = 10;
      cloud->is_dense = false;
 cloud->points.resize (cloud_b->width * cloud_b->height);
      *cloud=*cloud_a+*cloud_b;
      std::cout << "CONCATENATED CLOUD " << cloud->size() << std::endl;







    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CloudHandler stairCloud;
    float index;
    PointT o;
    std::vector<Eigen::Vector4f> centers;

   // stairCloud.GraspCloud(std::string(argv[1]));
    stairCloud.GraspCloud(cloud);
    //stairCloud.PrintData();

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
   viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(stairCloud.GiveColoredCloud(),stairCloud.GiveNormals(),1,5,"normal",v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,0,0 , "normal");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 100, "normal");



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

