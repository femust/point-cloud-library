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

    if (argc<2)
    {
        std::cerr << "\033[1;31mGIVE ME A FILE\033[0m\n"<<std::endl;
        return 0;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CloudHandler stairCloud;
    float index;
    PointT o;
    std::vector<Eigen::Vector4f> centers;

    stairCloud.GraspCloud(std::string(argv[1]));
    stairCloud.StairsAndPapesDetection();

    int v1 = 0;
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
    viewer->addCoordinateSystem(1,0,0,0,"global",v1);
    viewer->addPointCloud<PointT> (stairCloud.GiveCloudPointer(),"view", v1);





    int v2=0;
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (1, 1, 1, v2);
    viewer->addCoordinateSystem (1);
    viewer->addPointCloud(stairCloud.GiveColoredCloud(),"sample",v2);
    //viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(stairCloud.GiveColoredCloud(),stairCloud.GiveNormals(),100,0.2,"normal",v2);
    centers =stairCloud.GiveCentroidPlanes();

//       o.x = 1.0;
//       o.y = 0;
//       o.z = 0;
//       viewer->addSphere(o,0.25,"sphere",v2);

for (auto it=centers.begin(); it != centers.end();++it)
{
    index = centers.begin() - it;
    o.x = (*it)[0];
    o.y = (*it)[1];
    o.z = (*it)[2];
    viewer->addSphere (o, 0.02, 250, 0, 0,std::to_string(index),v2);
}

while (!viewer->wasStopped ())
{
    viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}

return 0;

}

