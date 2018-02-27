#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

int
main (int, char**)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

     // Fill in the cloud data
     cloud.width    = 5;
     cloud.height   = 1;
     cloud.is_dense = false;
     cloud.points.resize (cloud.width * cloud.height);


       cloud.points[0].x = 0;
       cloud.points[0].y = -1;
       cloud.points[0].z = 0;

       cloud.points[1].x = 0;
       cloud.points[1].y = -1;
       cloud.points[1].z = 1;

       cloud.points[2].x = 0;
       cloud.points[2].y = 1;
       cloud.points[2].z = 1;

       cloud.points[3].x = 0;
       cloud.points[3].y = 1;
       cloud.points[3].z = 0;

       cloud.points[4].x = 0;
       cloud.points[4].y = 1;
       cloud.points[4].z = -120;




  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (cloud, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
  return (0);
}
