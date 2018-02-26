#ifndef CLOUDHANDLER_H
#define CLOUDHANDLER_H

#include <iostream>

#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



#include <typeinfo>


class CloudHandler {
public:
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;


CloudHandler();
~CloudHandler();

void GraspCloud(const std::string);
void GraspCloud(PointCloud::Ptr);

void FilterCloud();

void EstimatePointNormal();

//void CylinderSegmentation();

//void PlaneSegmentation();

void RegionGrowingMethod();

void StairsAndPapesDetection();


PointCloud::Ptr GiveCloudPointer() const;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GiveColoredCloud() const;

std::vector<pcl::PointCloud<PointT>::Ptr> GivePlanes() const;

SurfaceNormals::Ptr GiveNormals() const;

std::vector<Eigen::Vector4f> GiveCentroidPlanes() const;

void PrintData();

enum Mode {GRAPH = 0,SEPARATE};


private:

 Mode mode_ = GRAPH;
 PointCloud::Ptr _cloud;
 PointCloud::Ptr cloud_filtered;
 PointCloud::Ptr cloud_cylinder;
 SurfaceNormals::Ptr cloud_normals;


 pcl::PointCloud<pcl::PointXYZRGB>::Ptr _colored_cloud;

 std::vector<PointCloud::Ptr> _planes;
 std::vector<Eigen::Vector4f> _centroid_planes;

 double _verticalLimit = cos(15.0/ 180.0 * M_PI);
 double _horizontalLimit = cos(75.0/ 180.0 * M_PI);


 std::vector<pcl::PointIndices> clusters;
 std::vector<pcl::PointIndices::Ptr> eigen_inliers;
 std::vector <Eigen::Matrix3f> EigenVectors;
 std::vector <Eigen::Vector3f> EigenValues;


};






#endif // CLOUDHANDLER_H
