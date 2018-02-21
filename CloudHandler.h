#ifndef CLOUDHANDLER_H
#define CLOUDHANDLER_H


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



class CloudHandler {

private:
typedef pcl::PointXYZRGB PointT;

 pcl::PointCloud<PointT>::Ptr _cloud;
 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
 pcl::PointCloud<PointT>::Ptr cloud_filtered;
 pcl::PointCloud<PointT>::Ptr cloud_cylinder;
 std::vector<pcl::PointCloud<PointT>::Ptr> _planes;
 std::vector<Eigen::Vector4f> _centroid_planes;



public:

CloudHandler();
~CloudHandler();

void GraspCloud(const std::string);

void FilterCloud();

void EstimatePointNormal();

void CylinderSegmentation();

void PlaneSegmentation();

pcl::PointCloud <pcl::PointXYZRGB>::Ptr RegionGrowingMethod();

void StairsAndPapesDetection();


pcl::PointCloud<PointT>::Ptr GiveCloudPointer() const;

std::vector<pcl::PointCloud<PointT>::Ptr> GivePlanes() const;

std::vector<Eigen::Vector4f> GiveCentroidPlanes() const;

void PrintData();



};






#endif // CLOUDHANDLER_H
