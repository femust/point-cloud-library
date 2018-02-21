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



class CloudHandler {

public:

CloudHandler();
~CloudHandler();

void GraspCloud(const std::string);

void FilterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);

void EstimatePointNormal(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                         pcl::PointCloud<pcl::Normal>::Ptr);

void CylinderSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                          pcl::PointCloud<pcl::Normal>::Ptr,
                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);

void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                        pcl::PointCloud<pcl::Normal>::Ptr,
                                        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &,
                                        std::vector<Eigen::Vector4f> &);

void StairsAndPapesDetection();


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GiveCloudPointer() const;

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> GivePlanes() const;

 std::vector<Eigen::Vector4f> GiveCentroidPlanes() const;

void PrintData();

private:

 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud;
 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered;
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cylinder;
 std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> _planes;
 std::vector<Eigen::Vector4f> _centroid_planes;


};






#endif // CLOUDHANDLER_H
