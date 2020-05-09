#pragma once
#ifndef _MAPEADOR_H_
#define _MAPEADOR_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

constexpr char SUB_TARGET[] = "/camera/depth/points";
constexpr char PUB_TARGET[] = "/";

using Point3D = pcl::PointXYZ;
using Point3DRGB = pcl::PointXYZRGB;
using Point3DI= pcl::PointXYZI;
using Cloud_t = pcl::PointCloud<Point3D>;
using CloudPtr_t = Cloud_t::Ptr;
using Cloud2_t = pcl::PCLPointCloud2;
using Cloud2Ptr_t = Cloud2_t::Ptr;

using CloudPoint3DI_t = pcl::PointCloud<Point3DI>;
using CloudPoint3DIPtr_t = CloudPoint3DI_t::Ptr;

using CloudPoint3DRGB_t = pcl::PointCloud<Point3DRGB>;
using CloudPoint3DRGBPtr_t = CloudPoint3DRGB_t::Ptr;

class Mapper{
    private:
        CloudPoint3DIPtr_t last_keypoints_cloud;


        Cloud2Ptr_t VoxelGridFilter(const Cloud2Ptr_t);
        Cloud2Ptr_t Ransac(const Cloud2Ptr_t);
        void extract_harris3d_keypoints(const Cloud2Ptr_t input, const CloudPoint3DIPtr_t keypoints);
        CloudPtr_t PC2toPC(Cloud2Ptr_t);
        Cloud2Ptr_t PCtoPC2(CloudPtr_t);
        void findCorrespondences(const CloudPoint3DIPtr_t source, const CloudPoint3DIPtr_t target, std::vector<std::pair<unsigned, unsigned>>& correspondences);
    protected:
        ros::Publisher pub;
        ros::Subscriber sub;
        void PointCloudCallback(const Cloud2Ptr_t);  
    public:
        Mapper(ros::NodeHandle& nh);
        void bucle();
};

#endif /* _MAPEADOR_H_ */