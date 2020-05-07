#pragma once
#ifndef _MAPEADOR_H_
#define _MAPEADOR_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

constexpr char SUB_TARGET[] = "/camera/depth/points";
constexpr char PUB_TARGET[] = "";

using Cloud_t = pcl::PCLPointCloud2;
using CloudPtr_t = Cloud_t::Ptr;

class Mapper{
    private:
        CloudPtr_t VoxelGridFilter(const CloudPtr_t);
    protected:
        ros::Publisher pub;
        ros::Subscriber sub;
        void PointCloudCallback(const sensor_msgs::PointCloud2&);  
    public:
        Mapper(ros::NodeHandle& nh);
        void bucle();
};

#endif /* _MAPEADOR_H_ */