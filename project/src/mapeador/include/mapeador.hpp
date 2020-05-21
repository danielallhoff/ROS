#pragma once
#ifndef _MAPEADOR_H_
#define _MAPEADOR_H_

constexpr char SUB_TARGET[] = "/camera/depth/points";
constexpr char PUB_TARGET[] = "/mapper/cloud";

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

using FeatureType = pcl::PFHSignature125;
using CloudFeatureType_t = pcl::PointCloud<FeatureType>::Ptr;

using CloudNormals = pcl::PointCloud<pcl::Normal>;
using CloudNormals_t = pcl::PointCloud<pcl::Normal>::Ptr;


using Normal = pcl::Normal;
class Mapper{
    private:
        CloudPoint3DIPtr_t last_keypoints_cloud;
        CloudPoint3DIPtr_t cloudpoint;
        
        //CloudPoint3DRGB_t VoxelGridFilter(const CloudPoint3DRGB_t);
        Cloud2Ptr_t Ransac(const Cloud2Ptr_t);
        void detect_harris3d_keypoints(const CloudPoint3DIPtr_t& input, const CloudPoint3DIPtr_t& keypoints);
        CloudPtr_t PC2toPC(Cloud2Ptr_t);
        Cloud2Ptr_t PCtoPC2(CloudPtr_t);
        void findCorrespondences(const CloudFeatureType_t source, const CloudFeatureType_t target, std::vector<int>& correspondences);
        void PublishPointCloud(const CloudPoint3DIPtr_t);
        void extractDescriptors(CloudPoint3DIPtr_t input, CloudPoint3DIPtr_t keypoints, CloudFeatureType_t features);
        CloudPoint3DIPtr_t RegistrarNubes(const CloudPoint3DIPtr_t& source, const CloudPoint3DIPtr_t& target);
    protected:
        ros::Publisher pub;
        ros::Subscriber sub;
        void PointCloudCallback(const Cloud2Ptr_t);  
    public:
        Mapper(ros::NodeHandle& nh);
        void bucle();
        
};

#endif /* _MAPEADOR_H_ */