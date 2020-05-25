#pragma once
#ifndef _MAPEADOR_H_
#define _MAPEADOR_H_

constexpr char SUB_TARGET[] = "/camera/depth/points";
constexpr char PUB_TARGET[] = "/mapper/cloud";

using Cloud2_t = pcl::PCLPointCloud2;
using Cloud2Ptr_t = Cloud2_t::Ptr;

using Point3DI= pcl::PointXYZI;
using CloudPoint3DI_t = pcl::PointCloud<Point3DI>;
using CloudPoint3DIPtr_t = CloudPoint3DI_t::Ptr;

using Point3DRGB = pcl::PointXYZRGB;
using CloudPoint3DRGB_t = pcl::PointCloud<Point3DRGB>;
using CloudPoint3DRGBPtr_t = CloudPoint3DRGB_t::Ptr;

using FeatureType = pcl::PFHSignature125;
using CloudFeatureType_t = pcl::PointCloud<FeatureType>::Ptr;

using Normal = pcl::Normal;
using CloudNormals = pcl::PointCloud<Normal>;
using CloudNormals_t = pcl::PointCloud<Normal>::Ptr;

class Mapper{
    private:
        CloudPoint3DIPtr_t last_keypoints_cloud;
        CloudPoint3DIPtr_t cloudpoint;
        CloudPoint3DIPtr_t merged;
        //CloudPoint3DRGB_t VoxelGridFilter(const CloudPoint3DRGB_t);
        void rejectCorrespondencesRansac(const CloudPoint3DIPtr_t source_keypoints, const CloudPoint3DIPtr_t target_keypoints,  pcl::CorrespondencesPtr correspondences);
        void detect_harris3d_keypoints(CloudPoint3DIPtr_t& input, CloudPoint3DIPtr_t& keypoints);
        CloudPoint3DIPtr_t ConvertCloud2To3DI(const Cloud2Ptr_t&);
        void CalculateCorrespondences(const std::vector<int>& st_correspondences, const std::vector<int>& ts_correspondences, pcl::CorrespondencesPtr& correspondences_);
        void findCorrespondences(const CloudFeatureType_t source, const CloudFeatureType_t target, std::vector<int>& correspondences);
        void VoxelGridFilter(const CloudPoint3DIPtr_t& cloud, CloudPoint3DIPtr_t& filtered);
        void extractDescriptors(CloudPoint3DIPtr_t input, CloudPoint3DIPtr_t keypoints, CloudFeatureType_t features);
        void calculateInitialTransformation(const CloudPoint3DIPtr_t source_filtered, const CloudPoint3DIPtr_t target_keypoints, CloudPoint3DIPtr_t source_transformed, pcl::CorrespondencesPtr correspondences_);
        void PublishPointCloud(const CloudPoint3DIPtr_t);
        void RegistrarNubes(CloudFeatureType_t& source_filtered, CloudPoint3DIPtr_t& source_keypoints, const CloudPoint3DIPtr_t& target);
        void LoadClouds(std::vector<CloudPoint3DIPtr_t>& v, const char* const path);
    protected:
        ros::Publisher pub;
        ros::Subscriber sub;
        void PointCloudCallback(const Cloud2Ptr_t);  
    public:
        Mapper(ros::NodeHandle&, bool);
        void bucle();
        void RegistrarNubesGuardadas(const char* const);
};

#endif /* _MAPEADOR_H_ */