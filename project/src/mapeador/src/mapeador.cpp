#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/io/pcd_io.h>
#include <eigen32/Eigen/Dense>

#include "mapeador.hpp"

Mapper::Mapper(ros::NodeHandle& nh, bool useSub = true){
    srand(time(NULL));
    pub = nh.advertise<Cloud2_t>(PUB_TARGET, 1);
    if(useSub) sub = nh.subscribe(SUB_TARGET, 1, &Mapper::PointCloudCallback, this);
    merged = CloudPoint3DIPtr_t (new CloudPoint3DI_t());
    last_keypoints_cloud = CloudPoint3DIPtr_t (new CloudPoint3DI_t());
}

CloudPoint3DIPtr_t Mapper::ConvertCloud2To3DI(const Cloud2Ptr_t& source){
    CloudPoint3DIPtr_t converted_cloud(new CloudPoint3DI_t());
    pcl::PointCloud<Point3DRGB>::Ptr nube_rgb(new pcl::PointCloud<Point3DRGB>());

    pcl::fromPCLPointCloud2(*source, *nube_rgb);
    pcl::PointCloudXYZRGBtoXYZI(*nube_rgb, *converted_cloud);

    return converted_cloud;   
}

void Mapper::CalculateCorrespondences(const std::vector<int>& st_correspondences, const std::vector<int>& ts_correspondences, pcl::CorrespondencesPtr& correspondences_){
    std::vector<std::pair<unsigned, unsigned>> correspondences;
    for(std::size_t cIdx = 0; cIdx < st_correspondences.size(); ++cIdx){
        if(ts_correspondences[st_correspondences[cIdx]] == static_cast<int>(cIdx)){
            correspondences.push_back(std::make_pair(cIdx, st_correspondences[cIdx]));
        }
    }

    correspondences_->resize(correspondences.size());
    for (std::size_t cIdx = 0; cIdx < correspondences.size(); ++cIdx) {
        (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
        (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
    }
}

void Mapper::RegistrarNubes(CloudFeatureType_t& source_features, CloudPoint3DIPtr_t& source_keypoints, const CloudPoint3DIPtr_t& target){
    std::cout << "Detecting keypoints\n";
    
    // Filters both PointClouds
    CloudPoint3DIPtr_t target_filtered(new CloudPoint3DI_t());
    VoxelGridFilter(target, target_filtered);

    // Detects keypoints in both clouds
    CloudPoint3DIPtr_t target_keypoints(new CloudPoint3DI_t());
    detect_harris3d_keypoints(target_filtered, target_keypoints);

    std::cout << "Extracting features\n";        
    // Extracts keypoints into features
    CloudFeatureType_t target_features(new pcl::PointCloud<FeatureType>);
    extractDescriptors(target_filtered, target_keypoints, target_features);
    
    std::cout << "Find correspondences\n";
    // Finds correspondences between feature-type cloud
    std::vector<int> source_target_correspondences;
    std::vector<int> target_source_correspondences;
    findCorrespondences(target_features, source_features, target_source_correspondences);
    std::cout << "Despues de target-source\n";
    findCorrespondences(source_features, target_features, source_target_correspondences);
    std::cout << "Despues de source-target\n";
    
    //Convert correspondences to pcl::Correspondences
    pcl::CorrespondencesPtr correspondences_(new pcl::Correspondences);
    CalculateCorrespondences(source_target_correspondences, target_source_correspondences, correspondences_);  
    
    std::cout << "Correspondences calculated.Total correspondences: " << correspondences_->size() <<  "\n";
    //Filter correspondences using Random sample Consensus
    rejectCorrespondencesRansac(source_keypoints, target_keypoints, correspondences_);
    
    std::cout << "Removed bad correspondences via ransac. Total correspondences: " << correspondences_->size() <<  "\n";
    
    //Determine initial Point3DRGB
    CloudPoint3DIPtr_t source_transformed_(new CloudPoint3DI_t);
    CloudPoint3DIPtr_t source_registered_(new CloudPoint3DI_t);
    calculateInitialTransformation(source_keypoints, target_keypoints, source_transformed_,correspondences_);
    
    std::cout << "Applied initial transformation \n";
    //Determine final transformation and add to the final Point3DI
    pcl::Registration<Point3DI, Point3DI>::Ptr registration(new pcl::IterativeClosestPoint<Point3DI, Point3DI>());
    registration->setInputSource(source_transformed_);
    registration->setInputTarget(target);
    registration->setMaxCorrespondenceDistance(0.05f);
    registration->setRANSACOutlierRejectionThreshold(0.05f);
    registration->setTransformationEpsilon(0.000001f);
    registration->setMaximumIterations(1000);
    //Final cloud saved
    registration->align(*merged);
    std::cout << "Applied final transformation \n";

    source_features = target_features;
    source_keypoints = target_keypoints;

}

//Calculate initial transformation and return the transformed point cloud in source_transformed
void Mapper::calculateInitialTransformation(const CloudPoint3DIPtr_t source_keypoints, const CloudPoint3DIPtr_t target_keypoints, CloudPoint3DIPtr_t source_transformed, pcl::CorrespondencesPtr correspondences_){
    Eigen::Matrix4f initial_transformation_matrix;
    pcl::registration::TransformationEstimation<Point3DI, Point3DI>::Ptr transform_estimation(new pcl::registration::TransformationEstimationSVD<Point3DI, Point3DI>);
    transform_estimation->estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences_, initial_transformation_matrix);
    pcl::transformPointCloud(*source_keypoints,*source_transformed,initial_transformation_matrix);
}

constexpr int k { 1 };
//Find correspondences between both clouds. The result is a vector correspondences of int, associating which point from the cloud1 corresponds to which from cloud2
void Mapper::findCorrespondences(const CloudFeatureType_t source,  const CloudFeatureType_t target, std::vector<int>& correspondences){    
    correspondences.resize(source->size());
    
    // Search for near matches
    pcl::KdTreeFLANN<FeatureType> descriptor;
    descriptor.setInputCloud(target);

    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for(std::size_t i = 0; i < source->size(); ++i){
        descriptor.nearestKSearch(*source, i, k,k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
}

void Mapper::bucle(){
    ros::Rate rate(1);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

void Mapper::LoadClouds(std::vector<CloudPoint3DIPtr_t>& v, const char* const path){
    boost::filesystem::path p(path);
    boost::filesystem::directory_iterator iter(p), end_itr;
    std::vector<std::string> files;

    BOOST_FOREACH(boost::filesystem::path const& i, std::make_pair(iter, end_itr)){
        if(is_regular_file(i)){
            files.push_back(i.string());
        }
    }

    std::sort(files.begin(), files.end());

    for(std::string s: files){
        CloudPoint3DIPtr_t cloud(new CloudPoint3DI_t);
        std::cout << "Reading file: " << s << '\n';
        if(pcl::io::loadPCDFile<Point3DI>(s, *cloud) == -1){
            std::cerr << "Error loading file " << s << '\n';
            exit(-1); 
        }
        v.push_back(cloud);
        std::cout << "Loaded " << v.size() << " clouds\n";
        if(v.size() == 10) return;
    }
}

void Mapper::RegistrarNubesGuardadas(const char* const directory_name){
    std::vector<CloudPoint3DIPtr_t> C;
    std::cout << "Loading clouds\n";
    LoadClouds(C, directory_name);
    std::cout << "Loaded all clouds\n";
    
    //Extract first keypoints
    CloudFeatureType_t source_features(new pcl::PointCloud<FeatureType>);
    CloudPoint3DIPtr_t source_filtered(new CloudPoint3DI_t());
    
    std::cout << "Processing first cloud\n";
    VoxelGridFilter(C[0], source_filtered);
    detect_harris3d_keypoints(source_filtered, last_keypoints_cloud);
    extractDescriptors(source_filtered, last_keypoints_cloud, source_features);

    for(int i=1; i<C.size()-1;++i){
        
        RegistrarNubes(source_features, last_keypoints_cloud, C[i]);
        PublishPointCloud(merged);
        std::cout << "Registered " << i + 1 << " clouds\n";
    }
}

// Source ->? Cloud
// Target ->? last_cloud
void Mapper::PointCloudCallback(const Cloud2Ptr_t cloud){
    std::cout << "Received pointcloud, starting operations\n"; 
    CloudPoint3DIPtr_t converted_cloud = ConvertCloud2To3DI(cloud);
    CloudPoint3DIPtr_t source_filtered(new CloudPoint3DI_t());
    CloudFeatureType_t source_features(new pcl::PointCloud<FeatureType>);

    //When already exists keypoints, pair them
    if(last_keypoints_cloud != NULL){
        RegistrarNubes(source_features, last_keypoints_cloud, converted_cloud);
        PublishPointCloud(merged);
    }
    else{
        last_keypoints_cloud = converted_cloud;
        VoxelGridFilter(converted_cloud, source_filtered);
        detect_harris3d_keypoints(source_filtered, last_keypoints_cloud);
    }
}

int main(int argc, char** argv){

    if(argc != 2 && argc != 3){
        std::cout << "USAGE:\n\n\trosrun mapeador mapeador_node -[SP|LP] <pointcloud_directory>\n";
        return -1;
    }

    if(!std::strcmp("-SP", argv[1])){
        ros::init(argc, argv, "mapeador");
        ros::NodeHandle nh;
        Mapper mapper(nh, true);
        mapper.bucle();
    }
    else if(!std::strcmp("-LP", argv[1])){
        if(argc != 3){
            std::cerr << "Directory not specified.\n";
            return -1;
        }
        else{
            ros::init(argc, argv, "mapeador");
            ros::NodeHandle nh;
            Mapper mapper(nh, false);
            mapper.RegistrarNubesGuardadas(argv[2]);
        }
    }

    return 0;
}

///////////////////////////
///// PRIVATE METHODS /////
///////////////////////////

void Mapper::VoxelGridFilter(const CloudPoint3DIPtr_t& cloud, CloudPoint3DIPtr_t& filtered){
    pcl::VoxelGrid<Point3DI> vgf;

    vgf.setInputCloud(cloud);
    vgf.setLeafSize(0.025f, 0.025f, 0.025f);
    vgf.filter(*filtered);
}

void Mapper::PublishPointCloud(const CloudPoint3DIPtr_t cloud){
    Cloud2Ptr_t publish_cloud(new Cloud2_t());
    pcl::toPCLPointCloud2(*cloud, *publish_cloud);
    publish_cloud->header.frame_id = "camera_depth_optical_frame";
    pub.publish(*publish_cloud);
}

void Mapper::rejectCorrespondencesRansac(const CloudPoint3DIPtr_t source_keypoints, const CloudPoint3DIPtr_t target_keypoints,  pcl::CorrespondencesPtr correspondences){
    pcl::registration::CorrespondenceRejectorSampleConsensus<Point3DI> rejectorRANSAC;
    //rejectorRANSAC.setInputSource(last_keypoints_cloud);
    rejectorRANSAC.setInputSource(source_keypoints);
    //rejectorRANSAC.setInputTarget(keypoints);
    rejectorRANSAC.setInputTarget(target_keypoints);
    rejectorRANSAC.setInlierThreshold(0.01);
    rejectorRANSAC.setMaximumIterations(1000);
    rejectorRANSAC.setInputCorrespondences(correspondences);
    rejectorRANSAC.getCorrespondences(*correspondences);
}

void Mapper::extractDescriptors(CloudPoint3DIPtr_t input, CloudPoint3DIPtr_t keypoints, CloudFeatureType_t features){

    pcl::PFHEstimation<Point3DI, Normal, FeatureType>::Ptr feature_extractor(new pcl::PFHEstimation<Point3DI, Normal, FeatureType>);
    pcl::FeatureFromNormals<Point3DI, Normal, FeatureType>::Ptr feature_from_normals = (pcl::FeatureFromNormals<Point3DI, Normal, FeatureType>::Ptr) feature_extractor;
    
    feature_extractor->setKSearch(50);
    feature_extractor->setSearchSurface(input);
    feature_extractor->setInputCloud(keypoints);
    //if(feature_from_normals){
        CloudNormals_t normals(new CloudNormals);      
        pcl::NormalEstimation<Point3DI, Normal> normal_estimation;
        normal_estimation.setSearchMethod(pcl::search::Search<Point3DI>::Ptr(new pcl::search::KdTree<Point3DI>));
        normal_estimation.setRadiusSearch(0.01);
        normal_estimation.setInputCloud(input);
        normal_estimation.compute(*normals);
        feature_from_normals->setInputNormals(normals);    
    //}
    
    feature_extractor->compute(*features);
}

void Mapper::detect_harris3d_keypoints(CloudPoint3DIPtr_t& input, CloudPoint3DIPtr_t& keypoints){
    
    pcl::HarrisKeypoint3D<Point3DI, Point3DI>* harris3D = new pcl::HarrisKeypoint3D<Point3DI, Point3DI>(pcl::HarrisKeypoint3D<Point3DI, Point3DI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius(0.01f);
    harris3D->setRadiusSearch(0.01f);
    std::cout << "Halfway\n";
    pcl::Keypoint<Point3DI, Point3DI>::Ptr keypoint_detector;
    keypoint_detector.reset(harris3D);
    keypoint_detector->setInputCloud(input);
    std::cout << "Computing\n";
    keypoint_detector->compute(*keypoints);
}