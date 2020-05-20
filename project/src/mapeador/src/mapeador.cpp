#include "mapeador.hpp"
#include <pcl/memory.h>
#include <eigen32/Eigen/Dense>

Mapper::Mapper(ros::NodeHandle& nh){
    srand(time(NULL));
    pub = nh.advertise<Cloud2_t>(PUB_TARGET, 1);
    sub = nh.subscribe(SUB_TARGET, 1, &Mapper::PointCloudCallback, this);
}

CloudPoint3DIPtr_t ConvertCloud2To3DI(const Cloud2Ptr_t& source){
    CloudPoint3DIPtr_t converted_cloud(new CloudPoint3DI_t());
    pcl::PointCloud<Point3DRGB>::Ptr nube_rgb(new pcl::PointCloud<Point3DRGB>());

    pcl::fromPCLPointCloud2(*source, *nube_rgb);
    pcl::PointCloudXYZRGBtoXYZI(*nube_rgb, *converted_cloud);

    return converted_cloud;   
}

// Source ->? Cloud
// Target ->? last_cloud
void Mapper::PointCloudCallback(const Cloud2Ptr_t cloud){
    std::cout << "Received pointcloud, starting operations\n"; 
    CloudPoint3DIPtr_t converted_cloud = ConvertCloud2To3DI(cloud);
    
    //Cloud2Ptr_t filtered_point_cloud = VoxelGridFilter(cloud);
    /*CloudPoint3DIPtr_t keypoints(new CloudPoint3DI_t());
    extract_harris3d_keypoints(cloud, keypoints);
    PublishPointCloud(keypoints);
    std::cout << "Keypoints extracted from cloud. Number of keypoints: " << keypoints->size() << "\n";
*/
    //When already exists keypoints, pair them
    if(last_keypoints_cloud != NULL){
        // Detects keypoints in both clouds
        CloudPoint3DIPtr_t source_keypoints(new CloudPoint3DI_t());
        CloudPoint3DIPtr_t target_keypoints(new CloudPoint3DI_t());
        detect_harris3d_keypoints(converted_cloud, source_keypoints);
        detect_harris3d_keypoints(last_keypoints_cloud, target_keypoints);

        // Extracts keypoints into features
        CloudFeatureType_t source_features(new pcl::PointCloud<FeatureType>);
        CloudFeatureType_t target_features(new pcl::PointCloud<FeatureType>);
        extractDescriptors(converted_cloud, source_keypoints, source_features);
        extractDescriptors(last_keypoints_cloud, target_keypoints, target_features);

        // Finds correspondences between feature-type cloud
        std::vector<int> source_target_correspondences;
        std::vector<int> target_source_correspondences;
        findCorrespondences(target_features, source_features, target_source_correspondences);
        std::cout << "Despues de target-source\n";
        findCorrespondences(source_features, target_features, source_target_correspondences);
        std::cout << "Despues de source-target\n";
        

        std::vector<std::pair<unsigned, unsigned>> correspondences;
        for(std::size_t cIdx = 0; cIdx < source_target_correspondences.size(); ++cIdx){
            if(target_source_correspondences[source_target_correspondences[cIdx]] == static_cast<int>(cIdx)){
                correspondences.push_back(std::make_pair(cIdx, source_target_correspondences[cIdx]));
            }
        }

        //Calculate correspondences
        /*std::vector<std::pair<unsigned, unsigned>> correspondences;
        findCorrespondences(last_keypoints_cloud, keypoints, correspondences);
        std::cout << "Correspondences calculated: " << correspondences.size() << "\n";
        CloudPoint3DIPtr_t source_transformed_(new CloudPoint3DI_t);
        CloudPoint3DIPtr_t source_registered_(new CloudPoint3DI_t);
        */

        //Convert correspondences to pcl::Correspondences
        pcl::CorrespondencesPtr correspondences_(new pcl::Correspondences);
        correspondences_->resize(correspondences.size());
        for (std::size_t cIdx = 0; cIdx < correspondences.size(); ++cIdx) {
            (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
            (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
        }
        std::cout << "Correspondences calculated.Total correspondences: " << correspondences_->size() <<  "\n";
        //Filter correspondences using Random sample Consensus
        pcl::registration::CorrespondenceRejectorSampleConsensus<Point3DI> rejectorRANSAC;
        //rejectorRANSAC.setInputSource(last_keypoints_cloud);
        rejectorRANSAC.setInputSource(source_keypoints);
        //rejectorRANSAC.setInputTarget(keypoints);
        rejectorRANSAC.setInputTarget(target_keypoints);
        rejectorRANSAC.setInlierThreshold(0.01);
        rejectorRANSAC.setMaximumIterations(1000);
        rejectorRANSAC.setInputCorrespondences(correspondences_);
        rejectorRANSAC.getCorrespondences(*correspondences_);
        std::cout << "Removed bad correspondences via ransac. Total correspondences: " << correspondences_->size() <<  "\n";
       
        //Determine initial Point3DRGB
        CloudPoint3DIPtr_t source_transformed_(new CloudPoint3DI_t);
        CloudPoint3DIPtr_t source_registered_(new CloudPoint3DI_t);
        pcl::registration::TransformationEstimation<Point3DI, Point3DI>::Ptr transform_estimation(new pcl::registration::TransformationEstimationSVD<Point3DI, Point3DI>);
        
        Eigen::Matrix4f initial_transformation_matrix;
        Eigen::Matrix4f transformation_matrix;
        //Calculate transformation matrix requires intensity
        //transform_estimation->estimateRigidTransformation(*last_keypoints_cloud, *keypoints, *correspondences_, initial_transformation_matrix);
        transform_estimation->estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences_, initial_transformation_matrix);
        
        pcl::transformPointCloud(*converted_cloud, *source_transformed_, initial_transformation_matrix);
        
        std::cout << "Applied initial transformation \n";
        //Determine final transformation and add to the final Point3DI
        pcl::Registration<Point3DI, Point3DI>::Ptr registration(new pcl::IterativeClosestPoint<Point3DI, Point3DI>());
        registration->setInputSource(source_transformed_);
        registration->setInputTarget(last_keypoints_cloud);
        registration->setMaxCorrespondenceDistance(0.05f);
        registration->setRANSACOutlierRejectionThreshold(0.05f);
        registration->setTransformationEpsilon(0.000001f);
        registration->setEuclideanFitnessEpsilon(1);
        registration->setMaximumIterations(50);
        //Final cloud saved
        registration->align(*source_registered_);
        transformation_matrix = registration->getFinalTransformation();
        std::cout << "Applied final transformation \n";
        
        // Reconstructs surface based on transformed and target cloud
        CloudPoint3DIPtr_t merged(new CloudPoint3DI_t);
        *merged = *source_registered_;
        //*merged = *source_transformed_;
        *merged += *last_keypoints_cloud;
        
        pcl::VoxelGrid<Point3DI> voxel_grid;
        voxel_grid.setInputCloud(merged);
        voxel_grid.setLeafSize(0.002f, 0.002f, 0.002f);
        voxel_grid.setDownsampleAllData(true);
        voxel_grid.filter(*merged);
        std::cout << "Merged and filtered \n";

        last_keypoints_cloud = merged;
        PublishPointCloud(merged);
    }else{

        last_keypoints_cloud = converted_cloud;
        //PublishPointCloud(last_keypoints_cloud);
    }
}

constexpr int k { 1 };
//Find correspondences between both clouds. The result is a vector correspondences of int, associating which point from the cloud1 corresponds to which from cloud2
void Mapper::findCorrespondences(const CloudFeatureType_t source,  const CloudFeatureType_t target, std::vector<int>& correspondences){
    
    correspondences.resize(source->size());
    
    // Search for near matches
    //pcl::KdTreeFLANN<Point3DI> descriptor;
    pcl::KdTreeFLANN<FeatureType> descriptor;
    descriptor.setInputCloud(target);

    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for(std::size_t i = 0; i < source->size(); ++i){
        //Find the nearest target point to source
        descriptor.nearestKSearch(*source, i, k,k_indices, k_squared_distances);
        //correspondences.push_back(std::make_pair(i,k_indices[0]));
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

int main(int argc, char** argv){
    ros::init(argc, argv, "mapeador");
    ros::NodeHandle nh;
    Mapper mapper(nh);
    mapper.bucle();
    return 0;
}


///////////////////////////
///// PRIVATE METHODS /////
///////////////////////////
/*
CloudPoint3DRGB_t Mapper::VoxelGridFilter(const CloudPoint3DRGB_t cloud){
    CloudPoint3DRGB_t cloud_filtered(new CloudPoint3DRGB_t());
    pcl::VoxelGrid<Point3DRGB> vgf;

    vgf.setInputCloud(cloud);
    vgf.setLeafSize(0.01f, 0.01f, 0.01f);
    vgf.filter(*cloud_filtered);

    return cloud_filtered;
}*/

void Mapper::PublishPointCloud(const CloudPoint3DIPtr_t cloud){
    Cloud2Ptr_t publish_cloud(new Cloud2_t());
    pcl::toPCLPointCloud2(*cloud, *publish_cloud);
    publish_cloud->header.frame_id = "camera_depth_optical_frame";
    pub.publish(*publish_cloud);
}

Cloud2Ptr_t Mapper::Ransac(const Cloud2Ptr_t cloud2){
    CloudPtr_t cloud = PC2toPC(cloud2);
    std::vector<int> inliers;
    CloudPtr_t final_cloud;
    pcl::SampleConsensusModelLine<Point3D>::Ptr model(new pcl::SampleConsensusModelLine<Point3D>(cloud));
    pcl::RandomSampleConsensus<Point3D> ransac(model);

    ransac.setDistanceThreshold(0.01f);
    ransac.computeModel();
    ransac.getInliers(inliers);
    pcl::copyPointCloud(*cloud, inliers, *final_cloud);

    return PCtoPC2(final_cloud);
}

//pcl::toPCLPointCloud2
//pcl::fromPCLPointCloud2
CloudPtr_t Mapper::PC2toPC(Cloud2Ptr_t source){
    CloudPtr_t result(new Cloud_t());
    pcl::fromPCLPointCloud2(*source, *result);
    return result;
}

Cloud2Ptr_t Mapper::PCtoPC2(CloudPtr_t source){
    Cloud2Ptr_t result(new Cloud2_t());
    pcl::toPCLPointCloud2(*source, *result);
    return result;
}

void Mapper::extractDescriptors(CloudPoint3DIPtr_t input, CloudPoint3DIPtr_t keypoints, CloudFeatureType_t features){

    pcl::PFHEstimation<Point3DI, Normal, FeatureType>::Ptr feature_extractor(new pcl::PFHEstimation<Point3DI, Normal, FeatureType>);
    //pcl::FeatureFromNormals<Point3DI, Normal, FeatureType>::Ptr feature_from_normals = std::dynamic_pointer_cast<pcl::FeatureFromNormals<Point3DI, Normal, FeatureType>>(feature_extractor);
    
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
        feature_extractor->setInputNormals(normals);    
    //}
    
    feature_extractor->compute(*features);
}

void Mapper::detect_harris3d_keypoints(const CloudPoint3DIPtr_t& input, const CloudPoint3DIPtr_t& keypoints){
    
    //pcl::HarrisKeypoint3D<Point3DRGB, Point3DI>* harris3D = new pcl::HarrisKeypoint3D<Point3DRGB, Point3DI>();
    //pcl::HarrisKeypoint3D<Point3DI, Point3DI>* harris3D = new pcl::HarrisKeypoint3D<Point3DI, Point3DI>();
    pcl::HarrisKeypoint3D<Point3DI, Point3DI>* harris3D = new pcl::HarrisKeypoint3D<Point3DI, Point3DI>(pcl::HarrisKeypoint3D<Point3DI, Point3DI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius(0.01f);
    harris3D->setRadiusSearch(0.01f);
    
    pcl::Keypoint<Point3DI, Point3DI>::Ptr keypoint_detector;
    keypoint_detector.reset(harris3D);
    keypoint_detector->setInputCloud(input);
    keypoint_detector->compute(*keypoints);

    /*
    std::cout << "Before conversion\n";
    pcl::PointCloud<Point3DRGB>::Ptr nube_rgb(new pcl::PointCloud<Point3DRGB>());
    pcl::PointCloud<Point3DI>::Ptr nube_I(new pcl::PointCloud<Point3DI>());
    pcl::fromPCLPointCloud2(*input, *nube_rgb);

    pcl::PointCloudXYZRGBtoXYZI(*nube_rgb, *nube_I);
    std::cout << nube_I->size() << "\n";
    std::cout << "After conversion\n";
    harris3D->setInputCloud(nube_I);

    harris3D->compute(*keypoints);*/
}