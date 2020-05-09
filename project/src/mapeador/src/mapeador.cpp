#include "mapeador.hpp"

Mapper::Mapper(ros::NodeHandle& nh){
    srand(time(NULL));
    pub = nh.advertise<Cloud2_t>(PUB_TARGET, 1);
    sub = nh.subscribe(SUB_TARGET, 1, &Mapper::PointCloudCallback, this);
}

void Mapper::PointCloudCallback(const Cloud2Ptr_t cloud){
    Cloud2Ptr_t filtered_point_cloud = VoxelGridFilter(cloud);
    CloudPoint3DIPtr_t keypoints(new CloudPoint3DI_t());
    extract_harris3d_keypoints(filtered_point_cloud, keypoints);
    
    //When already exists keypoints, pair them
    if(last_keypoints_cloud != NULL){
        std::vector<std::pair<unsigned, unsigned>> correspondences;
        findCorrespondences(last_keypoints_cloud, keypoints, correspondences);
        

        //Filter correspondences using Random sample Consensus
        pcl::registration::CorrespondenceRejectorSampleConsensus<Point3DI> rejectorRANSAC;
        rejectorRANSAC.setInputSource(last_keypoints_cloud);
        rejectorRANSAC.setInputTarget(keypoints);
        rejectorRANSAC.setInputCorrespondences(correspondences);
        rejectorRANSAC.getCorrespondences(*correspondences);

        //Determine nitial transformation
        pcl::registration::TransformationEstimation<Point3DI, Point3DI>::Ptr transform_estimation(new pcl::registration::TransformationEstimationSVD<Point3DI, Point3DI>);
        
        Eigen::Matrix4f initial_transfomation_matrix;
        Eigen::Matrix4f transfomation_matrix;
        
        transformation_estimation->estimateRigidTransformation(*last_keypoints_cloud, *keypoints, *corespondences);

        CloudPoint3DRGBPtr_t source_segmented(new CloudPoint3DRGB_t());
        CloudPoint3DRGBPtr_t target_segmented(new CloudPoint3DRGB_t());

        pcl::transformPointCloud(source_segmented,target_segmented , initial_transformation_matrix);
        //Determine final transformation and add to the final cloud
        pcl::Registration<Point3DRGB, Point3DRGB>::Ptr registration(new pcl::IterativeClosestPoint<Point3DRGB, Point3DRGB>());
        registration.setInputSource(/*source_transformed*/);
        registration.setInputTarget(/*target_transformed*/);
        registration.setMaxCorrespondenceDistance(0.05f);
        registration.setRANSACOutlierRejectionThreshold(0.05f);
        registration.setTransformationEpsilon(0.000001f);
        registration.setMaximumIterations(1000);
        registration.align(/*source_registered*/);
        transfomation_matrix = registration.getFinalTransformation();
        
    }else{
        last_keypoints_cloud = keypoints;
        
    }
}

constexpr int k { 1 };
//Find correspondences between both clouds. The result is a vector correspondences of int, associating which point from the cloud1 corresponds to which from cloud2
void Mapper::findCorrespondences(const CloudPoint3DIPtr_t source,  const CloudPoint3DIPtr_t target, std::vector<std::pair<unsigned, unsigned>>& correspondences){
    pcl::KdTreeFLANN<Point3DI> descriptor;
    descriptor.setInputCloud(target);

    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for(std::size_t i = 0; i < source->size(); ++i){
        //Find the nearest target point to source
        descriptor.nearestKSearch(*source, i, k,k_indices, k_squared_distances);
        correspondences.push_back(std::make_pair(i,k_indices[0]));
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

Cloud2Ptr_t Mapper::VoxelGridFilter(const Cloud2Ptr_t cloud){
    Cloud2Ptr_t cloud_filtered(new Cloud2_t());
    pcl::VoxelGrid<Cloud2_t> vgf;

    vgf.setInputCloud(cloud);
    vgf.setLeafSize(0.01f, 0.01f, 0.01f);
    vgf.filter(*cloud_filtered);

    return cloud_filtered;
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

void Mapper::extract_harris3d_keypoints(const Cloud2Ptr_t input, const CloudPoint3DIPtr_t keypoints){
    //pcl::HarrisKeypoint3D<Point3DRGB, Point3DI>::HARRIS
    pcl::HarrisKeypoint3D<Point3DRGB, Point3DI>* harris3D = new pcl::HarrisKeypoint3D<Point3DRGB, Point3DI>();
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius(0.01f);
    harris3D->setRadiusSearch(0.01f);
    pcl::PointCloud<Point3DRGB>::Ptr nube(new pcl::PointCloud<Point3DRGB>());
    pcl::fromPCLPointCloud2(*input, *nube);
    harris3D->setInputCloud(nube);
    //pcl::Keypoint<Point3DRGB, Point3DI>::PointCloudOut
    harris3D->compute(*keypoints);
}