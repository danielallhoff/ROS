#include "mapeador.hpp"

Mapper::Mapper(ros::NodeHandle& nh){
    srand(time(NULL));
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUB_TARGET, 1);
    sub = nh.subscribe(SUB_TARGET, 1, &Mapper::PointCloudCallback, this);
}

void Mapper::PointCloudCallback(const sensor_msgs::PointCloud2& cloud){

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

CloudPtr_t Mapper::VoxelGridFilter(const CloudPtr_t cloud){
    CloudPtr_t cloud_filtered(new Cloud_t());
    pcl::VoxelGrid<Cloud_t> vgf;

    vgf.setInputCloud(cloud);
    vgf.setLeafSize(0.01f, 0.01f, 0.01f);
    vgf.filter(*cloud_filtered);

    return cloud_filtered;
}