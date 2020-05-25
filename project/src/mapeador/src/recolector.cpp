#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

constexpr unsigned NUM_ZEROS { 3 };
unsigned num_clouds = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr ConvertCloud2To3DI(const pcl::PCLPointCloud2::Ptr& source){
    pcl::PointCloud<pcl::PointXYZI>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::fromPCLPointCloud2(*source, *nube_rgb);
    pcl::PointCloudXYZRGBtoXYZI(*nube_rgb, *converted_cloud);

    return converted_cloud;   
}

std::string FormatName(unsigned zeros, unsigned num){
    std::string numero(std::to_string(num));
    std::string res = std::string(std::max(0, (int)(zeros-numero.length())),'0').append(numero);
    return res;
}

void Callback(const pcl::PCLPointCloud2::Ptr cloud){
    std::cout << "Received pointcloud no. " << num_clouds << ".\n"; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr converted_cloud = ConvertCloud2To3DI(cloud);

    pcl::io::savePCDFileASCII("/home/alfonso/ROS/pc" + FormatName(NUM_ZEROS, num_clouds) + ".pcd", *converted_cloud);
    ++num_clouds;
}

void bucle(unsigned iterations){
    ros::Rate rate(0.05f);

    while(iterations-num_clouds){
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "mapeador_recolector");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, Callback);
    
    bucle(200);
    
    return 0;
}