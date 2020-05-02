#!/usr/bin/env python

import pcl
import pcl.pcl_visualization
import ros_numpy
import rospy
import cv2 as cv
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class Mapeador:
    def __init__(self):
        self.vel_sub = rospy.Subscriber('/camera/depth/points',PointCloud2, self.callback)
        #Last point cloud extracted(filtered and keypoints)
        self.last_point_cloud = None
        #World point cloud
        self.M = None

    def voxel_grid_filter(self, point_cloud):
        '''Returns filtered point cloud using VoxelGrid'''
        sor = point_cloud.make_voxel_grid_filter()
        sor.set_leaf_size(0.01, 0.01, 0.01)
        cloud_filtered = sor.filter()
        return cloud_filtered

    def ransac(self, point_cloud):
        model = pcl.SampleConsensusModelLine(point_cloud)
        ransac = pcl.RandomSampleConsensus(model)
        ransac.set_DistanceThreshold(0.01)
        ransac.computeModel()
        inliers = ransac.get_Inliers()
        len_inliers = len(inliers)
        if len_inliers != 0:
            finalpoints = np.zeros((len_inliers, 3), dtype=np.float32)
            for i in range(0, len_inliers):
                finalpoints[i][0] = point_cloud[inliers[i]][0]
                finalpoints[i][1] = point_cloud[inliers[i]][1]
                finalpoints[i][2] = point_cloud[inliers[i]][2]
        final = pcl.PointCloud()
        final.from_array(finalpoints)
        return final


    def extract_sift(self,cloud, cloud_normals):
        min_scale = 0.01
        n_octaves = 3
        n_scales_per_octave = 4
        min_contrast = 0.001
        tree = cloud.make_kdtree()
        sift = cloud_makeSIFTKeypoint()
        sift.set_SearchMethod(tree)
        sift.set_Scales(min_scale, n_octaves, n_scales_per_octave)
        sift.set_MinimumContrast(0.00)
        result = sift.compute()        
        return result

    def extract_normals(self,cloud):
        ne = cloud.make_NormalEstimation()
        tree = cloud.make_kdtree()
        ne.set_SearchMethod(tree)
        ne.set_RadiusSearch(0.5)
        
        cloud_normals = ne.compute()

        return cloud_normals

    def pair_key_points(self, cloud, cloud_next):
        pass

    def callback(self, pointcloud):
        #Point cloud in ros point cloud format
        pc = ros_numpy.numpify(pointcloud)
        points = np.zeros((pc.shape[0],3))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        #Point cloud in pcl format
        p = pcl.PointCloud(np.array(points, dtype= np.float32))

        
        if self.last_point_cloud is None:
            self.last_point_cloud = p
        #Pair pointclouds
        else:
            #Extract keypoints new point cloud
            p_filtered = self.voxel_grid_filter(p)
            #Extract normals
            cloud_normals = self.extract_normals(p_filtered)
            #Extract key points
            key_points = self.extract_sift(p_filtered, cloud_normals)
            




def main():
    print("Start")

    mapper = Mapeador()
    
    rospy.init_node('RobotMapper', anonymous=True)
    try:
        print("Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()