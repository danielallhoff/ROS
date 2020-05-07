#!/usr/bin/env python

import pcl
import pcl.pcl_visualization
import ros_numpy
import rospy
import cv2 as cv
import numpy as np
import struct
import ctypes
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class Mapeador:
    def __init__(self):
        self.vel_sub = rospy.Subscriber('/camera/depth/points',PointCloud2, self.callback)
        #Last point cloud extracted(filtered and keypoints)
        self.last_key_points = None
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


    def extract_harris(self,cloud):
        min_scale = 0.01
        n_octaves = 3
        n_scales_per_octave = 4
        min_contrast = 0.001
        
        harris3d = cloud.make_HarrisKeypoint3D()
        #print(dir(harris3d))
        harris3d.setRadius(0.01)
        harris3d.set_RadiusSearch(0.01)
        harris3d.set_NonMaxSupression(True)
        result = harris3d.compute()        
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
    #https://answers.ros.org/question/344096/subscribe-pointcloud-and-convert-it-to-numpy-in-python/
    
    '''def pc2_to_numpy(self, pointcloud):
        gen = pc2.read_points(pointcloud, skip_nans = True)
        int_data = list(gen)
        xyz  = np.array([[0,0,0]])
        rgb  = np.array([[0,0,0]])
        for x in int_data:
            test = x[3] 
           
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)
        return xyz, rgb'''
    
    def callback(self, pointcloud):
        
        #Obtain numpy points from pointcloud
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pointcloud)
        #xyz, rgb = self.pc2_to_numpy(pointcloud)
        
        #Point cloud in pcl format
        p = pcl.PointCloud(np.array(xyz, dtype= np.float32))

        
        if self.last_key_points is None:
            self.last_key_points = p
        #Pair pointclouds
        else:
            print("Calculando")
            #Extract keypoints new point cloud
            p_filtered = self.voxel_grid_filter(p)
            #Extract normals for sift
            #cloud_normals = self.extract_normals(p_filtered)
            #Extract key points
            key_points = self.extract_harris(p_filtered)
            
            #Pair last_point_cloud with key_points

            




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