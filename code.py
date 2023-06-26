"""Python module which takes the source and template points clods and do the template matching"""

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import statistics
import pandas as pd
from sklearn.preprocessing import StandardScaler  # to standardize the features
from sklearn.decomposition import PCA
from itertools import chain
import math
import copy,itertools
from matplotlib import pyplot as plt
from plyfile import PlyData, PlyElement
from code_help import *



def NormalizeData(data):
    """Function to normalise the points data of a point cloud
    :params
        -data : numpy array
    """
    return (data - np.min(data)) / (np.max(data) - np.min(data))

def pcr_caluclator(scene_coords):
    """Function to calculate average point cloud resolution (PCR)
    :params
        - scene_coords : numpy array of x,y,z coordinates of all points of pointcloud.
                        Dimensions = (N,3) #N is number of points in pointcloud

    """
    feat1tree = cKDTree(scene_coords)
    dists, nn_inds = feat1tree.query(scene_coords, k=2)
    dists_lis = []
    for i in range(len(dists)):
        dists_lis.append(dists[i][1])
    res = statistics.pstdev(dists_lis)
    return res

def extract_fpfh(pcd, pcr):
    """Function to calculate FPFH feature descriptors.
    :params
        - pcd : point cloud whose FPFH has to be determined
    """
    #radius_normal = pcr*100
    radius_normal = 0.1
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    #radius_feature = pcr*1000
    radius_feature = 0.25
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return np.array(fpfh.data).T

def fpfh_visualise(scene_feats,pcd):
    """Function to visualise the FPFH features
    :params
        -scene_feats : numpy array of dimension (N,33). FPFH features calculated are 33 length vector for each point
        - pcd : point cloud whose FPFH are calculated.

    This functions reduces 33 dimensions of FPFH to 3 dimension with Principal component reduction.
    These values are passed a values of colors to pointclouds and visualised
    """
    total_kp_feats_arr = np.array(scene_feats)
    df = pd.DataFrame(total_kp_feats_arr)
    scalar = StandardScaler()
    scaled_data = pd.DataFrame(scalar.fit_transform(df))  # scaling the data

    pca = PCA(n_components=3)
    # pca = PCA()
    pca.fit(scaled_data)
    data_pca = pca.transform(scaled_data)
    # data_pca = pd.DataFrame(data_pca)
    data_pca = np.array(data_pca)
    #print(data_pca)
    #print("after normalisation")
    data_pca = NormalizeData(data_pca)
    #print(data_pca)
    new_pcl = o3d.geometry.PointCloud()
    new_pcl.points = pcd.points
    new_pcl.colors = o3d.utility.Vector3dVector(data_pca[:, :3])
    #o3d.visualization.draw_geometries([new_pcl])
    return new_pcl


def knn(temp_feats, scene_feats, k):
    """Function to calculate K nearest neighbors for points of template in points of source point cloud in feature space (FPFH)
    :params
        - temp_feats : numpy array of dimension (M,33). FPFH feature descriptors of template point cloud
        - scene_feats : numpy array of dimension (N,33). FPFH feature descriptors of scene point cloud
        - K : int. Number of nearest neighbors per point of template.
         M is the number of points in template point cloud
         N is the number of points in source point cloud
        """
    feat1tree = cKDTree(scene_feats)
    dists, nn_inds = feat1tree.query(temp_feats, k=k)
    return nn_inds

def keypoint_features(kp, feats):
    """Calculate FPFH features of all keypoints
    :params
        kp: numpy array of size (N,3). N is number of keypoints
        feats : numpy array of size (M,33). M is total number of points in source point cloud

    """
    kp = np.array(kp)  # Convert kp to a NumPy array
    kp_feats = feats[kp]  # Use NumPy array indexing to select the features
    return kp_feats


def icp_knn(temp,pcd,transformation):
    """Function to calculate how good the ICP Registration result is.
    :params
        - temp : template point cloud
        - pcd : source point cloud
        - transformation : 4X4 rigid transformation matrix from ICP Registration.

        This functions calculates the average of nearest neighbour distances of the transformed template in the source point cloud.
        If the vales are too high then it indicates bas ICP Registration. Too low values represent perfect Registration.

        """

    temp_copy = copy.deepcopy(temp)
    temp_copy.transform(transformation)

    pcd_arr = np.asarray(pcd.points)
    temp_Arr = np.asarray(temp_copy.points)
    tree = cKDTree(pcd_arr)
    dists, nn_inds = tree.query(temp_Arr, k=1)
    res = sum(map(lambda i: i * i, dists))
    final = res/len(dists)
    return final

def draw_registration_result(source, target, transformation):
    """ Function to visualise the performance of ICP registration.
    After ICP registration, pass the final transformation to visualise the result
     :params
        - source : template point cloud
        - target : source point cloud
        - transformation : 4X4 rigid transformation matrix from ICP Registration.

    This transformation is applied on template point cloud and visualised
       """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # target_temp.transform(reg_p2l.transformation)
    #aabb = source_temp.get_oriented_bounding_box()
    aabb = source_temp.get_axis_aligned_bounding_box()
    #print(aabb.get_box_points())
    # aabb=o3d.geometry.AxisAlignedBoundingBox.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    #o3d.visualization.draw_geometries([source_temp])
    o3d.visualization.draw_geometries([source_temp, target_temp, aabb])

def final_colured_pcd_vis(transformations,temp_pcd,pcd,label_colour,label_txt,label_lst):
    """Function which takes all the transformations for a template and uses the help of bounding box on transformed
    template to get all the points of 3D template matching.
    :params
        - transformations : list of size c. Each element is a 4X4 rigid transformation matrix.
                            c is the number of repetitions of the template estimated.
        - temp_pcd : template point cloud
        - pcd : source point cloud
        - label_color : list of size 3 with RGB float values
        - label_txt : str
        - label_list : list of strings of size (N,) where N is numbers of points in pcd

    """
    Points_to_colour = []
    bb_list = []
    for trans in transformations:
        src1 = copy.deepcopy(temp_pcd)
        src1.transform(trans)
        aabb1 = src1.get_axis_aligned_bounding_box()
        aabb1.color = [1, 0, 0]
        bb_list.append(aabb1)
        pts = aabb1.get_point_indices_within_bounding_box(pcd.points)
        Points_to_colour.append(pts)
    #return Points_to_colour
    pts_to_color = list(itertools.chain.from_iterable(Points_to_colour))
    #print(pts_to_color)
    print("i am here")
    print(label_lst)
    print(len(label_lst))
    new_pcl = o3d.geometry.PointCloud()
    new_pcl.points = pcd.points
    new_pcl.normals = pcd.normals
    new_pcl_color = np.asarray(pcd.colors)
    for i in pts_to_color:
        new_pcl_color[i] = label_colour
        label_lst[i] = label_txt

    new_pcl.colors = o3d.utility.Vector3dVector(new_pcl_color)
    o3d.visualization.draw_geometries([new_pcl])

    return new_pcl,label_lst

def template_matching_Coluring(temp_pcd,pcd, label_colour,label_txt, label_list):
    """Function to find 3D template matches in source point cloud
    :params
        - temp_pcd : template point cloud
        - pcd : source point cloud
        - label_color : list of size 3 with RGB float values
        - label_txt : str
        - label_list : list of strings of size (N,) where N is numbers of points in pcd """

    """visualise Point Clouds"""
    #o3d.visualization.draw_geometries([pcd])
    #o3d.visualization.draw_geometries([temp_pcd])

    """Total number of points of Point Clouds"""
    pcd_points_arr = np.array(pcd.points)
    temp_pcd_points_arr = np.asarray(temp_pcd.points)
    no_of_pcd_points = len(pcd_points_arr)
    no_of_temp_pcd_points = len(temp_pcd_points_arr)
    temp_pts_mean =np.mean(temp_pcd_points_arr,axis=0)
    print(no_of_temp_pcd_points)

    tree = cKDTree(pcd_points_arr)
    distances, temp_pcd_indices = tree.query(temp_pcd_points_arr)  #temp_pcd_indices is a ndarray which stores the indices of pcd that belong to temp pcd


    pcr = pcr_caluclator(pcd_points_arr)
    print("pcr = ", pcr)

    """visualise FPFH """
    scene_feats = extract_fpfh(pcd, pcr)
    fpfh_pointcloud = fpfh_visualise(scene_feats, pcd)
    #o3d.visualization.draw_geometries([fpfh_pointcloud])

    temp_fpfh_features = keypoint_features(temp_pcd_indices,scene_feats)
    #fpfh_visualise(temp_fpfh_features,temp_pcd)

    """KNN"""
    knn_indexes = knn(temp_fpfh_features,scene_feats,k=50)
    all_knn_indices = list(chain.from_iterable(knn_indexes))

    knn_pointcloud = pcd.select_by_index(all_knn_indices)
    knn_pointcloud.paint_uniform_color([1, 1, 1])

    knn_point_cloud = pcd.select_by_index(all_knn_indices)
    pcr_for_knn = pcr_caluclator(np.asarray(knn_point_cloud.points))
    print("pcr for knn = ",pcr_for_knn)
    #o3d.visualization.draw_geometries([knn_pointcloud])

    """DBSCAN"""
    db_clust_points = int(no_of_temp_pcd_points * 2 / 100)
    db_eps = pcr_for_knn*3
    print("db_clust_points = ",db_clust_points)

    if no_of_temp_pcd_points < 1000:
        dbscan_est = 0.0135
    else:
        dbscan_est = pcr_for_knn * 0.035 / 0.004

    print(" db_eps",db_eps)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            knn_point_cloud.cluster_dbscan(eps=dbscan_est, min_points=db_clust_points, print_progress=True))
    #print(labels)
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors = colors[:, :3]
    colors[labels < 0] = 0
    knn_point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #print(len(new_pcl.points))
    dbscan_pc = knn_point_cloud
    #o3d.visualization.draw_geometries([dbscan_pc])

    #print(labels)
    dbscan_cluster_without_noise_indices = []
    for i in range(len(labels)):
        if labels[i] >= 0:
            dbscan_cluster_without_noise_indices.append(i)
    dbscan_cluster_without_noise_indices = np.asarray(dbscan_cluster_without_noise_indices)
    #print(dbscan_cluster_without_noise_indices)
    nice_dbscan = knn_point_cloud.select_by_index(dbscan_cluster_without_noise_indices)
    # o3d.visualization.draw_geometries([nice_dbscan])

    """Correspondence matchings"""
    final_list_transformations = []
    distances, dbscan_indices = tree.query(np.asarray(knn_point_cloud.points))

    cluster_indices = [[] for _ in range(max_label+1)] #cluster_indices = [ [], [], [] ] nested list of cluster indices
    for i, value in enumerate(labels):
        if value >= 0:
            cluster_indices[value].append(dbscan_indices[i])
    #cluster_indices = [np.where(labels == i)[0].tolist() for i in range(max_label+1)]  #cluster_indices = [ [], [], [] ] nested list of cluster indices
    cluster_min_vals = []
    cluster_min_val_transformations = []
    for i in range(len(cluster_indices)):
        cluster_i = pcd.select_by_index(cluster_indices[i])
        #o3d.visualization.draw_geometries([cluster_i])
        cluster_feats = keypoint_features(cluster_indices[i], scene_feats)
        clust_feats_tree = cKDTree(cluster_feats)
        dists, indxs = clust_feats_tree.query(temp_fpfh_features, k=1)
        correspondence_indxs_li = np.asarray(cluster_indices[i])[np.array(indxs)]
        correspondence_indxs = correspondence_indxs_li.tolist() #correspondence_indxs is list of pcd indexes
        print("number of points in cluster ",i," is = ",len(cluster_indices[i]))
        if len(cluster_indices[i]) < (no_of_temp_pcd_points*0.25):
            print("bad cluster")
            continue
        else:
            print("good cluster")
            #correspondence_cluster = pcd.select_by_index(correspondence_indxs)
            #o3d.visualization.draw_geometries([correspondence_cluster])

            """findind seed translation and rotation & ICP"""
            correspondence_pts = pcd_points_arr[correspondence_indxs]
            correspondence_pts_mean = np.mean(correspondence_pts,axis=0)

            """set of rotations"""
            angle_step = (2 * np.pi) / 16
            transformations = []
            for i in range(16):
                angle = angle_step * (i + 1)  # Example rotation of 45 degrees
                rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz((0, angle, 0))
                #print(rotation_matrix)
                translation = correspondence_pts_mean - np.dot(rotation_matrix, temp_pts_mean)
                #print(translation)
                FR = np.column_stack((rotation_matrix, translation))
                hr = np.asarray([[0., 0., 0., 1.]])
                R_T = np.vstack((FR, hr))
                transformations.append(R_T)
            #now i have list of transformations for a cluster
            icp_transformations = []
            icp_filter_distances_lst = []
            for transformation_t in transformations:
                reg_p2p = o3d.pipelines.registration.registration_icp(
                    temp_pcd, pcd, 0.02, transformation_t,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane())
                t = reg_p2p.transformation
                icp_transformations.append(t)
                icp_filter_dist = icp_knn(temp_pcd, pcd, t)
                icp_filter_distances_lst.append(icp_filter_dist)
            #done all icp transformations for 1 cluster
            #now get correct transformations from 20
            min_val = min(icp_filter_distances_lst)
            min_idx = icp_filter_distances_lst.index(min_val)
            final_t = icp_transformations[min_idx]
            print("min val = ",min_val)
            #draw_registration_result(temp_pcd, pcd, final_t)
            cluster_min_vals.append(min_val)
            cluster_min_val_transformations.append(final_t)
            #final_list_transformations.append(final_t)


    sorted_cluster_min_vals = sorted(cluster_min_vals)
    exponent = math.floor(math.log10(sorted_cluster_min_vals[1]))
    exponent = exponent + 1
    val = 1 * (10 ** (exponent))
    print("value == ", val)
    for i in range(len(cluster_min_vals)):
        if cluster_min_vals[i] <= val:
            final_list_transformations.append(cluster_min_val_transformations[i])

    final_pcd , final_label_list = final_colured_pcd_vis(final_list_transformations, temp_pcd, pcd, label_colour, label_txt, label_list)
    return final_pcd, final_label_list,fpfh_pointcloud,knn_pointcloud,dbscan_pc

