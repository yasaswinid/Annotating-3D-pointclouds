"""Python module to read and write point clouds from and to .ply files respectively"""

import open3d as o3d
import numpy as np
from plyfile import PlyData, PlyElement
#from collections import defaultdict


def reaaad_plydata(src_filepath):
    """Function to read a point cloud from .ply file path
    :params
        -src_filepath : str.
    """
    pcd = o3d.io.read_point_cloud(src_filepath)
    flg = False
    labels_list = []
    labels_dict = {"None": (1.0, 1.0, 1.0)}

    plydata = PlyData.read(src_filepath)
    vertex_data = plydata['vertex']
    attribute_names = [prop.name for prop in vertex_data.properties]

    for name in attribute_names:
        if name == 'label':
            flg = True
            break

    if flg == True:
        print("flag is true")
        label = np.array(vertex_data['label'])
        labels_list = encode(label)
        #print(labels_list)
        red = np.array(vertex_data['red'])
        green = np.array(vertex_data['green'])
        blue = np.array(vertex_data['blue'])
        colors_lst = np.column_stack((red, green, blue))

        C = {}
        for key, value in zip(labels_list, colors_lst):
            C[key] = value

        # Convert C to a dictionary
        labels_dict = dict(C)
        labels_dict["None"] = (1.0, 1.0, 1.0)
    else:
        labels_list = np.full(100000, 'None', dtype=object)

    #print("labels_dicitionay = ", labels_dict)

    colors_list = np.array([labels_dict[key] for key in labels_list])
    #print("length of colors list = ",len(colors_list))
    #print("colours list = " , colors_list)

    return np.asarray(colors_list),labels_dict,np.asarray(labels_list)

def write_ply_labels(self,labelledPcd,filename, final_label_lst, labels_dict):
    """ Function to stored labelled point cloud
    :Params
        - labelledPcd = 3D point cloud
        - filename = String
        - self = parent class
    """
    coordinates = np.asarray(labelledPcd.points)
    colors_list = np.array([self.labels_dict[key] for key in self.labels_list])
    colors = colors_list
    normals = np.asarray(labelledPcd.normals)
    alpha = [1.0]*len(coordinates)
    final_label_lst = decode(self.labels_list)
    prop = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('nx', 'f4'), ('ny', 'f4'), ('nz', 'f4'), ('red', 'f4'),
            ('green', 'f4')
        , ('blue', 'f4'), ('alpha', 'f4'), ('label', 'i4', (15,))]
    vertex_all = np.empty(len(coordinates), dtype=prop)
    for i_prop in range(0, 3):
        vertex_all[prop[i_prop][0]] = coordinates[:, i_prop]
    for i_prop in range(0, 3):
        vertex_all[prop[i_prop+3][0]] = normals[:, i_prop]
    for i_prop in range(0, 3):
        vertex_all[prop[i_prop + 6][0]] = colors[:, i_prop]
    vertex_all[prop[9][0]] = alpha
    vertex_all[prop[10][0]] = final_label_lst

    ply = PlyData([PlyElement.describe(vertex_all, 'vertex')], text=True)
    ply.write(filename)
    print("DONE")


def decode(labels_lst):
    """Function to convert a list of labels to a numpy array of ints
    params:
        -labels_lst : list of string of size N. N is number of points in source point cloud

    :return - a 2D numpy array of size (N,15).

    Each value of labels_lst is a string. This string is converted into a 1D array of size (1,15) by converting letters
    to ASCII values
    """
    final_lab_lst = []
    for label in labels_lst:
        lab_idx = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        for i in range(len(label)):
            lab_idx[i] = ord(label[i])
        final_lab_lst.append(lab_idx)
    #print(final_lab_lst)
    return np.asarray(final_lab_lst)

def encode(labels_lst):
    """Function to convert a 2D array to a list of labels
    :params
        -labels_lst : 2D numpy array of size (N,15). N is number of points in source point cloud

    :return - list of strings (labels)
        """
    labels_lst_final = []
    for label_list in labels_lst:
        string = ""
        for val in label_list:
            if val > 0:
                string = string+chr(val)
            else:
                break
        labels_lst_final.append(string)

    return labels_lst_final