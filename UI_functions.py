import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLAxisItem, GLTextItem
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QPushButton, QGroupBox, QVBoxLayout,  QGridLayout, QStackedWidget, QWidget, QColorDialog, QComboBox, QLineEdit, QFileDialog
from PyQt5.QtWidgets import QApplication, QTableWidget, QTableWidgetItem, QStyledItemDelegate
from PyQt5.QtGui import QFont, QVector3D, QColor
import os
from code import *
from code_help import *
from sklearn.neighbors import KDTree



def load_source_pointcloud(self):
    file_name, _ = QFileDialog.getOpenFileName(self,"Open File", "", "All Files (*.*)")
    if file_name:
        self.spc_name_label.setText("Source pointcloud : " + os.path.basename(file_name))
        self.src_filepath = file_name
        self.pcd = o3d.io.read_point_cloud(self.src_filepath)
        self.pcd_points = np.asarray(self.pcd.points)
        clrs_arr, self.labels_dict, self.labels_list = reaaad_plydata(self.src_filepath)

        self.pcd.colors = o3d.utility.Vector3dVector(np.asarray(clrs_arr))
        #o3d.visualization.draw_geometries([self.pcd])

        self.scatter.setData(pos=self.pcd_points, size=2, color=clrs_arr)
        display_labels(self,self.labels_dict)

        self.temp_filepath = "None"
        self.temp_pcd = None
        self.temp_points = None
        self.label_color = [1.0, 0.0, 0.0]
        self.label = "None"
        self.labeled_pc = None
        self.blkout = True
        self.fpfh_pc = None
        self.knn_pc = None
        self.dbscan_pc = None
        self.tmp_label.setText("No file found")
        self.k = self.repts_value1.currentText()
        self.clrr.setStyleSheet('')
        self.clrr1.setStyleSheet('')
        self.template_label_val.clear()
        self.template_label_val1.clear()
        self.temp_save_name.setText("Enter template name")

def show_boundingbox(self):
    self.widget.addItem(self.box)
    # Create GLAxisItem and add it to the GLBoxItem
    self.axis = GLAxisItem()
    self.axis.setSize(0.3, 0.3, 0.3)
    self.axis.setParentItem(self.box)
    self.axis.translate(*self.center)

    #add X,Y,Z axis annottaions and colours for reperesentation
    label_x = GLTextItem(text='X', color=QColor(0, 0, 255))
    label_y = GLTextItem(text='Y', color=QColor(255, 255, 0))
    label_z = GLTextItem(text='Z', color=QColor(0, 255, 0))

    label_z.setData(pos=np.asarray([0.0, 0.0, 0.1]))
    label_z.setParentItem(self.axis)
    label_y.setData(pos=np.asarray([0.0, 0.1, 0.0]))
    label_y.setParentItem(self.axis)
    label_x.setData(pos=np.asarray([0.1, 0.0, 0.0]))
    label_x.setParentItem(self.axis)

def reset_boundingbox(self):
    self.widget.removeItem(self.box)
    self.size = np.array([0.1, 0.1, 0.1])
    anchor = np.array([0, 0, 0])  # Anchor point of the box
    self.center = anchor + self.size / 2.0  # Calculate the center position of the box
    self.box = gl.GLBoxItem(size=QVector3D(*self.size), color=(255, 0, 0, 255), glOptions='opaque')
    self.show_bbox()

def move_boundingbox(self,dir):
    value = 1
    if dir == "+X":
        self.box.translate(value / 100, 0.0, 0.0, local=True)
    elif dir == "-X":
        self.box.translate(-value / 100, 0.0, 0.0, local=True)
    elif dir == "+Y":
        self.box.translate(0.0, value / 100, 0.0, local=True)
    elif dir == "-Y":
        self.box.translate(0.0, -value / 100, 0.0, local=True)
    elif dir == "-Z":
        self.box.translate(0.0, 0.0, -value / 100, local=True)
    elif dir == "+Z":
        self.box.translate(0.0, 0.0, value / 100, local=True)

def span_boundingbox(self,dim):
    if dim == "+len":
        self.size[0] = self.size[0] + 0.01
    elif dim == "-len":
        self.size[0] = self.size[0] - 0.01
    elif dim == "-wid":
        self.size[1] = self.size[1] - 0.01
    elif dim == "-ht":
        self.size[2] = self.size[2] - 0.01
    elif dim == "+wid":
        self.size[1] = self.size[1] + 0.01
    elif dim == "+ht":
        self.size[2] = self.size[2] + 0.01
    elif dim == "-sc":
        self.size[0] -= 0.01
        self.size[1] -= 0.01
        self.size[2] -= 0.01
    elif dim == "+sc":
        self.size[0] += 0.01
        self.size[1] += 0.01
        self.size[2] += 0.01

    self.box.setSize(self.size[0], self.size[1], self.size[2])

def get_box_center(self):
    transform = self.box.transform()
    size = self.box.size()
    # Calculate the center position based on the transformation matrix and box size
    center = QVector3D(0, 0, 0)
    for i in range(8):
        vertex = QVector3D(size[0] / 2, size[1] / 2, size[2] / 2)
        vertex = transform.map(vertex)
        center += vertex
    center /= 8
    return center

def calculate_box_vertices(self):
    center = get_box_center(self)
    size = self.box.size()
    half_size = [size[0] / 2, size[1] / 2, size[2] / 2]
    vertices = [
            [center.x() - half_size[0], center.y() - half_size[1], center.z() - half_size[2]],
            [center.x() - half_size[0], center.y() - half_size[1], center.z() + half_size[2]],
            [center.x() - half_size[0], center.y() + half_size[1], center.z() - half_size[2]],
            [center.x() - half_size[0], center.y() + half_size[1], center.z() + half_size[2]],
            [center.x() + half_size[0], center.y() - half_size[1], center.z() - half_size[2]],
            [center.x() + half_size[0], center.y() - half_size[1], center.z() + half_size[2]],
            [center.x() + half_size[0], center.y() + half_size[1], center.z() - half_size[2]],
            [center.x() + half_size[0], center.y() + half_size[1], center.z() + half_size[2]], ]
    return np.array(vertices)

def crop_temp_pts(self, vrtx_arr):
    bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(vrtx_arr))

    if self.blkout == False:
        print("hey! situ of blakout")

        pts = np.asarray(self.pcd.points)
        labels = self.labels_list
        unlabelled_indices = []
        for i in range(len(labels)):
            if labels[i] == 'None':
                unlabelled_indices.append(i)
        unlabelled_points = pts[unlabelled_indices]
        f_pts = bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(np.asarray(unlabelled_points)))
        fin_pts = []
        for i in f_pts:
            fin_pts.append(np.asarray(unlabelled_points)[i])
    else:
        f_pts = bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(np.asarray(self.pcd.points)))
        fin_pts = []
        for i in f_pts:
            fin_pts.append(np.asarray(self.pcd.points)[i])
    return np.asarray(fin_pts)

def crop_template(self):
    vrtx_arr = calculate_box_vertices(self)
    final_pts = crop_temp_pts(self,vrtx_arr)
    self.scatter.setData(pos=final_pts)
    self.temp_points = final_pts
    tree = cKDTree(np.asarray(self.pcd.points))
    distances, idxs = tree.query(np.asarray(final_pts))
    self.temp_pcd = self.pcd.select_by_index(idxs)
    self.widget.removeItem(self.box)

def save_template(self):
    o3d.io.write_point_cloud(self.temp_save_name.text(), self.temp_pcd)

def template_CROP_submit(self):
    if self.k == "1":
        self.labels_dict[(self.label)] = self.label_color
        print(self.labels_dict)
        pts = self.temp_points
        tree = cKDTree(np.asarray(self.pcd.points))
        distances, idxs = tree.query(np.asarray(pts))
        for i in idxs:
            self.labels_list[i] = self.label
        clrs_arr = np.array([self.labels_dict[key] for key in self.labels_list])

        self.scatter.setData(pos=np.asarray(self.pcd.points), size=2, color=clrs_arr)
        self.labeled_pc = o3d.geometry.PointCloud()
        self.labeled_pc.points = o3d.utility.Vector3dVector(np.asarray(self.pcd.points))
        self.labeled_pc.colors = o3d.utility.Vector3dVector(clrs_arr)
        self.labeled_pc.normals = self.pcd.normals
        display_labels(self,self.labels_dict)
    else:
        submit(self)

def submit(self):
    self.labels_dict[(self.label)] = self.label_color
    print(self.labels_dict)

    self.labeled_pc,self.labels_list, self.fpfh_pc, self.knn_pc, self.dbscan_pc = template_matching_Coluring(self.temp_pcd,
                                                                                                  self.pcd,
                                                                                                  self.label_color,
                                                                                                  self.label, self.labels_list)

    colors_list = np.array([self.labels_dict[key] for key in self.labels_list])
    self.scatter.setData(pos=np.asarray(self.pcd.points), color=np.asarray(colors_list))
    display_labels(self,self.labels_dict)

def load_template_pointcloud(self):
    file_name, _ = QFileDialog.getOpenFileName(self, "Open File", "", "All Files (*.*)")
    if file_name:
        self.tmp_label.setText("Template pointcloud : " + os.path.basename(file_name))
        self.temp_filepath = file_name
        self.temp_pcd = o3d.io.read_point_cloud(self.temp_filepath)
        self.temp_points = np.asarray(self.temp_pcd.points)
        self.scatter.setData(pos=self.temp_points)

def reload_and_label(self):
    self.temp_filepath = "None"
    self.temp_pcd = None
    self.temp_points = None
    self.tmp_label.setText("No file found")
    self.blkout = True
    self.pcd.colors = self.labeled_pc.colors

    self.k = self.repts_value1.currentText()

    self.label_color = [1.0, 0.0, 0.0]
    self.label = "None"
    self.clrr.setStyleSheet('')
    self.clrr1.setStyleSheet('')
    self.template_label_val.clear()
    self.template_label_val1.clear()
    self.temp_save_name.setText("Enter template name")

    clrs_arr = np.array([self.labels_dict[key] for key in self.labels_list])
    self.scatter.setData(pos=np.asarray(self.pcd.points), size=2, color=clrs_arr)
    display_labels(self,self.labels_dict)

def display_labels(self,labels_dictionary):
    row_count = len(labels_dictionary)
    self.table_widget.setRowCount(row_count)
    row = 0
    for label_id, rgb in labels_dictionary.items():
        label_item = QTableWidgetItem(str((label_id)))
        rgb_item = QTableWidgetItem()
        # Convert float RGB values to integers
        rgb_int = (int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255))
        rgb_item.setBackground(QColor(*rgb_int))
        self.table_widget.setItem(row, 0, label_item)
        self.table_widget.setItem(row, 1, rgb_item)
        row += 1
    self.table_widget.resizeColumnsToContents()
    self.table_widget.resizeRowsToContents()


def save_labelled_pc(self):
    name = self.eneter_labeled_pc_name_txt.text()
    write_ply_labels(self,self.labeled_pc,name ,self.labels_dict, self.labels_list )

def display_toggle_of_template(self):
    if self.blkout == True:
        pts = np.asarray(self.pcd.points)
        labels = self.labels_list
        unlabelled_indices = []
        for i in range(len(labels)):
            if labels[i] == 'None':
                unlabelled_indices.append(i)
        unlabelled_points = pts[unlabelled_indices]
        self.scatter.setData(pos=np.asarray(unlabelled_points), size=2, color=(255, 255, 255, 255))
        self.blkout = False
    else:
        clrs_arr = np.array([self.labels_dict[key] for key in self.labels_list])
        self.scatter.setData(pos=np.asarray(self.pcd.points), size=2, color=clrs_arr)
        self.blkout = True








