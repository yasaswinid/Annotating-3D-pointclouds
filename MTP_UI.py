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
from UI_functions import *

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.src_filepath = "None"
        self.temp_filepath = "None"
        self.pcd_points = None
        self.pcd = None
        self.blkout = True
        self.temp_pcd = None
        self.temp_points = None
        self.k = ">1"
        self.label_color = [1.0, 0.0, 0.0]
        self.label = "None"
        self.labels_dict = {}
        self.labels_list = []
        self.labeled_pc = None

        self.fpfh_pc = None
        self.knn_pc = None
        self.dbscan_pc = None


    def initUI(self):
        layout = QGridLayout()
        self.setLayout(layout)

        # create the view widget for 3D point clouds
        self.widget = gl.GLViewWidget()
        self.pcd_points = np.random.normal(size=(1, 3))
        self.scatter = gl.GLScatterPlotItem(pos=self.pcd_points, size=2)
        self.widget.addItem(self.scatter)
        self.widget.opts['distance'] = 1

        # create the bounding box
        self.size = np.array([0.1, 0.1, 0.1])
        anchor = np.array([0, 0, 0])  # Anchor point of the box
        self.center = anchor + self.size / 2.0  # Calculate the center position of the box
        self.box = gl.GLBoxItem(size=QVector3D(*self.size), color=(255, 0, 0, 255), glOptions='translucent')

        """Source Point Cloud UI box"""
        #UI button controls for loading source 3D point cloud
        self.label1 = QLabel('Source Point Cloud')
        self.label1.setAlignment(QtCore.Qt.AlignCenter)
        self.label1.setStyleSheet('font-size: 16px;color:red; ')
        font = QFont()
        font.setWeight(QFont.Bold)
        self.label1.setFont(font)
        self.load_spc_btn = QPushButton('Load Source Points Cloud')
        self.spc_name_label = QLabel('No file found')
        self.spc_name_label.setAlignment(QtCore.Qt.AlignCenter)
        self.load_spc_btn.clicked.connect(self.open_file_src)

        #UI layout for loading source 3D point cloud
        group_box = QGroupBox()
        group_box_layout = QVBoxLayout()
        group_box.setLayout(group_box_layout)
        group_box_layout.addWidget(self.label1)
        group_box_layout.addWidget(self.load_spc_btn)
        group_box_layout.addWidget(self.spc_name_label)
        group_box.setStyleSheet('QGroupBox { background-color: lightgrey; }')

        """ Template Point Cloud UI box """
        self.label2 = QLabel('Template : ')
        self.label2.setFont(font)
        self.label2.setStyleSheet('font-size: 16px;color:red; ')
        self.temp_crop_button = QPushButton('Crop')
        #self.temp_crop_button.clicked.connect(lambda: self.set_mode("crop"))
        self.temp_crop_button.setStyleSheet('background-color: #BDB76B; ')
        self.temp_load_button = QPushButton('Load')
        #self.temp_load_button.clicked.connect(lambda: self.set_mode("load"))
        self.temp_load_button.setStyleSheet('background-color: #E9967A; ')

        """ Template CROP Tab"""
        #UI button controls for cropping 3D point cloud
        self.blkout_btn = QPushButton('TOGGLE')
        self.blkout_btn.clicked.connect(self.black_out)
        self.pick_bb = QPushButton('Pick a bounding box')
        self.pick_bb.clicked.connect(self.show_bbox)
        self.reset_bb = QPushButton('Reset bounding box')
        self.reset_bb.clicked.connect(self.reset_bbox)
        self.X_plus = QPushButton('X+')
        self.X_minus = QPushButton('X-')
        self.Z_plus = QPushButton('Z+')
        self.Z_minus = QPushButton('Z-')
        self.Y_minus = QPushButton('Y-')
        self.Y_plus = QPushButton('Y+')
        self.Scale_plus = QPushButton('Scale+')
        self.Scale_minus = QPushButton('Scale-')
        self.X_plus.clicked.connect(lambda: self.move_bbox("+X"))
        self.X_minus.clicked.connect(lambda: self.move_bbox("-X"))
        self.Z_plus.clicked.connect(lambda: self.move_bbox("+Z"))
        self.Z_minus.clicked.connect(lambda: self.move_bbox("-Z"))
        self.Y_minus.clicked.connect(lambda: self.move_bbox("-Y"))
        self.Y_plus.clicked.connect(lambda: self.move_bbox("+Y"))
        self.Scale_plus.clicked.connect(lambda: self.span_bbox("+sc"))
        self.Scale_minus.clicked.connect(lambda: self.span_bbox("-sc"))
        self.length_lab = QLabel('length(X)')
        self.len_plus = QPushButton('+')
        self.len_minus = QPushButton('-')
        self.width_lab = QLabel('width(Y)')
        self.wid_plus = QPushButton('+')
        self.wid_minus = QPushButton('-')
        self.height_lab = QLabel('height(Z)')
        self.hgt_plus = QPushButton('+')
        self.hgt_minus = QPushButton('-')
        self.len_plus.clicked.connect(lambda: self.span_bbox("+len"))
        self.len_minus.clicked.connect(lambda: self.span_bbox("-len"))
        self.wid_plus.clicked.connect(lambda: self.span_bbox("+wid"))
        self.wid_minus.clicked.connect(lambda: self.span_bbox("-wid"))
        self.hgt_plus.clicked.connect(lambda: self.span_bbox("+ht"))
        self.hgt_minus.clicked.connect(lambda: self.span_bbox("-ht"))
        self.CROP_BTN = QPushButton('CROP')
        self.CROP_BTN.clicked.connect(self.crop_temp)
        self.CROP_BTN.setStyleSheet('background-color: lightblue; ')
        self.temp_save_name = QLineEdit("Enter template name")
        self.save_temp_btn = QPushButton('SAVE')
        self.save_temp_btn.clicked.connect(self.save_temp)
        self.temp_reps1 = QLabel('Est. Template Repetitons')
        self.repts_value1 = QComboBox(self)
        self.repts_value1.addItem("1")
        self.repts_value1.addItem(">1")
        self.repts_value1.setCurrentIndex(1)
        # Connect the currentIndexChanged signal to a slot
        self.repts_value1.currentIndexChanged.connect(self.on_combobox_changed)
        self.template_label = QLabel('Enter Label')
        self.template_label_val = QLineEdit()
        self.pick_clr1 = QPushButton('Pick the color')
        self.clrr1 = QPushButton()
        self.pick_clr1.clicked.connect(self.openColorPicker1)
        self.temp_crop_submit_btn = QPushButton('Submit')
        self.temp_crop_submit_btn.clicked.connect(self.temp_crop_submit)

        # UI layout for cropping template from 3D source point cloud
        group_box_crop = QGroupBox()
        group_box_crop_layout = QGridLayout()
        group_box_crop.setLayout(group_box_crop_layout)
        group_box_crop.setStyleSheet(
            'QGroupBox {background-color: #BDB76B; }')

        group_box_crop_layout.addWidget(self.blkout_btn, 0, 0, 1, 1)
        group_box_crop_layout.addWidget(self.pick_bb, 0, 1, 1, 1)
        group_box_crop_layout.addWidget(self.reset_bb, 0, 2, 1, 1)
        group_box_crop_layout.addWidget(self.X_minus, 1, 0)
        group_box_crop_layout.addWidget(self.Z_plus, 1, 1)
        group_box_crop_layout.addWidget(self.X_plus, 1, 2)
        group_box_crop_layout.addWidget(self.Y_minus, 2, 0)
        group_box_crop_layout.addWidget(self.CROP_BTN, 2, 1)
        group_box_crop_layout.addWidget(self.Y_plus, 2, 2)
        group_box_crop_layout.addWidget(self.Scale_plus, 3, 0)
        group_box_crop_layout.addWidget(self.Z_minus, 3, 1)
        group_box_crop_layout.addWidget(self.Scale_minus, 3, 2)
        group_box_crop_layout.addWidget(self.length_lab, 4, 0)
        group_box_crop_layout.addWidget(self.len_plus, 4, 1)
        group_box_crop_layout.addWidget(self.len_minus, 4, 2)
        group_box_crop_layout.addWidget(self.width_lab, 5, 0)
        group_box_crop_layout.addWidget(self.wid_plus, 5, 1)
        group_box_crop_layout.addWidget(self.wid_minus, 5, 2)
        group_box_crop_layout.addWidget(self.height_lab, 6, 0)
        group_box_crop_layout.addWidget(self.hgt_plus, 6, 1)
        group_box_crop_layout.addWidget(self.hgt_minus, 6, 2)
        group_box_crop_layout.addWidget(self.temp_reps1, 7, 0, 1, 2)
        group_box_crop_layout.addWidget(self.repts_value1, 7, 2, 1, 1)
        group_box_crop_layout.addWidget(self.temp_save_name, 8, 0, 1, 2)
        group_box_crop_layout.addWidget(self.save_temp_btn, 8, 2, 1, 1)
        group_box_crop_layout.addWidget(self.template_label, 9, 0)
        group_box_crop_layout.addWidget(self.template_label_val, 9, 1, 1, 2)
        group_box_crop_layout.addWidget(self.pick_clr1, 10, 0)
        group_box_crop_layout.addWidget(self.clrr1, 10, 1, 1, 1)
        group_box_crop_layout.addWidget(self.temp_crop_submit_btn, 10, 2, 1, 1)

        """Template load Tab"""
        # UI button controls for loading a template 3D point cloud
        self.load_tpc_btn = QPushButton('Load Template Points Cloud')
        self.load_tpc_btn.clicked.connect(self.open_file_tmp)
        self.tmp_label = QLabel('No file found')
        self.template_label1 = QLabel('Enter Label')
        self.template_label_val1 = QLineEdit()
        self.template_label_val1_txt = self.template_label_val1.text()
        self.pick_clr = QPushButton('Pick the label color')
        self.clrr = QPushButton()
        # Connect button signal to color picker function
        self.pick_clr.clicked.connect(self.openColorPicker)
        self.submit_btn_load = QPushButton('Submit to Label')
        self.submit_btn_load.clicked.connect(self.submit)

        #UI layout for loading a template
        group_box_load = QGroupBox()
        group_box_load_layout = QGridLayout()
        group_box_load.setLayout(group_box_load_layout)
        group_box_load.setStyleSheet('QGroupBox { background-color: #E9967A; }')

        group_box_load_layout.addWidget(self.load_tpc_btn, 0, 0, 2, 3, QtCore.Qt.AlignBottom)
        group_box_load_layout.addWidget(self.tmp_label, 2, 0, 1, 3, QtCore.Qt.AlignCenter)
        group_box_load_layout.addWidget(self.template_label1, 5, 0, 1, 1)
        group_box_load_layout.addWidget(self.template_label_val1, 5, 1, 1, 2)
        group_box_load_layout.addWidget(self.pick_clr, 6, 0, 5, 2, QtCore.Qt.AlignTop)
        group_box_load_layout.addWidget(self.clrr, 6, 2)
        group_box_load_layout.addWidget(self.submit_btn_load, 7, 0, 1, 3)

        # Create stacked widget for LOAD and CROP options of template and set as instance variable
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(group_box_crop)
        self.stacked_widget.addWidget(group_box_load)
        self.stacked_widget.setCurrentIndex(0)  # Set initial visible widget
        # Connect button signals to switch between stacked widgets (LOAD and CROP)
        self.temp_crop_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(0))
        self.temp_load_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))

        self.rload_to_lbl_btn = QPushButton("Reload to label")
        self.rload_to_lbl_btn.clicked.connect(self.reload_to_lbl)

        # labels table
        self.table_widget = QTableWidget(self)
        self.table_widget.setColumnCount(2)
        self.table_widget.setHorizontalHeaderLabels(["Label Name", "Label Color"])
        self.labels_details_tag = QLabel("LABELS:")

        #Right section of UI which is view portion
        self.view_label = QLabel('View')
        self.view_pc_btn = QPushButton('View Source Pointcloud')
        self.view_pc_btn.clicked.connect(lambda: self.view_pc("SPC"))
        self.view_temp_pc_btn = QPushButton('View Template Pointcloud')
        self.view_temp_pc_btn.clicked.connect(lambda: self.view_pc("TPC"))
        self.view_intm_stges_label = QLabel('View intermediate stages')
        self.view_fpfh_pc_btn = QPushButton('View FPFH feature descriptors')
        self.view_fpfh_pc_btn.clicked.connect(lambda: self.view_pc("FPFHPC"))
        self.view_knn_pc_btn = QPushButton('View KNN stage result')
        self.view_knn_pc_btn.clicked.connect(lambda: self.view_pc("KNNPC"))
        self.view_dbscan_pc_btn = QPushButton('View DBSCAN stage result')
        self.view_dbscan_pc_btn.clicked.connect(lambda: self.view_pc("DBSCANPC"))
        self.view_labelled_pc_btn = QPushButton('View Labelled Pointcloud')
        self.view_labelled_pc_btn.clicked.connect(lambda: self.view_pc("LPC"))
        self.save_labelled_pc_btn = QPushButton('SAVE Labelled Pointcloud')
        self.save_labelled_pc_btn.clicked.connect(self.save_labelled_pc)
        self.eneter_labeled_pc_name_txt = QLineEdit("Enter the name")


        #whole UI layout
        layout.addWidget(group_box, 0, 0, 3, 3, QtCore.Qt.AlignBottom)
        layout.addWidget(self.label2, 4, 0)
        layout.addWidget(self.temp_crop_button, 4, 1)
        layout.addWidget(self.temp_load_button, 4, 2)
        layout.addWidget(self.stacked_widget, 5, 1, 4, 2, Qt.AlignTop)
        layout.addWidget(self.rload_to_lbl_btn,9,2)
        layout.addWidget(self.labels_details_tag, 11, 0, 1, 1)
        layout.addWidget(self.table_widget, 11, 1, 2, 2)
        layout.addWidget(self.widget, 0, 5, 15, 10)
        layout.addWidget(self.view_label, 0, 15, 1, 3)
        layout.addWidget(self.view_pc_btn, 1, 15, 1, 3)
        layout.addWidget(self.view_temp_pc_btn, 2, 15, 1, 3)
        layout.addWidget(self.view_intm_stges_label, 4, 15, 1, 3)
        layout.addWidget(self.view_fpfh_pc_btn, 5, 15, 1, 3)
        layout.addWidget(self.view_knn_pc_btn, 6, 15, 1, 3)
        layout.addWidget(self.view_dbscan_pc_btn, 7, 15, 1, 3)
        layout.addWidget(self.view_labelled_pc_btn, 9, 15, 1, 3)
        layout.addWidget(self.save_labelled_pc_btn, 11, 15, 1, 3)
        layout.addWidget(self.eneter_labeled_pc_name_txt, 10, 15, 1, 3)

        self.setWindowTitle('Annotate 3D Pointcloud')
        self.setGeometry(150, 100, 1700, 900)
        self.show()

    # functionalities of the above UI controls
    def open_file_src(self):
        """Function helps to load source point cloud .ply file from the system"""
        load_source_pointcloud(self)

    def show_bbox(self):
        """Function to Initiate a Bounding Box for cropping """
        show_boundingbox(self)

    def reset_bbox(self):
        """Function to reset Bounding box dimensions"""
        reset_boundingbox(self)

    def move_bbox(self, dir):
        """Function takes a parameter
            -dir = axis and direction for bounding box to move.
            Function to move the bounding box
        """
        move_boundingbox(self,dir)

    def span_bbox(self, var):
        """Function to change Bounding box dimensions
        Parameter :
        -var : a variable indicating length/width/height along with inc or dec
        """
        span_boundingbox(self,var)

    def crop_temp(self):
        """Function to CROP the 3D points enclosed by bounding box"""
        crop_template(self)

    def save_temp(self):
        """Function to save the cropped template """
        save_template(self)

    def on_combobox_changed(self, flg):
        """Function which sets the Est. template repetitions"""
        selected_item = self.repts_value1.currentText()
        self.k = selected_item
        print("Selected item:", selected_item)

    def openColorPicker1(self):
        """Function to assign label color to CROP tab of template UI"""
        color = QColorDialog.getColor()
        if color.isValid():
            self.clrr1.setStyleSheet(f"background-color: {color.name()};")
            self.label_color = color.getRgbF()[:3]
            print("color = ",self.label_color)
            self.label = self.template_label_val.text()
            print("text = : ",self.template_label_val.text())

    def temp_crop_submit(self):
        """Function which uses template and source 3D point cloud for template matching and labelling
         where template if cropped from source point cloud
         """
        template_CROP_submit(self)

    def open_file_tmp(self):
        """Function to load the template point cloud"""
        load_template_pointcloud(self)

    def openColorPicker(self):
        """Function to assign label color to LOAD tab of template UI"""
        color = QColorDialog.getColor()
        if color.isValid():
            self.clrr.setStyleSheet(f"background-color: {color.name()};")
            self.label_color = color.getRgbF()[:3]
            print("color = ",self.label_color)
            self.label = self.template_label_val1.text()
            print("text = : ",self.template_label_val1.text())

    def submit(self):
        """Function which uses template and source 3D point cloud for template matching and labelling
                where template if loaded from system
                """
        submit(self)

    def reload_to_lbl(self):
        """Function which enables relabeling of a labelled point cloud"""
        reload_and_label(self)



    def view_pc(self,flg):
        """Function which enables to view the different stages of template matching"""

        if flg == "SPC":
            self.scatter.setData(pos=self.pcd_points, color=(255,255,255,255))
        elif flg == "TPC":
            self.scatter.setData(pos=self.temp_points, color=(255,255,255,255))
        elif flg == "FPFHPC":
            fpfh_pts = np.asarray(self.fpfh_pc.points)
            fpfh_clrs = np.asarray(self.fpfh_pc.colors)
            self.scatter.setData(pos=fpfh_pts, color=fpfh_clrs)
        elif flg == "KNNPC":
            knn_pts = np.asarray(self.knn_pc.points)
            knn_colrs = np.asarray(self.knn_pc.colors)
            self.scatter.setData(pos=knn_pts, color=(255,255,255,255))
        elif flg == "DBSCANPC":
            dbscan_pts = np.asarray(self.dbscan_pc.points)
            dbscan_clrs = np.asarray(self.dbscan_pc.colors)
            self.scatter.setData(pos=dbscan_pts, color=dbscan_clrs)
        elif flg == "LPC":
            labeled_pc_pts = np.asarray(self.labeled_pc.points)
            clrs_arr = np.array([self.labels_dict[key] for key in self.labels_list])
            self.scatter.setData(pos=labeled_pc_pts, color=np.asarray(clrs_arr))

    def save_labelled_pc(self):
        """Function to save the finally labelled Point cloud"""
        save_labelled_pc(self)

    def black_out(self):
        """Functionality of a TOGGLE button in UI
        which toggles between unlabelled points and labelled points of point cloud
        which helps in cropping template
        """
        display_toggle_of_template(self)

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()
    app.exec_()
