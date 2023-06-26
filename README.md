# Annotating 3D point clouds
This project aims to annotate different parts of a 3D point cloud by utilizing a 3D template matching method. The process becomes easier when multiple repetitions of a part can be annotated simultaneously. The project includes a user interface (UI) that allows users to load the source and template point clouds. They can provide additional information such as the part name and the desired color for labeling representation. Upon submission, the system labels the repetitions of the part in the source point cloud. The UI also provides an option to directly label without finding repetitions. Users can reload the labeled point cloud and select another part for annotation. By repeating this process, complete 3D point cloud can be annotated iteratively. 

## Setup
:information_source: *Currently supports Python 3.9.5*

### via git (manually)
```bash
git clone https://github.com/yasaswinid/Annotating-3D-pointclouds.git  # 1. Clone repository
pip install -r Requirements.txt  # 2. Install requirements
# 3. Copy point clouds into `pointclouds` folder.
python3 MTP_UI.py  # 4. Start labelling
PYOPENGL_PLATFORM=mesa python MTP_UI.py # 5. If running in WSO2 or Linux
```

## 3D Template Matching
This project does template matching in 3 steps :
### 1. Feature Descriptor Estimation:
* After Data Pre-processing, calculate feature vectors for each point in the source and template point clouds by Fast Point Feature Histogram (FPFH) method
### 2. Correspondence Estimation:
* Perform the K-nearest neighbors (KNN) search in the feature space to find similar points between the source and template point clouds.
* Apply DBSCAN clustering to segment the scene points into distinct clusters based on density.
* Estimate correspondences between each cluster & template points
### 3. Transformation Estimation:
* Estimate the rigid transformation matrix based on the correspondences between clusters and template points.
* Refine the transformation using the Iterative Closest Point (ICP) algorithm to minimize the distance between corresponding points.
* Utilize Uniform Random Rotations (URR) to generate a set of seed transformations for ICP registration.
* Use specific uniform rotations around the Y-axis as seed transformations for more efficient and accurate registration.

## Labelling Instructions & Steps
For labelling the 3D point cloud, user needs  two 3D points clouds which are source and template point clouds.

###  Loading Source Point Clouds
#### Load the Source 3D Point Cloud :
      * Click on the "Load Source Point Cloud" button to load a .ply format point cloud file .

### Load the Template Point Cloud & Label
#### Load the Template 3D Point Cloud if already exists :
##### • Load a pre-existing template:
        - Click the "Load" button.
        - Click on the "Load Template Point Cloud" button to load the template.
        – Enter the label name for the part in the text box.
        – Pick a color that represents the label by clicking on the "Pick the color" button.
        – Click on the "Submit to label" button to match the templates in the source and obtain a labeled point cloud

### Crop the Template Point Cloud & Label
If a template is not already available, follow these steps to crop a template from the source point cloud & Annotate:
#### 1.Pick a Bounding Box:
       - Click ’Crop’ button
       - Click on the "Pick a bounding box" button.
       - Move the bounding box along the positive X and negative X axes using ’X+’ and ’X-’ button respectively. Follow the same for Y and Z axes using the respective buttons.
       - Increase or decrease the dimensions of the bounding box using the "+" and "-" buttons for length, width, and height.
       -  Once the bounding box encloses the desired template, click on the "Crop" button to extract the template from the source point cloud.
#### 2.TOGGLE BUTTON :
     Clicking on ’TOGGLE’ button will toggle the display of pointcloud between unlabelled 3D points and labelled 3D points of pointcloud. It makes the cropping process easy.
#### 3. Save the Template (optional):
       - Click on the "Save" button to save the template for future reference.
       - Enter the template name in .ply format in the adjacent text box.
#### 4. Labeling and Annotation of Cropped Template:
      • Estimated Template Repetitions:
            - Select an option from the "Est. Template Repetitions" drop-down box.
            - Choose "1" if there are no repetitions and you only want to color and label the selected crop.
            - Select ">1" if you want to label all repetitions of the template.
      • Enter the Part Label:
            - Enter the desired label name in the text box provided.
      • Pick a Color:
            - Click on the "Pick the color" button to select a color that represents the chosen label.
      • Submit:
            - Click on the "Submit" button to match all the templates in the source and obtain

### Reloading and Saving
     • Reloading for Labeling Another Part:
          - To label another part of the already labeled point cloud, click on the "Reload to label" button at the bottom right end.
          - The labeled point cloud will load as the new source point cloud, and you can continue the process further by loading or cropping the template as stated in the above steps..
     • Saving the Labeled Point Cloud:
          – On the right side of the UI, enter a name for the labeled point cloud in .ply format in the text box.
          – Click on the "Save labeled point cloud" button to save the labeled point cloud.

# Bounding box & Controls

|                               Button                                 | Description                                          |
| :------------------------------------------------------------------: | ---------------------------------------------------- |
|                             *Navigation*                             |                                                      |
|                          `Y`+                                        | Translates the Bounding Box along +ve Y axis         |
|                          `X`+                                        | Translates the Bounding Box along +ve X axis         |
|                          `Z`+                                        | Translates the Bounding Box along +ve Z axis         |
|                          `Y`-                                        | Translates the Bounding Box along -ve Y axis         |
|                          `X`-                                        | Translates the Bounding Box along -ve X axis         |
|                          `Y`-                                        | Translates the Bounding Box along -ve Z axis         |
|                             *Diemsions*                              |                                                      |
|                         length(`X`)     +                            | Increase the bounding box length along `X` axis      |
|                         length(`X`)     -                            | Decrease the bounding box length along `X` axis      |
|                         width(`Y`)     +                             | Increase the bounding box length along `Y` axis      |
|                         width(`Y`)     -                             | Decrease the bounding box length along `Y` axis      |
|                         Height(`Z`)     +                            | Increase the bounding box length along `Z` axis      |
|                         Height(`Z`)     -                            | Decrease the bounding box length along `Z` axis      |
|                         Scale+                                       | Increase the bounding box in all dimensions          |
|                         Scale-                                       | Decrease the bounding box in all dimensions          |
|                              *General*                               |                                                      |
|                          Right Mouse Button                          | Rotates the camera                                   |
|                             Mouse Wheel                              | Zooms in & out the Point Cloud                       |

# Supported Label Format
Its important to note that the label name that is entered in UI should be less than 15 letters. Because this code uses a plyfile library to modify the loaded source point cloud's .ply file to add an attribute called LABEL. This attribute stores the label information in the annotated point cloud with individual letters stored with their ASCII values.
The code snippet for this can be seen in code_help.py. 
```bash
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
```

## Dataset
We have used BuildingNet dataset which is available at : https://buildingnet.org/

  
