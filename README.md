# Annotating 3D point clouds
This is a project focusing on labelling different parts of a 3D point cloud. 

## Setup
:information_source: *Currently supports Python 3.9.5*

### via git (manually)
```bash
git clone https://github.com/yasaswinid/Annotating-3D-pointclouds.git  # 1. Clone repository
pip install -r Requirements.txt  # 2. Install requirements
# 3. Copy point clouds into `pointclouds` folder.
python3 UI.py  # 4. Start labelCloud
```

## Labeling
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

### Crop the Template Point Cloud
If a template is not already available, follow these steps to crop a template from the source point cloud:

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
  ##### • Estimated Template Repetitions:
  - Select an option from the "Est. Template Repetitions" drop-down box.
  - Choose "1" if there are no repetitions and you only want to color and label the selected crop.
  - Select ">1" if you want to label all repetitions of the template.
  ##### • Enter the Part Label:
  - Enter the desired label name in the text box provided.
  ##### • Pick a Color:
  - Click on the "Pick the color" button to select a color that represents the chosen label.
  ##### • Submit:
  - Click on the "Submit" button to match all the templates in the source and obtain




  
