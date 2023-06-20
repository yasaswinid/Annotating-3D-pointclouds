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
### Source Point Cloud 
Load the Source point cloud directly from UI.
### Template Point Cloud
To load template point cloud, two options are available : 
* Crop a part of Source point cloud as template
* Load the already available template point cloud file

  #### Load Template Point Cloud
  * Click on Load button
  * Load the template pointcloud file
  * Enter the Label name and pick the label color
  * Click 'Submit to Label' button to Annotate the template

  
