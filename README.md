## Lidar Obstacle Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

This project was built as part of the **Udacity Sensor Fusion Nanodegree Program**.

The main objective of this project is to preprocess, cluster and draw bounding boxes around obstacles in a sequence of lidar point cloud data using custom c++ algorithms and PCL(Point Cloud Library) implemented algorithms.

**Workflow

**Filtering and Cropping**
This is the initial stage of the lidar preprocessing pipeline. Most of the task here involves downsampling point cloud data to reduce the number of points for fast processing. We use PCL implementation of voxel grid filtering to downsample the cloud data. Another important point of this stage is to remove cloud points which are beyond the sides of the road.

**Segmentation**
At this stage, we use our custom C++ implementation of the Random Sample Consensus (RANSAC) algorithm to separate the ground plane from the obstacle plane. More details: https://en.wikipedia.org/wiki/Random_sample_consensus

**Clustering**
At this stage also, we use our custom C++ implementation of the Kd-tree data structure to efficiently extract clusters from the obstacle plane. We then draw bounding boxes around each cluster which represents an obstacle within our ego vehichle environment.



## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/henrhie/Lidar-Obstacle-Detection.git
$> cd Lidar-Obstacle-Detection
$> mkdir build && cd build  **if build folder is not available
$> cmake ..  **can skip this step. project already built
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
