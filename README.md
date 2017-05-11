# SLAMRecon #
## 1. Introduction ##
SLAMRecon is a real-time 3D dense mapping system based on RGB-D camera. The output is a reconstructed indoor scene model. As we know, there are some previous 3D Dense Mapping Systems, like Kinect Fusion. Kinect Fusion using ICP to do registration between adjacent two frames. And ICP will fail when scan some flat area. However, Kinect Fusion doesn’t do any local or global optimization for camera poses. During scan, the reconstruction will drift because of accumulated errors. In our work, we used Orb-Slam to help us estimate camera pose of each frame. Because of orb-slam’s local and global optimization, we can avoid obvious drift when scan an indoor scene. We use an integration and re-integration framework to handle changing camera poses. Once a camera pose changes because of optimization, the system will do re-integration process to remove data influenced by old camera pose from scene volume and integrate data based on new pose to the scene volume.

The system is built based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [InfiniTAM](https://github.com/victorprad/InfiniTAM). 
More information about SLAMRecon can be found in our project homepage: <http://irc.cs.sdu.edu.cn/SLAMRecon>

The system is developed by Hao Li, Huayong Xu and Guowei Wan.

## 2. License ##
SLAMRecon is released under a [GPLv3 license]. More detailed information can be find in [LICENSE.txt](). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md]().  

## 3. System Building ##
The system is developed on Visual Studio 2013. You can also build the system on other versions(>=2010) of Visual Studio. It is not difficult to transform the code to Linux platform.

### 3.1 Hardware Requirements ###
We have tested the system on computer with this configuration:  
> Rrocessor: Intel(R)Core(TM)i7-6700K CPU @ 4.00GHz    
> RAM: 16.0GB  
> Graphics Card: NVIDIA GeForce GTX 980 Ti(6GB)  

To use SLAMRecon system, we suggest you employ a compuer that has similar performance to ours. NVIDIA Graphics Card is necessary.

### 3.2 Software Requirements ###

- **CUDA 7.5**   
  Other versions(>=6.0) of CUDA is also supported. But you have to modify the vs configuration file.  
  CUDA can be available at <https://developer.nvidia.com/cuda-downloads>.
- **QT 5.5.1**  
  Other versions(>=5.0) of CUDA is also supported. But you have to re-config the QT in vs.  
  QT can be available at <https://www.qt.io/download>.
- **OPENCV 2.4.8**  
  You'd better use OPENCV 2.4.8, or you have to re-config the project in vs. Set system variables **OPENCV** = $OPENCV\_INSTALL_PATH\build. 
  OPENCV can be available at <http://opencv.org/releases.html>.
- **OPENNI 2.2**  
  OPENNI can be available at <https://structure.io/openni>.

### 3.3 Building ###
Just open Visual Studio solution file(SLAMRecon.sln).    
Modify configurations if you install softwares that versions are different from ones listed in 2.2.   
We used x64 solution platform, if you have a win32 system, you should also modify configurations.
  
## 4. Dataset ##
The system support to acquire RGB-D images from both RGB-D camera and files stored on the disk. A example data can be download [here](http://irc.cs.sdu.edu.cn/SLAMRecon). We suggest you put the example data in $PROJECT_FOLDER/data folder. The parameter file for example data is $PROJECT\_FOLDER/data/FILES\_PARAM3.yaml. 

### 4.1 Download ORB Vocabulary ###
ORB Vocabulary can be download [here](). You should put ORB Vocabulary in $PROJECT_FOLDER/data folder. 

### 4.2 More dataset ###
More data can be got from TUM Dataset: <http://vision.in.tum.de/data/datasets/rgbd-dataset/download>