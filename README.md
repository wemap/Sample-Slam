# SolAR Sample SLAM

[![License](https://img.shields.io/github/license/SolARFramework/Sample-Slam?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)


The SolAR **SLAM samples** show a SolAR pipeline for augmented reality based on a SLAM (Simultaneous Localization And Mapping).


| ![](./Multi/cube.jpg) ![](./Multi/pointcloud.jpg) | ![](./Plugin/plugin.jpg) |
|:-:|:-:|
| StandAlone/Multithread | Plugin | 


## How to run

### Bag Of Word Vocabulary

:warning: Don't forget to download the [fbow vocabularies](https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip) unzip this archive and put the `akaze.fbow` in the `./data` folder.

### Camera Calibration

We provide a defaut camera calibration file that contains intrinsic parameters of the camera logitech C920.
If you want to change the calibration parameters of the camera, edit the `camera_calibration.yml` file.

### Initialization

SolAR SLAM supports two initialization modes: **fiducial marker based** or **markerless based**.

* For **fiducial marker** based:
	* SolAR SLAM samples require a fiducial marker to initialize. So you need to print the [Fiducial Marker](./SolARSample_SLAM_Mono/FiducialMarker.gif) and put it into the scene.

	* If you want to change your fiducial marker, you can edit the `fiducialMarker.yml`.
	
	* Set the `hasPose` parameter value of `SolARSLAMBootstrapper` component to `1` in the configuration file.
	
* For **markerless** based:
    * Not any marker is required. Just set the `hasPose` parameter value of `SolARSLAMBootstrapper` component to `0` in the configuration file.

### Run tests using a video file

Download [freiburg3_long_office_household_validation-rgb.avi](https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi) file from TUM RGB-D benchmark dataset, and put this video into the `./data` folder.
From the binary directory, run following command for testing SolAR SLAM:

* For mono thread test:
<pre><code>SolARSample_SLAM_Mono.exe ..\..\data\SolARSample_SLAM_TUM_conf.xml</code></pre>

* For multithreading test:
<pre><code>SolARSample_SLAM_Multi.exe ..\..\data\SolARSample_SLAM_TUM_conf.xml</code></pre>

* For pipeline plugin test:
<pre><code>SolARPipelineTest_SLAM.exe ..\..\data\SolARPipelineTest_SLAM_TUM_conf.xml</code></pre>

Press `escape` to quit the application.

### Run tests using a webcam

Change the `deviceID` parameter in the configuration file to the corresponding ID of the camera that you use. From the binary directory, launch the execution for testing SolAR SLAM.

*  When the application is started, if you use the fiducial marker based initialization, point the camera to the fiducial marker (you can see a virtual cube on the marker). Otherwise, point the camera in front of the scene.

* Then move the camera to initialize the SLAM (recommend moving horizontally). Note that for the fiducial marker based, make sure the fiducial marker is always in view of the camera. The initialization is successful when appearing green points in the image.

* :warning: If the initialization is still not successful after a few seconds (This is normally due to texture-less scenes), you can add some objects into the scene and restart.

* Now you can move the camera freely. The SolAR SLAM can track the camera pose and at the same time, you see in a dedicated window the 3D point cloud reconstructed by the SLAM (only for tests of mono and multi thread). Note that the fiducial marker is only used for the initialization, it can be removed from the scene later.

* :warning: In the case of lost tracking, you can return the camera to the previous views to relocalize camera pose instead of restarting.

* Press `escape` to quit the application.

## Contact 
Website https://solarframework.github.io/

Contact framework.solar@b-com.com



