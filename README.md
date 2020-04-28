SolAR SLAM
=============

SolAR is an open-source framework released under Apache license 2.0 making possible to easily create your own camera pose estimation solution to develop Augmented Reality applications. 
SolAR is dedicated to Augmented Reality (AR).
It offers a C++ SDK to easily and quickly develop and use custom solutions for camera pose estimation. It provides developers with a full chain from low-level vision components development to camera pose estimation pipelines and AR service development.

The SolAR SLAM samples show a SolAR pipeline for augmented reality based on a SLAM.

Run whether the SolARSlamSampleMono.exe (showing a mono thread demonstration of the SolAR SLAM), or the SolARSlamSampleMulti.exe (showing a multi thread demonstration of the SolAR SLAM).
Move the camera to initialize the SLAM. You will see in a dedicated window the 3D point cloud reconstructed by the SLAM.

You can also test the TestSlamPlugin.exe (showing an application loading the SLAM pipeline embedded into a dedicated SolAR Module). Here, you need to print the FiducialMarker.gif. When the application is started, just point to the fiducial marker to initialize the SLAM. 

Press escape to quit the application.

If you want to change the calibration parameters of the camera, edit the camera_calibration.yml.

To change properties of the components of the SLAM pipeline, edit the .xml files.


*   Website https://solarframework.github.io/

*   Contact framework.solar@b-com.com


