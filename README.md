SolAR SLAM
=============

SolAR is an open-source framework released under Apache license 2.0 making possible to easily create your own camera pose estimation solution to develop Augmented Reality applications.
SolAR is dedicated to Augmented Reality (AR).
It offers a C++ SDK to easily and quickly develop and use custom solutions for camera pose estimation. It provides developers with a full chain from low-level vision components development to camera pose estimation pipelines and AR service development.

The SolAR SLAM samples show a SolAR pipeline for augmented reality based on a SLAM. It includes 3 samples as follows:

*	__SolARSlamSampleMono.exe__ : showing a mono thread demonstration of the SolAR SLAM.
*	__SolARSlamSampleMulti.exe__ : showing a multi thread demonstration of the SolAR SLAM.
*	__TestSlamPlugin.exe__ : showing an application loading the SLAM pipeline embedded into a dedicated SolAR Module.

## How to run ##

SolAR SLAM samples require a fiducial marker to initialize. So you need to print the __FiducialMarker.gif__ and put it into the scene.

Run whether the SolARSlamSampleMono.exe, the SolARSlamSampleMulti.exe, or the TestSlamPlugin.exe. When the application is started, point the camera to the fiducial marker (you can see a virtual cube on the marker). Then move camera around the fiducial marker (recommend moving left and right) to initialize the SLAM. The initialization is successful when appearing green points in the image.

:warning: If the initialization is still not successful after a few seconds (This is normally due to texture-less scenes), you can add some objects into the scene and restart.

Now you can move the camera freely. The SolAR SLAM is able to track the camera pose and at the same time you see in a deddicated window the 3D point cloud reconstructed by the SLAM (only for SolARSlamSampleMono.exe and SolARSlamSampleMulti.exe). Note that the fiducial marker is only used for the initialization, it can be removed from the scene later.

:warning: In the case of lost tracking, you can return the camera to the previous views to relocalize camera pose instead of restarting.

Press escape to quit the application.

If you want to change the calibration parameters of the camera, edit the camera_calibration.yml.

To change properties of the components of the SLAM pipeline, edit the .xml files.


*   Website https://solarframework.github.io/

*   Contact framework.solar@b-com.com
