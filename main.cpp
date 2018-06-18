/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <string>
#include "constants.h"
#include "datastructure/Frame.h"
#include <boost/log/core.hpp>
#include <ctime>
#include<fstream>

#include "opencv2\highgui.hpp"



const char space_key = 32;
const char escape_key = 27;

void keyBoard(unsigned char key){
    switch (key) {
    case 's': {
        saving_images = !saving_images;
        std::cout << "image saved: " << views.size() << std::endl;
        break;

    }
    case 't': {
        triangulation_first = !triangulation_first;
        std::cout << "tringulaiton started: " << triangulation_first << std::endl;
        break;
    }
    case 'd': {
        viewerGL.SetPointCloudToDisplay(poseGraph->getMap()->getPointCloud());
        std::cout << "drawing started "  << std::endl;
        break;
    }
    case 'p': {
        processing = !processing;
        std::cout << "processing started: " << processing << std::endl;
        break;
    }
    case 'x': {
        std::cout << "views poped: " << processing << std::endl;
        views.pop_back();
        break;
    }
    case space_key: {
        std::cout << "make pause/!pause " << std::endl;
        pause=!pause;
        break;
    }
    case escape_key: {
        std::cout << "exit " << std::endl;
        exit_=true;
        break;
    }
    default:
        break;
    }
}




void DrawPointCloud(Transform3Df relative_pose)
{
    std::string window_name = "cloud_points";
    cv::namedWindow( window_name, 0);cv::resizeWindow(window_name, 640, 480);
    cv::Mat reprojected(cv::Size(640,480), CV_8UC3);
    reprojected.setTo(0);

    std::vector<cv::Point3f> worldCVPoints;
    std::set<cv::Point3f> worldCVPointsSet;
    std::vector<cv::Point2f> projected3D;


    std::vector<SRef<CloudPoint>> Map= *(poseGraph->getMap()->getPointCloud());
    for(auto point : Map){
        worldCVPoints.push_back(cv::Point3f(point->getX(), point->getY(),point->getZ()));
    }
    if( worldCVPoints.size()!=0){

        CamCalibration intrinsic_param;
        CamDistortion distorsion_param;

        intrinsic_param=camera->getIntrinsicsParameters();
        distorsion_param=camera->getDistorsionParameters();

        cv::Mat m_camMatrix;
        cv::Mat m_camDistorsion;
        m_camMatrix.create(3, 3, CV_32FC1);
        m_camDistorsion.create(5, 1, CV_32FC1);

        m_camDistorsion.at<float>(0, 0)  = dist(0);
        m_camDistorsion.at<float>(1, 0)  = dist(1);
        m_camDistorsion.at<float>(2, 0)  = dist(2);
        m_camDistorsion.at<float>(3, 0)  = dist(3);
        m_camDistorsion.at<float>(4, 0)  = dist(4);

        m_camMatrix.at<float>(0, 0) = intrinsic_param(0,0);
        m_camMatrix.at<float>(0, 1) = intrinsic_param(0,1);
        m_camMatrix.at<float>(0, 2) = intrinsic_param(0,2);
        m_camMatrix.at<float>(1, 0) = intrinsic_param(1,0);
        m_camMatrix.at<float>(1, 1) = intrinsic_param(1,1);
        m_camMatrix.at<float>(1, 2) = intrinsic_param(1,2);
        m_camMatrix.at<float>(2, 0) = intrinsic_param(2,0);
        m_camMatrix.at<float>(2, 1) = intrinsic_param(2,1);
        m_camMatrix.at<float>(2, 2) = intrinsic_param(2,2);

        // Rotation and Translation from input pose
        cv::Mat Rvec;   Rvec.create(3, 3, CV_32FC1);
        cv::Mat Tvec;   Tvec.create(3, 1, CV_32FC1);

        Rvec.at<float>(0,0) = relative_pose(0,0);
        Rvec.at<float>(0,1) = relative_pose(0,1);
        Rvec.at<float>(0,2) = relative_pose(0,2);

        Rvec.at<float>(1,0) = relative_pose(1,0);
        Rvec.at<float>(1,1) = relative_pose(1,1);
        Rvec.at<float>(1,2) = relative_pose(1,2);

        Rvec.at<float>(2,0) = relative_pose(2,0);
        Rvec.at<float>(2,1) = relative_pose(2,1);
        Rvec.at<float>(2,2) = relative_pose(2,2);

        Tvec.at<float>(0,0) = relative_pose(0,3);
        Tvec.at<float>(1,0) = relative_pose(1,3);
        Tvec.at<float>(2,0) = relative_pose(2,3);

        cv::Mat rodrig;
        cv::Rodrigues(Rvec,rodrig);
//        std::cout << " rodrig \n";
//        std::cout << rodrig <<std::endl;
//        std::cout << " Tvec \n";
//        std::cout << Tvec <<std::endl;

        cv::projectPoints(worldCVPoints, rodrig, Tvec, m_camMatrix, m_camDistorsion, projected3D);

//        std::cout << projected3D.size() << "\n\n";

        for (int i = 0; i<projected3D.size(); i++) {
            cv::circle(reprojected, projected3D[i], 2.0, cv::Scalar(0, 0, 255), 2.0,8,0);
        }

        cv::imshow("cloud_points", reprojected);
        }
}


void DrawPnpMatches(const std::vector<SRef<Point2Df>> & imagePoints,const std::vector<SRef<Point3Df>> & worldPoints,Transform3Df relative_pose)
{

    std::vector<cv::Point2f> imageCVPoints;
    std::vector<cv::Point3f> worldCVPoints;

    for (int i=0;i<imagePoints.size();++i) {
        Point2Df point2D = *(imagePoints.at(i));
        Point3Df point3D = *(worldPoints.at(i));
        imageCVPoints.push_back(cv::Point2f(point2D.getX(), point2D.getY()));
        worldCVPoints.push_back(cv::Point3f(point3D.getX(), point3D.getY(),point3D.getZ()));
    }

    std::string window_name = "reprojectedPoints";
    cv::namedWindow( window_name, 0);cv::resizeWindow(window_name, 640, 480);
    cv::Mat reprojected(cv::Size(640,480), CV_8UC3);
    reprojected.setTo(0);

    std::vector<cv::Point2f> projected3D;


if( worldCVPoints.size()!=0){

    CamCalibration intrinsic_param;
    CamDistortion distorsion_param;

    intrinsic_param=camera->getIntrinsicsParameters();
    distorsion_param=camera->getDistorsionParameters();

    cv::Mat m_camMatrix;
    cv::Mat m_camDistorsion;
    m_camMatrix.create(3, 3, CV_32FC1);
    m_camDistorsion.create(5, 1, CV_32FC1);

    m_camDistorsion.at<float>(0, 0)  = dist(0);
    m_camDistorsion.at<float>(1, 0)  = dist(1);
    m_camDistorsion.at<float>(2, 0)  = dist(2);
    m_camDistorsion.at<float>(3, 0)  = dist(3);
    m_camDistorsion.at<float>(4, 0)  = dist(4);

    m_camMatrix.at<float>(0, 0) = intrinsic_param(0,0);
    m_camMatrix.at<float>(0, 1) = intrinsic_param(0,1);
    m_camMatrix.at<float>(0, 2) = intrinsic_param(0,2);
    m_camMatrix.at<float>(1, 0) = intrinsic_param(1,0);
    m_camMatrix.at<float>(1, 1) = intrinsic_param(1,1);
    m_camMatrix.at<float>(1, 2) = intrinsic_param(1,2);
    m_camMatrix.at<float>(2, 0) = intrinsic_param(2,0);
    m_camMatrix.at<float>(2, 1) = intrinsic_param(2,1);
    m_camMatrix.at<float>(2, 2) = intrinsic_param(2,2);

    // Rotation and Translation from input pose
    cv::Mat Rvec;   Rvec.create(3, 3, CV_32FC1);
    cv::Mat Tvec;   Tvec.create(3, 1, CV_32FC1);

    Rvec.at<float>(0,0) = relative_pose(0,0);
    Rvec.at<float>(0,1) = relative_pose(0,1);
    Rvec.at<float>(0,2) = relative_pose(0,2);

    Rvec.at<float>(1,0) = relative_pose(1,0);
    Rvec.at<float>(1,1) = relative_pose(1,1);
    Rvec.at<float>(1,2) = relative_pose(1,2);

    Rvec.at<float>(2,0) = relative_pose(2,0);
    Rvec.at<float>(2,1) = relative_pose(2,1);
    Rvec.at<float>(2,2) = relative_pose(2,2);

    Tvec.at<float>(0,0) = relative_pose(0,3);
    Tvec.at<float>(1,0) = relative_pose(1,3);
    Tvec.at<float>(2,0) = relative_pose(2,3);

    cv::Mat rodrig;
    cv::Rodrigues(Rvec,rodrig);
//    std::cout << " rodrig \n";
//    std::cout << rodrig <<std::endl;
//    std::cout << " Tvec \n";
//    std::cout << Tvec <<std::endl;

    cv::projectPoints(worldCVPoints, rodrig, Tvec, m_camMatrix, m_camDistorsion, projected3D);

//    std::cout << projected3D.size() << "\n\n";

    for (int i = 0; i<projected3D.size(); i++) {
        cv::circle(reprojected, imageCVPoints[i], 2.0, cv::Scalar(0, 255, 0),2.0,8,0);
        cv::circle(reprojected, projected3D[i], 2.0, cv::Scalar(0, 0, 255), 2.0,8,0);
    }

    cv::imshow("reprojectedPoints", reprojected);
    }
}


bool ParseConfigFile(const std::string &filePath){
	indexCurrentFrame = 0;
	streamSource = ""; 
	indexFirstKeyFrame = 0;
	indexSecondKeyFrame = 0;

	std::string dummy[4];
	std::ifstream ox;

	ox.open(filePath);
	std::cout << "<SLAM config: >" << std::endl;
	if (ox.is_open()) {
		for (int i = 0; i < 4; ++i) {
			ox >> dummy[i];
		}
		streamSource = dummy[0];
		calibCameraSource = dummy[1];
		indexFirstKeyFrame = std::stoi(dummy[2]);
		indexSecondKeyFrame = std::stoi(dummy[3]);
		std::cout << "	# stream mode: " << streamSource << std::endl;
		std::cout << "	# calib file: " << calibCameraSource << std::endl;
		std::cout << "	# triangulation pair: (" << indexFirstKeyFrame << "," << indexSecondKeyFrame << ")" << std::endl << std::endl;
		ox.close();

		return true;
	}
	else{
		ox.close();
		std::cout << " can't read slam config slam file from: " << filePath << std::endl;
		return false;
	}
}



void init(std::string configFile)
{
    // component creation

       xpcf::ComponentFactory::createComponent<SolARCameraOpencv>(gen(input::devices::ICamera::UUID), camera);
       xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader);

#ifdef USE_FREE
       xpcf::ComponentFactory::createComponent<SolARKeypointDetectorOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
       xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorORBOpencv>(gen(features::IDescriptorsExtractor::UUID), descriptorExtractor);
#else
       xpcf::ComponentFactory::createComponent<SolARKeypointDetectorNonFreeOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
       xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSURF64Opencv>(gen(features::IDescriptorsExtractor::UUID ), descriptorExtractor);
#endif

//     xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherKNNOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);

#ifdef USE_FREE
       xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherKNNOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);
#else
       xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherRadiusOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);

#endif
       xpcf::ComponentFactory::createComponent<SolARSideBySideOverlayOpencv>(gen(display::ISideBySideOverlay::UUID ), overlay);
       xpcf::ComponentFactory::createComponent<SolAR2DOverlayOpencv>(gen(display::I2DOverlay::UUID ), overlay2d);

       xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);
       xpcf::ComponentFactory::createComponent<SolARGeometricMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ), matchesFilterGeometric);
       xpcf::ComponentFactory::createComponent<SolARFundamentalMatrixEstimationOpencv>(gen(solver::pose::I2DTransformFinder::UUID ), fundamentalFinder);
       xpcf::ComponentFactory::createComponent<SolARSVDFundamentalMatrixDecomposerOpencv>(gen(solver::pose::I2DTO3DTransformDecomposer::UUID ), fundamentalDecomposer);
       xpcf::ComponentFactory::createComponent<SolARSVDTriangulationOpencv>(gen(solver::map::ITriangulator::UUID ), mapper);
       xpcf::ComponentFactory::createComponent<SolARMapFilterOpencv>(gen(solver::map::IMapFilter::UUID), mapFilter);

       xpcf::ComponentFactory::createComponent<SolARMapperOpencv>(gen(solver::map::IMapper::UUID), poseGraph);
       xpcf::ComponentFactory::createComponent<SolARPoseEstimationPnpEPFL>(gen(solver::pose::I3DTransformFinder::UUID), PnP);
//       xpcf::ComponentFactory::createComponent<SolARPoseEstimationPnpOpencv>(gen(solver::pose::I3DTransformFinder::UUID), PnP);

       xpcf::ComponentFactory::createComponent<SolAR2D3DCorrespondencesFinderOpencv>(gen(solver::pose::I2D3DCorrespondencesFinder::UUID), corr2D3DFinder);


       ParseConfigFile(configFile.c_str());


#ifdef USE_FREE
       keypointsDetector->setType(features::KeypointDetectorType::ORB);
#else
       keypointsDetector->setType(features::KeypointDetectorType::SURF);
#endif
       // load camera parameters from yml input file
       camera->loadCameraParameters(calibCameraSource);
       PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
       K = camera->getIntrinsicsParameters();
       dist = camera->getDistorsionParameters();
      
	   // Open camera or file
	   if (streamSource == "camera")
	   {
		   camera->start(0);
	   }
	   else
	   {
		   camera->start(streamSource); 

		   //std::cout << "	# loading frames: ";
		   //// this part to load image from video:
		   //for (int k = 0; k < 30; ++k) {
			  // camera->getNextImage(view_current);
			  // views.push_back(view_current);
		   //}
		   //std::cout << views.size() << std::endl;
       }
}


bool fullTriangulation(const std::vector<SRef<Point2Df>>& pt2d_1,
    const std::vector<SRef<Point2Df>>& pt2d_2,
    const std::vector<DescriptorMatch>&matches,
    const std::pair<int, int>&working_views,
    const Transform3Df&p1,
    const std::vector<Transform3Df>&p2,
    const CamCalibration&cam,
    const CamDistortion&dist,
    Transform3Df&corrected_pose,
    std::vector<SRef<CloudPoint>>& cloud)
{

    if (p2.size() != 4) {
        std::cerr << " number of decomposed poses supposed to be 4.." << std::endl;
        return false;
    }
    std::vector<SRef<CloudPoint>>   pcloud;
    std::vector<SRef<CloudPoint>>   pcloud1;
    std::vector<bool> tmp_status;

    //check if points are triangulated --in front-- of cameras for all 4 ambiguations
    for (int i = 0; i < 4; i++)
    {
        corrected_pose = p2[i];
        pcloud.clear(); pcloud1.clear();
        double reproj_error1 = mapper->triangulate(pt2d_1, pt2d_2, matches, working_views, p1, corrected_pose, cam, dist, pcloud);
        double reproj_error2 = mapper->triangulate(pt2d_2, pt2d_1, matches, working_views, corrected_pose, p1, cam, dist, pcloud1);
        if (mapFilter->checkFrontCameraPoints(pcloud, corrected_pose, tmp_status) && mapFilter->checkFrontCameraPoints(pcloud1, p1, tmp_status) && reproj_error1 < 100.0 && reproj_error2 < 100.0)
        {
            // points are in front of camera and reproj error is acceptable
            break;
        }
        if (i == 3)
        {
            return false;
        }

    }

    mapFilter->filterPointCloud(pcloud, tmp_status, cloud);

    return true;
}


SRef<Frame> createAndInitFrame(SRef<Image>&img)
{
    std::chrono::time_point<std::chrono::system_clock> now1 = std::chrono::system_clock::now();
    SRef<Frame> resul = xpcf::utils::make_shared<Frame>() ;

    std::vector< SRef<Keypoint>> keyPoints  ;
    SRef<DescriptorBuffer>  descriptors ;

    keypointsDetector->detect(img, keyPoints);
    descriptorExtractor->extract(img, keyPoints, descriptors);

    resul->InitKeyPointsAndDescriptors(keyPoints , descriptors);

    std::chrono::time_point<std::chrono::system_clock> now2 = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now2 - now1).count() ;

//    std::cout << " create frame " << ms << std::endl ;

    return resul ;
}

void getMatchedKeyPoints( std::vector<SRef<Keypoint>> & keyPoints1,  std::vector<SRef<Keypoint>> & keyPoints2,  std::vector<DescriptorMatch> & matches, std::vector<SRef<Point2Df>> & matchedKeyPoints1, std::vector<SRef<Point2Df>> & matchedKeyPoints2){
  matchedKeyPoints1.reserve(matches.size()); // allocate memory
  for( int i = 0; i < matches.size(); i++ ){
       matchedKeyPoints1.push_back(xpcf::utils::make_shared<Point2Df>(keyPoints1[ matches[i].getIndexInDescriptorA()]->getX(),keyPoints1[ matches[i].getIndexInDescriptorA()]->getY()));
       matchedKeyPoints2.push_back(xpcf::utils::make_shared<Point2Df>(keyPoints2[ matches[i].getIndexInDescriptorB()]->getX(),keyPoints2[ matches[i].getIndexInDescriptorB()]->getY()));
     }
}

void getPoint2DFromKeyPoint(std::vector<SRef<Keypoint>> & keyPoints, std::vector<SRef<Point2Df>> & to2D)
{
    to2D.reserve(keyPoints.size());
    for (int i = 0; i < keyPoints.size(); i++)
    {
        to2D.push_back(xpcf::utils::make_shared<Point2Df>(keyPoints[i]->getX(), keyPoints[i]->getY()));
    }
}




bool addFrameToMapAsKeyFrame(SRef<Frame> & frame, int newIndex)
{
    SRef<Keyframe> referenceKeyFrame = frame->getReferenceKeyFrame();
    Transform3Df poseFrame = frame->m_pose;
    Transform3Df poseKeyFrame = referenceKeyFrame->m_pose;

    std::vector<SRef<Point2Df>> pointsFrame;
    std::vector<SRef<Point2Df>> pointsKeyFrame;

    std::vector<SRef<Keypoint>> kp1,kp2;
    kp1=referenceKeyFrame->getKeyPoints();
    kp2=frame->getKeyPoints();

    getMatchedKeyPoints(kp1, kp2, frame->getUnknownMatchesWithReferenceKeyFrame(), pointsKeyFrame, pointsFrame);

    std::vector<SRef<CloudPoint>> newMapPoints;
    std::pair<int, int> corres(referenceKeyFrame->m_idx , newIndex);

    // Triangulate new points
    mapper->triangulate(pointsFrame, pointsKeyFrame, frame->getUnknownMatchesWithReferenceKeyFrame(), corres, frame->m_pose, referenceKeyFrame->m_pose, K, dist, newMapPoints);

    // check point cloud
    std::vector<bool> tmp_status;
    if (!mapFilter->checkFrontCameraPoints(newMapPoints, frame->m_pose, tmp_status))
    {
        // not good triangulation : do not add key frame
        std::cout << "not good triangulation : do not add key frame " << std::endl;
        return false;
    }
    //filter point cloud
    std::vector<SRef<CloudPoint>> filteredPoints;
    mapFilter->filterPointCloud(newMapPoints, tmp_status, filteredPoints);



    referenceKeyFrame->addVisibleMapPoints(filteredPoints);
    poseGraph->getMap()->addCloudPoints(filteredPoints);

    SRef<Keyframe> newKeyFrame = xpcf::utils::make_shared<Keyframe>(frame->getDescriptors(), newIndex, frame->m_pose, frame->getKeyPoints());

    newKeyFrame->addVisibleMapPoints(frame->getCommonMapPointsWithReferenceKeyFrame());
    newKeyFrame->addVisibleMapPoints(filteredPoints);

    // update visibility of common points
    std::vector<SRef<CloudPoint>> & commonPoints = frame->getCommonMapPointsWithReferenceKeyFrame();
    std::vector<DescriptorMatch> & knownMatches = frame->getKnownMatchesWithReferenceKeyFrame();
    for (int i = 0; i < commonPoints.size(); i++)
    {
        commonPoints[i]->m_visibility[newIndex] = knownMatches[i].getIndexInDescriptorB();// Clean needed : Try to do this somewhere else

    }

    std::cout << " add new keyframe  with " << filteredPoints.size() << "points" << std::endl;
    poseGraph->addNewKeyFrame(newKeyFrame);

    // update viewer
    viewerGL.AddKeyFrameCameraPose(newKeyFrame->m_pose);

    return true;
}


bool init_mapping(SRef<Image>&view_1,SRef<Image>&view_2){
 SRef<Frame> frame1 = createAndInitFrame(view_1);
 SRef<Frame> frame2 = createAndInitFrame(view_2);

  std::vector<DescriptorMatch>  matches;

  SRef<DescriptorBuffer> d1=frame1->getDescriptors();
  SRef<DescriptorBuffer> d2=frame2->getDescriptors();
  matcher->match(d1, d2, matches);

 std::vector<SRef<Point2Df>>    matchedKeypoints1;
 std::vector<SRef<Point2Df>>    matchedKeypoints2;
 std::vector<SRef<Keypoint>>    kp1,kp2;
 kp1=frame1->getKeyPoints();
 kp2=frame2->getKeyPoints();

 getMatchedKeyPoints(kp1, kp2 , matches, matchedKeypoints1 , matchedKeypoints2);
 int vizPoints0 = matches.size();

  // Draw the matches in a dedicated image
 overlay->drawMatchesLines(view_1, view_2, viewerImage1, matchedKeypoints1, matchedKeypoints2, vizPoints0);

 std::vector<DescriptorMatch>  ggmatches;

 matchesFilterGeometric->filter(matches,ggmatches,frame1->getKeyPoints(), frame2->getKeyPoints());

 std::vector<SRef<Point2Df>> ggmatchedKeypoints1;
 std::vector<SRef<Point2Df>> ggmatchedKeypoints2;
 kp1=frame1->getKeyPoints();
 kp2=frame2->getKeyPoints();

 getMatchedKeyPoints(kp1, kp2 ,ggmatches, ggmatchedKeypoints1 ,  ggmatchedKeypoints2 );

 int vizPoints2 = int(ggmatches.size());
 overlay->drawMatchesLines(view_1 ,view_2, viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2,vizPoints2);

 viewer->display("original matches", viewerImage1,27,1280,480);
 viewer->display("filtred matches (epipolar)", viewerImage3, 27,1280,480);

 fundamentalFinder->find(ggmatchedKeypoints1, ggmatchedKeypoints2,F);
 std::vector<Transform3Df>  poses;
 fundamentalDecomposer->decompose(F,K,dist,poses);
 Transform3Df  pose_canonique;
 pose_canonique.setIdentity() ;
 std::pair<int,int>working_view = std::make_pair(0,1);
 Transform3Df pose_final;
 std::vector<SRef<CloudPoint>> tempCloud ;
     if(fullTriangulation(ggmatchedKeypoints1,ggmatchedKeypoints2,ggmatches, working_view,pose_canonique,poses,K,dist,pose_final,tempCloud)){

         SRef<Keyframe> kframe1 = xpcf::utils::make_shared<Keyframe>(view_1,frame1->getDescriptors(),0,pose_canonique, frame1->getKeyPoints());
         SRef<Keyframe> kframe2 = xpcf::utils::make_shared<Keyframe>(view_2,frame2->getDescriptors(),1,pose_final,frame2->getKeyPoints());


         kframe1->addVisibleMapPoints(tempCloud);
         kframe2->addVisibleMapPoints(tempCloud) ;
         poseGraph->initMap(kframe1,kframe2,tempCloud,ggmatches);

		 bool saving_cloud = true;
		 if (saving_cloud) {
			 std::ofstream oy("D:/cloud_solar.txt");
			 oy << tempCloud.size() << std::endl;
			 for (int j = 0; j < tempCloud.size(); ++j) {
				 oy << tempCloud[j]->getX() << " " << tempCloud[j]->getY() << " " << tempCloud[j]->getZ() << std::endl;
			 }
			 oy.close();
		 }

         Point3Df gravity  ;
         float maxDist ;
         poseGraph->getMap()->computeGravity(gravity , maxDist) ;
         nbFrameSinceKeyFrame = 0;

         // update viewer
         viewerGL.m_glcamera.resetview(math_vector_3f(gravity.getX(), gravity.getY(), gravity.getZ()), maxDist);
         viewerGL.m_glcamera.rotate_180();
         viewerGL.AddKeyFrameCameraPose(pose_canonique);
         viewerGL.AddKeyFrameCameraPose(pose_final);


         return true;
     }
     else{
         std::cerr<<"can't find good baseline, select another pair of images for triangulation.."<<std::endl;
         return false;
     }

}

SRef<Image>currentMatchImage;
SRef<Image>projected_image;

bool tracking(SRef<Image>&view){
    nbFrameSinceKeyFrame++ ;
    SRef<Frame> newFrame = createAndInitFrame(view);
    poseGraph->associateReferenceKeyFrameToFrame(newFrame) ;
    newFrame->setNumberOfFramesSinceLastKeyFrame(nbFrameSinceKeyFrame);
    std::vector<DescriptorMatch>new_matches, new_matches_filtred;
    SRef<Keyframe> referenceKeyFrame = newFrame->getReferenceKeyFrame() ;
    SRef<DescriptorBuffer> d1=referenceKeyFrame->getDescriptors();
    SRef<DescriptorBuffer> d2=newFrame->getDescriptors();

    matcher->match(d1,d2, new_matches);
    matchesFilterGeometric->filter(new_matches,new_matches_filtred, referenceKeyFrame->getKeyPoints(), newFrame->getKeyPoints());

    std::vector<SRef<Keypoint>> kp1,kp2;
    std::vector<SRef<Point2Df>>current_kp1;
    std::vector<SRef<Point2Df>>current_kp2;

    kp1=referenceKeyFrame->getKeyPoints();
    kp2=newFrame->getKeyPoints();
    getMatchedKeyPoints(kp1, kp2, new_matches_filtred, current_kp1, current_kp2);

   
    overlay->drawMatchesLines(referenceKeyFrame->m_view, view, currentMatchImage, current_kp1, current_kp2,current_kp1.size());
    viewer->display("current matches", currentMatchImage, &escape_key,1280,480);

    std::vector<SRef<Point2Df>>pt2d;
    std::vector<SRef<Point3Df>>pt3d;
    std::vector<SRef<CloudPoint>> foundPoints;
    std::vector<DescriptorMatch> foundMatches;
    std::vector<DescriptorMatch> remainingMatches;


    corr2D3DFinder->find(referenceKeyFrame->getVisibleMapPoints(),referenceKeyFrame->m_idx,new_matches_filtred, newFrame->getKeyPoints(), foundPoints,  pt3d,pt2d , foundMatches , remainingMatches);

    std::vector<SRef<Point2Df>>imagePoints_inliers;
    std::vector<SRef<Point3Df>>worldPoints_inliers;

    Transform3Df pose_current;
    if(PnP->estimate(pt2d,pt3d,imagePoints_inliers, worldPoints_inliers, pose_current) == FrameworkReturnCode::_SUCCESS /*&&
            worldPoints_inliers.size()> 50*/){
       // std::cout<<" pnp inliers size: "<<worldPoints_inliers.size()<<" / "<<pt3d.size()<<std::endl;


		std::cout << " number of iniers: " << imagePoints_inliers.size() << std::endl;
		std::vector<int>color = { 0,0,255 };
		overlay2d->drawCircles(imagePoints_inliers, 2.0, 2.0, view);
		viewer->display("inlier image", view);
		
		newFrame->m_pose = pose_current;
        DrawPointCloud(pose_current);
        DrawPnpMatches(pt2d,pt3d,pose_current);


        newFrame->m_pose = pose_current;
        viewerGL.SetRealCameraPose(pose_current);

        newFrame->addCommonMapPointsWithReferenceKeyFrame(foundPoints);
        newFrame->setUnknownMatchesWithReferenceKeyFrame(remainingMatches);
		newFrame->setKnownMatchesWithReferenceKeyFrame(foundMatches);

		// I need to check this function

		int isKeyFrameCandidate = poseGraph->isKeyFrameCandidate(newFrame);
		if (isKeyFrameCandidate != -1) // try to add key frame if success tracking
		{
			
			std::cout << "/.trying to add keyframe./" << std::endl;

			if (addFrameToMapAsKeyFrame(newFrame, isKeyFrameCandidate))
			{
				nbFrameSinceKeyFrame = 0;
			}
		}

        newFrame->setKnownMatchesWithReferenceKeyFrame(foundMatches);

        if (isKeyFrameCandidate != -1) // try to add key frame if success tracking
        {

            if (addFrameToMapAsKeyFrame(newFrame, isKeyFrameCandidate))
            {
                nbFrameSinceKeyFrame = 0;
            }
        }
        return true;
    }else{
       // std::cout<<"new keyframe creation.."<<std::endl;
        return false;
    }
}



void idle(){
    if(exit_){
        exit(0);
    }
    if(pause && streamSource != "camera"){
        return;
    }
    camera->getNextImage(view_current);
    frameCount++;
    viewer->display("current view",view_current, 27);

    // in case of video as input : simulate keyboard input
    bool tryProcess = false;
    if (streamSource != "camera")
    {
        if (frameCount == indexFirstKeyFrame)
        {
            keyBoard('s');
        }
        else if (frameCount == indexSecondKeyFrame)
        {
            keyBoard('s');
        }

    }
    if(saving_images)
    {
      views.push_back(view_current);
      saving_images=false;
    }
    if (triangulation_first&& views.size()>1) {
        if(init_mapping(views[0],views[1])){
            triangulation_first = false;
            keyBoard('d');  // display
            keyBoard('p'); // processing
        }else{
            views.clear();
        }
    }
    if(processing && !triangulation_first)
    {
       tracking(view_current);
    }
}

int main (int argc, char* argv[]){

    boost::log::core::get()->set_logging_enabled(false);
    std::string configFile=std::string("slamConfig.txt");
    if(argc==2)
        configFile=std::string(argv[1]);

    init(configFile);

    viewerGL.callBackIdle = idle ;
    viewerGL.callbackKeyBoard = keyBoard;
    viewerGL.InitViewer(640 , 480);
    return 0;

}






