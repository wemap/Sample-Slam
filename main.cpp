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
std::vector<SRef<Image>>static_views;

const int static_views_no = 10;
int static_views_counter = 4;
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
    case '+': {
        ++static_views_counter;
        std::cout<<" track view: "<<static_views_counter<<std::endl;
        break;
    }
    case '-': {
        --static_views_counter;
        std::cout<<" track view: "<<static_views_counter<<std::endl;
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
       xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherHammingBruteForceOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);
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
       xpcf::ComponentFactory::createComponent<SolARPoseEstimationPnpOpencv>(gen(solver::pose::I3DTransformFinder::UUID), PnP);
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
	   else{
           camera->start(streamSource);
		   //std::cout << "	# loading frames: ";
		   //// this part to load image from video:
       }


       std::cout<<"<loading static view: ";
       static_views.resize(static_views_no);
       for(int k = 0; k < static_views_no; ++k){
           std::stringstream buff;
           buff<<std::setfill('0')<<std::setw(5)<<k;
           std::string path_temp = streamSource +   buff.str() + ".png";
           std::cout<<" temp: "<<path_temp<<std::endl;
           imageLoader->loadImage(path_temp,static_views[k]);
           std::cout<<"     ->img size: "<<static_views[k]->getWidth()<<" "<<static_views[k]->getHeight()<<std::endl;
           viewer->display("__view",static_views[k]);
       }
       std::cout<<" done"<<std::endl;

}
bool debug_coplanarity(){

    std::ifstream pt1_log("D:/debug_triangulation/imgpts1_good.txt");
    std::ifstream pt2_log("D:/debug_triangulation/imgpts2_good.txt");

    std::ifstream P_log("D:/debug_triangulation/P.txt");
    std::ifstream P1_log("D:/debug_triangulation/P1.txt");
    std::ifstream K_log("D:/debug_triangulation/K.txt");


    int pt_no1, pt_no2, match_no;

    pt1_log>>pt_no1;
    pt2_log>>pt_no2;

    std::cout<<" number of points to triangulate"<<pt_no1<<std::endl;
    std::vector<SRef<Point2Df>>pt2d_1,pt2d_2;
    std::vector<DescriptorMatch>  matches;
    CamCalibration cam;
    CamDistortion dist;
    Transform3Df corrected_pose, p1;
    std::pair<int, int>working_views = std::make_pair(0,1);
    std::vector<bool> tmp_status;
    std::vector<SRef<CloudPoint>>   pcloud;
    std::vector<SRef<CloudPoint>>   pcloud1;


    pt2d_1.resize(pt_no1);
    pt2d_2.resize(pt_no2);


    for(int i = 0; i <pt_no1; ++i){
        float x1,y1;
        pt1_log>>x1; pt1_log>>y1;
        pt2d_1[i] =   sptrnms::make_shared<Point2Df>(x1,y1);

        float x2,y2;
        pt2_log>>x2; pt2_log>>y2;
        pt2d_2[i] =   sptrnms::make_shared<Point2Df>(x2,y2);

        /*
        std::cout<<"pt1: "<<pt2d_1[i]->getX()<<" "<<pt2d_1[i]->getY()<<std::endl;
        std::cout<<"pt2: "<<pt2d_2[i]->getX()<<" "<<pt2d_2[i]->getY()<<std::endl;
       cv::waitKey(0);*/

    }



    for(int i = 0; i < 3; ++i){
        for(int j = 0; j<  3; ++j){
            K_log>>cam(i,j);
        }
    }

    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 4; ++j){
            P1_log>>corrected_pose(i,j);
            P_log>>p1(i,j);
        }
    }

    for(int  i = 0; i < 5; ++i){
        dist(i,0) = 0.f;
    }

    std::cout<<"----P1: "<<std::endl;
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j<  4; ++j){
            std::cout<<p1(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;

    std::cout<<"----P2: "<<std::endl;
    for(int i = 0; i < 3; ++i){
         for(int j = 0; j<  4; ++j){
               std::cout<<corrected_pose(i,j)<<" ";
             }
          std::cout<<std::endl;
        }
    std::cout<<std::endl;

    std::cout<<"----K: "<<std::endl;
    for(int i = 0; i < 3; ++i){
         for(int j = 0; j<  3; ++j){
               std::cout<<cam(i,j)<<" ";
             }
          std::cout<<std::endl;
        }
    std::cout<<std::endl;

    std::cout<<"----dist: "<<std::endl;
    for(int i = 0; i < 5; ++i){

        std::cout<<dist(i,0)<<" ";
        }
    std::cout<<std::endl;

    double reproj_error1 = mapper->triangulate(pt2d_1, pt2d_2, matches, working_views, p1, corrected_pose, cam, dist, pcloud);
    double reproj_error2 = mapper->triangulate(pt2d_2, pt2d_1, matches, working_views, corrected_pose, p1, cam, dist, pcloud1);

    std::cout<<"    err1: :"<<reproj_error1<<"   err2: "<<reproj_error2<<std::endl;
    std::ofstream oxx("D:/solar_cloud_temp.txt");
    oxx<<pcloud.size()<<std::endl;
    for(const auto &p: pcloud){
        oxx<<p->getX()<<" "<<p->getY()<<" "<<p->getZ()<<std::endl;
    }
    oxx.close();

    std::cout<<" "<<mapFilter->checkFrontCameraPoints(pcloud, corrected_pose, tmp_status)<<" "
                   <<mapFilter->checkFrontCameraPoints(pcloud1, p1, tmp_status)<<std::endl;

return true;

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
                        std::vector<SRef<CloudPoint>>& cloud){

    if (p2.size() != 4) {
        std::cerr << " number of decomposed poses supposed to be 4.." << std::endl;
        return false;
    }
    std::vector<SRef<CloudPoint>>   pcloud;
    std::vector<SRef<CloudPoint>>   pcloud1;
    std::vector<bool> tmp_status;

    //check if points are triangulated --in front-- of cameras for all 4 ambiguations
    for (int i = 0; i < 4; i++){
        corrected_pose = p2[i];
        pcloud.clear(); pcloud1.clear();

        double reproj_error1 = mapper->triangulate(pt2d_1, pt2d_2, matches, working_views, p1, corrected_pose, cam, dist, pcloud);
        double reproj_error2 = mapper->triangulate(pt2d_2, pt2d_1, matches, working_views, corrected_pose, p1, cam, dist, pcloud1);

        /*
        std::cout<<"    err1: :"<<reproj_error1<<"   err2: "<<reproj_error2<<std::endl;
        std::cout<<" "<<mapFilter->checkFrontCameraPoints(pcloud, corrected_pose, tmp_status)<<" "
                       <<mapFilter->checkFrontCameraPoints(pcloud1, p1, tmp_status)<<std::endl;
        */
        if (mapFilter->checkFrontCameraPoints(pcloud, corrected_pose, tmp_status) && mapFilter->checkFrontCameraPoints(pcloud1, p1, tmp_status) && reproj_error1 < 100.0 && reproj_error2 < 100.0)
        {
            // points are in front of camera and reproj error is acceptable
            break;
        }
        if (i == 3){
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



bool addFrameToMapAsKeyFrame(SRef<Frame> & frame,SRef<Image> &view, int newIndex)

{


    SRef<Keyframe> referenceKeyFrame = frame->getReferenceKeyFrame();
    Transform3Df poseFrame = frame->m_pose;
    Transform3Df poseKeyFrame = referenceKeyFrame->m_pose;

    std::vector<SRef<Point2Df>> pointsFrame;
    std::vector<SRef<Point2Df>> pointsKeyFrame;

    std::vector<SRef<Keypoint>> kp1,kp2;
    kp1=referenceKeyFrame->getKeyPoints();
    kp2=frame->getKeyPoints();

    // triangulate all points from keyFrame?
    getMatchedKeyPoints(kp1, kp2, frame->getUnknownMatchesWithReferenceKeyFrame(), pointsKeyFrame, pointsFrame);

    std::cout<<"    ->match points: "<<pointsFrame.size()<<std::endl;

    std::vector<SRef<CloudPoint>> newMapPoints;
    std::pair<int, int> corres(referenceKeyFrame->m_idx , newIndex);

    // Triangulate new points
    mapper->triangulate(pointsFrame, pointsKeyFrame, frame->getUnknownMatchesWithReferenceKeyFrame(), corres, frame->m_pose, referenceKeyFrame->m_pose, K, dist, newMapPoints);

    std::cout<<"    ->new 3d points: "<<newMapPoints.size()<<std::endl;

    // check point cloud
    std::vector<bool> tmp_status;
    if (!mapFilter->checkFrontCameraPoints(newMapPoints, frame->m_pose, tmp_status)){
        // not good triangulation : do not add key frame
        std::cout << "not good triangulation : do not add key frame " << std::endl;
        return false;
    }
    //filter point cloud
    std::vector<SRef<CloudPoint>> filteredPoints;
    mapFilter->filterPointCloud(newMapPoints, tmp_status, filteredPoints);


    std::cout<<"  filtred point size: "<<filteredPoints.size()<<std::endl;


    referenceKeyFrame->addVisibleMapPoints(filteredPoints);
    poseGraph->getMap()->addCloudPoints(filteredPoints);

    SRef<Keyframe> newKeyFrame = xpcf::utils::make_shared<Keyframe>(view,frame->getDescriptors(), newIndex, frame->m_pose, frame->getKeyPoints());

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
bool init_mapping(SRef<Image>&view_1,SRef<Image>&view_2, const std::string& path_cloud = std::string()){
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
            std::cout<<" pnp inliers size: "<<worldPoints_inliers.size()<<" / "<<pt3d.size()<<std::endl;

            newFrame->m_pose = pose_current.inverse();
            viewerGL.SetRealCameraPose(pose_current.inverse());

            newFrame->addCommonMapPointsWithReferenceKeyFrame(foundPoints);
            newFrame->setUnknownMatchesWithReferenceKeyFrame(remainingMatches);
            newFrame->setKnownMatchesWithReferenceKeyFrame(foundMatches);

         /*
            if (addFrameToMapAsKeyFrame(newFrame, 2)) {
                nbFrameSinceKeyFrame = 0;
            }
            */
            /*
            if (isKeyFrameCandidate != -1) // try to add key frame if success tracking
            if (addFrameToMapAsKeyFrame(newFrame,view, isKeyFrameCandidate))

            {

                if (addFrameToMapAsKeyFrame(newFrame, isKeyFrameCandidate))
                {
                    nbFrameSinceKeyFrame = 0;
                }
            }
            */
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
void idle_static(){
    if(exit_){
        exit(0);
    }
    if(triangulation_first){
        std::string path_cloud = "";
        if(init_mapping(static_views[2],static_views[3], path_cloud)){
             triangulation_first = false;
             std::cout<<" done"<<std::endl;
        }
    }
    if(processing && !triangulation_first)
    {
        std::cout<<" frame to track.."<<std::endl;
        viewer->display("fram to track",static_views[static_views_counter]);
       tracking(static_views[static_views_counter]);
            //   ++static_views_counter;
    }
}
int main (int argc, char* argv[]){

    boost::log::core::get()->set_logging_enabled(false);
    std::string configFile=std::string("D:/AmineSolar/source/slam/Sample-Slam/slamStaticConfig.txt");
//    std::string configFile=std::string("D:/AmineSolar/source/slam/Sample-Slam/slamConfig.txt");

    if(argc==2)
        configFile=std::string(argv[1]);

    init(configFile);
 //   debug_coplanarity();
 //   debug_triangulation();
 //   viewerGL.callBackIdle = idle ;
    viewerGL.callBackIdle = idle_static;

    viewerGL.callbackKeyBoard = keyBoard;
    viewerGL.InitViewer(640 , 480);
    return 0;
}
}




