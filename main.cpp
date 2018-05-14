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


#include <ctime>


void keyBord(unsigned char key){
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
    default:
        break;
    }
}
void init(){
    // component creation

       xpcf::ComponentFactory::createComponent<SolARCameraOpencv>(gen(input::devices::ICamera::UUID), camera);
       xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader);

       xpcf::ComponentFactory::createComponent<SolARKeypointDetectorNonFreeOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
       xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSURF64Opencv>(gen(features::IDescriptorsExtractor::UUID ), descriptorExtractor);

   //    xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherKNNOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);

       xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherRadiusOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);

       xpcf::ComponentFactory::createComponent<SolARSideBySideOverlayOpencv>(gen(display::ISideBySideOverlay::UUID ), overlay);
       xpcf::ComponentFactory::createComponent<SolAR2DOverlayOpencv>(gen(display::I2DOverlay::UUID ), overlay2d);

       xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);
       xpcf::ComponentFactory::createComponent<SolARGeometricMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ), matchesFilterGeometric);
       xpcf::ComponentFactory::createComponent<SolARFundamentalMatrixEstimationOpencv>(gen(solver::pose::I2DTransformFinder::UUID ), fundamentalFinder);
       xpcf::ComponentFactory::createComponent<SolARSVDFundamentalMatrixDecomposerOpencv>(gen(solver::pose::I2DTO3DTransformDecomposer::UUID ), fundamentalDecomposer);
       xpcf::ComponentFactory::createComponent<SolARSVDTriangulationOpencv>(gen(solver::map::ITriangulator::UUID ), mapper);
       xpcf::ComponentFactory::createComponent<SolARMapperOpencv>(gen(solver::map::IMapper::UUID), poseGraph);
       xpcf::ComponentFactory::createComponent<SolARPoseEstimationPnpOpencv>(gen(solver::pose::I3DTransformFinder::UUID), PnP);

       xpcf::ComponentFactory::createComponent<SolAR2D3DCorrespondencesFinderOpencv>(gen(solver::pose::I2D3DCorrespondencesFinder::UUID), corr2D3DFinder);


       keypointsDetector->setType(features::KeypointDetectorType::SURF);       
       // load camera parameters from yml input file
        std::string cameraParameters = std::string("calib.yml");

       camera->loadCameraParameters(cameraParameters);
       PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
       K = camera->getIntrinsicsParameters();
       dist = camera->getDistorsionParameters();

       camera->start(0);
}


SRef<Frame> createAndInitFrame(SRef<Image>&img){
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
bool init_mapping(SRef<Image>&view_1,SRef<Image>&view_2, bool verbose){
 SRef<Frame> frame1 = createAndInitFrame(views[0]);
 SRef<Frame> frame2 = createAndInitFrame((views[1])) ;

 std::cout<<"--< Descriptors matching: "<<std::endl;
 matcher->match(frame1->getDescriptors(), frame2->getDescriptors(), matches);
 std::cout<<"     #original matches: "<<matches.size()<<std::endl;
 if(verbose){
     int vizPoints0 = int(matches.size());
    matchedKeypoints1.clear();
    matchedKeypoints2.clear();
    for( int i = 0; i < matches.size(); i++ ){
     matchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(frame1->getKeyPoints()[ matches[i].getIndexInDescriptorA()]->getX(),frame1->getKeyPoints()[ matches[i].getIndexInDescriptorA()]->getY()));
     matchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(frame2->getKeyPoints()[ matches[i].getIndexInDescriptorB()]->getX(),frame2->getKeyPoints()[ matches[i].getIndexInDescriptorB()]->getY()));
     }
    // Draw the matches in a dedicated image
     overlay->drawMatchesLines(views[0], views[1], viewerImage1, matchedKeypoints1, matchedKeypoints2, vizPoints0);
    int vizPoints1 = int(gmatches.size());
    overlay->drawMatchesLines(views[0],views[1], viewerImage2, gmatchedKeypoints1, gmatchedKeypoints2,vizPoints1);
 }
 matchesFilterGeometric->filter(matches,ggmatches,frame1->getKeyPoints(), frame2->getKeyPoints());
 if(verbose){
    std::cout<<"    #filtred matches: "<<ggmatches.size()<<std::endl;
    ggmatchedKeypoints1.clear();
    ggmatchedKeypoints2.clear();
    for( int i = 0; i < ggmatches.size(); i++ ){
       ggmatchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(frame1->getKeyPoints()[ggmatches[i].getIndexInDescriptorA()]->getX(),frame1->getKeyPoints()[ ggmatches[i].getIndexInDescriptorA()]->getY()));
       ggmatchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(frame2->getKeyPoints()[ggmatches[i].getIndexInDescriptorB()]->getX(),frame2->getKeyPoints()[ ggmatches[i].getIndexInDescriptorB()]->getY()));
    }
    int vizPoints2 = int(ggmatches.size());
    overlay->drawMatchesLines(views[0], views[1], viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2,vizPoints2);
    viewer->display("original matches", viewerImage1, &escape_key,1280,480);
    viewer->display("filtred matches (epipolar)", viewerImage3, &escape_key,1280,480);
    }
   std::cout<<"--<Triangulation: "<<std::endl;
   std::cout<<"    #Fundamental calculation"<<std::endl;
   fundamentalFinder->find(ggmatchedKeypoints1, ggmatchedKeypoints2,F);
   std::cout<<"    #Fundamental decomposition"<<std::endl;
   fundamentalDecomposer->decompose(F,K,dist,poses);
   pose_canonique.setIdentity() ;

   std::cout<<"    #Full triangulation"<<std::endl;
   std::cout<<"    #pt2d1 size: "<<ggmatchedKeypoints1.size()<<std::endl;
   std::cout<<"    #pt2d2 size: "<<ggmatchedKeypoints2.size()<<std::endl;
   std::cout<<"    #matches size: "<<ggmatches.size()<<std::endl;

   std::pair<int,int>working_view = std::make_pair(0,1);
   std::cout<<"   #full triangulation+filtering"<<std::endl;
   std::vector<SRef<CloudPoint>> tempCloud ;
     if(mapper->triangulateFull(ggmatchedKeypoints1,ggmatchedKeypoints2,ggmatches, working_view,pose_canonique,poses,K,dist,pose_final,tempCloud)){
         std::cout<<"   #final cloud size: "<<tempCloud.size()<<std::endl;
        /* std::cout<<"gravity center: "<<gravity<<std::endl;*/
         // to do : move these two lines elsewhere :
         SRef<Keyframe> kframe1 = xpcf::utils::make_shared<Keyframe>(view_1,frame1->getDescriptors(),0,pose_canonique, frame1->getKeyPoints());
         SRef<Keyframe> kframe2 = xpcf::utils::make_shared<Keyframe>(view_2,frame2->getDescriptors(),1,pose_final,frame2->getKeyPoints());
         kframe1->addVisibleMapPoints(tempCloud);
         kframe2->addVisibleMapPoints(tempCloud) ;
         std::cout<<"   #map init"<<std::endl;
         poseGraph->initMap(kframe1,kframe2,tempCloud,ggmatches);
         std::cout<<"--<Pose graph: "<<std::endl;
         std::cout<<"     # kframe(t): "<<kframe1->m_idx<<std::endl;
         std::cout<<"     # kframe(t+1): "<<kframe2->m_idx<<std::endl;
         Point3Df gravity  ;
         float maxDist ;

         std::cout << " compute gravity" << std::endl ;
         poseGraph->getMap()->computeGravity(gravity , maxDist) ;
         viewerGL.m_glcamera.resetview(math_vector_3f(gravity.getX(), gravity.getY(), gravity.getZ()), maxDist);
         viewerGL.m_glcamera.rotate_180();
		 nbFrameSinceKeyFrame = 0 ;
         return true;
     }
     else{
         std::cerr<<"can't find good baseline, select another pair of images for triangulation.."<<std::endl;
         return false;
     }

}
bool tracking(SRef<Image>&view, const int kframe_idx, bool verbose){	
	nbFrameSinceKeyFrame++ ;
    auto t0 = std::clock();
	SRef<Frame> newFrame = createAndInitFrame(view);
    poseGraph->associateReferenceKeyFrameToFrame(newFrame) ;
    newFrame->setNumberOfFramesSinceLastKeyFrame(nbFrameSinceKeyFrame);
//    std::cout<<"    ->time image pross: "<<std::clock() - t0<<std::endl;
    std::vector<DescriptorMatch>new_matches, new_matches_filtred;
	SRef<Keyframe> referenceKeyFrame = newFrame->getReferenceKeyFrame() ;
    auto t1 = std::clock();
    matcher->match(referenceKeyFrame->getDescriptors(), newFrame->getDescriptors(), new_matches);
//    std::cout<<"    ->time matching:  "<<std::clock() - t1<<std::endl;
    auto t2 = std::clock();
    matchesFilterGeometric->filter(new_matches,new_matches_filtred, referenceKeyFrame->getKeyPoints(), newFrame->getKeyPoints());
    std::cout<<"    ->time filtering: "<<std::clock() - t2<<std::endl;
    std::vector<SRef<Point2Df>>current_kp1;
    std::vector<SRef<Point2Df>>current_kp2;
    for( int i = 0; i < new_matches_filtred.size(); i++ ){
       current_kp1.push_back(xpcf::utils::make_shared<Point2Df>(referenceKeyFrame->getKeyPoints()[new_matches_filtred[i].getIndexInDescriptorA()]->getX(),
                             referenceKeyFrame->getKeyPoints()[ new_matches_filtred[i].getIndexInDescriptorA()]->getY()));
       current_kp2.push_back(xpcf::utils::make_shared<Point2Df>(newFrame->getKeyPoints()[new_matches_filtred[i].getIndexInDescriptorB()]->getX()
                             ,newFrame->getKeyPoints()[ new_matches_filtred[i].getIndexInDescriptorB()]->getY()));
    }

    SRef<Image>currentMatchImage;
    overlay->drawMatchesLines(referenceKeyFrame->m_view, view, currentMatchImage, current_kp1, current_kp2,current_kp1.size());
    viewer->display("current matches", currentMatchImage, &escape_key,1280,480);

    std::vector<SRef<Point2Df>>pt2d;
    std::vector<SRef<Point3Df>>pt3d;
    std::vector<SRef<CloudPoint>> foundPoints ;
    std::vector<DescriptorMatch> remainingMatches ;
    auto t3 = std::clock();
    corr2D3DFinder->find(referenceKeyFrame->getVisibleMapPoints(),referenceKeyFrame->m_idx,new_matches_filtred, newFrame->getKeyPoints(), foundPoints,  pt3d,pt2d , remainingMatches);
//    std::cout<<"    ->time 2d3d: "<<std::clock() - t3<<std::endl;
    SRef<Image>projected_image;
    auto t4 = std::clock();
    if(PnP->estimate(pt2d,pt3d,pose_current, true) == FrameworkReturnCode::_SUCCESS){
        std::cout<<"    ->time pnp: "<<std::clock() - t4<<std::endl;

        newFrame->m_pose = pose_current ;
        viewerGL.SetRealCameraPose(pose_current);

        newFrame->addCommonMapPointsWithReferenceKeyFrame(foundPoints);
        newFrame->setMatchesWithReferenceKeyFrame(remainingMatches);
        if (poseGraph->tryToAddKeyFrame(newFrame)) // try to add key frame if success tracking
        {
            nbFrameSinceKeyFrame = 0 ;
            std::cout << " add new key frame in the map"  << std::endl ;
        }

        return true;
    }else{
        return false;
    }	
}
/*
SRef<Image>  im1 ;
SRef<Image> im2 ;
bool first = true  ;*/



void my_idle(){
    if(triangulation_first = true){
        imageLoader->loadImage(std::string("D:/view_0.png"),view_current);
        views.push_back(view_current);
        imageLoader->loadImage(std::string("D:/view_1.png"),view_current);
        views.push_back(view_current);
        init_mapping(views[0],views[1], true);
        triangulation_first = false;
    }
    imageLoader->loadImage(std::string("D:/view_2.png"),view_current);
    tracking(view_current,1,true);
}

void idle(){
    camera->getNextImage(view_current);
    viewer->display("current view",view_current, &escape_key);
    if(saving_images){
      views.push_back(view_current);
      saving_images=false;
    }
    if (triangulation_first&& views.size()>1) {
        if(init_mapping(views[0],views[1], true)){
            triangulation_first = false;
        }else{
            views.clear();
        }
    }
    if(processing){
       tracking(view_current,1 /*kframe_idx*/,true);
    }
}
int main (int argc, char* argv[]){
    init();
//    viewerGL.callBackIdle = idle ;
    viewerGL.callBackIdle = idle ;

    viewerGL.callbackKeyBoard = keyBord;
    viewerGL.InitViewer(640 , 480);
    return 0;
}


/*
int main(){
    init();
    if(triangulation_first = true){
        imageLoader->loadImage(std::string("D:/view_0.png"),view_current);
        views.push_back(view_current);
        imageLoader->loadImage(std::string("D:/view_1.png"),view_current);
        views.push_back(view_current);
        init_mapping(views[0],views[1], true);
        triangulation_first = false;
    }
    while(true){
        imageLoader->loadImage(std::string("D:/view_2.png"),view_current);
        tracking(view_current,1,true);
    }
  return 0;
}*/



