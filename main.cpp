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
        viewerGL.SetPointCloudToDisplay(&gcloud);
        std::cout << "drawing started "  << std::endl;
        break;
    }
    case 'p': {
        processing = !processing;
        std::cout << "processing started: " << processing << std::endl;
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
    //   std::string cameraParameters = std::string("D:/AmineSolar/source/slam/build-SolARTriangulationSample/mycamera_calibration0.yml");
       std::string cameraParameters = std::string("D:/Development/SDK/SolARFramework/mycamera_calibration0.yml");

       camera->loadCameraParameters(cameraParameters);
       PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
       K = camera->getIntrinsicsParameters();
       dist = camera->getDistorsionParameters();

       camera->start(0);
}

SRef<Frame> createAndInitFrame(SRef<Image>&img)
{
    SRef<Frame> resul = xpcf::utils::make_shared<Frame>() ;

    std::vector< SRef<Keypoint>> keyPoints  ;
    SRef<DescriptorBuffer>  descriptors ;

    keypointsDetector->detect(img, keyPoints);
    descriptorExtractor->extract(img, keyPoints, descriptors);

    resul->InitKeyPointsAndDescriptors(keyPoints , descriptors);

    return resul ;
}

bool debug_reprojection(){
    init();
    cv::namedWindow("debug_window",0);
    cv::Mat view = cv::imread("D:/AmineSolar/mySLAM/my_validdata_2/0002.png");
    if (!view.empty()) {
        std::cout << " --------------------<debug 2d-3d reprojection>" << std::endl;
        std::cout << "	#loading 2d-3d correspondances:" << std::endl;
        int corr_no = 1817;
        std::vector<SRef<Point2Df>>pt2d_temp;
        std::vector<SRef<Point3Df>>pt3d_temp;
        pt3d_temp.resize(corr_no); pt2d_temp.resize(corr_no);
        std::string corr_path = "D:/corr_cvSlam.txt";
        std::ifstream oxCorr(corr_path);
        std::string dummy;
        for (int i = 0; i < corr_no; ++i) {
            float v[5];
            for (int j = 0; j < 5; ++j) {
                oxCorr >> dummy;
                v[j] = std::stof(dummy);
            }
            pt3d_temp[i] = sptrnms::make_shared<Point3Df>(v[0], v[1], v[2]);
            pt2d_temp[i] = sptrnms::make_shared<Point2Df>(v[3], v[4]);
     //       std::cout << " pt3d: " << pt3d_temp[i]->getX() << " "<<pt3d_temp[i]->getY()<<" "<<
      //                    pt3d_temp[i]->getZ()<<"   pt2d: " << pt2d_temp[i]->getX()<<" "<< pt2d_temp[i]->getY() << std::endl;
       //     cv::waitKey(0);
        }
        std::string thirdImagePath = "D:/AmineSolar/source/slam/build-SolARTriangulationSample/0002.png";
        if (imageLoader->loadImage(thirdImagePath,view_current) != FrameworkReturnCode::_SUCCESS)
        {
           LOG_ERROR("Cannot load image with path {}", std::string());
           return -1;
        }

        SRef<Pose> pose_pnp;
        PnP->estimate(pt2d_temp,pt3d_temp,pose_current);

        std::cout<<" pose solar from pnp: "<<std::endl;

        for(int ii = 0; ii < 4; ++ii){
            for(int jj = 0; jj < 4; ++jj){
                std::cout<<pose_current(ii,jj)<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl<<std::endl;
        std::cout<<"  number of correspondances: "<<pt2d_temp.size()<<"  "<<pt3d_temp.size()<<std::endl;
 //       PnP->reproject(view_current,pose_current,K,dist,pt2d_temp,pt3d_temp);
        viewer->display("2d-3d corr", view_current, &escape_key,640,480);

        cv::waitKey(0);

        /*
        std::pair<int, int>working_pair = std::make_pair<int, int>(0, 1);
        if (vo->computePoseFromPnP(view, working_pair, rvec, t, R, pt3d_temp, pt2d_temp, paramPnP)) {
            current_pose = cv::Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0),
                R(1, 0), R(1, 1), R(1, 2), t(1),
                R(2, 0), R(2, 1), R(2, 2), t(2));

            std::cout << "		# pose: " << t(0) << " " << t(1) << " " << t(2) << std::endl;
            std::cout << "		# tracking: good" << std::endl;
            cv::waitKey(0);
            return true;
        }
        else {
            std::cout << "		# tracking: bad" << std::endl;
            return false;
        }
        */
    }

}
void computeGravity(std::vector<SRef<CloudPoint>>&cloud,cv::Vec3f&grav, float& maxDist){
   grav =  cv::Vec3f(0, 0, 0);
   maxDist  =0.f;

    cv::Vec3f vr_temp(0, 0, 0);
    int count = 0;
    for (int i = 0; i < cloud.size(); ++i) {
        vr_temp = cv::Vec3f(cloud[i]->getX(), cloud[i]->getX(), cloud[i]->getY());
        grav = grav + vr_temp;
        count++;
    }
    maxDist = 0;
    if (count > 0) {
        grav= (1.f / (float)count)*gravity;
        for (int i = 0; i < cloud.size(); ++i) {
            vr_temp = cv::Vec3f(cloud[i]->getX(),cloud[i]->getY(), cloud[i]->getZ());
            cv::Vec3f temp = vr_temp - grav;
            maxDist = MAX(maxDist, (float)norm(vr_temp - grav));
        }
    }
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
     if(mapper->triangulateFull(ggmatchedKeypoints1,ggmatchedKeypoints2,ggmatches, working_view,pose_canonique,poses,K,dist,pose_final,gcloud)){
         std::cout<<"   #final cloud size: "<<gcloud.size()<<std::endl;
         computeGravity(gcloud,gravity, maxDist);
         std::cout<<"gravity center: "<<gravity<<std::endl;
         // to do : move these two lines elsewhere :
         viewerGL.m_glcamera.resetview(math_vector_3f(gravity[0], gravity[1], gravity[2]), maxDist);
         viewerGL.m_glcamera.rotate_180();
         SRef<Keyframe> kframe1 = xpcf::utils::make_shared<Keyframe>(view_1,frame1->getDescriptors(),0,pose_canonique, frame1->getKeyPoints());
         SRef<Keyframe> kframe2 = xpcf::utils::make_shared<Keyframe>(view_2,frame2->getDescriptors(),1,pose_final,frame2->getKeyPoints());
         std::cout<<"   #map init"<<std::endl;
         poseGraph->initMap(kframe1,kframe2,gcloud,ggmatches);
         std::cout<<"--<Pose graph: "<<std::endl;
         std::cout<<"     # kframe(t): "<<kframe1->m_idx<<std::endl;
         std::cout<<"     # kframe(t+1): "<<kframe2->m_idx<<std::endl;
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
	SRef<Frame> newFrame = createAndInitFrame(view);
    poseGraph->AssociateReferenceKeyFrameToFrame(newFrame) ;
    newFrame->setNumberOfFramesSinceLastKeyFrame(nbFrameSinceKeyFrame);

    std::vector<DescriptorMatch>new_matches, new_matches_filtred;

	SRef<Keyframe> referenceKeyFrame = newFrame->getReferenceKeyFrame() ;
    matcher->match(referenceKeyFrame->getDescriptors(), newFrame->getDescriptors(), new_matches);
   
    matchesFilterGeometric->filter(new_matches,new_matches_filtred, referenceKeyFrame->getKeyPoints(), newFrame->getKeyPoints());

    newFrame->setMatchesWithReferenceKeyFrame(new_matches_filtred);

    std::vector<SRef<Point2Df>>pt2d;
    std::vector<SRef<Point3Df>>pt3d;
//    poseGraph->find2D3DCorrespondances(kframe_idx,new_matches_filtred,keypoints3,pt2d,pt3d);
//    std::cout<<"trying to find 2d/3d correspondences!"<<std::endl;
    corr2D3DFinder->find(gcloud,kframe_idx,new_matches_filtred, newFrame->getKeyPoints(),pt3d,pt2d);
    SRef<Image>projected_image;
    if(PnP->estimate(pt2d,pt3d,pose_current) == FrameworkReturnCode::_SUCCESS)
    {
        newFrame->m_pose = pose_current ;
        viewerGL.SetRealCameraPose(pose_current);
        if(verbose){
            PnP->reproject(view,pose_current,K,dist,pt2d,pt3d, projected_image, false);
            viewer->display("pnp reprojection image", projected_image, &escape_key,640,480);
        }
        return true;
    }else{
        return false;
    }
	
	if (poseGraph->tryToAddKeyFrame(newFrame))
    {
        nbFrameSinceKeyFrame = 0 ;
        std::cout << " add new key frame in the map"  << std::endl ;
        // triangulate
    }
	
	
}
void idle(){
    camera->getNextImage(view_current);
    viewer->display("view current", view_current, &escape_key);
    cv::waitKey(1);
    if(saving_images){
        views.push_back(view_current);
        saving_images=false;
    }
    if (triangulation_first&& views.size()>1) {
        init_mapping(views[0],views[1], true);
        triangulation_first = false;
    }
    if(processing){
       std::cout<<"TRACKING THREAD"<<std::endl;
       tracking(view_current,1 /*kframe_idx*/,true);
    }
}
int main(int argc, char* argv[]){
//    debug_reprojection();
    init();
    viewerGL.callBackIdle = idle ;
    viewerGL.callbackKeyBoard = keyBord;
    viewerGL.InitViewer(640 , 480);
   //
   //


   // gcloud
}


