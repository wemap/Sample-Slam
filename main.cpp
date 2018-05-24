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
	   xpcf::ComponentFactory::createComponent<SolARMapFilterOpencv>(gen(solver::map::IMapFilter::UUID), mapFilter);
	   
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

	getMatchedKeyPoints(referenceKeyFrame->getKeyPoints(), frame->getKeyPoints(), frame->getMatchesWithReferenceKeyFrame(), pointsKeyFrame, pointsFrame);

	std::vector<SRef<CloudPoint>> newMapPoints; 
	std::pair<int, int> corres(referenceKeyFrame->m_idx , newIndex);

	// Triangulate new points 
	mapper->triangulate(pointsFrame, pointsKeyFrame, frame->getMatchesWithReferenceKeyFrame(), corres, frame->m_pose, referenceKeyFrame->m_pose, K, dist, newMapPoints);
	

	// filter new point cloud
	std::vector<bool> tmp_status;
	if (!mapFilter->checkFrontCameraPoints(newMapPoints, frame->m_pose, tmp_status))
	{
		// not good triangulation : do not add key frame
		std::cout << "not good triangulation : do not add key frame " << std::endl; 
		return false; 
	}
	std::vector<SRef<CloudPoint>> filteredPoints;
	mapFilter->filterPointCloud(newMapPoints, tmp_status, filteredPoints);
	
	Transform3Df pose_final; 

	referenceKeyFrame->addVisibleMapPoints(filteredPoints);
	poseGraph->getMap()->addCloudPoints(filteredPoints);
	
	SRef<Keyframe> newKeyFrame = xpcf::utils::make_shared<Keyframe>(frame->getDescriptors(), newIndex, frame->m_pose, frame->getKeyPoints());
	
	newKeyFrame->addVisibleMapPoints(frame->getCommonMapPointsWithReferenceKeyFrame()); 
	newKeyFrame->addVisibleMapPoints(filteredPoints);
	
	
	std::cout << " add new keyframe  with " << filteredPoints.size() << "points" << std::endl;
	poseGraph->addNewKeyFrame(newKeyFrame);

	return true; 
}


bool init_mapping(SRef<Image>&view_1,SRef<Image>&view_2){
 SRef<Frame> frame1 = createAndInitFrame(views[0]);
 SRef<Frame> frame2 = createAndInitFrame((views[1])) ;

  std::vector<DescriptorMatch>  matches;


 matcher->match(frame1->getDescriptors(), frame2->getDescriptors(), matches); 

 std::vector<SRef<Point2Df>>    matchedKeypoints1;
 std::vector<SRef<Point2Df>>    matchedKeypoints2;

 getMatchedKeyPoints(frame1->getKeyPoints(), frame2->getKeyPoints() , matches, matchedKeypoints1 , matchedKeypoints2);
 int vizPoints0 = matches.size();

  // Draw the matches in a dedicated image
 overlay->drawMatchesLines(views[0], views[1], viewerImage1, matchedKeypoints1, matchedKeypoints2, vizPoints0);

 std::vector<DescriptorMatch>  ggmatches;

 matchesFilterGeometric->filter(matches,ggmatches,frame1->getKeyPoints(), frame2->getKeyPoints());

 std::vector<SRef<Point2Df>> ggmatchedKeypoints1;
 std::vector<SRef<Point2Df>> ggmatchedKeypoints2;

 getMatchedKeyPoints(frame1->getKeyPoints(), frame2->getKeyPoints() ,ggmatches, ggmatchedKeypoints1 ,  ggmatchedKeypoints2 );

 int vizPoints2 = int(ggmatches.size());
 overlay->drawMatchesLines(views[0], views[1], viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2,vizPoints2);

 viewer->display("original matches", viewerImage1, &escape_key,1280,480);
 viewer->display("filtred matches (epipolar)", viewerImage3, &escape_key,1280,480);

 fundamentalFinder->find(ggmatchedKeypoints1, ggmatchedKeypoints2,F);
 fundamentalDecomposer->decompose(F,K,dist,poses);
 pose_canonique.setIdentity() ;
 std::pair<int,int>working_view = std::make_pair(0,1);
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

SRef<Image>currentMatchImage;
SRef<Image>projected_image;

bool tracking(SRef<Image>&view){
	nbFrameSinceKeyFrame++ ;
    SRef<Frame> newFrame = createAndInitFrame(view);
    poseGraph->associateReferenceKeyFrameToFrame(newFrame) ;
    newFrame->setNumberOfFramesSinceLastKeyFrame(nbFrameSinceKeyFrame);
    std::vector<DescriptorMatch>new_matches, new_matches_filtred;
	SRef<Keyframe> referenceKeyFrame = newFrame->getReferenceKeyFrame() ;
    matcher->match(referenceKeyFrame->getDescriptors(), newFrame->getDescriptors(), new_matches);
    matchesFilterGeometric->filter(new_matches,new_matches_filtred, referenceKeyFrame->getKeyPoints(), newFrame->getKeyPoints());
    
	std::vector<SRef<Point2Df>>current_kp1;
    std::vector<SRef<Point2Df>>current_kp2;

	getMatchedKeyPoints(referenceKeyFrame->getKeyPoints(), newFrame->getKeyPoints(), new_matches_filtred, current_kp1, current_kp2);


   /*
    overlay->drawMatchesLines(referenceKeyFrame->m_view, view, currentMatchImage, current_kp1, current_kp2,current_kp1.size());
    viewer->display("current matches", currentMatchImage, &escape_key,1280,480);*/

    std::vector<SRef<Point2Df>>pt2d;
    std::vector<SRef<Point3Df>>pt3d;
    std::vector<SRef<CloudPoint>> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
    std::vector<DescriptorMatch> remainingMatches;


    corr2D3DFinder->find(referenceKeyFrame->getVisibleMapPoints(),referenceKeyFrame->m_idx,new_matches_filtred, newFrame->getKeyPoints(), foundPoints,  pt3d,pt2d , foundMatches , remainingMatches);
    
    std::vector<SRef<Point2Df>>imagePoints_inliers;
    std::vector<SRef<Point3Df>>worldPoints_inliers;


    if(PnP->estimate(pt2d,pt3d,imagePoints_inliers, worldPoints_inliers,pose_current, true) == FrameworkReturnCode::_SUCCESS /*&&
            worldPoints_inliers.size()> 50*/){
       // std::cout<<" pnp inliers size: "<<worldPoints_inliers.size()<<" / "<<pt3d.size()<<std::endl;
        newFrame->m_pose = pose_current ;
        viewerGL.SetRealCameraPose(pose_current);

        newFrame->addCommonMapPointsWithReferenceKeyFrame(foundPoints);
        newFrame->setMatchesWithReferenceKeyFrame(remainingMatches);

		int isKeyFrameCandidate = poseGraph->isKeyFrameCandidate(newFrame);
		if (isKeyFrameCandidate != -1) // try to add key frame if success tracking
		{
			
			if (addFrameToMapAsKeyFrame(newFrame, isKeyFrameCandidate))
			{
				nbFrameSinceKeyFrame = 0;
				// update visibility of old points in new key frame 
				for (int i = 0; i < foundPoints.size(); i++)
				{
					foundPoints[i]->m_visibility[isKeyFrameCandidate] = foundMatches[i].getIndexInDescriptorB();// Clean needed : Try to do this somewhere else

				}
			}
		}
        return true;
    }else{
       // std::cout<<"new keyframe creation.."<<std::endl;
        return false;
    }	
}

bool mapping(SRef<Image>&view, bool verbose){
    /*
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
*/
    return true;
}


SRef<Image>  image1;
SRef<Image>  image2;
SRef<Image>  image3; 
SRef<Image>  image4;


void my_idle(){
    
		if(triangulation_first){
		
		imageLoader->loadImage(std::string("bview0.jpg"), image1);
        views.push_back(image1);
		
        imageLoader->loadImage(std::string("bview1.jpg"), image2);
        views.push_back(image2);
        init_mapping(views[0],views[1]);
        triangulation_first = false;
		std::cout << " allo " << std::endl; 
		imageLoader->loadImage(std::string("bview2.jpg"), image3);
		imageLoader->loadImage(std::string("bview3.jpg"), image4);
		
    }
   
	tracking(image3);

   
}


void idle(){
    camera->getNextImage(view_current);
    viewer->display("current view",view_current, &escape_key);
    if(saving_images){
      views.push_back(view_current);
      saving_images=false;
    }
    if (triangulation_first&& views.size()>1) {
        if(init_mapping(views[0],views[1])){
            triangulation_first = false;
        }else{
            views.clear();
        }
    }
    if(processing){
       tracking(view_current);
    }
}
int main (int argc, char* argv[]){
    
	boost::log::core::get()->set_logging_enabled(false);
	init();
	viewerGL.callBackIdle = idle ;
	//viewerGL.callBackIdle = my_idle;

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



