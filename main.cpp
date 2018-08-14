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
#include <vector>

#include "opencv2/core.hpp" // TO REMOVE
#include "opencv2/calib3d.hpp"

#include <boost/log/core.hpp>

// ADD COMPONENTS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleOpengl_traits.h"

#include "xpcf/xpcf.h"

#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/I2Dto3DTransformDecomposer.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/pose/I3DTransformFinderfrom2D3D.h"
#include "api/display/ISideBySideOverlay.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::OPENGL;
using namespace SolAR::MODULES::TOOLS;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv){

//#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
//#endif

    LOG_ADD_LOG_TO_CONSOLE();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */
    SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    if(xpcfComponentManager->load("conf_SLAM.xml")!=org::bcom::xpcf::_SUCCESS)
    {
        LOG_ERROR("Failed to load the configuration file conf_SLAM.xml")
        return -1;
    }

    // declare and create components
    LOG_INFO("Start creating components");

 // component creation

    auto camera =xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto keypointsDetector =xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    auto descriptorExtractorAKAZE2 =xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
    auto descriptorExtractorORB =xpcfComponentManager->create<SolARDescriptorsExtractorORBOpencv>()->bindTo<features::IDescriptorsExtractor>();
    auto matcherKNN =xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto matcherBF =xpcfComponentManager->create<SolARDescriptorMatcherHammingBruteForceOpencv>()->bindTo<features::IDescriptorMatcher>();
    auto matchesFilterGeometric =xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    auto keypointsReindexer =xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
    auto poseFinderFrom2D2D =xpcfComponentManager->create<SolARPoseFinderFrom2D2DOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D2D>();
    auto fundamentalFinder =xpcfComponentManager->create<SolARFundamentalMatrixEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    auto fundamentalDecomposer =xpcfComponentManager->create<SolARSVDFundamentalMatrixDecomposerOpencv>()->bindTo<solver::pose::I2DTO3DTransformDecomposer>();
    auto mapper =xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
    auto mapFilter =xpcfComponentManager->create<SolARMapFilterOpencv>()->bindTo<solver::map::IMapFilter>();
    auto poseGraph =xpcfComponentManager->create<SolARMapperOpencv>()->bindTo<solver::map::IMapper>();
    auto PnP =xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
    auto corr2D3DFinder =xpcfComponentManager->create<SolAR2D3DCorrespondencesFinderOpencv>()->bindTo<solver::pose::I2D3DCorrespondencesFinder>();

    auto overlaySBS =xpcfComponentManager->create<SolARSideBySideOverlayOpencv>()->bindTo<display::ISideBySideOverlay>();
    auto imageViewerFrame1 =xpcfComponentManager->create<SolARImageViewerOpencv>("frame1")->bindTo<display::IImageViewer>();
    auto imageViewerFrame2 =xpcfComponentManager->create<SolARImageViewerOpencv>("frame2")->bindTo<display::IImageViewer>();
    auto imageViewerMatches =xpcfComponentManager->create<SolARImageViewerOpencv>("matches")->bindTo<display::IImageViewer>();
    auto viewer3DPoints =xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();

    /* in dynamic mode, we need to check that components are well created*/
    /* this is needed in dynamic mode */
    if ( !camera || !keypointsDetector || !descriptorExtractorORB || !descriptorExtractorAKAZE2 || !matcherKNN || !matcherBF || !matchesFilterGeometric || !keypointsReindexer || !poseFinderFrom2D2D ||
         !fundamentalFinder ||!fundamentalDecomposer || !mapper || !mapFilter || !poseGraph || !PnP || !corr2D3DFinder ||
         !overlaySBS || !imageViewerFrame1 || !imageViewerFrame2 || !imageViewerMatches || !viewer3DPoints)
    {
        LOG_ERROR("One or more component creations have failed");
        return -1;
    }

    // declarations
    SRef<Image>                                         view1, view2;
    SRef<Frame>                                         frame1 = xpcf::utils::make_shared<Frame>();
    SRef<Frame>                                         frame2 = xpcf::utils::make_shared<Frame>();
    std::vector<SRef<Keypoint>>                         keyPointsView1, keyPointsView2;
    SRef<DescriptorBuffer>                              descriptorsView1, descriptorsView2;
    std::vector<DescriptorMatch>                        matches, filteredMatches;
    std::vector<SRef<Point2Df>>                         matchedKeypointsView1,matchedKeypointsView2;

    Transform2Df                                        F;
    std::vector<Transform3Df>                           posesCandidates;

    Transform3Df                                        poseFrame1 = Transform3Df::Identity();
    Transform3Df                                        poseFrame2;
    Transform3Df                                        corrected_pose;

    std::vector<SRef<CloudPoint>>                       cloud;

    SRef<DescriptorBuffer> d1 = frame1->getDescriptors();
    SRef<DescriptorBuffer> d2 = frame2->getDescriptors();

    SRef<Image>                                         view_current;
    std::vector<SRef<Image>>                            views;
    SRef<Image>                                         currentMatchImage;
    SRef<Image>                                         projected_image;

    SRef<Image>                                         imageSBSMatches;
    SRef<Image>                                         viewerImage2;
    SRef<Image>                                         viewerImage3;
    SRef<std::vector<SRef<CloudPoint>>>                 cloud_current;
    CamCalibration                                      K;
    CamDistortion                                       dist;



    // initialize pose estimation with the camera intrinsic parameters (please refeer to the use of intrinsec parameters file)
    PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    fundamentalDecomposer->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    mapper->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());

    LOG_DEBUG("Intrincic parameters : \n {}", camera->getIntrinsicsParameters());

    if (camera->start() != FrameworkReturnCode::_SUCCESS) // videoFile
    {
        LOG_ERROR("Camera cannot start");
        return -1;
    }

    // Here, Capture the two first keyframe view1, view2
    bool imageCaptured = false;
    while (!imageCaptured)
    {
        if (camera->getNextImage(view1) == SolAR::FrameworkReturnCode::_ERROR_)
            break;
        if (imageViewerFrame1->display(view1) == SolAR::FrameworkReturnCode::_STOP)
            imageCaptured = true;
    }

    imageCaptured = false;
    while (!imageCaptured)
    {
        if (camera->getNextImage(view2) == SolAR::FrameworkReturnCode::_ERROR_)
            break;
        if (imageViewerFrame2->display(view2) == SolAR::FrameworkReturnCode::_STOP)
            imageCaptured = true;
    }

    // Detect keypoints, extract descriptors and create frame for the first two view
    keypointsDetector->detect(view1, keyPointsView1);
    descriptorExtractorAKAZE2->extract(view1, keyPointsView1, descriptorsView1);
    frame1->InitKeyPointsAndDescriptors(keyPointsView1, descriptorsView1);

    keypointsDetector->detect(view2, keyPointsView2);
    descriptorExtractorAKAZE2->extract(view2, keyPointsView2, descriptorsView2);
    frame2->InitKeyPointsAndDescriptors(keyPointsView2, descriptorsView2);

    // Match keypoint between the two first frame, filtered and display them
    matcherKNN->match(descriptorsView1, descriptorsView2, matches);
    matchesFilterGeometric->filter(matches, filteredMatches, keyPointsView1, keyPointsView2);
    keypointsReindexer->reindex(keyPointsView1, keyPointsView2, filteredMatches, matchedKeypointsView1, matchedKeypointsView2);
    overlaySBS->drawMatchesLines(view1, view2, imageSBSMatches, matchedKeypointsView1, matchedKeypointsView2);

    while (true)
    {
        if (imageViewerMatches->display(imageSBSMatches) == SolAR::FrameworkReturnCode::_STOP)
                break;
    }

    std::vector<SRef<Point2Df>> inliersView1, inliersView2;

    // Estimate the pose of of the second frame (the first frame being the reference of our coordinate system)
    poseFinderFrom2D2D->estimate(matchedKeypointsView1, matchedKeypointsView2, poseFrame1, poseFrame2, inliersView1, inliersView2);
    LOG_DEBUG("Estimate pose of the camera for the frame 2: \n {}", poseFrame2.matrix());


    // Triangulate
    std::vector<SRef<CloudPoint>> temp_cloud;
    std::vector<bool> mapFilterStatus;
    std::pair<int, int> working_view = std::make_pair(0, 1);

/*
    // Compare to OpenCV !
    K = camera->getIntrinsicsParameters();

    cv::Mat Kcv(3,3,CV_64FC1);
    for(int i  = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            double k = K(i,j);
            Kcv.at<double>(i,j) =k;
        }
    }

    std::vector<cv::Point2f> points_view1;
    std::vector<cv::Point2f> points_view2;

    points_view1.resize(matchedKeypointsView1.size());
    points_view2.resize(matchedKeypointsView2.size());
    for( int i = 0; i < matchedKeypointsView1.size(); i++ ){
        points_view1[i].x=matchedKeypointsView1.at(i)->getX();
        points_view1[i].y=matchedKeypointsView1.at(i)->getY();

        points_view2[i].x=matchedKeypointsView2.at(i)->getX();
        points_view2[i].y=matchedKeypointsView2.at(i)->getY();
    }
    cv::Mat RotE(3,3,CV_64FC1); cv::Mat RotF(3,3,CV_64FC1);
    cv::Mat PosE(3,1,CV_64FC1); cv::Mat PosF(3,1,CV_64FC1);
    cv::Point2f pp; pp.x=K(0,2); pp.y=K(1,2);
    cv::Mat Fcv = cv::findFundamentalMat(points_view1, points_view2);

    cv::Mat Ecv = cv::findEssentialMat(points_view1, points_view2, K(0,0), pp);
    LOG_DEBUG("E :\n{}", Ecv);
    cv::recoverPose(Ecv, points_view1, points_view2, RotE, PosE, K(0,0), pp);
    cv::Mat Kcvt; cv::transpose(Kcv, Kcvt);
    cv::recoverPose(Kcvt * Fcv * Kcv, points_view1, points_view2, RotF, PosF, K(0,0), pp);
    LOG_DEBUG("Kt * F * K: \n {}", Kcvt * Fcv * Kcv);
    LOG_DEBUG("Estimate rotation RecoverPose from E :\n{}", RotE);
    LOG_DEBUG("Estimate translation RecoverPose from E :\n{}", PosE);
    LOG_DEBUG("Estimate rotation RecoverPose from F :\n{}", RotF);
    LOG_DEBUG("Estimate translation RecoverPose from F :\n{}", PosF);
*/
    double reproj_error = mapper->triangulate(inliersView1,inliersView2, filteredMatches, working_view,poseFrame1, poseFrame2,temp_cloud);

//    mapFilter->filterPointCloud(temp_cloud, mapFilterStatus, cloud);

    while (true)
    {
        if (viewer3DPoints->display(temp_cloud, poseFrame2) == FrameworkReturnCode::_STOP)
                break;
    }

    return 0;
}



