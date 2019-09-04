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

 #define USE_FREE
 //#define USE_IMAGES_SET

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <boost/log/core.hpp>

// ADD MODULES TRAITS HEADERS HERE

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleOpengl_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleFBOW_traits.h"
#include "SolARModuleCeres_traits.h"


#ifndef USE_FREE
#include "SolARModuleNonFreeOpencv_traits.h"
#endif

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/features/IMatchesFilter.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "core/Log.h"

#include "opencv2/highgui.hpp"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::FBOW;
using namespace SolAR::MODULES::CERES;
#ifndef USE_FREE
using namespace SolAR::MODULES::NONFREEOPENCV;
#endif
using namespace SolAR::MODULES::OPENGL;
using namespace SolAR::MODULES::TOOLS;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv) {

	cv::namedWindow("toto", 0);
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif

	LOG_ADD_LOG_TO_CONSOLE();

  try{

        /* instantiate component manager*/
        /* this is needed in dynamic mode */
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        std::string configxml = std::string("conf_SLAM.xml");
        if (argc == 2)
            configxml = std::string(argv[1]);
        if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
        {
            LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
                return -1;
        }

        // declare and create components
        //LOG_INFO("Start creating components fro");
		std::cout << "Start creating components from: " << configxml << std::endl;


        // component creation
#ifdef USE_IMAGES_SET
        auto camera = xpcfComponentManager->create<SolARImagesAsCameraOpencv>()->bindTo<input::devices::ICamera>();
#else
		auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
		LOG_INFO("	-<SolARCameraOpencv: loaded>-");
#endif
#ifdef USE_FREE
        auto keypointsDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
		LOG_INFO("	-<SolARKeypointDetectorOpencv: loaded>-");
        auto descriptorExtractor = xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
		LOG_INFO("	-<SolARDescriptorsExtractorAKAZE2Opencv: loaded>-");
#else
        auto  keypointsDetector = xpcfComponentManager->create<SolARKeypointDetectorNonFreeOpencv>()->bindTo<features::IKeypointDetector>();
        auto descriptorExtractor = xpcfComponentManager->create<SolARDescriptorsExtractorSURF64Opencv>()->bindTo<features::IDescriptorsExtractor>();
#endif

        //   auto descriptorExtractorORB =xpcfComponentManager->create<olARDescriptorsExtractorORBOpencv>()->bindTo<features::IDescriptorsExtractor>();
        SRef<features::IDescriptorMatcher> matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
		LOG_INFO("	-<SolARDescriptorMatcherKNNOpencv: loaded>-");
        SRef<solver::pose::I3DTransformFinderFrom2D2D> poseFinderFrom2D2D = xpcfComponentManager->create<SolARPoseFinderFrom2D2DOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D2D>();
		LOG_INFO("	-<SolARPoseFinderFrom2D2DOpencv: loaded>-");
        SRef<solver::map::ITriangulator> triangulator = xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
		LOG_INFO("	-<SolARPoseFinderFrom2D2DOpencv: loaded>-");
        SRef<features::IMatchesFilter> matchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
		LOG_INFO("	-<SolARGeometricMatchesFilterOpencv: loaded>-");
        SRef<solver::pose::I3DTransformSACFinderFrom2D3D> PnP = xpcfComponentManager->create<SolARPoseEstimationSACPnpOpencv>()->bindTo<solver::pose::I3DTransformSACFinderFrom2D3D>();
		LOG_INFO("	-<SolARPoseEstimationSACPnpOpencv: loaded>-");
        SRef<solver::pose::I2D3DCorrespondencesFinder> corr2D3DFinder = xpcfComponentManager->create<SolAR2D3DCorrespondencesFinderOpencv>()->bindTo<solver::pose::I2D3DCorrespondencesFinder>();
		LOG_INFO("	-<SolAR2D3DCorrespondencesFinderOpencv: loaded>-");
        SRef<solver::map::IMapFilter> mapFilter = xpcfComponentManager->create<SolARMapFilter>()->bindTo<solver::map::IMapFilter>();
		LOG_INFO("	-<SolARMapFilter: loaded>-");
        SRef<solver::map::IMapper> mapper = xpcfComponentManager->create<SolARMapper>()->bindTo<solver::map::IMapper>();
		LOG_INFO("	-<SolARMapper: loaded>-");
        SRef<solver::map::IKeyframeSelector> keyframeSelector = xpcfComponentManager->create<SolARKeyframeSelector>()->bindTo<solver::map::IKeyframeSelector>();
		LOG_INFO("	-<SolARKeyframeSelector: loaded>-");
        SRef<display::IMatchesOverlay> matchesOverlay = xpcfComponentManager->create<SolARMatchesOverlayOpencv>()->bindTo<display::IMatchesOverlay>();
		LOG_INFO("	-<SolARMatchesOverlayOpencv: loaded>-");
        SRef<display::IMatchesOverlay> matchesOverlayBlue = xpcfComponentManager->create<SolARMatchesOverlayOpencv>("matchesBlue")->bindTo<display::IMatchesOverlay>();
		LOG_INFO("	-<SolARMatchesOverlayOpencv: loaded>-");
        SRef<display::IMatchesOverlay> matchesOverlayRed = xpcfComponentManager->create<SolARMatchesOverlayOpencv>("matchesRed")->bindTo<display::IMatchesOverlay>();
		LOG_INFO("	-<SolARMatchesOverlayOpencv: loaded>-");
        SRef<display::IImageViewer> imageViewer = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
		LOG_INFO("	-<SolARImageViewerOpencv: loaded>-");
        SRef<display::I3DPointsViewer> viewer3DPoints = xpcfComponentManager->create<SolAR3DPointsViewerOpengl>()->bindTo<display::I3DPointsViewer>();
		LOG_INFO("	-<SolAR3DPointsViewerOpengl: loaded>-");
        SRef<reloc::IKeyframeRetriever> kfRetriever = xpcfComponentManager->create<SolARKeyframeRetrieverFBOW>()->bindTo<reloc::IKeyframeRetriever>();
		LOG_INFO("	-<SolARKeyframeRetrieverFBOW: loaded>-");
		SRef<solver::map::IBundler> bundler = xpcfComponentManager->create<SolARBundlerCeres>()->bindTo<solver::map::IBundler>();
		LOG_INFO("-<SolARBundlerCeres loaded>-");

        // declarations
        SRef<Image>                                         view1, view2, view;
        SRef<Keyframe>                                      keyframe1;
        SRef<Keyframe>                                      keyframe2;
        std::vector<Keypoint>								keypointsView1, keypointsView2, keypoints;
        SRef<DescriptorBuffer>                              descriptorsView1, descriptorsView2, descriptors;
        std::vector<DescriptorMatch>                        matches;

        Transform3Df                                        poseFrame1 = Transform3Df::Identity();
        Transform3Df                                        poseFrame2;
        Transform3Df                                        newFramePose;
        Transform3Df                                        lastPose;

        std::vector<CloudPoint>								cloud, filteredCloud;

        std::vector<Transform3Df>                           keyframePoses;
        std::vector<Transform3Df>                           framePoses;

        SRef<Frame>                                         newFrame;
        SRef<Frame>											frameToTrack;

        SRef<Keyframe>                                      referenceKeyframe;
        SRef<Keyframe>                                      newKeyframe;

        SRef<Image>                                         imageMatches, imageMatches2;
        SRef<Map>                                           map;
		std::vector<CloudPoint>								localMap;

        bool												isLostTrack = false;

        // initialize pose estimation with the camera intrinsic parameters (please refeer to the use of intrinsec parameters file)
        PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
        poseFinderFrom2D2D->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
        triangulator->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
        LOG_DEBUG("Intrincic parameters : \n {}", camera->getIntrinsicsParameters());

        if (camera->start() != FrameworkReturnCode::_SUCCESS)
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
#ifdef USE_IMAGES_SET
            imageViewer->display(view1);
#else
            if (imageViewer->display(view1) == SolAR::FrameworkReturnCode::_STOP)
#endif
            {
                keypointsDetector->detect(view1, keypointsView1);
                descriptorExtractor->extract(view1, keypointsView1, descriptorsView1);
                keyframe1 = xpcf::utils::make_shared<Keyframe>(keypointsView1, descriptorsView1, view1, poseFrame1);
                mapper->update(map, keyframe1);
                keyframePoses.push_back(poseFrame1); // used for display
                kfRetriever->addKeyframe(keyframe1); // add keyframe for reloc
                imageCaptured = true;
            }
        }

        bool bootstrapOk = false;
        while (!bootstrapOk)
        {
            if (camera->getNextImage(view2) == SolAR::FrameworkReturnCode::_ERROR_)
                break;

            keypointsDetector->detect(view2, keypointsView2);

            descriptorExtractor->extract(view2, keypointsView2, descriptorsView2);
            SRef<Frame> frame2 = xpcf::utils::make_shared<Frame>(keypointsView2, descriptorsView2, view2, keyframe1);
            matcher->match(descriptorsView1, descriptorsView2, matches);
            int nbOriginalMatches = matches.size();
            matchesFilter->filter(matches, matches, keypointsView1, keypointsView2);

            matchesOverlay->draw(view2, imageMatches, keypointsView1, keypointsView2, matches);
            if (imageViewer->display(imageMatches) == SolAR::FrameworkReturnCode::_STOP)
                return 1;

            if (keyframeSelector->select(frame2, matches))
            {
                // Estimate the pose of of the second frame (the first frame being the reference of our coordinate system)
                poseFinderFrom2D2D->estimate(keypointsView1, keypointsView2, poseFrame1, poseFrame2, matches);
                LOG_INFO("Nb matches for triangulation: {}\\{}", matches.size(), nbOriginalMatches);
                LOG_INFO("Estimate pose of the camera for the frame 2: \n {}", poseFrame2.matrix());
                frame2->setPose(poseFrame2);

                // Triangulate
                keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
                triangulator->triangulate(keyframe2, matches, cloud);
                //double reproj_error = triangulator->triangulate(keypointsView1, keypointsView2, matches, std::make_pair(0, 1), poseFrame1, poseFrame2, cloud);
                mapFilter->filter(poseFrame1, poseFrame2, cloud, filteredCloud);
                keyframePoses.push_back(poseFrame2); // used for display
                mapper->update(map, keyframe2, filteredCloud, matches);
                kfRetriever->addKeyframe(keyframe2); // add keyframe for reloc
				keyframe1->addNeighborKeyframe(keyframe2->m_idx, filteredCloud.size());
				keyframe2->addNeighborKeyframe(keyframe1->m_idx, filteredCloud.size());


				std::vector<SRef<Keyframe>> kFramesToBundle = mapper->getKeyframes();
				std::vector<CloudPoint>cloudToBundle = mapper->getGlobalMap()->getPointCloud();
				CamCalibration cameraToBundle = camera->getIntrinsicsParameters();
				CamDistortion distToBundle = camera->getDistorsionParameters();

				// I need to save the others  cloud points :)! changes solver method... !
				std::vector<SRef<Keyframe>>correctedKeyframes;
				std::vector<CloudPoint>correctedCloud;
				CamCalibration correctedCalib;
				CamDistortion correctedDist;
				correctedKeyframes.resize(mapper->getKeyframes().size());
				for (unsigned i = 0; i < mapper->getKeyframes().size(); ++i) {
					Transform3Df tt = Transform3Df::Identity();
					correctedKeyframes[i] = xpcf::utils::make_shared<Keyframe>(mapper->getKeyframe(i)->getKeypoints(),
																			   mapper->getKeyframe(i)->getDescriptors(),
																			   mapper->getKeyframe(i)->getView(),
																			   tt);
				}

				std::vector<int>selectedKeyframes; // = {1,7};
				double reproj_errorFinal = 0.f;
				reproj_errorFinal = bundler->solve(mapper->getKeyframes(),
												   mapper->getGlobalMap()->getPointCloud(),
												   camera->getIntrinsicsParameters(),
												   camera->getDistorsionParameters(),
												   selectedKeyframes,
												   correctedKeyframes,
												   correctedCloud,
												   correctedCalib,
												   correctedDist);

				mapper->update(correctedCloud, correctedKeyframes);
				LOG_INFO("reproj error after bundle: {}", reproj_errorFinal);
                bootstrapOk = true;
            }
        }

		LOG_INFO("Nb pc: {}", filteredCloud.size());

        referenceKeyframe = keyframe2;
        lastPose = poseFrame2;
		localMap = map->getPointCloud();

        // copy referenceKeyframe to frameToTrack
        frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
        frameToTrack->setReferenceKeyframe(referenceKeyframe);

        // Start tracking
        clock_t start, end;
        int count = 0;
        start = clock();

		// functions

		CamCalibration calib = camera->getIntrinsicsParameters();
		CamDistortion dist = camera->getDistorsionParameters();

		auto project3DPoint = [&calib, &dist](Transform3Df &invPose, CloudPoint &point3D) {						
			float k1 = dist[0];
			float k2 = dist[1];
			float p1 = dist[2];
			float p2 = dist[3];
			float k3 = dist[4];

			Vector3f point(point3D.getX(), point3D.getY(), point3D.getZ());

			Vector3f pointInCamRef = invPose * point;
			if (pointInCamRef(2) > 0) {
				float x = pointInCamRef(0) / pointInCamRef(2);
				float y = pointInCamRef(1) / pointInCamRef(2);

				float r2 = x * x + y * y;
				float r4 = r2 * r2;
				float r6 = r4 * r2;
				float r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
				float xx = x * r_coeff + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
				float yy = y * r_coeff + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);

				float u = calib(0, 0)*xx + calib(0, 2);
				float v = calib(1, 1)*yy + calib(1, 2);

				return Point2Df(u, v);
			}
			else
				return Point2Df(-100.f, -100.f);
		};

		auto findNearest2DPoint = [](Point2Df &pt2D, std::vector<Keypoint> &keypoints, const float thres = 2.f) {
			for (int i = 0; i < keypoints.size(); i++) {
				if (std::fabsf(pt2D.getX() - keypoints[i].getX()) + std::fabsf(pt2D.getY() - keypoints[i].getY()) < thres)
					return i;
			}
			return -1;
		};


		auto refineCameraPose = [&camera, &project3DPoint, &findNearest2DPoint](std::vector<CloudPoint> &localMap, SRef<Keyframe> refKF, SRef<Frame> newFrame, std::vector<Point2Df> &pt2d, std::vector<Point3Df> &pt3d) {
			// get inverse pose
			Transform3Df invPose;
			invPose = newFrame->getPose().inverse();

			// get all keypoints of the new frame
			std::vector<Keypoint> keypoints = newFrame->getKeypoints();

			//  projection points
			for (auto it : localMap) {
				Point2Df pt2D = project3DPoint(invPose, it);
				int idxFound = findNearest2DPoint(pt2D, keypoints, 2.f);
				if (idxFound != -1) {
					pt2d.push_back(Point2Df(keypoints[idxFound].getX(), keypoints[idxFound].getY()));
					pt3d.push_back(Point3Df(it.getX(), it.getY(), it.getZ()));
				}
			}									
		};

		// check need to make a new keyframe based on all existed keyframes
		auto checkNeedNewKfWithAllKfs = [&kfRetriever](SRef<Keyframe> &referenceKeyframe, SRef<Frame> &newFrame) {
			std::vector < SRef <Keyframe>> ret_keyframes;
			if (kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
				for (auto it : ret_keyframes) {
					if (it->m_idx != referenceKeyframe->m_idx) {
						referenceKeyframe = it;
						LOG_INFO("Update new reference keyframe *** {}", referenceKeyframe->m_idx);
						return true;
					}
				}
				LOG_INFO("Find same reference keyframe, need make new keyframe");
				return false;
			}
			else
				return false;
		};


		// check need to make a new keyframe based on neighboring keyframes
		auto checkNeedNewKfwithSomeKfs = [&kfRetriever, &mapper](SRef<Keyframe> &referenceKeyframe, SRef<Frame> &newFrame) {
			// get all neighbor of referencekeyframe
			std::map<unsigned int, unsigned int> refKfNeighbors = referenceKeyframe->getNeighborKeyframes();

			std::set<unsigned int> allNeighbors;

			for (auto it = refKfNeighbors.begin(); it != refKfNeighbors.end(); it++) {
				allNeighbors.insert(it->first);
				SRef<Keyframe> tmpKf = mapper->getKeyframe(it->first);
				std::map<unsigned int, unsigned int> posNeighbors = tmpKf->getNeighborKeyframes();
				for (auto it_tmp = posNeighbors.begin(); it_tmp != posNeighbors.end(); it_tmp++)
					allNeighbors.insert(it_tmp->first);
			}

			std::vector < SRef <Keyframe>> ret_keyframes;
			
			if (kfRetriever->retrieve(newFrame, allNeighbors, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
				for (auto it : ret_keyframes) {
					if (it->m_idx != referenceKeyframe->m_idx) {
						referenceKeyframe = it;
						LOG_INFO("Update new reference keyframe *** {}", referenceKeyframe->m_idx);
						return true;
					}
				}
				LOG_INFO("Find same reference keyframe, need make new keyframe");
				return false;
			}
			else
				return false;
		};

		/// process to add a new keyframe
		auto processNewKeyframe = [&kfRetriever, &mapper, &localMap, &referenceKeyframe, &frameToTrack, &newKeyframe](SRef<Frame> newFrame) {
			// Some principal tasks for add a new keyframe
			LOG_INFO("Create new keyframe");
			// 0. Make new Keyframe
			newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);

			// 1. Add to BOW retrieval
			kfRetriever->addKeyframe(newKeyframe);

			// 2. Update cloud point seen from this keyframe

			// 3. Update links in the covisibility graph

			// 4. Triangulation and update neighbor

			// 5. Check and fuse cloud point

			// 6. Update reference keyframe
			referenceKeyframe = newKeyframe;

			// 7. Update frame to track
			frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
			frameToTrack->setReferenceKeyframe(referenceKeyframe);

			// 8. Update local map
			localMap.clear();
			mapper->getLocalMap(referenceKeyframe, localMap);			
		};

        while (true)
        {
            // Get current image
            camera->getNextImage(view);
            count++;
            keypointsDetector->detect(view, keypoints);
            descriptorExtractor->extract(view, keypoints, descriptors);
            newFrame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, referenceKeyframe);
            // match current keypoints with the keypoints of the Keyframe
            SRef<DescriptorBuffer> frameToTrackDescriptors = frameToTrack->getDescriptors();
            matcher->match(frameToTrackDescriptors, descriptors, matches);
            matchesFilter->filter(matches, matches, frameToTrack->getKeypoints(), keypoints);

            std::vector<Point2Df> pt2d;
            std::vector<Point3Df> pt3d;
            std::vector<CloudPoint> foundPoints;
            std::vector<DescriptorMatch> foundMatches;
            std::vector<DescriptorMatch> remainingMatches;
            corr2D3DFinder->find(frameToTrack, newFrame, matches, map, pt3d, pt2d, foundMatches, remainingMatches);
            //LOG_INFO("found matches {}, Remaining Matches {}", foundMatches.size(), remainingMatches.size());
            // display matches
            if (isLostTrack) {
                if (imageViewer->display(view) == FrameworkReturnCode::_STOP)
                    break;
            }
            else {
                matchesOverlayBlue->draw(view, imageMatches, referenceKeyframe->getKeypoints(), keypoints, foundMatches);
                matchesOverlayRed->draw(imageMatches, imageMatches2, referenceKeyframe->getKeypoints(), keypoints, remainingMatches);
                if (imageViewer->display(imageMatches2) == FrameworkReturnCode::_STOP)
                    break;
            }

            std::vector<Point2Df> imagePoints_inliers;
            std::vector<Point3Df> worldPoints_inliers;
            if (PnP->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, newFramePose, lastPose) == FrameworkReturnCode::_SUCCESS) {
                LOG_INFO(" pnp inliers size: {} / {}", worldPoints_inliers.size(), pt3d.size());
				LOG_INFO("Estimated pose: \n {}", newFramePose.matrix());

				lastPose = newFramePose;
				
				// Set the pose of the new frame
				newFrame->setPose(newFramePose);
				
				/// refine pose and find new reference keyframe based on the local cloud points
				//std::vector<Point2Df> pt2d_new;
				//std::vector<Point3Df> pt3d_new;
				//std::vector<Point2Df> imagePoints_inliers_new;
				//std::vector<Point3Df> worldPoints_inliers_new;
				//refineCameraPose(localMap, referenceKeyframe, newFrame, pt2d_new, pt3d_new);
				//PnP->estimate(pt2d_new, pt3d_new, imagePoints_inliers_new, worldPoints_inliers_new, newFramePose, lastPose);
				//LOG_INFO(" pnp inliers size: {} / {}", worldPoints_inliers_new.size(), pt3d_new.size());
				//LOG_INFO("Refined pose: \n {}", newFramePose.matrix());
				//newFrame->setPose(newFramePose);
				//lastPose = newFramePose;

                // If the camera has moved enough, create a keyframe and map the scene
                if (keyframeSelector->select(newFrame, foundMatches))
                //if ((nbFrameTracking == 0) && (foundMatches.size() < 0.7 * referenceKeyframe->getVisibleMapPoints().size()))
                {
					if (checkNeedNewKfWithAllKfs(referenceKeyframe, newFrame)) {
						LOG_INFO("Update new reference keyframe with id {}", referenceKeyframe->m_idx);
						frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
						frameToTrack->setReferenceKeyframe(referenceKeyframe);

						// update local map
						localMap.clear();
						mapper->getLocalMap(referenceKeyframe, localMap);
					}
					else {
						LOG_INFO("Create new keyframe");
						// create a new keyframe from the current frame
						newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);

						// triangulate with the reference keyframe
						std::vector<CloudPoint>newCloud, filteredCloud;
						triangulator->triangulate(newKeyframe, remainingMatches, newCloud);

						// remove abnormal 3D points from the new cloud
						mapFilter->filter(referenceKeyframe->getPose(), newFramePose, newCloud, filteredCloud);
						LOG_DEBUG("Number of matches: {}, number of 3D points:{}", remainingMatches.size(), filteredCloud.size());

						// Add neighborhood
						newKeyframe->addNeighborKeyframe(referenceKeyframe->m_idx, filteredCloud.size());
						referenceKeyframe->addNeighborKeyframe(newKeyframe->m_idx, filteredCloud.size());
						
						//  Add new keyframe with the cloud to the mapper
						mapper->update(map, newKeyframe, filteredCloud, remainingMatches, foundMatches);
						keyframePoses.push_back(newKeyframe->getPose());
						referenceKeyframe = newKeyframe;
						frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
						frameToTrack->setReferenceKeyframe(referenceKeyframe);
						kfRetriever->addKeyframe(referenceKeyframe); // add keyframe for reloc

						// update local map
						localMap.clear();
						mapper->getLocalMap(referenceKeyframe, localMap);

						LOG_DEBUG(" cloud current size: {} \n", map->getPointCloud().size());
					}
                }
                else
                {
					// update frame to track
					frameToTrack = newFrame;
                }

				framePoses.push_back(newFramePose); // used for display

                isLostTrack = false;	// tracking is good

            }
            else {
                LOG_INFO("Pose estimation has failed");
                isLostTrack = true;		// lost tracking

                // reloc
                std::vector < SRef <Keyframe>> ret_keyframes;
                if (kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
                    referenceKeyframe = ret_keyframes[0];
                    frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
                    frameToTrack->setReferenceKeyframe(referenceKeyframe);
                    lastPose = referenceKeyframe->getPose();

					// update local map
					localMap.clear();
					mapper->getLocalMap(referenceKeyframe, localMap);

                    LOG_INFO("Retrieval Success");
                }
                else
                    LOG_INFO("Retrieval Failed");
            }
			
			LOG_INFO("Nb of Local Map: {}", localMap.size());
			LOG_INFO("Index of current reference keyframe: {}", referenceKeyframe->m_idx);
            // display point cloud
            if (viewer3DPoints->display(map->getPointCloud(), lastPose, keyframePoses, framePoses, localMap) == FrameworkReturnCode::_STOP)
                break;
        }

        // display stats on frame rate
        end = clock();
        double duration = double(end - start) / CLOCKS_PER_SEC;
        printf("\n\nElasped time is %.2lf seconds.\n", duration);
        printf("Number of processed frame per second : %8.2f\n", count / duration);

    }
    catch (xpcf::Exception &e)
    {
        LOG_DEBUG("{}", e.what());
        return -1;
    }

	return 0;
}



