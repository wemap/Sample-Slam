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
#include <set>
#include <cmath>
#include <boost/log/core.hpp>

// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IBundler.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "api/features/IMatchesFilter.h"
#include "api/display/I2DOverlay.h"
#include "api/display/IMatchesOverlay.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/geom/IProject.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"

#define MIN_WEIGHT_NEIGHBOR_KEYFRAME 50
#define MIN_POINT_DISTANCE 0.04
#define NB_NEWKEYFRAMES_LOOP 5


using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

int main(int argc, char **argv) {

#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif

	LOG_ADD_LOG_TO_CONSOLE();

	/* instantiate component manager*/
	/* this is needed in dynamic mode */
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

    std::string configxml = std::string("conf_SLAM_Mono.xml");
	if (argc == 2)
		configxml = std::string(argv[1]);
	if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
	{
		LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
			return -1;
	}

	// declare and create components
	LOG_INFO("Start creating components");

	// component creation    
    LOG_INFO("Resolving camera ");
    auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
	// storage components
    LOG_INFO("---- Resolving storage components ----");
    LOG_INFO("Resolving point cloud manager");
	auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
    LOG_INFO("Resolving key frames manager");
	auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
    LOG_INFO("Resolving covisibility graph");
	auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
    LOG_INFO("Resolving key frame retriever");
	auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
    LOG_INFO("Resolving key mapper");
	auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
		
	// processing components
    LOG_INFO("---- Resolving processing components ----");
    LOG_INFO("Resolving key points detector");
    auto  keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
    LOG_INFO("Resolving descriptor extractor");
    auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
    LOG_INFO("Resolving descriptor matcher");
    auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>("Matcher");
    LOG_INFO("Resolving triangulator");
    auto triangulator = xpcfComponentManager->resolve<solver::map::ITriangulator>();
    LOG_INFO("Resolving matches filter");
    auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
    LOG_INFO("Resolving pnp ransac");
    auto pnpRansac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>();
    LOG_INFO("Resolving pnp ");
    auto pnp = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>();
    LOG_INFO("Resolving correspondences finder");
    auto corr2D3DFinder = xpcfComponentManager->resolve<solver::pose::I2D3DCorrespondencesFinder>();
    LOG_INFO("Resolving map filter");
    auto mapFilter = xpcfComponentManager->resolve<solver::map::IMapFilter>();        
    LOG_INFO("Resolving key frame selector");
    auto keyframeSelector = xpcfComponentManager->resolve<solver::map::IKeyframeSelector>();
    LOG_INFO("Resolving matches overlay");
    auto matchesOverlay = xpcfComponentManager->resolve<display::IMatchesOverlay>();
    LOG_INFO("Resolving image viewer");
    auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
    LOG_INFO("Resolving viewer3D points");
    auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();        
    LOG_INFO("Resolving projector");
    auto projector = xpcfComponentManager->resolve<geom::IProject>();
    LOG_INFO("Resolving bundler");
    auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
	LOG_INFO("Resolving loop detector");
	auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
	LOG_INFO("Resolving loop corrector");
	auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
    LOG_INFO("Resolving 3D overlay");
    auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
    LOG_INFO("Resolving 2D overlay");
    auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();
	LOG_INFO("Resolving bootstrapper");
	auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();

	LOG_INFO("Loaded all components");

	// declarations
	SRef<Image>                                         view;		
	std::vector<Keypoint>								keypoints;
	SRef<DescriptorBuffer>                              descriptors;
	std::vector<DescriptorMatch>                        matches;
	Transform3Df                                        poseFrame;
	Transform3Df                                        newFramePose;
	Transform3Df                                        lastPose;	
	std::vector<Transform3Df>                           keyframePoses;
	std::vector<Transform3Df>                           framePoses;
	SRef<Frame>                                         newFrame;
	SRef<Frame>											frameToTrack;
	SRef<Keyframe>                                      referenceKeyframe, updatedRefKf;
	SRef<Image>                                         imageMatches;
	std::vector<SRef<CloudPoint>>						localMap;
    CamCalibration                                      calibration;
    CamDistortion                                       distortion;

	bool												isLostTrack = false;
	double												bundleReprojError;		

	// initialize pose estimation with the camera intrinsic parameters (please refer to the use of intrinsic parameters file)
    calibration = camera->getIntrinsicsParameters();
    distortion = camera->getDistortionParameters();
    overlay3D->setCameraParameters(calibration, distortion);
    pnpRansac->setCameraParameters(calibration, distortion);
    pnp->setCameraParameters(calibration, distortion);
    triangulator->setCameraParameters(calibration, distortion);
    projector->setCameraParameters(calibration, distortion);
	loopCorrector->setCameraParameters(calibration, distortion);
	bootstrapper->setCameraParameters(calibration, distortion);
    LOG_DEBUG("Intrincic parameters : \n {}", distortion);

	if (camera->start() != FrameworkReturnCode::_SUCCESS)
	{
		LOG_ERROR("Camera cannot start");
		return -1;
	}

	// Load map from file
	SRef<Keyframe>	keyframe1, keyframe2;
	if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
		keyframesManager->getKeyframe(0, keyframe2);
	}
	else {
		LOG_INFO("Initialization from scratch");
		if (bootstrapper->run() == FrameworkReturnCode::_ERROR_)
			return 1;
		keyframesManager->getKeyframe(0, keyframe1);
		keyframesManager->getKeyframe(1, keyframe2);
		keyframePoses.push_back(keyframe1->getPose());
		keyframePoses.push_back(keyframe2->getPose());
	}
	
	LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
	LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());

	// update data function
	auto updateData = [&](const SRef<Keyframe> &refKf, std::vector<SRef<CloudPoint>> &localMap, SRef<Keyframe> & referenceKeyframe, SRef<Frame> &frameToTrack)
	{
		referenceKeyframe = refKf;
		frameToTrack = xpcf::utils::make_shared<Frame>(referenceKeyframe);
		frameToTrack->setReferenceKeyframe(referenceKeyframe);
		localMap.clear();
		// get local point cloud
		mapper->getLocalPointCloud(refKf, MIN_WEIGHT_NEIGHBOR_KEYFRAME, localMap);
	};	

	// checkDisparityDistance
	std::function<bool(const SRef<Frame>&)> checkDisparityDistance = [&referenceKeyframe, &mapper, &projector, &pointCloudManager](const SRef<Frame>& newFrame) -> bool {
		const std::vector<Keypoint> &refKeypoints = referenceKeyframe->getKeypoints();
		const std::map<uint32_t, uint32_t> &refMapVisibility = referenceKeyframe->getVisibility();
		std::vector<SRef<CloudPoint>> cpRef;
		std::vector<Point2Df> projected2DPts, ref2DPts;

		for (auto const & it : refMapVisibility) {
			SRef<CloudPoint> cloudPoint;
			if (pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
				cpRef.push_back(cloudPoint);
				ref2DPts.push_back(Point2Df(refKeypoints[it.first].getX(), refKeypoints[it.first].getY()));
			}
		}
		projector->project(cpRef, projected2DPts, newFrame->getPose());

		unsigned int imageWidth = newFrame->getView()->getWidth();
		double totalMatchesDist = 0.0;
		for (int i = 0; i < projected2DPts.size(); i++)
		{
			Point2Df pt1 = ref2DPts[i];
			Point2Df pt2 = projected2DPts[i];
			totalMatchesDist += (pt1 - pt2).norm() / imageWidth;
		}
		double meanMatchesDist = totalMatchesDist / projected2DPts.size();
		LOG_DEBUG("Keyframe Selector Mean Matches Dist: {}", meanMatchesDist);
		return (meanMatchesDist > 0.07);
	};


	// check need to make a new keyframe based on all existed keyframes
	std::function<bool(const SRef<Frame>&)> checkNeedNewKfWithAllKfs = [&](const SRef<Frame>& newFrame) -> bool {
		std::vector < uint32_t> ret_keyframesId, neighborsKfs;
		covisibilityGraph->getNeighbors(newFrame->getReferenceKeyframe()->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME, neighborsKfs);
		std::set<uint32_t> candidates;
		for (const auto &it : neighborsKfs)
			candidates.insert(it);
		if (keyframeRetriever->retrieve(newFrame, candidates, ret_keyframesId) == FrameworkReturnCode::_SUCCESS) {
			keyframesManager->getKeyframe(ret_keyframesId[0], updatedRefKf);
			LOG_INFO("Update new reference keyframe with id {}", updatedRefKf->getId());
			return true;
		}
		else {
			LOG_INFO("Need to make new keyframe");
			return false;
		}
	};

	// Update keypoint visibility, descriptor in cloud point
	auto updateAssociateCloudPoint = [&](SRef<Keyframe> & newKf) {
		const std::map<uint32_t, uint32_t> &newkf_mapVisibility = newKf->getVisibility();
		std::map<uint32_t, int> kfCounter;
		// calculate the number of connections to other keyframes
		for (auto const &it : newkf_mapVisibility) {
			SRef<CloudPoint> cloudPoint;
			if (pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {				
				const std::map<uint32_t, uint32_t> &cpKfVisibility = cloudPoint->getVisibility();
				for (auto const &it_kf : cpKfVisibility)
					kfCounter[it_kf.first]++;
				// add new visibility to cloud point
				cloudPoint->addVisibility(newKf->getId(), it.first);
				// update view direction
				Transform3Df poseNewKf = newKf->getPose();
				Vector3f newViewDirection(poseNewKf(0, 3) - cloudPoint->getX(), poseNewKf(1, 3) - cloudPoint->getY(), poseNewKf(2, 3) - cloudPoint->getZ());
				cloudPoint->addNewViewDirection(newViewDirection);
				// update descriptor
				cloudPoint->addNewDescriptor(newKf->getDescriptors()->getDescriptor(it.first));
			}			
		}

		// Add to covisibility graph
		for (auto const &it : kfCounter)
			if (it.first != newKf->getId())
				covisibilityGraph->increaseEdge(newKf->getId(), it.first, it.second);
	};

	// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
	auto findMatchesAndTriangulation = [&](const SRef<Keyframe> & newKf, const std::vector<uint32_t> &idxBestNeighborKfs, std::vector<SRef<CloudPoint>> &cloudPoint) {
		const std::map<unsigned int, unsigned int> &newKf_mapVisibility = newKf->getVisibility();
		const SRef<DescriptorBuffer> &newKf_des = newKf->getDescriptors();
		const std::vector<Keypoint> & newKf_kp = newKf->getKeypoints();
		const Transform3Df& newKf_pose = newKf->getPose();

		// Vector indices keypoints have no visibility to map point
		std::vector<bool> checkMatches(newKf_kp.size(), false);
		for (int i = 0; i < newKf_kp.size(); ++i)
			if (newKf_mapVisibility.find(i) != newKf_mapVisibility.end()) {
				checkMatches[i] = true;
			}

		// Triangulate to neighboring keyframes
		for (int i = 0; i < idxBestNeighborKfs.size(); ++i) {
			// get non map point view keypoints
			std::vector<int> newKf_indexKeypoints;
			for (int j = 0; j < checkMatches.size(); ++j)
				if (!checkMatches[j])
					newKf_indexKeypoints.push_back(j);

			// get neighbor keyframe i
			SRef<Keyframe> tmpKf;
			keyframesManager->getKeyframe(idxBestNeighborKfs[i], tmpKf);
			const Transform3Df &tmpKf_pose = tmpKf->getPose();

			// Matching based on BoW
			std::vector < DescriptorMatch> tmpMatches, goodMatches;
			keyframeRetriever->match(newKf_indexKeypoints, newKf_des, tmpKf, tmpMatches);

			// matches filter based epipolar lines
			matchesFilter->filter(tmpMatches, tmpMatches, newKf_kp, tmpKf->getKeypoints(), newKf_pose, tmpKf_pose, camera->getIntrinsicsParameters());

			// find info to triangulate				
			const std::map<unsigned int, unsigned int> & tmpMapVisibility = tmpKf->getVisibility();
			for (int j = 0; j < tmpMatches.size(); ++j) {
				unsigned int idx_newKf = tmpMatches[j].getIndexInDescriptorA();
				unsigned int idx_tmpKf = tmpMatches[j].getIndexInDescriptorB();
				if ((!checkMatches[idx_newKf]) && (tmpMapVisibility.find(idx_tmpKf) == tmpMapVisibility.end())) {
					goodMatches.push_back(tmpMatches[j]);
				}
			}

			// triangulation
			std::vector<SRef<CloudPoint>> tmpCloudPoint, tmpFilteredCloudPoint;
			std::vector<int> indexFiltered;
			if (goodMatches.size() > 0)
				triangulator->triangulate(newKf_kp, tmpKf->getKeypoints(), newKf_des, tmpKf->getDescriptors(), goodMatches, 
					std::make_pair(newKf->getId(), idxBestNeighborKfs[i]), newKf_pose, tmpKf_pose, tmpCloudPoint);

			// filter cloud points
			if (tmpCloudPoint.size() > 0)
				mapFilter->filter(newKf_pose, tmpKf_pose, tmpCloudPoint, tmpFilteredCloudPoint, indexFiltered);
			for (int i = 0; i < indexFiltered.size(); ++i) {
				checkMatches[goodMatches[indexFiltered[i]].getIndexInDescriptorA()] = true;
				cloudPoint.push_back(tmpFilteredCloudPoint[i]);
			}
		}

	};

	// check and fuse cloud point
	auto fuseCloudPoint = [&](const SRef<Keyframe> &newKeyframe, const std::vector<uint32_t> &idxNeigborKfs, std::vector<SRef<CloudPoint>> &newCloudPoint) {
		std::vector<bool> checkMatches(newCloudPoint.size(), true);
		std::vector<SRef<DescriptorBuffer>> desNewCloudPoint;
		for (auto const &it_cp : newCloudPoint) {
			desNewCloudPoint.push_back(it_cp->getDescriptor());
		}			

		for (int i = 0; i < idxNeigborKfs.size(); ++i) {
			// get a neighbor
			SRef<Keyframe> neighborKf;
			if (keyframesManager->getKeyframe(idxNeigborKfs[i], neighborKf) == FrameworkReturnCode::_ERROR_)
				continue;
			const std::map<uint32_t, uint32_t> &mapVisibilitiesNeighbor = neighborKf->getVisibility();

			//  projection points
			std::vector< Point2Df > projected2DPts;
			projector->project(newCloudPoint, projected2DPts, neighborKf->getPose());

			std::vector<DescriptorMatch> allMatches;
			matcher->matchInRegion(projected2DPts, desNewCloudPoint, neighborKf, allMatches, 5.f);

			for (int j = 0; j < allMatches.size(); ++j) {
				int idxNewCloudPoint = allMatches[j].getIndexInDescriptorA();
				int idxKpNeighbor = allMatches[j].getIndexInDescriptorB();
				if (!checkMatches[idxNewCloudPoint])
					continue;

				const std::map<uint32_t, uint32_t> &newCloudPointVisibility = newCloudPoint[idxNewCloudPoint]->getVisibility();
				// check this cloud point is created from the same neighbor keyframe
				if (newCloudPointVisibility.find(idxNeigborKfs[i]) != newCloudPointVisibility.end())
					continue;
					
				// check if have a cloud point in the neighbor keyframe is coincide with this cloud point.
				auto it_cp = mapVisibilitiesNeighbor.find(idxKpNeighbor);
				if (it_cp == mapVisibilitiesNeighbor.end())
					continue;
				
				SRef<CloudPoint> existedCloudPoint; 
				if (pointCloudManager->getPoint(it_cp->second, existedCloudPoint) == FrameworkReturnCode::_SUCCESS) {
					const std::map<uint32_t, uint32_t> &existedCloudPointVisibility = existedCloudPoint->getVisibility();
					bool goodCheck = true;
					for (const auto &vExistedCP : existedCloudPointVisibility)
						if (newCloudPointVisibility.find(vExistedCP.first) != newCloudPointVisibility.end()) {
							goodCheck = false;
							break;
						}
					if (goodCheck && ((*existedCloudPoint - *newCloudPoint[idxNewCloudPoint]).magnitude() < MIN_POINT_DISTANCE)) {
						// add visibilities to this cloud point and keyframes
						std::vector<uint32_t> keyframeIds;
						for (auto const &vNewCP : newCloudPointVisibility) {
							existedCloudPoint->addVisibility(vNewCP.first, vNewCP.second);
							keyframeIds.push_back(vNewCP.first);
							SRef<Keyframe> tmpKeyframe;
							keyframesManager->getKeyframe(vNewCP.first, tmpKeyframe);
							tmpKeyframe->addVisibility(vNewCP.second, it_cp->second);
							// modify cloud point descriptor
							existedCloudPoint->addNewDescriptor(tmpKeyframe->getDescriptors()->getDescriptor(vNewCP.second));
							// modify view direction
							Transform3Df poseTmpKf = tmpKeyframe->getPose();
							Vector3f newViewDirection(poseTmpKf(0, 3) - existedCloudPoint->getX(), poseTmpKf(1, 3) - existedCloudPoint->getY(), poseTmpKf(2, 3) - existedCloudPoint->getZ());
							existedCloudPoint->addNewViewDirection(newViewDirection);

						}
						// update covisibility graph
						covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[0], 1);
						covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[1], 1);
						covisibilityGraph->increaseEdge(keyframeIds[0], keyframeIds[1], 1);
						// this new cloud point is existed
						checkMatches[idxNewCloudPoint] = false;
					}
				}
				
			}
		}

		std::vector<std::tuple<uint32_t, int, uint32_t>> tmpInfoMatches;
		std::vector<SRef<CloudPoint>> tmpNewCloudPoint;
		for (int i = 0; i < checkMatches.size(); ++i)
			if (checkMatches[i])
				tmpNewCloudPoint.push_back(newCloudPoint[i]);
		tmpNewCloudPoint.swap(newCloudPoint);
	};

	// process to add a new keyframe
	auto processNewKeyframe = [&](SRef<Frame> &newFrame) -> SRef<Keyframe> {
		// create a new keyframe from the current frame
		SRef<Keyframe> newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);
		// Add to keyframe manager
		keyframesManager->addKeyframe(newKeyframe);
		// Add to BOW retrieval			
		keyframeRetriever->addKeyframe(newKeyframe);
		// Update keypoint visibility, descriptor in cloud point and connections between new keyframe with other keyframes
		updateAssociateCloudPoint(newKeyframe);
		// get best neighbor keyframes
		std::vector<uint32_t> idxBestNeighborKfs;
		covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME, idxBestNeighborKfs);
		// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
		std::vector<SRef<CloudPoint>> newCloudPoint;
		findMatchesAndTriangulation(newKeyframe, idxBestNeighborKfs, newCloudPoint);
		LOG_DEBUG("Number of new 3D points before fusing: {}", newCloudPoint.size());
		if (newCloudPoint.size() > 0) {
			// fuse duplicate points
			std::vector<uint32_t> idxNeigborKfs;
			covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME - 10, idxNeigborKfs);
			fuseCloudPoint(newKeyframe, idxNeigborKfs, newCloudPoint);	
		}
		LOG_DEBUG("Keyframe: {} -> Number of new 3D points: {}", newKeyframe->getId(), newCloudPoint.size());
		// add new points to point cloud manager, update visibility map and covisibility graph
		for (auto const &point : newCloudPoint)
			mapper->addCloudPoint(point);
		return newKeyframe;
	};		

	// Prepare for tracking
	lastPose = keyframe2->getPose();
	updateData(keyframe2, localMap, referenceKeyframe, frameToTrack);

	// Start tracking
	clock_t start, end;
	int count = 0;
	int countNewKeyframes = 0;
	start = clock();
	while (true)
	{
		// Get current image
		camera->getNextImage(view);
		keypointsDetector->detect(view, keypoints);
		LOG_INFO("Number of keypoints: {}", keypoints.size());
		descriptorExtractor->extract(view, keypoints, descriptors);		
		newFrame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, referenceKeyframe);
		// match current keypoints with the keypoints of the Keyframe
		matcher->match(frameToTrack->getDescriptors(), descriptors, matches);			
		matchesFilter->filter(matches, matches, frameToTrack->getKeypoints(), keypoints);			

		std::vector<Point2Df> pt2d;
		std::vector<Point3Df> pt3d;
		std::vector<CloudPoint> foundPoints;
		std::vector<DescriptorMatch> foundMatches;
		std::vector<DescriptorMatch> remainingMatches;
		corr2D3DFinder->find(frameToTrack, newFrame, matches, pt3d, pt2d, foundMatches, remainingMatches);		
		LOG_INFO("Number of 2D-3D finder: {}", pt3d.size());
		// display matches
		imageMatches = view->copy();

		std::vector<Point2Df> imagePoints_inliers;
		std::vector<Point3Df> worldPoints_inliers;
		if (pnpRansac->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, newFramePose, lastPose) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG(" pnp inliers size: {} / {}", worldPoints_inliers.size(), pt3d.size());
			LOG_DEBUG("Estimated pose: \n {}", newFramePose.matrix());
			// Set the pose of the new frame
			newFrame->setPose(newFramePose);

			// refine pose and update map visibility of frame
			{
				// get all keypoints of the new frame
				const std::vector<Keypoint> &keypoints = newFrame->getKeypoints();

				//  projection points
				std::vector< Point2Df > projected2DPts;
				projector->project(localMap, projected2DPts, newFrame->getPose());
					
				std::vector<SRef<DescriptorBuffer>> desAllLocalMap;
				for (auto &it_cp : localMap) {
					desAllLocalMap.push_back(it_cp->getDescriptor());
				}

				// matches feature in region
				std::vector<DescriptorMatch> allMatches;
				matcher->matchInRegion(projected2DPts, desAllLocalMap, newFrame, allMatches, 5.f);

				std::vector<Point2Df> pt2d;
				std::vector<Point3Df> pt3d;
				std::map<unsigned int, unsigned int> newMapVisibility;


				for (auto &it_match : allMatches) {
					int idx_2d = it_match.getIndexInDescriptorB();
					int idx_3d = it_match.getIndexInDescriptorA();
					pt2d.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
					pt3d.push_back(Point3Df(localMap[idx_3d]->getX(), localMap[idx_3d]->getY(), localMap[idx_3d]->getZ()));
					newMapVisibility[idx_2d] = localMap[idx_3d]->getId();
				}
					
				// pnp optimization
				Transform3Df refinedPose;
				pnp->estimate(pt2d, pt3d, refinedPose, newFrame->getPose());
				newFrame->setPose(refinedPose);

				// update map visibility of current frame
				newFrame->addVisibilities(newMapVisibility);
				LOG_DEBUG("Number of map visibility of frame to track: {}", newMapVisibility.size());
				overlay2D->drawCircles(pt2d, imageMatches);
				overlay3D->draw(refinedPose, imageMatches);
			}
			LOG_DEBUG("Refined pose: \n {}", newFrame->getPose().matrix());
			lastPose = newFrame->getPose();

			// check need new keyframe
			if (!isLostTrack && keyframeSelector->select(newFrame, checkDisparityDistance)){
				if (keyframeSelector->select(newFrame, checkNeedNewKfWithAllKfs)) {
					updateData(updatedRefKf, localMap, referenceKeyframe, frameToTrack);
				}
				else {
					SRef<Keyframe> newKeyframe = processNewKeyframe(newFrame);
					// Local bundle adjustment
					std::vector<uint32_t> bestIdx;
					covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME, bestIdx);						
					bestIdx.push_back(newKeyframe->getId());
                    bundleReprojError = bundler->bundleAdjustment(calibration, distortion, bestIdx);
					// loop closure
					countNewKeyframes++;
					if (countNewKeyframes >= NB_NEWKEYFRAMES_LOOP) {
						LOG_INFO("Last keyframe id: {}", newKeyframe->getId());
						SRef<Keyframe> detectedLoopKeyframe;
						Transform3Df sim3Transform;
						std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
						if (loopDetector->detect(newKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
							// detected loop keyframe
							LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
							// performs loop correction 
							countNewKeyframes = 0;
							loopCorrector->correct(newKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);
						}
						else
							LOG_INFO("Cannot detect a loop closure from last keyframe");
					}
					// update data
					LOG_INFO("Update data");
					updateData(newKeyframe, localMap, referenceKeyframe, frameToTrack);
					// add keyframe pose to display
					keyframePoses.push_back(newKeyframe->getPose());
					LOG_INFO("Number of keyframe: {} -> cloud current size: {} \n", keyframesManager->getNbKeyframes(), pointCloudManager->getNbPoints());					
				}
			}
			else
			{
				// update frame to track
				frameToTrack = newFrame;
			}

			framePoses.push_back(newFrame->getPose()); // used for display

			isLostTrack = false;	// tracking is good

		}
		else {
			LOG_INFO("Pose estimation has failed");
			isLostTrack = true;		// lost tracking
			// reloc
			std::vector < uint32_t> retKeyframesId;
			if (keyframeRetriever->retrieve(newFrame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
				// update data
				SRef<Keyframe> bestRetKeyframe;
				keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe);
				updateData(bestRetKeyframe, localMap, referenceKeyframe, frameToTrack);
				lastPose = referenceKeyframe->getPose();
				LOG_INFO("Retrieval Success");
			}
			else
				LOG_INFO("Retrieval Failed");
		}

		LOG_DEBUG("Nb of Local Map / World Map: {} / {}", localMap.size(), pointCloudManager->getNbPoints());
		LOG_DEBUG("Index of current reference keyframe: {}", referenceKeyframe->getId());

		// display matches and a cube on the fiducial marker
		if (imageViewer->display(imageMatches) == FrameworkReturnCode::_STOP)
			break;

		// get all keyframes and point cloud
		keyframePoses.clear();
		std::vector<SRef<Keyframe>> allKeyframes;
		keyframesManager->getAllKeyframes(allKeyframes);
		for (auto const &it : allKeyframes)
			keyframePoses.push_back(it->getPose());
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloudManager->getAllPoints(pointCloud);
		// display point cloud 
		if (viewer3DPoints->display(pointCloud, lastPose, keyframePoses, framePoses, localMap) == FrameworkReturnCode::_STOP)
			break;

		count++;
	}

	// display stats on frame rate
	end = clock();
	double duration = double(end - start) / CLOCKS_PER_SEC;
	printf("\n\nElasped time is %.2lf seconds.\n", duration);
	printf("Number of processed frame per second : %8.2f\n", count / duration);

	// Save map
	mapper->saveToFile();

	return 0;
}
