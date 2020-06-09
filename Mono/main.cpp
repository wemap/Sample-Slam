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

//#define USE_IMAGES_SET

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
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
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

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"

#define MIN_THRESHOLD -1
#define MAX_THRESHOLD 220
#define NB_THRESHOLD 3
#define NB_POINTCLOUD_INIT 50
#define MIN_WEIGHT_NEIGHBOR_KEYFRAME 50
#define MIN_POINT_DISTANCE 0.04


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
#ifdef USE_IMAGES_SET
    auto camera = xpcfComponentManager->resolve<input::devices::ICamera>("ImagesAsCamera");
#else
    auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
#endif
	// storage components
	auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
	auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
	auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
	auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
	auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
		
	// processing components
    auto  keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
    auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
    auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
    auto poseFinderFrom2D2D = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D2D>();
    auto triangulator = xpcfComponentManager->resolve<solver::map::ITriangulator>();
    auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
    auto pnpRansac = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom2D3D>();
    auto pnp = xpcfComponentManager->resolve<solver::pose::I3DTransformFinderFrom2D3D>();
    auto corr2D3DFinder = xpcfComponentManager->resolve<solver::pose::I2D3DCorrespondencesFinder>();
    auto mapFilter = xpcfComponentManager->resolve<solver::map::IMapFilter>();        
    auto keyframeSelector = xpcfComponentManager->resolve<solver::map::IKeyframeSelector>();
    auto matchesOverlay = xpcfComponentManager->resolve<display::IMatchesOverlay>();
    auto matchesOverlayBlue = xpcfComponentManager->resolve<display::IMatchesOverlay>("matchesBlue");
    auto matchesOverlayRed = xpcfComponentManager->resolve<display::IMatchesOverlay>("matchesRed");
    auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
    auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();        
    auto projector = xpcfComponentManager->resolve<geom::IProject>();
    auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();

	// marker fiducial components
    auto binaryMarker = xpcfComponentManager->resolve<input::files::IMarker2DSquaredBinary>();
    auto imageFilterBinary = xpcfComponentManager->resolve<image::IImageFilter>();
    auto imageConvertor = xpcfComponentManager->resolve<image::IImageConvertor>();
    auto contoursExtractor = xpcfComponentManager->resolve<features::IContoursExtractor>();
    auto contoursFilter = xpcfComponentManager->resolve<features::IContoursFilter>();
    auto perspectiveController = xpcfComponentManager->resolve<image::IPerspectiveController>();
    auto patternDescriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractorSBPattern>();
    auto patternMatcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>("DescMatcherFiducial");
    auto patternReIndexer = xpcfComponentManager->resolve<features::ISBPatternReIndexer>();
    auto img2worldMapper = xpcfComponentManager->resolve<geom::IImage2WorldMapper>();
    auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
    auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();

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
	bool												bundling = true;
	double												bundleReprojError;		

	// components initialisation for marker detection
	SRef<DescriptorBuffer> markerPatternDescriptor;
	binaryMarker->loadMarker();
	patternDescriptorExtractor->extract(binaryMarker->getPattern(), markerPatternDescriptor);
	LOG_DEBUG("Marker pattern:\n {}", binaryMarker->getPattern().getPatternMatrix())
	// Set the size of the box to display according to the marker size in world unit
	overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().width, 0);
	overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height, 1);
	overlay3D->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(binaryMarker->getSize().height / 2.0f, 2);
	int patternSize = binaryMarker->getPattern().getSize();
	patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
	patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);
	img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(patternSize);
	img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(patternSize);
	img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(binaryMarker->getSize().width);
	img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(binaryMarker->getSize().height);

	// initialize pose estimation with the camera intrinsic parameters (please refer to the use of intrinsic parameters file)
    calibration = camera->getIntrinsicsParameters();
    distortion = camera->getDistortionParameters();
    overlay3D->setCameraParameters(calibration, distortion);
    pnpRansac->setCameraParameters(calibration, distortion);
    pnp->setCameraParameters(calibration, distortion);
    poseFinderFrom2D2D->setCameraParameters(calibration, distortion);
    triangulator->setCameraParameters(calibration, distortion);
    projector->setCameraParameters(calibration, distortion);
    LOG_DEBUG("Intrincic parameters : \n {}", distortion);

	if (camera->start() != FrameworkReturnCode::_SUCCESS)
	{
		LOG_ERROR("Camera cannot start");
		return -1;
	}

	// camera pose estimation function based on the fiducial marker
	auto detectFiducialMarker = [&imageConvertor, &imageFilterBinary, &contoursExtractor, &contoursFilter, &perspectiveController,
		&patternDescriptorExtractor, &patternMatcher, &markerPatternDescriptor, &patternReIndexer, &img2worldMapper, &pnp, &overlay3D](SRef<Image>& image, Transform3Df &pose){
		SRef<Image>                     greyImage, binaryImage;
		std::vector<Contour2Df>			contours;
		std::vector<Contour2Df>			filtered_contours;
		std::vector<SRef<Image>>        patches;
		std::vector<Contour2Df>			recognizedContours;
		SRef<DescriptorBuffer>          recognizedPatternsDescriptors;
		std::vector<DescriptorMatch>    patternMatches;
		std::vector<Point2Df>			pattern2DPoints;
		std::vector<Point2Df>			img2DPoints;
		std::vector<Point3Df>			pattern3DPoints;

		bool marker_found = false;
		// Convert Image from RGB to grey
		imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
		for (int num_threshold = 0; !marker_found && num_threshold < NB_THRESHOLD; num_threshold++)
		{
			// Compute the current Threshold valu for image binarization
			int threshold = MIN_THRESHOLD + (MAX_THRESHOLD - MIN_THRESHOLD)*((float)num_threshold / (float)(NB_THRESHOLD - 1));
			// Convert Image from grey to black and white
			imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("min")->setIntegerValue(threshold);
			imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("max")->setIntegerValue(255);
			// Convert Image from grey to black and white
			imageFilterBinary->filter(greyImage, binaryImage);
			// Extract contours from binary image
			contoursExtractor->extract(binaryImage, contours);
			// Filter 4 edges contours to find those candidate for marker contours
			contoursFilter->filter(contours, filtered_contours);
			// Create one warpped and cropped image by contour
			perspectiveController->correct(binaryImage, filtered_contours, patches);
			// test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
			if (patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
			{
				// From extracted squared binary pattern, match the one corresponding to the squared binary marker
				if (patternMatcher->match(markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == features::IDescriptorMatcher::DESCRIPTORS_MATCHER_OK)
				{
					// Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
					patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
					// Compute the 3D position of each corner of the marker
					img2worldMapper->map(pattern2DPoints, pattern3DPoints);
					// Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
					if (pnp->estimate(img2DPoints, pattern3DPoints, pose) == FrameworkReturnCode::_SUCCESS)
					{														
						marker_found = true;
					}
				}
			}
		}
		return marker_found;
	};

	// calculate distance between two center of camera poses
	auto centerCamDistance = [](const Transform3Df & pose1, const Transform3Df & pose2) {
		return std::sqrt(std::pow(pose1(0, 3) - pose2(0, 3), 2.f) + std::pow(pose1(1, 3) - pose2(1, 3), 2.f) +
			std::pow(pose1(2, 3) - pose2(2, 3), 2.f));
	};

	// calculate angle between z-axis of two camera poses
	auto angleCamDistance = [](const Transform3Df & pose1, const Transform3Df & pose2) {
		return std::acos(pose1(0, 2) * pose2(0, 2) + pose1(1, 2) * pose2(1, 2) + pose1(2, 2) * pose2(2, 2));
	};

	// Initialization by get two keyframes using pose estimation from fiducial marker
	SRef<Frame>						frame1, frame2;
	SRef<Keyframe>					keyframe1, keyframe2;
	std::vector<SRef<CloudPoint>>	cloud, filteredCloud;
	bool bootstrapOk = false;
	bool initFrame1 = false;

	// Load map from file
	if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
		bootstrapOk = true;
		keyframesManager->getKeyframe(0, keyframe2);
	}
	else
		LOG_INFO("Initialization from scratch");

	while (!bootstrapOk)
	{
		if (camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_)
			break;

		if (!detectFiducialMarker(view, poseFrame)) {
			if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
				return 1;
			continue;
		}
		if (!initFrame1) {
			initFrame1 = true;
			keypointsDetector->detect(view, keypoints);
			descriptorExtractor->extract(view, keypoints, descriptors);
			frame1 = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, poseFrame);
		}
		else {
			// check center camera distance
			Transform3Df poseFrame1 = frame1->getPose();
			if (centerCamDistance(poseFrame, poseFrame1) < 0.1) {
                overlay3D->draw(poseFrame, view);
				if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
					return 1;
				continue;
			}
			// feature extraction
			keypointsDetector->detect(view, keypoints);
			descriptorExtractor->extract(view, keypoints, descriptors);
			frame2 = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, poseFrame);
			// check angle camera distance
			if (angleCamDistance(poseFrame, poseFrame1) > 0.1) {
				frame1 = frame2;
				continue;
			}
			// matching
			matcher->match(frame1->getDescriptors(), frame2->getDescriptors(), matches);
			matchesFilter->filter(matches, matches, frame1->getKeypoints() , frame2->getKeypoints());
			matchesOverlay->draw(view, imageMatches, frame1->getKeypoints(), frame2->getKeypoints(), matches);
			if (imageViewer->display(imageMatches) == SolAR::FrameworkReturnCode::_STOP)
				return 1;
			// Triangulate
			cloud.clear();
			filteredCloud.clear();
			triangulator->triangulate(frame1->getKeypoints(), frame2->getKeypoints(), frame1->getDescriptors(), frame2->getDescriptors(), matches,
				std::make_pair(0, 1), frame1->getPose(), frame2->getPose(), cloud);
			mapFilter->filter(frame1->getPose(), frame2->getPose(), cloud, filteredCloud);
			std::cout << filteredCloud.size() << std::endl;
            if (filteredCloud.size() > NB_POINTCLOUD_INIT) {
				// add keyframes to keyframes manager
				keyframe1 = xpcf::utils::make_shared<Keyframe>(frame1);				
				keyframesManager->addKeyframe(keyframe1);
				keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
				keyframesManager->addKeyframe(keyframe2);
				keyframe2->setReferenceKeyframe(keyframe1);
				keyframePoses.push_back(keyframe1->getPose()); // used for display
				keyframePoses.push_back(keyframe2->getPose()); // used for display
				// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
				for (auto const &it : filteredCloud)
					mapper->addCloudPoint(it);
				// add keyframes to retriever
				keyframeRetriever->addKeyframe(keyframe1);
				keyframeRetriever->addKeyframe(keyframe2);
				// apply bundle adjustement 
				if (bundling) {
                    bundleReprojError = bundler->solve(calibration, distortion, { 0,1 });
				}
				bootstrapOk = true;
			}
			else {
				frame1 = frame2;
			}
		}							
	}
	
	LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
	LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());

	// update data function
	auto updateData = [&](const SRef<Keyframe> &refKf, std::vector<SRef<CloudPoint>> &localMap, SRef<Keyframe> & referenceKeyframe, SRef<Frame> &frameToTrack)
	{
		referenceKeyframe = refKf;
		LOG_INFO("Update new reference keyframe with id {}", referenceKeyframe->getId());
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
	std::function<bool(const SRef<Frame>&)> checkNeedNewKfWithAllKfs = [&referenceKeyframe, &mapper, &keyframeRetriever, &updatedRefKf, &keyframesManager](const SRef<Frame>& newFrame) -> bool {
		std::vector < uint32_t> ret_keyframesId;
		if (keyframeRetriever->retrieve(newFrame, ret_keyframesId) == FrameworkReturnCode::_SUCCESS) {
			if (ret_keyframesId[0] != referenceKeyframe->getId()) {
				keyframesManager->getKeyframe(ret_keyframesId[0], updatedRefKf);
				LOG_INFO("Update new reference keyframe with id {}", referenceKeyframe->getId());
				return true;
			}
			LOG_INFO("Find same reference keyframe, need make new keyframe");
			return false;
		}
		else
			return false;
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
				///Todo: update descriptor of cp: des_cp = ((des_cp * cp.getVisibility().size()) + des_buf) / (cp.getVisibility().size() + 1)
				cloudPoint->addVisibility(newKf->getId(), it.first);
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

			// check distance between two keyframes
			if ((centerCamDistance(newKf_pose, tmpKf_pose) < 0.05) || (angleCamDistance(newKf_pose, tmpKf_pose) > 0.5))
				continue;

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
				if (it_cp != mapVisibilitiesNeighbor.end()) {
					SRef<CloudPoint> existedCloudPoint; 
					if (pointCloudManager->getPoint(it_cp->second, existedCloudPoint) == FrameworkReturnCode::_SUCCESS) {
						if ((*existedCloudPoint - *newCloudPoint[idxNewCloudPoint]).magnitude() < MIN_POINT_DISTANCE) {
							// add visibilities to this cloud point and keyframes
							std::vector<uint32_t> keyframeIds;
							for (auto const &vNewCP : newCloudPointVisibility) {
								existedCloudPoint->addVisibility(vNewCP.first, vNewCP.second);
								keyframeIds.push_back(vNewCP.first);
								SRef<Keyframe> tmpKeyframe;
								keyframesManager->getKeyframe(vNewCP.first, tmpKeyframe);
								tmpKeyframe->addVisibility(vNewCP.second, it_cp->second);
							}
							// update covisibility graph
							covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[0], 1);
							covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[1], 1);
							covisibilityGraph->increaseEdge(keyframeIds[0], keyframeIds[1], 1);
							// this new cloud point is existed
							checkMatches[idxNewCloudPoint] = false;
							/// Todo: Modify cloud point descriptor
						}
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
	start = clock();
	while (true)
	{
		// Get current image
		camera->getNextImage(view);
		keypointsDetector->detect(view, keypoints);
		LOG_DEBUG("Number of keypoints: {}", keypoints.size());
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
					if (bundling) {
						// Local bundle adjustment
						std::vector<uint32_t> bestIdx;
						covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME, bestIdx);						
						bestIdx.push_back(newKeyframe->getId());
                        bundleReprojError = bundler->solve(calibration, distortion, bestIdx);
					}
					// update data
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
