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
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/solver/map/IMapper.h"
#include "api/display/I3DOverlay.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/solver/map/IBundler.h"
#include "api/loop/ILoopClosureDetector.h"
#include "api/loop/ILoopCorrector.h"
#include "api/slam/IBootstrapper.h"
#include "api/slam/ITracking.h"
#include "api/slam/IMapping.h"

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

	std::string configxml = std::string("conf_SLAM_Multi.xml");
	if (argc == 2)
		configxml = std::string(argv[1]);
	if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS)
	{
		LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
			return -1;
	}

	// declare and create components
	LOG_INFO("Start creating components");
	auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
	auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
	auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
	auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
	auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
	auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
	auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
	auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
	auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
	auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
	auto fiducialMarkerPoseEstimator = xpcfComponentManager->resolve<solver::pose::IFiducialMarkerPose>();
	auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
	auto globalBundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
	auto loopDetector = xpcfComponentManager->resolve<loop::ILoopClosureDetector>();
	auto loopCorrector = xpcfComponentManager->resolve<loop::ILoopCorrector>();
	auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
	auto bootstrapper = xpcfComponentManager->resolve<slam::IBootstrapper>();
	auto tracking = xpcfComponentManager->resolve<slam::ITracking>();
	auto mapping = xpcfComponentManager->resolve<slam::IMapping>();
	LOG_INFO("Loaded all components");

	// initialize pose estimation with the camera intrinsic parameters (please refeer to the use of intrinsec parameters file)
	CamCalibration calibration = camera->getIntrinsicsParameters();
	CamDistortion distortion = camera->getDistortionParameters();
	overlay3D->setCameraParameters(calibration, distortion);
	loopDetector->setCameraParameters(calibration, distortion);
	loopCorrector->setCameraParameters(calibration, distortion);
	fiducialMarkerPoseEstimator->setCameraParameters(calibration, distortion);
	bootstrapper->setCameraParameters(calibration, distortion);
	tracking->setCameraParameters(calibration, distortion);
	mapping->setCameraParameters(calibration, distortion);
	LOG_DEBUG("Intrincic parameters : \n {}", calibration);

	// get properties
	float minWeightNeighbor = mapping->bindTo<xpcf::IConfigurable>()->getProperty("minWeightNeighbor")->getFloatingValue();
	float reprojErrorThreshold = mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();

	if (camera->start() != FrameworkReturnCode::_SUCCESS)
	{
		LOG_ERROR("Camera cannot start");
		return -1;
	}

	// Load map from file
	bool stop = false;
	SRef<Keyframe> keyframe2;
	if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
		keyframesManager->getKeyframe(0, keyframe2);
	}
	else {
		LOG_INFO("Initialization from scratch");
		bool bootstrapOk = false;
		while (!bootstrapOk) {
			SRef<Image> image, view;
			camera->getNextImage(image);
			Transform3Df pose = Transform3Df::Identity();
			fiducialMarkerPoseEstimator->estimate(image, pose);
			if (bootstrapper->process(image, view, pose) == FrameworkReturnCode::_SUCCESS) {
				double bundleReprojError = bundler->bundleAdjustment(calibration, distortion);
				bootstrapOk = true;
			}
			if (!pose.isApprox(Transform3Df::Identity()))
				overlay3D->draw(pose, view);
			if (imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
				return 1;
		}
		keyframesManager->getKeyframe(1, keyframe2);
	}

	LOG_INFO("Number of initial point cloud: {}", pointCloudManager->getNbPoints());
	LOG_INFO("Number of initial keyframes: {}", keyframesManager->getNbKeyframes());

	// display point cloud function
	auto fnDisplay = [&keyframesManager, &pointCloudManager, &viewer3DPoints](const std::vector<Transform3Df>& framePoses) {
		// get all keyframes and point cloud
		std::vector<Transform3Df>   keyframePoses;
		std::vector<SRef<Keyframe>> allKeyframes;
		keyframesManager->getAllKeyframes(allKeyframes);
		for (auto const &it : allKeyframes)
			keyframePoses.push_back(it->getPose());
		std::vector<SRef<CloudPoint>> pointCloud;
		pointCloudManager->getAllPoints(pointCloud);
		// display point cloud 
		if (viewer3DPoints->display(pointCloud, framePoses.back(), keyframePoses, framePoses) == FrameworkReturnCode::_STOP)
			return false;
		else
			return true;
	};

	// Prepare for tracking
	tracking->updateReferenceKeyframe(keyframe2);

	// init display point cloud
	fnDisplay({ keyframe2->getPose() });

	xpcf::DropBuffer<SRef<Image>>												m_dropBufferCamImageCapture;
	xpcf::DropBuffer<SRef<Frame>>												m_dropBufferAddKeyframe;
	xpcf::DropBuffer<SRef<Keyframe>>											m_dropBufferNewKeyframe;
	xpcf::DropBuffer<SRef<Keyframe>>											m_dropBufferNewKeyframeLoop;
	xpcf::DropBuffer<SRef<Image>>												m_dropBufferDisplay;
	xpcf::DropBuffer< std::pair<SRef<Image>, std::vector<Keypoint>>>			m_dropBufferKeypoints;
	xpcf::DropBuffer<SRef<Frame>>												m_dropBufferFrameDescriptors;


	// Camera image capture task
	auto fnCamImageCapture = [&]()
	{
		SRef<Image> view;
		if (camera->getNextImage(view) != SolAR::FrameworkReturnCode::_SUCCESS)
		{
			stop = true;
			return;
		}
		m_dropBufferCamImageCapture.push(view);
	};

	// Keypoint detection task
	auto fnDetection = [&]()
	{
		SRef<Image> frame;
		if (!m_dropBufferCamImageCapture.tryPop(frame)) {
			xpcf::DelegateTask::yield();
			return;
		}
		std::vector<Keypoint> keypoints;
		keypointsDetector->detect(frame, keypoints);
		if (keypoints.size() > 0)
			m_dropBufferKeypoints.push(std::make_pair(frame, keypoints));
	};

	// Feature extraction task
	auto fnExtraction = [&]()
	{
		std::pair<SRef<Image>, std::vector<Keypoint>> frameKeypoints;
		if (!m_dropBufferKeypoints.tryPop(frameKeypoints)) {
			xpcf::DelegateTask::yield();
			return;
		}
		SRef<DescriptorBuffer> descriptors;
		descriptorExtractor->extract(frameKeypoints.first, frameKeypoints.second, descriptors);
		SRef<Frame> frame = xpcf::utils::make_shared<Frame>(frameKeypoints.second, descriptors, frameKeypoints.first);
		m_dropBufferFrameDescriptors.push(frame);
	};

	// Tracking task	
	std::vector<Transform3Df>   framePoses{keyframe2->getPose()};
	int count = 0;
	bool isStopMapping = false;
	auto fnTracking = [&]()
	{
		SRef<Frame> frame;
		if (!m_dropBufferFrameDescriptors.tryPop(frame)) {
			xpcf::DelegateTask::yield();
			return;
		}
		SRef<Keyframe> newKeyframe;
		if (m_dropBufferNewKeyframe.tryPop(newKeyframe))
		{
			tracking->updateReferenceKeyframe(newKeyframe);
			SRef<Frame> tmpFrame;
			m_dropBufferAddKeyframe.tryPop(tmpFrame);
			isStopMapping = false;
		}
		// tracking
		SRef<Image>	displayImage;
		if (tracking->process(frame, displayImage) == FrameworkReturnCode::_SUCCESS) {
			// used for display
			framePoses.push_back(frame->getPose());
			// draw cube
			overlay3D->draw(frame->getPose(), displayImage);
			// send frame to mapping task
			m_dropBufferAddKeyframe.push(frame);
		}

		if (imageViewer->display(displayImage) == SolAR::FrameworkReturnCode::_STOP) {
			stop = true;
		}

		// display point cloud
		if (!fnDisplay(framePoses))
			stop = true;
		++count;
	};

	// check need new keyframe
	int countNewKeyframes = 0;
	auto fnMapping = [&]()
	{
		SRef<Frame> frame;
		if (isStopMapping || (!m_dropBufferAddKeyframe.tryPop(frame))) {
			xpcf::DelegateTask::yield();
			return;
		}
		SRef<Keyframe> keyframe;
		if (mapping->process(frame, keyframe) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("New keyframe id: {}", keyframe->getId());
			// Local bundle adjustment
			std::vector<uint32_t> bestIdx;
			covisibilityGraph->getNeighbors(keyframe->getId(), minWeightNeighbor, bestIdx);
			bestIdx.push_back(keyframe->getId());
			double bundleReprojError = bundler->bundleAdjustment(calibration, distortion, bestIdx);
			mapper->pruning();
			countNewKeyframes++;
			m_dropBufferNewKeyframeLoop.push(keyframe);
		}

		if (keyframe) {
			isStopMapping = true;
			m_dropBufferNewKeyframe.push(keyframe);
		}

	};


	// loop closure task
	auto fnLoopClosure = [&]() {
		SRef<Keyframe> lastKeyframe;
		if ((countNewKeyframes < NB_NEWKEYFRAMES_LOOP) || (!m_dropBufferNewKeyframeLoop.tryPop(lastKeyframe))) {
			xpcf::DelegateTask::yield();
			return;
		}
		uint32_t lastKeyframeId = lastKeyframe->getId();
		SRef<Keyframe> detectedLoopKeyframe;
		Transform3Df sim3Transform;
		std::vector<std::pair<uint32_t, uint32_t>> duplicatedPointsIndices;
		if (loopDetector->detect(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices) == FrameworkReturnCode::_SUCCESS) {
			// detected loop keyframe
			LOG_INFO("Detected loop keyframe id: {}", detectedLoopKeyframe->getId());
			// performs loop correction 			
			loopCorrector->correct(lastKeyframe, detectedLoopKeyframe, sim3Transform, duplicatedPointsIndices);	
			countNewKeyframes = 0;
			// Loop optimisation
			globalBundler->bundleAdjustment(calibration, distortion);
			LOG_DEBUG("Global BA done");
			mapper->pruning();
		}
	};

	// instantiate and start tasks
	xpcf::DelegateTask taskCamImageCapture(fnCamImageCapture);
    xpcf::DelegateTask taskDetection(fnDetection);
    xpcf::DelegateTask taskExtraction(fnExtraction);
	xpcf::DelegateTask taskMapping(fnMapping);
	xpcf::DelegateTask taskLoopClosure(fnLoopClosure);

	taskCamImageCapture.start();
    taskDetection.start();
    taskExtraction.start();
	taskMapping.start();
	taskLoopClosure.start();

	// Start tracking
	clock_t start, end;		
	start = clock();
	while (!stop)
	{
        fnTracking();
	}

	// Stop tasks
	taskCamImageCapture.stop();
    taskDetection.stop();
    taskExtraction.stop();
	taskMapping.stop();	
	taskLoopClosure.stop();

	// display stats on frame rate
	end = clock();
	double duration = double(end - start) / CLOCKS_PER_SEC;
	printf("\n\nElasped time is %.2lf seconds.\n", duration);
	printf("Number of processed frame per second : %8.2f\n", count / duration);	

	// run global BA before exit
	bundler->bundleAdjustment(calibration, distortion);
	mapper->pruning();
	LOG_INFO("Nb keyframes of map: {}", keyframesManager->getNbKeyframes());
	LOG_INFO("Nb cloud points of map: {}", pointCloudManager->getNbPoints());

	// visualize final map	
	while (fnDisplay(framePoses)) {}

	// Save map
	mapper->saveToFile();

	return 0;
}
