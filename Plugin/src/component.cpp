#include "xpcf/module/ModuleFactory.h"
#include "PipeLineSlam.h"

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleFBOW_traits.h"
#include "core/Log.h"

#define USE_FREE
//#define USE_IMAGES_SET

#define MIN_THRESHOLD -1
#define MAX_THRESHOLD 220
#define NB_THRESHOLD 2

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::FBOW;
#ifndef USE_FREE
using namespace SolAR::MODULES::NONFREEOPENCV;
#endif
using namespace SolAR::MODULES::TOOLS;

namespace xpcf = org::bcom::xpcf;

// The pipeline component for the fiducial marker

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::PIPELINES::PipelineSlam)

namespace SolAR {
using namespace datastructure;
using namespace api::pipeline;
namespace PIPELINES {

PipelineSlam::PipelineSlam():ConfigurableBase(xpcf::toUUID<PipelineSlam>())
{
   addInterface<api::pipeline::IPipeline>(this);
    LOG_DEBUG(" Pipeline constructor");
    m_bootstrapOk=false;
    m_firstKeyframeCaptured = false;
    m_isLostTrack = false;
	m_stopFlag = false;
	m_startedOK = false;
}


PipelineSlam::~PipelineSlam()
{
     LOG_DEBUG(" Pipeline destructor")
}

FrameworkReturnCode PipelineSlam::init(SRef<xpcf::IComponentManager> xpcfComponentManager)
{
    // component creation
    try {
    #ifdef USE_IMAGES_SET
        m_camera = xpcfComponentManager->create<MODULES::OPENCV::SolARImagesAsCameraOpencv>()->bindTo<input::devices::ICamera>();
    #else
        m_camera =xpcfComponentManager->create<MODULES::OPENCV::SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    #endif

    #ifdef USE_FREE
        m_keypointsDetector =xpcfComponentManager->create<MODULES::OPENCV::SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
        m_descriptorExtractor =xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
    #else
		m_keypointsDetector = xpcfComponentManager->create<MODULES::OPENCV::SolARKeypointDetectorNonFreeOpencv>()->bindTo<features::IKeypointDetector>();
		m_descriptorExtractor = xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorsExtractorSURF64Opencv>()->bindTo<features::IDescriptorsExtractor>();
    #endif		
		m_matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
		m_poseFinderFrom2D2D = xpcfComponentManager->create<SolARPoseFinderFrom2D2DOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D2D>();
		m_triangulator = xpcfComponentManager->create<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
		m_matchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
		m_pnpRansac = xpcfComponentManager->create<SolARPoseEstimationSACPnpOpencv>()->bindTo<solver::pose::I3DTransformSACFinderFrom2D3D>();
		m_pnp = xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
		m_corr2D3DFinder = xpcfComponentManager->create<SolAR2D3DCorrespondencesFinderOpencv>()->bindTo<solver::pose::I2D3DCorrespondencesFinder>();
		m_mapFilter = xpcfComponentManager->create<SolARMapFilter>()->bindTo<solver::map::IMapFilter>();
		m_mapper = xpcfComponentManager->create<SolARMapper>()->bindTo<solver::map::IMapper>();
		m_keyframeSelector = xpcfComponentManager->create<SolARKeyframeSelector>()->bindTo<solver::map::IKeyframeSelector>();
		m_kfRetriever = xpcfComponentManager->create<SolARKeyframeRetrieverFBOW>()->bindTo<reloc::IKeyframeRetriever>();
		m_projector = xpcfComponentManager->create<SolARProjectOpencv>()->bindTo<geom::IProject>();
        m_sink = xpcfComponentManager->create<MODULES::TOOLS::SolARBasicSink>()->bindTo<sink::ISinkPoseImage>();
		// marker fiducial
		m_binaryMarker = xpcfComponentManager->create<SolARMarker2DSquaredBinaryOpencv>()->bindTo<input::files::IMarker2DSquaredBinary>();
		m_imageFilterBinary = xpcfComponentManager->create<SolARImageFilterBinaryOpencv>()->bindTo<image::IImageFilter>();
		m_imageConvertor = xpcfComponentManager->create<SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
		m_contoursExtractor = xpcfComponentManager->create<SolARContoursExtractorOpencv>()->bindTo<features::IContoursExtractor>();
		m_contoursFilter = xpcfComponentManager->create<SolARContoursFilterBinaryMarkerOpencv>()->bindTo<features::IContoursFilter>();
		m_perspectiveController = xpcfComponentManager->create<SolARPerspectiveControllerOpencv>()->bindTo<image::IPerspectiveController>();
		m_patternDescriptorExtractor = xpcfComponentManager->create<SolARDescriptorsExtractorSBPatternOpencv>()->bindTo<features::IDescriptorsExtractorSBPattern>();
		m_patternMatcher = xpcfComponentManager->create<SolARDescriptorMatcherRadiusOpencv>()->bindTo<features::IDescriptorMatcher>();
		m_patternReIndexer = xpcfComponentManager->create<SolARSBPatternReIndexer>()->bindTo<features::ISBPatternReIndexer>();
		m_img2worldMapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
		m_i2DOverlay = xpcfComponentManager->create<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();

        // load marker
        LOG_INFO("LOAD MARKER IMAGE ");
		if( m_binaryMarker->loadMarker()==FrameworkReturnCode::_ERROR_){
           return FrameworkReturnCode::_ERROR_;
		}
        LOG_INFO("MARKER IMAGE LOADED");

        m_patternDescriptorExtractor->extract(m_binaryMarker->getPattern(), m_markerPatternDescriptor);
		LOG_INFO("Marker pattern:\n {}", m_binaryMarker->getPattern().getPatternMatrix());
		
        int patternSize = m_binaryMarker->getPattern().getSize();

        m_patternDescriptorExtractor->bindTo<xpcf::IConfigurable>()->getProperty("patternSize")->setIntegerValue(patternSize);
        m_patternReIndexer->bindTo<xpcf::IConfigurable>()->getProperty("sbPatternSize")->setIntegerValue(patternSize);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(patternSize);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(patternSize);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(m_binaryMarker->getSize().width);
        m_img2worldMapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(m_binaryMarker->getSize().height);

        // initialize components requiring the camera intrinsic and distortion parameters
        m_pnp->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_pnpRansac->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_poseFinderFrom2D2D->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_triangulator->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_projector->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());   

        m_initOK = true;		
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR("Exception catched: {}", e.what());
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

bool PipelineSlam::detectFiducialMarker(SRef<Image>& image, Transform3Df &pose)
{
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
	m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);
	for (int num_threshold = 0; !marker_found && num_threshold < NB_THRESHOLD; num_threshold++)
	{
		// Compute the current Threshold valu for image binarization
		int threshold = MIN_THRESHOLD + (MAX_THRESHOLD - MIN_THRESHOLD)*((float)num_threshold / (float)(NB_THRESHOLD - 1));
		// Convert Image from grey to black and white
		m_imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("min")->setIntegerValue(threshold);
		m_imageFilterBinary->bindTo<xpcf::IConfigurable>()->getProperty("max")->setIntegerValue(255);
		// Convert Image from grey to black and white
		m_imageFilterBinary->filter(greyImage, binaryImage);
		// Extract contours from binary image
		m_contoursExtractor->extract(binaryImage, contours);
		// Filter 4 edges contours to find those candidate for marker contours
		m_contoursFilter->filter(contours, filtered_contours);
		// Create one warpped and cropped image by contour
		m_perspectiveController->correct(binaryImage, filtered_contours, patches);
		// test if this last image is really a squared binary marker, and if it is the case, extract its descriptor
		if (m_patternDescriptorExtractor->extract(patches, filtered_contours, recognizedPatternsDescriptors, recognizedContours) != FrameworkReturnCode::_ERROR_)
		{
			// From extracted squared binary pattern, match the one corresponding to the squared binary marker
			if (m_patternMatcher->match(m_markerPatternDescriptor, recognizedPatternsDescriptors, patternMatches) == features::IDescriptorMatcher::DESCRIPTORS_MATCHER_OK)
			{
				// Reindex the pattern to create two vector of points, the first one corresponding to marker corner, the second one corresponding to the poitsn of the contour
				m_patternReIndexer->reindex(recognizedContours, patternMatches, pattern2DPoints, img2DPoints);
				// Compute the 3D position of each corner of the marker
				m_img2worldMapper->map(pattern2DPoints, pattern3DPoints);
				// Compute the pose of the camera using a Perspective n Points algorithm using only the 4 corners of the marker
				if (m_pnp->estimate(img2DPoints, pattern3DPoints, pose) == FrameworkReturnCode::_SUCCESS)
				{
					marker_found = true;
				}
			}
		}
	}
	return marker_found;
}

void PipelineSlam::updateData(const SRef<Keyframe> refKf)
{	
	m_frameToTrack = xpcf::utils::make_shared<Frame>(m_referenceKeyframe);
	m_frameToTrack->setReferenceKeyframe(m_referenceKeyframe);
	m_idxLocalMap.clear();
	m_localMap.clear();
	m_mapper->getLocalMapIndex(m_referenceKeyframe, m_idxLocalMap);
	for (auto it : m_idxLocalMap)
		m_localMap.push_back(m_mapper->getGlobalMap()->getAPoint(it));
}

void PipelineSlam::updateReferenceKeyframe(const SRef<Keyframe> refKf)
{
	std::unique_lock<std::mutex> lock(globalVarsMutex);
	m_referenceKeyframe = refKf;
	LOG_DEBUG("Update new reference keyframe with id {}", m_referenceKeyframe->m_idx);
};

bool PipelineSlam::checkNeedNewKfWithAllKfs(const SRef<Frame>& newFrame)
{
	std::vector < SRef <Keyframe>> ret_keyframes;
	if (m_kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
		if (ret_keyframes[0]->m_idx != m_referenceKeyframe->m_idx) {
			m_updatedRefKf = ret_keyframes[0];
			LOG_DEBUG("Find new reference keyframe with id {}", m_updatedRefKf->m_idx);
			return true;
		}
		LOG_DEBUG("Find same reference keyframe, need make new keyframe");
		return false;
	}
	else
		return false;
};

bool PipelineSlam::checkDisparityDistance(const SRef<Frame>& newFrame) {
	const std::vector<CloudPoint> &cloudPoint = m_map->getPointCloud();
	const std::vector<Keypoint> &refKeypoints = m_referenceKeyframe->getKeypoints();
	const std::map<unsigned int, unsigned int> &refMapVisibility = m_referenceKeyframe->getVisibleMapPoints();
	std::vector<CloudPoint> cpRef;
	std::vector<Point2Df> projected2DPts, ref2DPts;

	for (auto it = refMapVisibility.begin(); it != refMapVisibility.end(); it++) {
		cpRef.push_back(cloudPoint[it->second]);
		ref2DPts.push_back(Point2Df(refKeypoints[it->first].getX(), refKeypoints[it->first].getY()));
	}
	m_projector->project(cpRef, projected2DPts, newFrame->getPose());

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
}
SRef<Keyframe> PipelineSlam::processNewKeyframe(SRef<Frame> newFrame)
{
	// create a new keyframe from the current frame
	SRef<Keyframe> newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);
	// Add to BOW retrieval			
	m_kfRetriever->addKeyframe(newKeyframe);
	// Update keypoint visibility, descriptor in cloud point and connections between new keyframe with other keyframes
	updateAssociateCloudPoint(newKeyframe);
	// get best neighbor keyframes
	std::vector<unsigned int> idxBestNeighborKfs = newKeyframe->getBestNeighborKeyframes(4);
	// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
	std::vector<std::tuple<unsigned int, int, unsigned int>> infoMatches; // first: index of kp in newKf, second: index of Kf, third: index of kp in Kf.
	std::vector<CloudPoint> newCloudPoint;
	findMatchesAndTriangulation(newKeyframe, idxBestNeighborKfs, infoMatches, newCloudPoint);
	if (newCloudPoint.size() > 0) {
		// fuse duplicate points
		std::vector<unsigned int> idxNeigborKfs = newKeyframe->getBestNeighborKeyframes(10);
		fuseCloudPoint(newKeyframe, idxNeigborKfs, infoMatches, newCloudPoint);
	}
	LOG_INFO("Keyframe: {} -> Number of new 3D points: {}", newKeyframe->m_idx, newCloudPoint.size());
	// mapper update
	m_mapper->update(m_map, newKeyframe, newCloudPoint, infoMatches);
	return newKeyframe;
}

void PipelineSlam::updateAssociateCloudPoint(SRef<Keyframe>& newKf)
{
	std::map<unsigned int, unsigned int> newkf_mapVisibility = newKf->getVisibleMapPoints();
	std::map<unsigned int, int> kfCounter;
	for (auto it = newkf_mapVisibility.begin(); it != newkf_mapVisibility.end(); it++) {
		CloudPoint &cp = m_map->getAPoint(it->second);
		// calculate the number of connections to other keyframes
		std::map<unsigned int, unsigned int> cpKfVisibility = cp.getVisibility();
		for (auto it_kf = cpKfVisibility.begin(); it_kf != cpKfVisibility.end(); it_kf++)
			kfCounter[it_kf->first]++;
		///// update descriptor of cp: des_cp = ((des_cp * cp.getVisibility().size()) + des_buf) / (cp.getVisibility().size() + 1)
		//// TO DO
		cp.visibilityAddKeypoint(newKf->m_idx, it->first);
	}

	for (auto it = kfCounter.begin(); it != kfCounter.end(); it++)
		if ((it->first != newKf->m_idx) && (it->second > 20)) {
			newKf->addNeighborKeyframe(it->first, it->second);
		}
}

void PipelineSlam::findMatchesAndTriangulation(SRef<Keyframe>& newKf, std::vector<unsigned int>& idxBestNeighborKfs, std::vector<std::tuple<unsigned int, int, unsigned int>>& infoMatches, std::vector<CloudPoint>& cloudPoint)
{
	const std::map<unsigned int, unsigned int> &newKf_mapVisibility = newKf->getVisibleMapPoints();
	const SRef<DescriptorBuffer> &newKf_des = newKf->getDescriptors();
	const std::vector<Keypoint> & newKf_kp = newKf->getKeypoints();
	Transform3Df newKf_pose = newKf->getPose();

	std::vector<bool> checkMatches(newKf_kp.size(), false);

	for (int i = 0; i < newKf_kp.size(); ++i)
		if (newKf_mapVisibility.find(i) != newKf_mapVisibility.end()) {
			checkMatches[i] = true;
		}

	for (int i = 0; i < idxBestNeighborKfs.size(); ++i) {
		std::vector<int> newKf_indexKeypoints;
		for (int j = 0; j < checkMatches.size(); ++j)
			if (!checkMatches[j])
				newKf_indexKeypoints.push_back(j);

		// get neighbor keyframe i
		SRef<Keyframe> &tmpKf = m_mapper->getKeyframe(idxBestNeighborKfs[i]);
		Transform3Df tmpPose = tmpKf->getPose();

		// check distance between two keyframes
		float distPose = std::sqrtf(std::powf(newKf_pose(0, 3) - tmpPose(0, 3), 2.f) + std::powf(newKf_pose(1, 3) - tmpPose(1, 3), 2.f) +
			std::powf(newKf_pose(2, 3) - tmpPose(2, 3), 2.f));
		if (distPose < 0.05)
			continue;

		// Matching based on BoW
		std::vector < DescriptorMatch> tmpMatches, goodMatches;
		m_kfRetriever->match(newKf_indexKeypoints, newKf_des, idxBestNeighborKfs[i], tmpMatches);

		// matches filter based epipolar lines
		m_matchesFilter->filter(tmpMatches, tmpMatches, newKf_kp, tmpKf->getKeypoints(), newKf->getPose(), tmpKf->getPose(), m_camera->getIntrinsicsParameters());

		// find info to triangulate				
		std::vector<std::tuple<unsigned int, int, unsigned int>> tmpInfoMatches;
		const std::map<unsigned int, unsigned int> & tmpMapVisibility = tmpKf->getVisibleMapPoints();
		for (int j = 0; j < tmpMatches.size(); ++j) {
			unsigned int idx_newKf = tmpMatches[j].getIndexInDescriptorA();
			unsigned int idx_tmpKf = tmpMatches[j].getIndexInDescriptorB();
			if ((!checkMatches[idx_newKf]) && (tmpMapVisibility.find(idx_tmpKf) == tmpMapVisibility.end())) {
				tmpInfoMatches.push_back(std::make_tuple(idx_newKf, idxBestNeighborKfs[i], idx_tmpKf));
				goodMatches.push_back(tmpMatches[j]);
			}
		}

		// triangulation
		std::vector<CloudPoint> tmpCloudPoint, tmpFilteredCloudPoint;
		std::vector<int> indexFiltered;
		if (goodMatches.size() > 0)
			m_triangulator->triangulate(newKf_kp, tmpKf->getKeypoints(), newKf_des, tmpKf->getDescriptors(), goodMatches,
				std::make_pair(newKf->m_idx, idxBestNeighborKfs[i]), newKf->getPose(), tmpKf->getPose(), tmpCloudPoint);

		// filter cloud points
		if (tmpCloudPoint.size() > 0)
			m_mapFilter->filter(newKf->getPose(), tmpKf->getPose(), tmpCloudPoint, tmpFilteredCloudPoint, indexFiltered);
		for (int i = 0; i < indexFiltered.size(); ++i) {
			checkMatches[std::get<0>(tmpInfoMatches[indexFiltered[i]])] = true;
			infoMatches.push_back(tmpInfoMatches[indexFiltered[i]]);
			cloudPoint.push_back(tmpFilteredCloudPoint[i]);
		}
	}
}

void PipelineSlam::fuseCloudPoint(SRef<Keyframe>& newKeyframe, std::vector<unsigned int>& idxNeigborKfs, std::vector<std::tuple<unsigned int, int, unsigned int>>& infoMatches, std::vector<CloudPoint>& newCloudPoint)
{
	std::vector<bool> checkMatches(newCloudPoint.size(), true);
	std::vector<SRef<DescriptorBuffer>> desNewCloudPoint;
	for (auto &it_cp : newCloudPoint) {
		desNewCloudPoint.push_back(it_cp.getDescriptor());
	}

	for (int i = 0; i < idxNeigborKfs.size(); ++i) {
		// get a neighbor
		SRef<Keyframe> &neighborKf = m_mapper->getKeyframe(idxNeigborKfs[i]);
		const std::map<unsigned int, unsigned int> mapVisibilitiesNeighbor = neighborKf->getVisibleMapPoints();

		//  projection points
		std::vector< Point2Df > projected2DPts;
		m_projector->project(newCloudPoint, projected2DPts, neighborKf->getPose());

		std::vector<DescriptorMatch> allMatches;
		m_matcher->matchInRegion(projected2DPts, desNewCloudPoint, neighborKf, allMatches, 5.f);

		for (int j = 0; j < allMatches.size(); ++j) {
			int idxNewCloudPoint = allMatches[j].getIndexInDescriptorA();
			int idxKpNeighbor = allMatches[j].getIndexInDescriptorB();
			if (!checkMatches[idxNewCloudPoint])
				continue;
			std::tuple<unsigned int, int, unsigned int> infoMatch = infoMatches[idxNewCloudPoint];

			// check this cloud point is created from the same neighbor keyframe
			if (std::get<1>(infoMatch) == idxNeigborKfs[i])
				continue;

			// check if have a cloud point in the neighbor keyframe is coincide with this cloud point.
			auto it_cp = mapVisibilitiesNeighbor.find(idxKpNeighbor);
			if (it_cp != mapVisibilitiesNeighbor.end()) {
				// fuse
				CloudPoint &old_cp = m_map->getAPoint(it_cp->second);
				old_cp.visibilityAddKeypoint(newKeyframe->m_idx, std::get<0>(infoMatch));
				old_cp.visibilityAddKeypoint(std::get<1>(infoMatch), std::get<2>(infoMatch));

				newKeyframe->addVisibleMapPoint(std::get<0>(infoMatch), it_cp->second);
				m_mapper->getKeyframe(std::get<1>(infoMatch))->addVisibleMapPoint(std::get<2>(infoMatch), it_cp->second);

				checkMatches[idxNewCloudPoint] = false;
			}
		}
	}
	std::vector<std::tuple<unsigned int, int, unsigned int>> tmpInfoMatches;
	std::vector<CloudPoint> tmpNewCloudPoint;
	for (int i = 0; i < checkMatches.size(); ++i)
		if (checkMatches[i]) {
			tmpInfoMatches.push_back(infoMatches[i]);
			tmpNewCloudPoint.push_back(newCloudPoint[i]);
		}
	tmpInfoMatches.swap(infoMatches);
	tmpNewCloudPoint.swap(newCloudPoint);
}

FrameworkReturnCode PipelineSlam::start(void* imageDataBuffer)
{
    if (m_initOK==false)
    {
        LOG_WARNING("Try to start the Fiducial marker pipeline without initializing it");
        return FrameworkReturnCode::_ERROR_;
    }
    m_stopFlag=false;

    m_sink->setImageBuffer((unsigned char*)imageDataBuffer);

    if (m_camera->start() != FrameworkReturnCode::_SUCCESS)
    {
        LOG_ERROR("Camera cannot start")
        return FrameworkReturnCode::_ERROR_;
    }

    // create and start threads
    auto getCameraImagesThread = [this](){;getCameraImages();};
    auto detectFirstKeyframeThread = [this](){; detectFirstKeyframe();};
    auto doBootStrapThread = [this](){;doBootStrap();};
    auto getKeyPointsThread = [this](){;getKeyPoints();};
    auto getDescriptorsThread = [this](){;getDescriptors();};
    auto getTrackingThread = [this](){;tracking();};
    auto getMappingThread = [this](){;mapping();};

    m_taskGetCameraImages = new xpcf::DelegateTask(getCameraImagesThread);
    m_taskDetectFirstKeyframe = new xpcf::DelegateTask(detectFirstKeyframeThread);
    m_taskDoBootStrap = new xpcf::DelegateTask(doBootStrapThread);
    m_taskGetKeyPoints = new xpcf::DelegateTask(getKeyPointsThread);
    m_taskGetDescriptors = new xpcf::DelegateTask(getDescriptorsThread);
    m_taskTracking = new xpcf::DelegateTask(getTrackingThread);
    m_taskMapping = new xpcf::DelegateTask(getMappingThread);

    m_taskGetCameraImages->start();
	m_taskDetectFirstKeyframe->start();
    m_taskDoBootStrap ->start();
    m_taskGetKeyPoints->start();
    m_taskGetDescriptors->start();
	m_taskTracking->start();
	m_taskMapping->start();

    LOG_INFO("Threads have started");
    m_startedOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode PipelineSlam::stop()
{
    m_stopFlag=true;
    m_camera->stop();

    if (m_taskGetCameraImages != nullptr)
        m_taskGetCameraImages->stop();
    if (m_taskDetectFirstKeyframe != nullptr)
		m_taskDetectFirstKeyframe->stop();
    if (m_taskDoBootStrap != nullptr)
        m_taskDoBootStrap->stop();
    if (m_taskGetKeyPoints != nullptr)
        m_taskGetKeyPoints->stop();
    if (m_taskGetDescriptors != nullptr)
        m_taskGetDescriptors->stop();
    if (m_taskTracking != nullptr)
		m_taskTracking->stop();
    if (m_taskMapping != nullptr)
		m_taskMapping->stop();

     if(!m_initOK)
     {
         LOG_WARNING("Try to stop a pipeline that has not been initialized");
         return FrameworkReturnCode::_ERROR_;
     }
     if (!m_startedOK)
     {
         LOG_WARNING("Try to stop a pipeline that has not been started");
         return FrameworkReturnCode::_ERROR_;
     }
     LOG_INFO("Pipeline has stopped: \n");

    return FrameworkReturnCode::_SUCCESS;
}

SourceReturnCode PipelineSlam::loadSourceImage(void* sourceTextureHandle, int width, int height)
{
    return SourceReturnCode::_NOT_IMPLEMENTED;
}

SinkReturnCode PipelineSlam::update(Transform3Df& pose)
{
    if(m_stopFlag)
        return SinkReturnCode::_ERROR;
    else
        return m_sink->tryGet(pose);
}

CameraParameters PipelineSlam::getCameraParameters()
{
    CameraParameters camParam;
    if (m_camera)
    {
        camParam = m_camera->getParameters();
    }
    return camParam;
}

void PipelineSlam::getCameraImages() {

	SRef<Image> view;
	if (m_stopFlag || !m_initOK || !m_startedOK)
		return;
	if (m_camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
		m_stopFlag = true;
		return;
	}
	m_CameraImagesBuffer.push(view);

#ifdef USE_IMAGES_SET
	std::this_thread::sleep_for(std::chrono::milliseconds(40));
#endif
};

void PipelineSlam::detectFirstKeyframe()
{
	if (m_stopFlag || !m_initOK || !m_startedOK || m_firstKeyframeCaptured || !m_CameraImagesBuffer.tryPop(m_camImage)) {
		xpcf::DelegateTask::yield();
		return;
	}
	
	if (detectFiducialMarker(m_camImage, m_poseKeyframe1)){
		m_keypointsDetector->detect(m_camImage, m_keypointsView1);
		m_descriptorExtractor->extract(m_camImage, m_keypointsView1, m_descriptorsView1);
		m_keyframe1 = xpcf::utils::make_shared<Keyframe>(m_keypointsView1, m_descriptorsView1, m_camImage, m_poseKeyframe1);
		m_mapper->update(m_map, m_keyframe1, {}, {}, {});
		m_keyframePoses.push_back(m_poseKeyframe1); // used for display
		m_kfRetriever->addKeyframe(m_keyframe1); // add keyframe for reloc
		m_firstKeyframeCaptured = true;
		LOG_INFO("Pose of keyframe 1: \n {}", m_poseKeyframe1.matrix());
		m_sink->set(m_poseKeyframe1, m_camImage);
	}
	else
		m_sink->set(m_camImage);
}

void PipelineSlam::doBootStrap()
{
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_firstKeyframeCaptured || m_bootstrapOk || !m_CameraImagesBuffer.tryPop(m_camImage)) {
		xpcf::DelegateTask::yield();
		return;
	}
	
	if (detectFiducialMarker(m_camImage, m_poseKeyframe2)) {
		float disTwoKeyframes = std::sqrtf(std::powf(m_poseKeyframe1(0, 3) - m_poseKeyframe2(0, 3), 2.f) + std::powf(m_poseKeyframe1(1, 3) - m_poseKeyframe2(1, 3), 2.f) +
			std::powf(m_poseKeyframe1(2, 3) - m_poseKeyframe2(2, 3), 2.f));

		if (disTwoKeyframes > 0.1) {
			m_keypointsDetector->detect(m_camImage, m_keypointsView2);
			m_descriptorExtractor->extract(m_camImage, m_keypointsView2, m_descriptorsView2);
			SRef<Frame> frame2 = xpcf::utils::make_shared<Frame>(m_keypointsView2, m_descriptorsView2, m_camImage, m_keyframe1);
			m_matcher->match(m_descriptorsView1, m_descriptorsView2, m_matches);
			int nbOriginalMatches = m_matches.size();
			m_matchesFilter->filter(m_matches, m_matches, m_keypointsView1, m_keypointsView2);

			if (m_keyframeSelector->select(frame2, m_matches))
			{
				LOG_INFO("Pose of keyframe 2: \n {}", m_poseKeyframe2.matrix());
				frame2->setPose(m_poseKeyframe2);
				LOG_INFO("Nb matches for triangulation: {}\\{}", m_matches.size(), nbOriginalMatches);
				// Triangulate
				m_keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
				m_triangulator->triangulate(m_keyframe2, m_matches, m_cloud);
				//double reproj_error = triangulator->triangulate(keypointsView1, keypointsView2, matches, std::make_pair(0, 1), poseFrame1, poseFrame2, cloud);
				m_mapFilter->filter(m_poseKeyframe1, m_poseKeyframe2, m_cloud, m_filteredCloud);
				m_keyframePoses.push_back(m_poseKeyframe2); // used for display
				m_mapper->update(m_map, m_keyframe2, m_filteredCloud, m_matches, {});
				m_kfRetriever->addKeyframe(m_keyframe2); // add keyframe for reloc
				m_bootstrapOk = true;
				m_lastPose = m_poseKeyframe2;
				updateReferenceKeyframe(m_keyframe2);
				updateData(m_keyframe2);
				LOG_INFO("Number of initial point cloud: {}", m_filteredCloud.size());
			}
		}
		m_sink->set(m_poseKeyframe2, m_camImage);
	}
	else
		m_sink->set(m_camImage);
}

void PipelineSlam::getKeyPoints() {
	SRef<Image>  camImage;

	if (m_stopFlag || !m_initOK || !m_startedOK || !m_bootstrapOk || !m_CameraImagesBuffer.tryPop(camImage)) {
		xpcf::DelegateTask::yield();
		return;
	}

	m_keypointsDetector->detect(camImage, m_keypoints);
	m_keypointsBuffer.push(std::make_pair(camImage, m_keypoints));
};

void PipelineSlam::getDescriptors()
{
	std::pair<SRef<Image>, std::vector<Keypoint>> frameKeypoints;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_keypointsBuffer.tryPop(frameKeypoints)) {
		xpcf::DelegateTask::yield();
		return;
	}

	m_descriptorExtractor->extract(frameKeypoints.first, frameKeypoints.second, m_descriptors);
	SRef<Frame> frame = xpcf::utils::make_shared<Frame>(frameKeypoints.second, m_descriptors, frameKeypoints.first);
	m_descriptorsBuffer.push(frame);
};

void PipelineSlam::tracking()
{
	SRef<Frame> newFrame;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_descriptorsBuffer.tryPop(newFrame)) {
		xpcf::DelegateTask::yield();
		return;
	}

	LOG_DEBUG("Number of keypoints: {}", newFrame->getKeypoints().size());
	SRef<Keyframe> newKeyframe;
	if (m_newKeyframeBuffer.tryPop(newKeyframe))
	{
		updateData(newKeyframe);
	}
	newFrame->setReferenceKeyframe(m_referenceKeyframe);
	m_matcher->match(m_frameToTrack->getDescriptors(), newFrame->getDescriptors(), m_matches);
	m_matchesFilter->filter(m_matches, m_matches, m_frameToTrack->getKeypoints(), newFrame->getKeypoints());

	std::vector<Point2Df> pt2d;
	std::vector<Point3Df> pt3d;
	std::vector<CloudPoint> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	m_corr2D3DFinder->find(m_frameToTrack, newFrame, m_matches, m_map, pt3d, pt2d, foundMatches, remainingMatches);
	// display matches
	SRef<Image> imageMatches = newFrame->getView()->copy();

	std::vector<Point2Df> imagePoints_inliers;
	std::vector<Point3Df> worldPoints_inliers;
	if (m_pnpRansac->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, m_pose, m_lastPose) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG(" pnp inliers size: {} / {}", worldPoints_inliers.size(), pt3d.size());
		// Set the pose of the new frame
		newFrame->setPose(m_pose);

		// refine pose and update map visibility of frame
		{
			// get all keypoints of the new frame
			const std::vector<Keypoint> &keypoints = newFrame->getKeypoints();

			//  projection points
			std::vector< Point2Df > projected2DPts;
			m_projector->project(m_localMap, projected2DPts, newFrame->getPose());

			std::vector<SRef<DescriptorBuffer>> desAllLocalMap;
			for (auto &it_cp : m_localMap) {
				desAllLocalMap.push_back(it_cp.getDescriptor());
			}
			std::vector<DescriptorMatch> allMatches;
			m_matcher->matchInRegion(projected2DPts, desAllLocalMap, newFrame, allMatches, 5.f);

			std::vector<Point2Df> pt2d;
			std::vector<Point3Df> pt3d;
			std::map<unsigned int, unsigned int> newMapVisibility;
			for (auto &it_match : allMatches) {
				int idx_2d = it_match.getIndexInDescriptorB();
				int idx_3d = it_match.getIndexInDescriptorA();
				pt2d.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
				pt3d.push_back(Point3Df(m_localMap[idx_3d].getX(), m_localMap[idx_3d].getY(), m_localMap[idx_3d].getZ()));
				newMapVisibility[idx_2d] = m_idxLocalMap[idx_3d];
			}

			// pnp optimization
			Transform3Df refinedPose;
			m_pnp->estimate(pt2d, pt3d, refinedPose, newFrame->getPose());
			newFrame->setPose(refinedPose);

			// update map visibility of current frame
			newFrame->addVisibleMapPoints(newMapVisibility);
			LOG_DEBUG("Number of map visibility of frame to track: {}", newMapVisibility.size());
			m_i2DOverlay->drawCircles(pt2d, imageMatches);
			//overlay3D->draw(refinedPose, imageMatches);
		}
		LOG_DEBUG("Refined pose: \n {}", newFrame->getPose().matrix());
		m_lastPose = newFrame->getPose();

		m_sink->set(m_lastPose, imageMatches);

		// update frame to track
		m_frameToTrack = newFrame;

		// add new frame to check need new keyframe
		m_addKeyframeBuffer.push(newFrame);

		m_framePoses.push_back(newFrame->getPose()); // used for display

		m_isLostTrack = false;	// tracking is good

	}
	else {
		LOG_DEBUG("Pose estimation has failed");
		m_isLostTrack = true;		// lost tracking

		// reloc
		std::vector < SRef <Keyframe>> ret_keyframes;
		if (m_kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Tracking: lost need update keyframe id {}", ret_keyframes[0]->m_idx);
			updateReferenceKeyframe(ret_keyframes[0]);
			updateData(ret_keyframes[0]);
			m_lastPose = m_referenceKeyframe->getPose();
			LOG_DEBUG("Retrieval Success");
		}
		else
			LOG_DEBUG("Retrieval Failed");

		m_sink->set(imageMatches);
	}

	LOG_DEBUG("Nb of Local Map / World Map: {} / {}", m_localMap.size(), m_map->getPointCloud().size());
	LOG_DEBUG("Index of current reference keyframe: {} / {}", m_referenceKeyframe->m_idx, m_mapper->getKeyframes().size());
}

void PipelineSlam::mapping()
{
	SRef<Frame> newFrame;
	if (m_stopFlag || !m_initOK || !m_startedOK || !m_addKeyframeBuffer.tryPop(newFrame)) {
		xpcf::DelegateTask::yield();
		return;
	}

	std::function<bool(const SRef<Frame>&)> _fnCheckDisparityDistance = [this](const SRef<Frame> &frame) ->bool { return checkDisparityDistance(frame); };
	std::function<bool(const SRef<Frame>&)> _fnCheckNeedNewKfWithAllKfs = [this](const SRef<Frame> &frame) ->bool { return checkNeedNewKfWithAllKfs(frame); };
	// check need new keyframe
	if (m_keyframeSelector->select(newFrame, _fnCheckDisparityDistance))
	{
		if (m_keyframeSelector->select(newFrame, _fnCheckNeedNewKfWithAllKfs)) {
			updateReferenceKeyframe(m_updatedRefKf);
			m_newKeyframeBuffer.push(m_updatedRefKf);
			LOG_INFO("Update new reference keyframe id: {} \n", m_updatedRefKf->m_idx);
		}
		else {
			SRef<Keyframe> newKeyframe = processNewKeyframe(newFrame);
			updateReferenceKeyframe(newKeyframe);
			m_keyframePoses.push_back(newKeyframe->getPose());
			LOG_INFO("Number of keyframe: {} -> cloud current size: {} \n", m_mapper->getKeyframes().size(), m_map->getPointCloud().size());
			m_newKeyframeBuffer.push(newKeyframe);
		}
	}		
}

}//namespace PIPELINES
}//namespace SolAR