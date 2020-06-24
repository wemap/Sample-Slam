#include "xpcf/module/ModuleFactory.h"
#include "pipelineSlam.h"
#include "core/Log.h"
#include "boost/log/core/core.hpp"
#include <cmath>

#define MIN_THRESHOLD -1
#define MAX_THRESHOLD 220
#define NB_THRESHOLD 3
#define NB_POINTCLOUD_INIT 50
#define MIN_WEIGHT_NEIGHBOR_KEYFRAME 50
#define MIN_POINT_DISTANCE 0.04
#define NB_NEWKEYFRAMES_BA 10

// The pipeline component for the fiducial marker

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::PIPELINES::PipelineSlam)

namespace SolAR {
using namespace datastructure;
using namespace api::pipeline;
namespace PIPELINES {

inline static float centerCamDistance(const Transform3Df & pose1, const Transform3Df & pose2) {
	return std::sqrt(std::pow(pose1(0, 3) - pose2(0, 3), 2.f) + std::pow(pose1(1, 3) - pose2(1, 3), 2.f) +
		std::pow(pose1(2, 3) - pose2(2, 3), 2.f));
}

inline static float angleCamDistance(const Transform3Df & pose1, const Transform3Df & pose2) {
	return std::acos(pose1(0, 2) * pose2(0, 2) + pose1(1, 2) * pose2(1, 2) + pose1(2, 2) * pose2(2, 2));
}

PipelineSlam::PipelineSlam():ConfigurableBase(xpcf::toUUID<PipelineSlam>())
{
    declareInterface<api::pipeline::IPipeline>(this);
#ifdef USE_IMAGES_SET
    declareInjectable<input::devices::ICamera>(m_camera, "ImagesAsCamera");
#else
    declareInjectable<input::devices::ICamera>(m_camera);
#endif
	declareInjectable<IPointCloudManager>(m_pointCloudManager);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
	declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
	declareInjectable<reloc::IKeyframeRetriever>(m_kfRetriever);
	declareInjectable<solver::map::IMapper>(m_mapper);

    declareInjectable<features::IKeypointDetector>(m_keypointsDetector);
    declareInjectable<features::IDescriptorsExtractor>(m_descriptorExtractor);
    declareInjectable<features::IDescriptorMatcher>(m_matcher);
    declareInjectable<solver::pose::I3DTransformFinderFrom2D2D>(m_poseFinderFrom2D2D);
    declareInjectable<solver::map::ITriangulator>(m_triangulator);
    declareInjectable<features::IMatchesFilter>(m_matchesFilter);
    declareInjectable<solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
    declareInjectable<solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
    declareInjectable<solver::map::IMapFilter>(m_mapFilter);    
    declareInjectable<solver::map::IKeyframeSelector>(m_keyframeSelector);    
    declareInjectable<geom::IProject>(m_projector);
    declareInjectable<sink::ISinkPoseImage>(m_sink);
    declareInjectable<source::ISourceImage>(m_source);
    declareInjectable<solver::map::IBundler>(m_bundler);
    // marker fiducial
    declareInjectable<input::files::IMarker2DSquaredBinary>(m_binaryMarker);
    declareInjectable<image::IImageFilter>(m_imageFilterBinary);
    declareInjectable<image::IImageConvertor>(m_imageConvertor);
    declareInjectable<image::IImageConvertor>(m_imageConvertorUnity, "ImageConvertorUnity");
    declareInjectable<features::IContoursExtractor>(m_contoursExtractor);
    declareInjectable<features::IContoursFilter>(m_contoursFilter);
    declareInjectable<image::IPerspectiveController>(m_perspectiveController);
    declareInjectable<features::IDescriptorsExtractorSBPattern>(m_patternDescriptorExtractor);
    declareInjectable<features::IDescriptorMatcher>(m_patternMatcher, "DescMatcherFiducial");
    declareInjectable<features::ISBPatternReIndexer>(m_patternReIndexer);
    declareInjectable<geom::IImage2WorldMapper>(m_img2worldMapper);
    declareInjectable<display::I2DOverlay>(m_i2DOverlay);

    m_bootstrapOk=false;
    m_firstKeyframeCaptured = false;
    m_isLostTrack = false;
	m_stopFlag = false;
	m_startedOK = false;
	m_startCaptureFirstKeyframe = false;

    LOG_DEBUG(" Pipeline constructor");	
}


PipelineSlam::~PipelineSlam()
{
     LOG_DEBUG(" Pipeline destructor")
}

FrameworkReturnCode PipelineSlam::init(SRef<xpcf::IComponentManager> xpcfComponentManager)
{    
    // component creation
    try {
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
        m_calibration = m_camera->getIntrinsicsParameters();
        m_distortion = m_camera->getDistortionParameters();
        m_pnp->setCameraParameters(m_calibration, m_distortion);
        m_pnpRansac->setCameraParameters(m_calibration, m_distortion);
        m_poseFinderFrom2D2D->setCameraParameters(m_calibration, m_distortion);
        m_triangulator->setCameraParameters(m_calibration, m_distortion);
        m_projector->setCameraParameters(m_calibration, m_distortion);

        m_initOK = true;	
		m_haveToBeFlip = false;
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

void PipelineSlam::updateData(const SRef<Keyframe> &refKf)
{	
	m_frameToTrack = xpcf::utils::make_shared<Frame>(m_referenceKeyframe);
	m_frameToTrack->setReferenceKeyframe(m_referenceKeyframe);
	m_localMap.clear();
	// get local point cloud
	m_mapper->getLocalPointCloud(refKf, MIN_WEIGHT_NEIGHBOR_KEYFRAME, m_localMap);
}

void PipelineSlam::updateReferenceKeyframe(const SRef<Keyframe> &refKf)
{
	std::unique_lock<std::mutex> lock(globalVarsMutex);
	m_referenceKeyframe = refKf;
	LOG_DEBUG("Update new reference keyframe with id {}", m_referenceKeyframe->getId());
};

bool PipelineSlam::checkNeedNewKfWithAllKfs(const SRef<Frame>& newFrame)
{
	std::vector < uint32_t> ret_keyframesId;
	if (m_kfRetriever->retrieve(newFrame, ret_keyframesId) == FrameworkReturnCode::_SUCCESS) {
		if (ret_keyframesId[0] != m_referenceKeyframe->getId()) {
			m_keyframesManager->getKeyframe(ret_keyframesId[0], m_updatedRefKf);
			LOG_DEBUG("Find new reference keyframe with id {}", m_updatedRefKf->getId());
			return true;
		}
		LOG_DEBUG("Find same reference keyframe, need make new keyframe");
		return false;
	}
	else
		return false;
};

bool PipelineSlam::checkDisparityDistance(const SRef<Frame>& newFrame) {
	const std::vector<Keypoint> &refKeypoints = m_referenceKeyframe->getKeypoints();
	const std::map<uint32_t, uint32_t> &refMapVisibility = m_referenceKeyframe->getVisibility();
	std::vector<SRef<CloudPoint>> cpRef;
	std::vector<Point2Df> projected2DPts, ref2DPts;

	for (auto const & it : refMapVisibility) {
		SRef<CloudPoint> cloudPoint;
		if (m_pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
			cpRef.push_back(cloudPoint);
			ref2DPts.push_back(Point2Df(refKeypoints[it.first].getX(), refKeypoints[it.first].getY()));
		}
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
SRef<Keyframe> PipelineSlam::processNewKeyframe(SRef<Frame> &newFrame)
{
	// create a new keyframe from the current frame
	SRef<Keyframe> newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);
	// Add to keyframe manager
	m_keyframesManager->addKeyframe(newKeyframe);
	// Add to BOW retrieval			
	m_kfRetriever->addKeyframe(newKeyframe);
	// Update keypoint visibility, descriptor in cloud point and connections between new keyframe with other keyframes
	updateAssociateCloudPoint(newKeyframe);
	// get best neighbor keyframes
	std::vector<uint32_t> idxBestNeighborKfs;
	m_covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME, idxBestNeighborKfs);
	// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
	std::vector<SRef<CloudPoint>> newCloudPoint;
	findMatchesAndTriangulation(newKeyframe, idxBestNeighborKfs, newCloudPoint);
	if (newCloudPoint.size() > 0) {
		// fuse duplicate points
		std::vector<uint32_t> idxNeigborKfs;
		m_covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME - 10, idxNeigborKfs);
		fuseCloudPoint(newKeyframe, idxNeigborKfs, newCloudPoint);
	}
	LOG_DEBUG("Keyframe: {} -> Number of new 3D points: {}", newKeyframe->getId(), newCloudPoint.size());
	// add new points to point cloud manager, update visibility map and covisibility graph
	for (auto const &point : newCloudPoint)
		m_mapper->addCloudPoint(point);
	return newKeyframe;
}

void PipelineSlam::updateAssociateCloudPoint(SRef<Keyframe>& newKf)
{
	const std::map<uint32_t, uint32_t> &newkf_mapVisibility = newKf->getVisibility();
	std::map<uint32_t, int> kfCounter;
	// calculate the number of connections to other keyframes
	for (auto const &it : newkf_mapVisibility) {
		SRef<CloudPoint> cloudPoint;
		if (m_pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
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
			m_covisibilityGraph->increaseEdge(newKf->getId(), it.first, it.second);
}

void PipelineSlam::findMatchesAndTriangulation(const SRef<Keyframe> & newKf, const std::vector<uint32_t> &idxBestNeighborKfs, std::vector<SRef<CloudPoint>> &cloudPoint)
{
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
		m_keyframesManager->getKeyframe(idxBestNeighborKfs[i], tmpKf);
		const Transform3Df &tmpKf_pose = tmpKf->getPose();

		// check distance between two keyframes
		if ((centerCamDistance(newKf_pose, tmpKf_pose) < 0.05) || (angleCamDistance(newKf_pose, tmpKf_pose) > 0.5))
			continue;

		// Matching based on BoW
		std::vector < DescriptorMatch> tmpMatches, goodMatches;
		m_kfRetriever->match(newKf_indexKeypoints, newKf_des, tmpKf, tmpMatches);

		// matches filter based epipolar lines
		m_matchesFilter->filter(tmpMatches, tmpMatches, newKf_kp, tmpKf->getKeypoints(), newKf_pose, tmpKf_pose, m_camera->getIntrinsicsParameters());

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
			m_triangulator->triangulate(newKf_kp, tmpKf->getKeypoints(), newKf_des, tmpKf->getDescriptors(), goodMatches,
				std::make_pair(newKf->getId(), idxBestNeighborKfs[i]), newKf_pose, tmpKf_pose, tmpCloudPoint);

		// filter cloud points
		if (tmpCloudPoint.size() > 0)
			m_mapFilter->filter(newKf_pose, tmpKf_pose, tmpCloudPoint, tmpFilteredCloudPoint, indexFiltered);
		for (int i = 0; i < indexFiltered.size(); ++i) {
			checkMatches[goodMatches[indexFiltered[i]].getIndexInDescriptorA()] = true;
			cloudPoint.push_back(tmpFilteredCloudPoint[i]);
		}
	}
}

void PipelineSlam::fuseCloudPoint(const SRef<Keyframe> &newKeyframe, const std::vector<uint32_t> &idxNeigborKfs, std::vector<SRef<CloudPoint>> &newCloudPoint)
{
	std::vector<bool> checkMatches(newCloudPoint.size(), true);
	std::vector<SRef<DescriptorBuffer>> desNewCloudPoint;
	for (auto const &it_cp : newCloudPoint) {
		desNewCloudPoint.push_back(it_cp->getDescriptor());
	}

	for (int i = 0; i < idxNeigborKfs.size(); ++i) {
		// get a neighbor
		SRef<Keyframe> neighborKf;
		if (m_keyframesManager->getKeyframe(idxNeigborKfs[i], neighborKf) == FrameworkReturnCode::_ERROR_)
			continue;
		const std::map<uint32_t, uint32_t> &mapVisibilitiesNeighbor = neighborKf->getVisibility();

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

			const std::map<uint32_t, uint32_t> &newCloudPointVisibility = newCloudPoint[idxNewCloudPoint]->getVisibility();
			// check this cloud point is created from the same neighbor keyframe
			if (newCloudPointVisibility.find(idxNeigborKfs[i]) != newCloudPointVisibility.end())
				continue;

			// check if have a cloud point in the neighbor keyframe is coincide with this cloud point.
			auto it_cp = mapVisibilitiesNeighbor.find(idxKpNeighbor);
			if (it_cp != mapVisibilitiesNeighbor.end()) {
				SRef<CloudPoint> existedCloudPoint;
				if (m_pointCloudManager->getPoint(it_cp->second, existedCloudPoint) == FrameworkReturnCode::_SUCCESS) {
					if ((*existedCloudPoint - *newCloudPoint[idxNewCloudPoint]).magnitude() < MIN_POINT_DISTANCE) {
						// add visibilities to this cloud point and keyframes
						std::vector<uint32_t> keyframeIds;
						for (auto const &vNewCP : newCloudPointVisibility) {
							existedCloudPoint->addVisibility(vNewCP.first, vNewCP.second);
							keyframeIds.push_back(vNewCP.first);
							SRef<Keyframe> tmpKeyframe;
							m_keyframesManager->getKeyframe(vNewCP.first, tmpKeyframe);
							tmpKeyframe->addVisibility(vNewCP.second, it_cp->second);
							// modify cloud point descriptor
							existedCloudPoint->addNewDescriptor(tmpKeyframe->getDescriptors()->getDescriptor(vNewCP.second));
							// modify view direction
							Transform3Df poseTmpKf = tmpKeyframe->getPose();
							Vector3f newViewDirection(poseTmpKf(0, 3) - existedCloudPoint->getX(), poseTmpKf(1, 3) - existedCloudPoint->getY(), poseTmpKf(2, 3) - existedCloudPoint->getZ());
							existedCloudPoint->addNewViewDirection(newViewDirection);
						}
						// update covisibility graph
						m_covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[0], 1);
						m_covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[1], 1);
						m_covisibilityGraph->increaseEdge(keyframeIds[0], keyframeIds[1], 1);
						// this new cloud point is existed
						checkMatches[idxNewCloudPoint] = false;
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

	if (!m_haveToBeFlip) {
		if (m_camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start")
				return FrameworkReturnCode::_ERROR_;
		}
	}

	// Load map from file
	if (m_mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
		m_bootstrapOk = true;
		m_keyframesManager->getKeyframe(0, m_keyframe2);
		m_lastPose = m_keyframe2->getPose();
		updateReferenceKeyframe(m_keyframe2);
		updateData(m_keyframe2);
	}
	else
		LOG_INFO("Initialization from scratch");

    // create and start threads
    auto getCameraImagesThread = [this](){;getCameraImages();};
    auto doBootStrapThread = [this](){;doBootStrap();};
    auto getKeyPointsThread = [this](){;getKeyPoints();};
    auto getDescriptorsThread = [this](){;getDescriptors();};
    auto getTrackingThread = [this](){;tracking();};
    auto getMappingThread = [this](){;mapping();};

    m_taskGetCameraImages = new xpcf::DelegateTask(getCameraImagesThread);
    m_taskDoBootStrap = new xpcf::DelegateTask(doBootStrapThread);
    m_taskGetKeyPoints = new xpcf::DelegateTask(getKeyPointsThread);
    m_taskGetDescriptors = new xpcf::DelegateTask(getDescriptorsThread);
    m_taskTracking = new xpcf::DelegateTask(getTrackingThread);
    m_taskMapping = new xpcf::DelegateTask(getMappingThread);

    m_taskGetCameraImages->start();
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
	// Save map
	m_mapper->saveToFile();

    return FrameworkReturnCode::_SUCCESS;
}

SourceReturnCode PipelineSlam::loadSourceImage(void* sourceTextureHandle, int width, int height)
{
	m_haveToBeFlip = true;
	return m_source->setInputTexture((unsigned char *)sourceTextureHandle, width, height);
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
	if (m_haveToBeFlip){
		m_source->getNextImage(view);
		m_imageConvertorUnity->convert(view, view, Image::ImageLayout::LAYOUT_RGB);
	}
	else if (m_camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
		m_stopFlag = true;
		return;
	}
	m_CameraImagesBuffer.push(view);

#ifdef USE_IMAGES_SET
	std::this_thread::sleep_for(std::chrono::milliseconds(40));
#endif
};

void PipelineSlam::doBootStrap()
{
	if (m_stopFlag || !m_initOK || !m_startedOK || m_bootstrapOk || !m_CameraImagesBuffer.tryPop(m_camImage)) {
		xpcf::DelegateTask::yield();
		return;
	}

	bool bootstrapOk = false;
	bool initFrame1 = false;


	if (detectFiducialMarker(m_camImage, m_poseFrame)) {
		if (!m_firstKeyframeCaptured) {
			m_firstKeyframeCaptured = true;
			m_keypointsDetector->detect(m_camImage, m_keypoints);
			m_descriptorExtractor->extract(m_camImage, m_keypoints, m_descriptors);
			m_frame1 = xpcf::utils::make_shared<Frame>(m_keypoints, m_descriptors, m_camImage, m_poseFrame);
		}
		else {
			Transform3Df poseFrame1 = m_frame1->getPose();
			if (centerCamDistance(m_poseFrame, poseFrame1) > 0.1) {
				// feature extraction
				m_keypointsDetector->detect(m_camImage, m_keypoints);
				m_descriptorExtractor->extract(m_camImage, m_keypoints, m_descriptors);
				m_frame2 = xpcf::utils::make_shared<Frame>(m_keypoints, m_descriptors, m_camImage, m_poseFrame);
				// check angle camera distance
				if (angleCamDistance(m_poseFrame, poseFrame1) < 0.1) {
					// matching
					m_matcher->match(m_frame1->getDescriptors(), m_frame2->getDescriptors(), m_matches);
					m_matchesFilter->filter(m_matches, m_matches, m_frame1->getKeypoints(), m_frame2->getKeypoints());
					// Triangulate
					m_cloud.clear();
					m_filteredCloud.clear();
					m_triangulator->triangulate(m_frame1->getKeypoints(), m_frame2->getKeypoints(), m_frame1->getDescriptors(), m_frame2->getDescriptors(), m_matches,
						std::make_pair(0, 1), m_frame1->getPose(), m_frame2->getPose(), m_cloud);
					m_mapFilter->filter(m_frame1->getPose(), m_frame2->getPose(), m_cloud, m_filteredCloud);

					if (m_filteredCloud.size() > NB_POINTCLOUD_INIT) {
						// add keyframes to keyframes manager
						m_keyframe1 = xpcf::utils::make_shared<Keyframe>(m_frame1);
						m_keyframesManager->addKeyframe(m_keyframe1);
						m_keyframe2 = xpcf::utils::make_shared<Keyframe>(m_frame2);
						m_keyframesManager->addKeyframe(m_keyframe2);
						m_keyframe2->setReferenceKeyframe(m_keyframe1);						
						// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
						for (auto const &it : m_filteredCloud)
							m_mapper->addCloudPoint(it);
						// add keyframes to retriever
						m_kfRetriever->addKeyframe(m_keyframe1);
						m_kfRetriever->addKeyframe(m_keyframe2);
						// apply bundle adjustement 
                        m_bundleReprojError = m_bundler->bundleAdjustment(m_calibration, m_distortion, { 0,1 });
						m_bootstrapOk = true;
						m_lastPose = m_keyframe2->getPose();
						updateReferenceKeyframe(m_keyframe2);
						updateData(m_keyframe2);
						LOG_INFO("Number of initial point cloud: {}", m_filteredCloud.size());
					}
					else {
						m_frame1 = m_frame2;
					}
				}
				else {
					m_frame1 = m_frame2;
				}
			}			
		}
		m_sink->set(m_poseFrame, m_camImage);
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
	m_corr2D3DFinder->find(m_frameToTrack, newFrame, m_matches, pt3d, pt2d, foundMatches, remainingMatches);
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
				desAllLocalMap.push_back(it_cp->getDescriptor());
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
				pt3d.push_back(Point3Df(m_localMap[idx_3d]->getX(), m_localMap[idx_3d]->getY(), m_localMap[idx_3d]->getZ()));
				newMapVisibility[idx_2d] = m_localMap[idx_3d]->getId();
			}

			// pnp optimization
			Transform3Df refinedPose;
			m_pnp->estimate(pt2d, pt3d, refinedPose, newFrame->getPose());
			newFrame->setPose(refinedPose);

			// update map visibility of current frame
			newFrame->addVisibilities(newMapVisibility);
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

		m_isLostTrack = false;	// tracking is good

	}
	else {
		LOG_DEBUG("Pose estimation has failed");
		m_isLostTrack = true;		// lost tracking
		// reloc
		std::vector < uint32_t> retKeyframesId;
		if (m_kfRetriever->retrieve(newFrame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Tracking: lost need update keyframe id {}", retKeyframesId[0]);
			SRef<Keyframe> bestRetKeyframe;
			m_keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe);
			updateReferenceKeyframe(bestRetKeyframe);
			updateData(bestRetKeyframe);
			m_lastPose = m_referenceKeyframe->getPose();
			LOG_DEBUG("Retrieval Success");
		}
		else
			LOG_DEBUG("Retrieval Failed");

		m_sink->set(imageMatches);
	}

	LOG_DEBUG("Nb of Local Map / World Map: {} / {}", m_localMap.size(), m_pointCloudManager->getNbPoints());
	LOG_DEBUG("Index of current reference keyframe: {} / {}", m_referenceKeyframe->getId(), m_keyframesManager->getNbKeyframes());
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
			LOG_INFO("Update new reference keyframe id: {} \n", m_updatedRefKf->getId());
		}
		else {
			// create new keyframe
			SRef<Keyframe> newKeyframe = processNewKeyframe(newFrame);
			// Local bundle adjustment
			std::vector<uint32_t> bestIdx;
			m_covisibilityGraph->getNeighbors(newKeyframe->getId(), MIN_WEIGHT_NEIGHBOR_KEYFRAME, bestIdx);
			bestIdx.push_back(newKeyframe->getId());
            m_bundleReprojError = m_bundler->bundleAdjustment(m_calibration,m_distortion, bestIdx);
			// global bundle adjustment
			m_countNewKeyframes++;
			if (m_countNewKeyframes == NB_NEWKEYFRAMES_BA) {
				m_countNewKeyframes = 0;
				m_bundleReprojError = m_bundler->bundleAdjustment(m_calibration, m_distortion);
				LOG_INFO("Global bundle adjustment -> error: {}", m_bundleReprojError);
			}
			// Update new reference keyframe 
			updateReferenceKeyframe(newKeyframe);
			LOG_INFO("Number of keyframe: {} -> cloud current size: {} \n", m_keyframesManager->getNbKeyframes(), m_pointCloudManager->getNbPoints());
			m_newKeyframeBuffer.push(newKeyframe);
		}
	}		
}

}//namespace PIPELINES
}//namespace SolAR
