#ifndef PIPELINESLAM_H
#define PIPELINESLAM_H

#if _WIN32
#ifdef PipelineSlam_API_DLLEXPORT
#define SOLARPIPELINESLAM_EXPORT_API __declspec(dllexport)
#else //SOLARPIPELINEFIDUCIALMARKER_API_DLLEXPORT
#define SOLARPIPELINESLAM_EXPORT_API __declspec(dllimport)
#endif //SOLARPIPELINEFIDUCIALMARKER_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINESLAM_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IPipeline.h"

// Add the headers to datastructures and component interfaces used by the pipeline

#include "api/input/devices/ICamera.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/solver/map/IMapFilter.h"
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
#include "api/solver/map/IBundler.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "core/Log.h"

#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"

#ifdef USE_OPENGL
    #include "api/sink/ISinkPoseTextureBuffer.h"
#else
    #include "api/sink/ISinkPoseImage.h"
#endif
#include "api/source/ISourceImage.h"

#include "xpcf/threading/SharedBuffer.h"
#include "xpcf/threading/DropBuffer.h"
#include "xpcf/threading/BaseTask.h"

//#define USE_IMAGES_SET
//#define VIDEO_INPUT

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::sink;
using namespace api::pipeline;
namespace PIPELINES {

class SOLARPIPELINESLAM_EXPORT_API PipelineSlam : public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IPipeline
{
public:
    PipelineSlam();
    ~PipelineSlam();

    //// @brief Initialization of the pipeline
    /// Initialize the pipeline by providing a reference to the component manager loaded by the PipelineManager.
    /// @param[in] componentManager a shared reference to the component manager which has loaded the components and configuration in the pipleine manager
    FrameworkReturnCode init(SRef<xpcf::IComponentManager> xpcfComponentManager) override;

    /// @brief Provide the camera parameters
    /// @return the camera parameters (its resolution and its focal)
    CameraParameters getCameraParameters() override;

    /// @brief Starts the pipeline and provides a texture buffer which will be updated when required.
    /// @param[in] textureHandle a pointer to the texture buffer which will be updated at each call of the update method.
#ifdef USE_OPENGL
    FrameworkReturnCode start(void* textureHandle) override;
#else
    FrameworkReturnCode start(void* imageDataBuffer) override;
#endif

    /// @brief Stop the pipeline.
    FrameworkReturnCode stop() override;

    /// @brief update the pipeline
    /// Get the new pose and update the texture buffer with the image that has to be displayed
    SinkReturnCode update(Transform3Df& pose) override;

    SourceReturnCode loadSourceImage(void* sourceTextureHandle, int width, int height) override;

    void unloadComponent () override final;

private:	
	// Image capture task
	void getCameraImages();

	// Bootstrap task
	void doBootStrap();

	// Keypoint detection task
	void getKeyPoints();

	// Feature extraction task
	void getDescriptors();

	// tracking  task
	void tracking();

	// mapping task
	void mapping();	

	// global bundle adjustment task
	void globalBundleAdjustment();


	// detect fiducial marker
	bool detectFiducialMarker(SRef<Image>& image, Transform3Df &pose);

	// update data to track
	void updateData(const SRef<Keyframe> &refKf);

	// update reference keyframe
	void updateReferenceKeyframe(const SRef<Keyframe> &refKf);

	// check need new keyframe based on FBoW
	bool checkNeedNewKfWithAllKfs(const SRef<Frame>& newFrame);

	// check need new keyframe based on disparity distance
	bool checkDisparityDistance(const SRef<Frame>& newFrame);

	// process to add a new keyframe
	SRef<Keyframe> processNewKeyframe(SRef<Frame> &newFrame);

	// Update keypoint visibility, descriptor in cloud point
	void updateAssociateCloudPoint(SRef<Keyframe> & newKf);

	// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
	void findMatchesAndTriangulation(const SRef<Keyframe> & newKf, const std::vector<uint32_t> &idxBestNeighborKfs, std::vector<SRef<CloudPoint>> &cloudPoint);

	// check and fuse cloud point
	void fuseCloudPoint(const SRef<Keyframe> &newKeyframe, const std::vector<uint32_t> &idxNeigborKfs, std::vector<SRef<CloudPoint>> &newCloudPoint);

private:

	// State flag of the pipeline
	bool m_stopFlag, m_initOK, m_startedOK;

	// mutex
	std::mutex globalVarsMutex;

	// storage components
	SRef<IPointCloudManager>							m_pointCloudManager;
	SRef<IKeyframesManager>								m_keyframesManager;
	SRef<ICovisibilityGraph>							m_covisibilityGraph;
	SRef<reloc::IKeyframeRetriever>						m_kfRetriever;
	SRef<solver::map::IMapper>							m_mapper;

	// components
    SRef<input::devices::ICamera>						m_camera;
    SRef<input::files::IMarker2DSquaredBinary>			m_binaryMarker;
    SRef<DescriptorBuffer>								m_markerPatternDescriptor;
    SRef<features::IDescriptorsExtractorSBPattern>		m_patternDescriptorExtractor;
    SRef<image::IImageFilter>							m_imageFilterBinary;
    SRef<image::IImageConvertor>						m_imageConvertor;
    SRef<image::IImageConvertor>						m_imageConvertorUnity;
    SRef<features::IContoursExtractor>					m_contoursExtractor ;
    SRef<features::IContoursFilter>						m_contoursFilter;
    SRef<image::IPerspectiveController>					m_perspectiveController;
    SRef<features::IDescriptorMatcher>					m_patternMatcher;
    SRef<features::ISBPatternReIndexer>					m_patternReIndexer;
    SRef<geom::IImage2WorldMapper>						m_img2worldMapper;

    SRef<features::IKeypointDetector>					m_keypointsDetector;
    SRef<features::IDescriptorsExtractor>				m_descriptorExtractor;
    SRef<features::IDescriptorMatcher>					m_matcher;
    SRef<features::IMatchesFilter>						m_matchesFilter;
    SRef<solver::pose::I3DTransformFinderFrom2D2D>		m_poseFinderFrom2D2D;
	SRef<solver::pose::I3DTransformFinderFrom2D3D>		m_pnp;
	SRef<solver::pose::I3DTransformSACFinderFrom2D3D>	m_pnpRansac;
	SRef<solver::map::ITriangulator>					m_triangulator;    
    SRef<solver::pose::I2D3DCorrespondencesFinder>		m_corr2D3DFinder;
    SRef<geom::IProject>								m_projector;
    SRef<solver::map::IMapFilter>						m_mapFilter;    
    SRef<solver::map::IKeyframeSelector>				m_keyframeSelector;    
	SRef<solver::map::IBundler>							m_bundler;

    // display stuff
    SRef<api::display::I2DOverlay>						m_i2DOverlay;


#ifdef USE_OPENGL
    SRef<sink::ISinkPoseTextureBuffer>					m_sink;
#else
    SRef<sink::ISinkPoseImage>							m_sink;
#endif   
	SRef<source::ISourceImage>							m_source;

	// SLAM variables
	Transform3Df										m_pose;
	SRef<Image>											m_camImage;
    Transform3Df                                        m_poseFrame;
	SRef<Frame>											m_frame1, m_frame2;
    SRef<Keyframe>                                      m_keyframe1, m_keyframe2;
	bool												m_firstKeyframeCaptured = false;
	bool												m_bootstrapOk = false;
	bool												m_startCaptureFirstKeyframe = false;
	bool												m_haveToBeFlip;
	int													m_countNewKeyframes = 0;

    CamCalibration                                      m_calibration;
    CamDistortion                                       m_distortion;
	std::vector<Keypoint>								m_keypoints;
	SRef<DescriptorBuffer>								m_descriptors;
	std::vector<DescriptorMatch>                        m_matches;
	std::vector<SRef<CloudPoint>>						m_cloud, m_filteredCloud;
    SRef<Keyframe>                                      m_referenceKeyframe;
    SRef<Keyframe>                                      m_updatedRefKf;
    SRef<Frame>											m_frameToTrack;
    Transform3Df                                        m_lastPose;
    bool                                                m_isLostTrack;

	std::vector<SRef<CloudPoint>>						m_localMap;
	double												m_bundleReprojError;



	xpcf::DropBuffer< SRef<Image>>						m_CameraImagesBuffer;
	xpcf::DropBuffer< std::pair< SRef<Image>, std::vector<Keypoint> >> m_keypointsBuffer;
	xpcf::DropBuffer< SRef<Frame >>						m_descriptorsBuffer;
	xpcf::DropBuffer<SRef<Frame>>						m_addKeyframeBuffer;
	xpcf::DropBuffer<SRef<Keyframe>>					m_newKeyframeBuffer;

	// tasks
    xpcf::DelegateTask*									m_taskGetCameraImages;
    xpcf::DelegateTask*									m_taskDetectFirstKeyframe;
    xpcf::DelegateTask*									m_taskDoBootStrap;
    xpcf::DelegateTask*									m_taskGetKeyPoints;
    xpcf::DelegateTask*									m_taskGetDescriptors;
    xpcf::DelegateTask*									m_taskTracking;
    xpcf::DelegateTask*									m_taskMapping;        
    xpcf::DelegateTask*									m_taskGlobalBA;        

};

}//namespace PIPELINES
}//namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::PipelineSlam,
                             "577ccd2c-de1b-402a-8829-496747598588",
                             "PipelineSlam",
                             "A pipeline to estimate the pose based on a Slam");
#endif // PIPELINESLAM_H
