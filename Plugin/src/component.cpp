#include "xpcf/module/ModuleFactory.h"
#include "PipeLineSlam.h"

#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"
#include "SolARModuleFBOW_traits.h"
#include "core/Log.h"

#define USE_FREE
#define ONE_THREAD 1

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
    m_firstImageCaptured = false;
    m_keyFrameDetectionOn = true;
    m_isLostTrack = false;
}


PipelineSlam::~PipelineSlam()
{
     LOG_DEBUG(" Pipeline destructor")
}

FrameworkReturnCode PipelineSlam::init(SRef<xpcf::IComponentManager> xpcfComponentManager)
{
    // component creation
//   #ifdef USE_IMAGES_SET
//       m_camera = xpcfComponentManager->create<MODULES::OPENCV::SolARImagesAsCameraOpencv>()->bindTo<input::devices::ICamera>();
    try {
    #ifdef VIDEO_INPUT
           m_camera = xpcfComponentManager->create<MODULES::OPENCV::SolARVideoAsCameraOpencv>()->bindTo<input::devices::ICamera>();
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

        m_matcher =xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
        m_poseFinderFrom2D2D =xpcfComponentManager->create<MODULES::OPENCV::SolARPoseFinderFrom2D2DOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D2D>();
        m_triangulator =xpcfComponentManager->create<MODULES::OPENCV::SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
        m_basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
        m_geomMatchesFilter =xpcfComponentManager->create<MODULES::OPENCV::SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
        m_PnP =xpcfComponentManager->create<MODULES::OPENCV::SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
        m_PnPSAC =xpcfComponentManager->create<MODULES::OPENCV::SolARPoseEstimationSACPnpOpencv>()->bindTo<solver::pose::I3DTransformSACFinderFrom2D3D>();
        m_corr2D3DFinder =xpcfComponentManager->create<MODULES::OPENCV::SolAR2D3DCorrespondencesFinderOpencv>()->bindTo<solver::pose::I2D3DCorrespondencesFinder>();
        m_mapFilter =xpcfComponentManager->create<MODULES::TOOLS::SolARMapFilter>()->bindTo<solver::map::IMapFilter>();
        m_mapper =xpcfComponentManager->create<MODULES::TOOLS::SolARMapper>()->bindTo<solver::map::IMapper>();
        m_keyframeSelector =xpcfComponentManager->create<MODULES::TOOLS::SolARKeyframeSelector>()->bindTo<solver::map::IKeyframeSelector>();
        m_kfRetriever = xpcfComponentManager->create<MODULES::FBOW::SolARKeyframeRetrieverFBOW>()->bindTo<reloc::IKeyframeRetriever>();
        m_sink = xpcfComponentManager->create<MODULES::TOOLS::SolARBasicSink>()->bindTo<sink::ISinkPoseImage>();

        if ( m_camera==nullptr || m_keypointsDetector==nullptr || m_descriptorExtractor==nullptr || m_matcher==nullptr ||
            m_poseFinderFrom2D2D==nullptr || m_triangulator==nullptr || m_mapFilter==nullptr || m_mapper==nullptr || m_keyframeSelector==nullptr || m_PnP==nullptr ||
            m_corr2D3DFinder==nullptr || m_geomMatchesFilter==nullptr || m_basicMatchesFilter==nullptr || m_kfRetriever==nullptr || m_sink==nullptr)
        {
           LOG_ERROR("One or more component creations have failed");
           return FrameworkReturnCode::_ERROR_  ;
        }

        // init relative to fiducial marker detection (will define the start of the process)
        m_binaryMarker =xpcfComponentManager->create<MODULES::OPENCV::SolARMarker2DSquaredBinaryOpencv>()->bindTo<input::files::IMarker2DSquaredBinary>();
        m_imageFilterBinary =xpcfComponentManager->create<MODULES::OPENCV::SolARImageFilterBinaryOpencv>()->bindTo<image::IImageFilter>();
        m_imageConvertor =xpcfComponentManager->create<MODULES::OPENCV::SolARImageConvertorOpencv>()->bindTo<image::IImageConvertor>();
        m_contoursExtractor =xpcfComponentManager->create<MODULES::OPENCV::SolARContoursExtractorOpencv>()->bindTo<features::IContoursExtractor>();
        m_contoursFilter =xpcfComponentManager->create<MODULES::OPENCV::SolARContoursFilterBinaryMarkerOpencv>()->bindTo<features::IContoursFilter>();
        m_perspectiveController =xpcfComponentManager->create<MODULES::OPENCV::SolARPerspectiveControllerOpencv>()->bindTo<image::IPerspectiveController>();
        m_projector = xpcfComponentManager->create<MODULES::OPENCV::SolARProjectOpencv>()->bindTo<geom::IProject>();
        m_patternDescriptorExtractor =xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorsExtractorSBPatternOpencv>()->bindTo<features::IDescriptorsExtractorSBPattern>();
        m_patternMatcher =xpcfComponentManager->create<MODULES::OPENCV::SolARDescriptorMatcherRadiusOpencv>()->bindTo<features::IDescriptorMatcher>();
        m_patternReIndexer = xpcfComponentManager->create<MODULES::TOOLS::SolARSBPatternReIndexer>()->bindTo<features::ISBPatternReIndexer>();
        m_img2worldMapper = xpcfComponentManager->create<MODULES::TOOLS::SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();

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



        // specific to the
        // initialize components requiring the camera intrinsic and distortion parameters
        m_PnP->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_PnPSAC->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_poseFinderFrom2D2D->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_triangulator->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());
        m_projector->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());

        m_i2DOverlay = xpcf::ComponentFactory::createInstance<SolAR2DOverlayOpencv>()->bindTo<api::display::I2DOverlay>();
        m_i2DOverlay->bindTo<xpcf::IConfigurable>()->getProperty("radius")->setUnsignedIntegerValue(1);

        m_initOK = true;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR("Exception catched: {}", e.what());
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}


// get images from camera

void PipelineSlam::getCameraImages(){

    SRef<Image> view;
    if (m_stopFlag || !m_initOK || !m_startedOK)
        return;
    if (m_camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
        m_stopFlag = true;
        return;
    }
    if(m_CameraImagesBuffer.empty())
         m_CameraImagesBuffer.push(view);

    return;
};



bool PipelineSlam::detectFiducialMarkerCore(SRef<Image>& image)
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


    bool poseComputed = false;

   // Convert Image from RGB to grey
    m_imageConvertor->convert(image, greyImage, Image::ImageLayout::LAYOUT_GREY);

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
            if (m_PnP->estimate(img2DPoints, pattern3DPoints, m_pose) == FrameworkReturnCode::_SUCCESS)
            {
                poseComputed = true;
                LOG_DEBUG("Fiducial marker detected")
             }
        }
    }

    return poseComputed;

}


void PipelineSlam::detectFiducialMarker()
{
    SRef<Image>                     camImage, greyImage, binaryImage;
    std::vector<SRef<Contour2Df>>   contours;
    std::vector<SRef<Contour2Df>>   filtered_contours;
    std::vector<SRef<Image>>        patches;
    std::vector<SRef<Contour2Df>>   recognizedContours;
    SRef<DescriptorBuffer>          recognizedPatternsDescriptors;
    std::vector<DescriptorMatch>    patternMatches;
    std::vector<SRef<Point2Df>>     pattern2DPoints;
    std::vector<SRef<Point2Df>>     img2DPoints;
    std::vector<SRef<Point3Df>>     pattern3DPoints;

    if (m_stopFlag || !m_initOK || !m_startedOK)
        return ;
    if(m_firstImageCaptured){
        return ;
    }

    if (!m_CameraImagesBuffer.tryPop(camImage))
        return  ;

    if ( detectFiducialMarkerCore(camImage)){
        m_sink->set(m_pose, camImage);
        poseFrame1=m_pose;
        m_keypointsDetector->detect(camImage, keypointsView1);
        m_descriptorExtractor->extract(camImage, keypointsView1, descriptorsView1);
        keyframe1 = xpcf::utils::make_shared<Keyframe>(keypointsView1, descriptorsView1, camImage, poseFrame1);
        m_mapper->update(m_map, keyframe1);
        m_kfRetriever->addKeyframe(keyframe1); // add keyframe for reloc
        m_firstImageCaptured = true;
    }
    else
        m_sink->set(camImage);

    return ;
}


void PipelineSlam::doBootStrap()
{
    SRef<Image>  camImage;
    Transform3Df poseFrame2;
    SRef<Keyframe>                                      keyframe2;
    std::vector<Keypoint>                         keypointsView2;
    SRef<DescriptorBuffer>                              descriptorsView2;
    std::vector<SRef<CloudPoint>>                       cloud, filteredCloud;

    if (m_stopFlag || !m_initOK || !m_startedOK)
        return ;
    if(!m_firstImageCaptured)
        return ;
    if(m_bootstrapOk)
        return ;

    if (!m_CameraImagesBuffer.tryPop(camImage))
        return  ;

    if(detectFiducialMarkerCore(camImage)){
        m_sink->set(m_pose, camImage);

        m_keypointsDetector->detect(camImage, keypointsView2);
        m_descriptorExtractor->extract(camImage, keypointsView2, descriptorsView2);

        poseFrame2=m_pose;
        m_matcher->match(descriptorsView1, descriptorsView2, matches);

        int nbOriginalMatches = matches.size();

        /* filter matches to remove redundancy and check geometric validity */
         m_basicMatchesFilter->filter(matches, matches, keypointsView1, keypointsView2);
         m_geomMatchesFilter->filter(matches, matches, keypointsView1, keypointsView2);


        SRef<Frame> frame2 = xpcf::utils::make_shared<Frame>(keypointsView2, descriptorsView2, camImage, keyframe1);
        frame2->setPose(poseFrame2);

        if (m_keyframeSelector->select(frame2, matches)) {

             LOG_INFO("Nb matches for triangulation: {}\\{}", matches.size(), nbOriginalMatches);
             LOG_INFO("Estimate pose of the camera for the frame 2: \n {}", poseFrame2.matrix());

             std::vector<DescriptorMatch> emptyMatches;
             m_keyFrameBuffer.push(std::make_tuple(frame2,keyframe1,emptyMatches,matches));

             m_bootstrapOk = true;
             LOG_INFO("BootStrap is validated \n");
        }

    }
    else {
        m_sink->set(camImage);
    }

    return ;
}

// extract key points
void PipelineSlam::getKeyPoints(){
    SRef<Image>  camImage;

    if (m_stopFlag || !m_initOK || !m_startedOK)
        return ;
    if(!m_bootstrapOk)
        return ;

    if (!m_CameraImagesBuffer.tryPop(camImage))
        return  ;

    std::vector<Keypoint> kp;
    m_keypointsDetector->detect(camImage, kp);
    if(m_outBufferKeypoints.empty())
        m_outBufferKeypoints.push(std::make_pair(camImage,kp));

    return;
};

// compute descriptors
void PipelineSlam::getDescriptors(){

    std::pair<SRef<Image>, std::vector<Keypoint> > kp ;
    SRef<DescriptorBuffer> camDescriptors;
    SRef<Frame> frame;

    if (m_stopFlag || !m_initOK || !m_startedOK)
        return ;
    if(!m_bootstrapOk)
        return ;

    if (!m_outBufferKeypoints.tryPop(kp)) {
        return ;
    }
    m_descriptorExtractor->extract(kp.first, kp.second, camDescriptors);
    frame=xpcf::utils::make_shared<Frame>(kp.second,camDescriptors,kp.first);

    if(m_outBufferDescriptors.empty())
        m_outBufferDescriptors.push(frame) ;

    return;
};

// A new keyFrame has been detected :
// - the triangulation has been performed
// - the Map is updated accordingly
//
void PipelineSlam::mapUpdate(){
    std::tuple<SRef<Keyframe>, SRef<Keyframe>, std::vector<DescriptorMatch>, std::vector<DescriptorMatch>, std::vector<CloudPoint>  >   element;
    std::vector<DescriptorMatch>                        foundMatches, remainingMatches;
    std::vector<CloudPoint>                       newCloud;
    std::vector<CloudPoint>                       filteredCloud;

    if (m_stopFlag || !m_initOK || !m_startedOK)
        return ;

    if(!m_bootstrapOk)
        return ;

    if (!m_outBufferTriangulation.tryPop(element)) {
        return ;
    }
    SRef<Keyframe>                                      newKeyframe,refKeyframe;

    newKeyframe=std::get<0>(element);
    refKeyframe=std::get<1>(element);
    foundMatches=std::get<2>(element);
    remainingMatches=std::get<3>(element);
    newCloud=std::get<4>(element);

    std::map<unsigned int, CloudPoint> frameVisibility = newKeyframe->getReferenceKeyframe()->getVisibleMapPoints();
    std::map<unsigned int, unsigned int> visibleKeypoints= newKeyframe->getReferenceKeyframe()->getVisibleKeypoints();
    frameVisibility = newKeyframe->getVisibleMapPoints();

    LOG_DEBUG(" frame pose estimation :\n {}", newKeyframe->getPose().matrix());
    LOG_DEBUG("Number of matches: {}, number of 3D points:{}", remainingMatches.size(), newCloud.size());
    //newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);
    m_mapFilter->filter(refKeyframe->getPose(), newKeyframe->getPose(), newCloud, filteredCloud);
    frameVisibility = newKeyframe->getVisibleMapPoints();
    LOG_INFO(" cur KF frameVisibility   : {} ", frameVisibility.size());
    m_mapper->update(m_map, newKeyframe, filteredCloud, remainingMatches, foundMatches);
    frameVisibility = newKeyframe->getVisibleMapPoints();

    m_referenceKeyframe = newKeyframe;
    m_frameToTrack = xpcf::utils::make_shared<Frame>(m_referenceKeyframe);
    m_frameToTrack->setReferenceKeyframe(m_referenceKeyframe);
    m_kfRetriever->addKeyframe(m_referenceKeyframe); // add keyframe for reloc
    m_keyframePoses.push_back(newKeyframe->getPose());
    LOG_DEBUG(" cloud current size: {} \n", m_map->getPointCloud().size());

    m_keyFrameDetectionOn = true;					// re - allow keyframe detection

    return;

};

// A new keyFrame has been detected :
// - perform triangulation
// - the resulting cloud will be used to update the Map
//
void PipelineSlam::doTriangulation(){
    std::tuple<SRef<Frame>,SRef<Keyframe>,std::vector<DescriptorMatch>,std::vector<DescriptorMatch> > element;
    SRef<Frame>                                         newFrame;
    SRef<Keyframe>                                      newKeyframe;
    SRef<Keyframe>                                      refKeyFrame;
    std::vector<DescriptorMatch>                        foundMatches;
    std::vector<DescriptorMatch>                        remainingMatches;
    std::vector<CloudPoint>								newCloud;

    if (m_stopFlag || !m_initOK || !m_startedOK)
        return ;

    if(!m_bootstrapOk)
        return ;


    if (!m_keyFrameBuffer.tryPop(element) ){
        return ;
    }

    newFrame=std::get<0>(element);
    refKeyFrame=std::get<1>(element);
    foundMatches=std::get<2>(element);
    remainingMatches=std::get<3>(element);

    newKeyframe = xpcf::utils::make_shared<Keyframe>(newFrame);
	
    if(remainingMatches.size())
            m_triangulator->triangulate(newKeyframe, remainingMatches, newCloud);
    //triangulator->triangulate(refKeyFrame->getKeypoints(), newFrame->getKeypoints(), remainingMatches,std::make_pair<int,int>((int)refKeyFrame->m_idx+0,(int)(refKeyFrame->m_idx+1)),
    //                    refKeyFrame->getPose(), newFrame->getPose(), newCloud);
    if(m_outBufferTriangulation.empty())
        m_outBufferTriangulation.push(std::make_tuple(newKeyframe,refKeyFrame,foundMatches,remainingMatches,newCloud));

    return;
};


// Processing of input frames :
// - perform match+PnP
// - test if current frame can promoted to a keyFrame.
// - in that case, push it in the output buffer to be processed by the triangulation thread
//

void PipelineSlam::processFrames(){

     SRef<Frame> newFrame;
     SRef<Keyframe> refKeyFrame;
     SRef<Image> camImage;
     std::vector< Keypoint > keypoints;
     SRef<DescriptorBuffer> descriptors;
     SRef<DescriptorBuffer> refDescriptors;
     std::vector<DescriptorMatch> matches;

     std::vector<Point2Df> pt2d;
     std::vector<Point3Df> pt3d;
     std::vector<CloudPoint> foundPoints;
     std::vector<DescriptorMatch> foundMatches;
     std::vector<DescriptorMatch> remainingMatches;
     std::vector<Point2Df> imagePoints_inliers;
     std::vector<Point3Df> worldPoints_inliers;

     std::vector < SRef <Keyframe>> ret_keyframes;

     if (m_stopFlag || !m_initOK || !m_startedOK)
         return ;

     if(!m_bootstrapOk)
         return ;

     if (m_isLostTrack && !m_outBufferTriangulation.empty() && !m_keyFrameBuffer.empty())
         return;

     // test if a triangulation has been performed on a previously keyframe candidate
     if(!m_outBufferTriangulation.empty()){
         //if so update the map
         mapUpdate();
     }

     /*compute matches between reference image and camera image*/
     if(!m_outBufferDescriptors.tryPop(newFrame)){
             return;
     }

     // referenceKeyframe can be changed outside : let's make a copy.
     if (!m_keyframeRelocBuffer.empty()) {
         m_referenceKeyframe = m_keyframeRelocBuffer.pop();
         m_frameToTrack = xpcf::utils::make_shared<Frame>(m_referenceKeyframe);
         m_frameToTrack->setReferenceKeyframe(m_referenceKeyframe);
         m_lastPose = m_referenceKeyframe->getPose();
     }

     newFrame->setReferenceKeyframe(m_referenceKeyframe);
     refKeyFrame = newFrame->getReferenceKeyframe();
     camImage=newFrame->getView();
     keypoints=newFrame->getKeypoints();
     descriptors=newFrame->getDescriptors();

//     m_i2DOverlay->drawCircles(keypoints,camImage);

     refDescriptors= m_frameToTrack->getDescriptors();
     m_matcher->match(refDescriptors, descriptors, matches);

     /* filter matches to remove redundancy and check geometric validity */
     m_basicMatchesFilter->filter(matches, matches, m_frameToTrack->getKeypoints(), keypoints);
     m_geomMatchesFilter->filter(matches, matches, m_frameToTrack->getKeypoints(), keypoints);

     std::map<unsigned int, CloudPoint> frameVisibility = m_frameToTrack->getReferenceKeyframe()->getVisibleMapPoints();
     m_corr2D3DFinder->find(m_frameToTrack, newFrame, matches, foundPoints, pt3d, pt2d, foundMatches, remainingMatches);

     if (m_PnPSAC->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, m_pose , m_lastPose) == FrameworkReturnCode::_SUCCESS){
        LOG_DEBUG(" frame pose  :\n {}", m_pose.matrix());
        LOG_DEBUG(" pnp inliers size: {} / {}",worldPoints_inliers.size(), pt3d.size());

        m_lastPose = m_pose;
        std::vector<Point2Df>	point2D;
        std::vector<CloudPoint> cloud;
        cloud=m_map->getPointCloud();
        m_projector->project(cloud, point2D, m_pose);
        m_i2DOverlay->drawCircles(point2D,camImage);
        // update new frame
        newFrame->setPose(m_pose);
        // update last frame
        m_frameToTrack = newFrame;

        // If the camera has moved enough, create a keyframe and map the scene
        if ( m_keyFrameDetectionOn &&  m_keyframeSelector->select(newFrame, foundMatches) ){
            m_keyFrameDetectionOn=false;
            LOG_INFO("New key Frame ")
            m_keyFrameBuffer.push(std::make_tuple(newFrame,refKeyFrame,foundMatches,remainingMatches));
        }
        m_isLostTrack = false;
        m_sink->set(m_pose, camImage);
     }
     else {
         LOG_DEBUG (" No valid pose was found");
         m_sink->set(camImage);
         m_isLostTrack = true;
         if ( m_kfRetriever->retrieve(newFrame, ret_keyframes) == FrameworkReturnCode::_SUCCESS) {
             LOG_INFO("Retrieval Success based on FBOW");
             m_keyframeRelocBuffer.push(ret_keyframes[0]);
             m_isLostTrack = false;
         }
//         else if ( detectFiducialMarkerCore(camImage)){
//              m_lastPose = m_pose;
//              m_isLostTrack = false;
//         }

        else{
             LOG_INFO("Retrieval Failed");
         }
     }

     return;
};

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
    auto detectFiducialMarkerThread = [this](){;detectFiducialMarker();};
    auto doBootStrapThread = [this](){;doBootStrap();};
    auto getKeyPointsThread = [this](){;getKeyPoints();};
    auto getDescriptorsThread = [this](){;getDescriptors();};
    auto processFramesThread = [this](){;processFrames();};
    auto doTriangulationThread = [this](){;doTriangulation();};
    auto mapUpdateThread = [this](){;mapUpdate();};

#if ONE_THREAD
    auto allTasksThread=[this](){;allTasks();};
    m_taskAll= new xpcf::DelegateTask(allTasksThread);
    m_taskAll->start();
#else
    m_taskGetCameraImages = new xpcf::DelegateTask(getCameraImagesThread);
    m_taskDetectFiducialMarker = new xpcf::DelegateTask(detectFiducialMarkerThread);
    m_taskDoBootStrap = new xpcf::DelegateTask(doBootStrapThread);
    m_taskGetKeyPoints = new xpcf::DelegateTask(getKeyPointsThread);
    m_taskGetDescriptors = new xpcf::DelegateTask(getDescriptorsThread);
    m_taskProcessFrames = new xpcf::DelegateTask(processFramesThread);
    m_taskDoTriangulation = new xpcf::DelegateTask(doTriangulationThread);
    m_taskMapUpdate = new xpcf::DelegateTask(mapUpdateThread);

    m_taskGetCameraImages->start();
    m_taskDetectFiducialMarker->start();
    m_taskDoBootStrap ->start();
    m_taskGetKeyPoints->start();
    m_taskGetDescriptors->start();
    m_taskProcessFrames ->start();
    m_taskDoTriangulation->start();
//    m_taskMapUpdate->start();
#endif

    LOG_INFO("Threads have started");
    m_startedOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode PipelineSlam::stop()
{
    m_stopFlag=true;
    m_camera->stop();

#if ONE_THREAD
    if (m_taskAll != nullptr)
            m_taskAll->stop();
#else
    if (m_taskGetCameraImages != nullptr)
        m_taskGetCameraImages->stop();
    if (m_taskDetectFiducialMarker != nullptr)
        m_taskDetectFiducialMarker->stop();
    if (m_taskDoBootStrap != nullptr)
        m_taskDoBootStrap->stop();
    if (m_taskGetKeyPoints != nullptr)
        m_taskGetKeyPoints->stop();
    if (m_taskGetDescriptors != nullptr)
        m_taskGetDescriptors->stop();
    if (m_taskProcessFrames != nullptr)
        m_taskProcessFrames->stop();
    if (m_taskDoTriangulation != nullptr)
        m_taskDoTriangulation->stop();
//    if (m_taskMapUpdate != nullptr)
//        m_taskMapUpdate->stop();
#endif

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


void PipelineSlam::allTasks(){
    getCameraImages();
    detectFiducialMarker();
    doBootStrap();
    getKeyPoints();
    getDescriptors();
    doTriangulation();
    processFrames();
//    mapUpdate();
}

}//namespace PIPELINES
}//namespace SolAR
