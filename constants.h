#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "SolARImageLoaderOpencv.h"
#include "SolARKeypointDetectorNonFreeOpencv.h"
#include "SolARDescriptorsExtractorSIFTOpencv.h"
#include "SolARDescriptorsExtractorSURF64Opencv.h"

#include "SolARImageViewerOpencv.h"
#include "SolAR2DOverlayOpencv.h"
#include "SolARCameraOpencv.h"

//#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARSideBySideOverlayOpencv.h"
#include "SolAR2DOverlayOpencv.h"
#include "SolARGeometricMatchesFilterOpencv.h"
#include "SolARFundamentalMatrixEstimationOpencv.h"
#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolARSVDTriangulationOpencv.h"
#include "SolAR2D3DcorrespondencesFinderOpencv.h"
#include "SolARMapperOpencv.h"
#include "SolARPoseEstimationPnpOpencv.h"
#include "freeglut.h"
#include "gl_camera.h"

//#define MYDEBUG

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::MODULES::NONFREEOPENCV;

namespace xpcf  = org::bcom::xpcf;

// declarations
   xpcf::utils::uuids::string_generator                 gen;

   SRef<input::devices::ICamera>                        camera;

   SRef<image::IImageLoader>                            imageLoader;
   SRef<features::IKeypointDetector>                    keypointsDetector;
   SRef<features::IDescriptorsExtractor>                descriptorExtractor;

   SRef<features::IDescriptorMatcher>                   matcher;
   SRef<display::IImageViewer>                          viewer;

   SRef<solver::pose::I2DTransformFinder>              fundamentalFinder;
   SRef<solver::pose::I2DTO3DTransformDecomposer>      fundamentalDecomposer;
   SRef<solver::map::ITriangulator>                    mapper;
   SRef<solver::map::IMapper>                          poseGraph;
   SRef<solver::pose::I3DTransformFinder>              PnP;
   SRef<solver::pose::I2D3DCorrespondencesFinder>      corr2D3DFinder;

   SRef<display::ISideBySideOverlay>                   overlay;
   SRef<display::I2DOverlay>                           overlay2d;

   SRef<Image>                                         view_1;
   SRef<Image>                                         view_2;
   SRef<Image>                                         view_current;
   std::vector<SRef<Image>>                            views;
   SRef<Keyframe>                                      kframe1;
   SRef<Keyframe>                                      kframe2;
   SRef<Keyframe>                                      kframe3;

   std::vector< SRef<Keypoint>>                        keypoints1;
   std::vector< SRef<Keypoint>>                        keypoints2;
   std::vector< SRef<Keypoint>>                        keypoints3;

   SRef<DescriptorBuffer>                              descriptors1;
   SRef<DescriptorBuffer>                              descriptors2;
   SRef<DescriptorBuffer>                              descriptors3;

   std::vector<DescriptorMatch>                        matches;

   std::vector<SRef<Point2Df>>                         matchedKeypoints1;
   std::vector<SRef<Point2Df>>                         matchedKeypoints2;

   std::vector<SRef<Point2Df>>                         gmatchedKeypoints1;
   std::vector<SRef<Point2Df>>                         gmatchedKeypoints2;

   std::vector<SRef<Point2Df>>                         ggmatchedKeypoints1;
   std::vector<SRef<Point2Df>>                         ggmatchedKeypoints2;
   std::vector<SRef<CloudPoint>>                       gcloud;



   SRef<Image>                                         viewerImage1;
   SRef<Image>                                         viewerImage2;
   SRef<Image>                                         viewerImage3;

   SRef<features::IMatchesFilter>                      matchesFilterGeometric;
   std::vector<DescriptorMatch>                        gmatches;
   std::vector<DescriptorMatch>                        ggmatches;

   CamCalibration                                      K;
   CamDistortion                                       dist;
   Transform2Df                                        F;
   std::vector<SRef<Pose>>                             poses;
   SRef<Pose>                                          pose_canonique ;
   SRef<Pose>                                          pose_final ;
   SRef<Pose>                                          pose_current ;
   // The escape key to exit the sample
   char escape_key = 27;

   cv::Vec3f                                          gravity;
   float                                              maxDist;

   int w,h;
   gl_camera g_camera;

   bool saving_images = false;
   bool triangulation_first = true;
   bool drawing = false;
   bool processing = false;

   // constants/ interactive


#endif // CONSTANTS_H
