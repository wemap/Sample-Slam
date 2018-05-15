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
#include "OpenGLViewer.h"

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



   SRef<Image>                                         viewerImage1;
   SRef<Image>                                         viewerImage2;
   SRef<Image>                                         viewerImage3;

   SRef<features::IMatchesFilter>                      matchesFilterGeometric;

   CamCalibration                                      K;
   CamDistortion                                       dist;
   Transform2Df                                        F;
   std::vector<Transform3Df>                           poses;
   Transform3Df                                        pose_canonique ;
   Transform3Df                                        pose_final ;
   Transform3Df                                        pose_current ;
   // The escape key to exit the sample
   char escape_key = 27;

    int                                                 nbFrameSinceKeyFrame ;


    OpenGLViewer                                       viewerGL ;


   bool saving_images = false;
   bool triangulation_first = true;
   bool processing = false;

   // constants/ interactive


#endif // CONSTANTS_H
