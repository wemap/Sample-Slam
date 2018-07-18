#ifndef CONSTANTS_H
#define CONSTANTS_H

//#define USE_FREE


#include "SolARImageLoaderOpencv.h"
#ifdef USE_FREE
#include "SolARDescriptorsExtractorORBOpencv.h"
#include "SolARKeypointDetectorOpencv.h"
#else
#include "SolARKeypointDetectorNonFreeOpencv.h"
#include "SolARDescriptorsExtractorSIFTOpencv.h"
#include "SolARDescriptorsExtractorSURF64Opencv.h"
#endif

#include "SolARImageViewerOpencv.h"
#include "SolAR2DOverlayOpencv.h"
#include "SolARCameraOpencv.h"

#ifdef USE_FREE
#include "SolARDescriptorMatcherKNNOpencv.h"
#include "SolARDescriptorMatcherHammingBruteForceOpencv.h"
#else
//#include "SolARDescriptorMatcherKNNOpencv.h"
#endif
#include "SolARDescriptorMatcherRadiusOpencv.h"
#include "SolARImageViewerOpencv.h"
#include "SolARSideBySideOverlayOpencv.h"
#include "SolAR2DOverlayOpencv.h"
#include "SolARGeometricMatchesFilterOpencv.h"
#include "SolARFundamentalMatrixEstimationOpencv.h"
#include "SolARSVDFundamentalMatrixDecomposerOpencv.h"
#include "SolARSVDTriangulationOpencv.h"
#include "SolAR2D3DcorrespondencesFinderOpencv.h"
#include "SolARMapFilterOpencv.h"
#include "SolARMapperOpencv.h"
#include "SolARPoseEstimationPnpOpencv.h"
#include "SolARPoseEstimationPnpEPFL.h"
#include "freeglut.h"
#include "gl_camera.h"
#include "OpenGLViewer.h"

//#define MYDEBUG

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::MODULES::OPENCV;
#ifdef USE_FREE
using namespace SolAR::MODULES::OPENCV;
#else
using namespace SolAR::MODULES::NONFREEOPENCV;
#endif


namespace xpcf  = org::bcom::xpcf;

// declarations
   xpcf::utils::uuids::string_generator                 gen;

   SRef<input::devices::ICamera>                        camera;

   SRef<image::IImageLoader>                            imageLoader;
   SRef<features::IKeypointDetector>                    keypointsDetector;
   SRef<features::IDescriptorsExtractor>                descriptorExtractor;

   SRef<features::IMatchesFilter>                      matchesFilterGeometric;

   SRef<features::IDescriptorMatcher>                   matcher;
   SRef<display::IImageViewer>                          viewer;

   SRef<solver::pose::I2DTransformFinder>              fundamentalFinder;
   SRef<solver::pose::I2DTO3DTransformDecomposer>      fundamentalDecomposer;
   SRef<solver::map::ITriangulator>                    mapper;
   SRef<solver::map::IMapFilter>					   mapFilter;
   SRef<solver::map::IMapper>                          poseGraph;
   SRef<solver::pose::I3DTransformFinder>              PnP;
   SRef<solver::pose::I2D3DCorrespondencesFinder>      corr2D3DFinder;

   SRef<display::ISideBySideOverlay>                   overlay;
   SRef<display::I2DOverlay>                           overlay2d;

   SRef<Image>                                         view_current;
   std::vector<SRef<Image>>                            views;
   SRef<Image>                                         currentMatchImage;
   SRef<Image>                                         projected_image;

   SRef<Image>                                         viewerImage1;
   SRef<Image>                                         viewerImage2;
   SRef<Image>                                         viewerImage3;

  

   CamCalibration                                      K;
   CamDistortion                                       dist;
   Transform2Df                                        F;
    int												   nbFrameSinceKeyFrame ;
    OpenGLViewer                                       viewerGL ;
    gl_camera                                          viewer3D;


	std::string											streamSource; // camera or path offline video 
	std::string											calibCameraSource; // path calibration camera yml
	// parameters used in case of video as input
	int													indexFirstKeyFrame; 
	int													indexSecondKeyFrame; 
	int													indexCurrentFrame; 
	int													frameCount; 
    int                                                 w;
    int                                                 h;

   bool saving_images = false;
   bool triangulation_first = true;
   bool processing = false;
   bool pause_exec = false;
   bool exit_ = false;




#endif // CONSTANTS_H
