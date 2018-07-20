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
#include "constants.h"
#include "datastructure/Frame.h"
#include <boost/log/core.hpp>
#include <ctime>
#include <fstream>

#include "opencv2/highgui.hpp"

#include <boost/filesystem.hpp>

#include "drawingUtils.h"


namespace xpcf = org::bcom::xpcf;
using namespace org::bcom::xpcf;

std::vector<Transform3Df> keyframe_poses;
std::vector<Transform3Df> frame_poses;

struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry &entry) const
    {
        return entry.path().leaf().string();
    }
};

void read_directory(const std::string &name, std::vector<std::string> &v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
}

const char space_key = 32;
const char escape_key = 27;
std::vector<SRef<Image>> static_views;
bool adding_once = true;
const int static_views_no = 9;
int frame_counter;
int frame_begin = 4;

void ogl_callback_keyBoard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 's':
    {
        saving_images = !saving_images;
        LOG_INFO(" image saved: {}", views.size());

        break;
    }
    case 't':
    {
        triangulation_first = !triangulation_first;
           LOG_INFO(" triangulation started: {} ", triangulation_first);
        break;
    }
    case 'd':
    {
        viewerGL.SetPointCloudToDisplay(poseGraph->getMap()->getPointCloud());
          LOG_INFO(" drawing started ");
        break;
    }
    case 'p':
    {
        processing = !processing;
          LOG_INFO("processing started: {} ", processing);
        break;
    }
    case 'x':
    {
        LOG_INFO("views poped: {} ", processing );

        views.pop_back();
        break;
    }
    case space_key:
    {
        LOG_INFO("make pause/!pause ");
        pause_exec = !pause_exec;
        break;
    }
    case escape_key:
    {
        LOG_INFO("Exit");
        exit_ = true;
        break;
    }
    default:
        break;
    }
}

bool ParseConfigFile(const std::string &filePath)
{
    indexCurrentFrame = 0;
    streamSource = "";
    indexFirstKeyFrame = 0;
    indexSecondKeyFrame = 0;

    std::string dummy[4];
    std::ifstream ox;

    ox.open(filePath);
    LOG_INFO("<SLAM config: >");
    if (ox.is_open())
    {
        for (int i = 0; i < 4; ++i)
        {
            ox >> dummy[i];
        }
        streamSource = dummy[0];
        calibCameraSource = dummy[1];
        indexFirstKeyFrame = std::stoi(dummy[2]);
        indexSecondKeyFrame = std::stoi(dummy[3]);

        LOG_INFO("	# stream mode: {} " ,streamSource);
        LOG_INFO("	# calib file: {} ", calibCameraSource);
        LOG_INFO("	# triangulation pair: {}, {}",  indexFirstKeyFrame , indexSecondKeyFrame);

        ox.close();

        return true;
    }
    else
    {
        ox.close();
        LOG_INFO("	 can't read slam config slam file from: {} " ,filePath);

        return false;
    }
}

void init(std::string configFile)
{
    // component creation

    camera = xpcf::ComponentFactory::createInstance<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    imageLoader = xpcf::ComponentFactory::createInstance<SolARImageLoaderOpencv>()->bindTo<image::IImageLoader>();

#ifdef USE_FREE
    keypointsDetector = xpcf::ComponentFactory::createInstance<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
    descriptorExtractor = xpcf::ComponentFactory::createInstance<SolARDescriptorsExtractorORBOpencv>()->bindTo<features::IDescriptorsExtractor>();
#else
    LOG_INFO("USE non Free descriptors");

    keypointsDetector = xpcf::ComponentFactory::createInstance<SolARKeypointDetectorNonFreeOpencv>()->bindTo<features::IKeypointDetector>();
    descriptorExtractor = xpcf::ComponentFactory::createInstance<SolARDescriptorsExtractorSURF64Opencv>()->bindTo<features::IDescriptorsExtractor>();
#endif

#ifdef USE_FREE
    matcher = xpcf::ComponentFactory::createInstance<SolARDescriptorMatcherHammingBruteForceOpencv>()->bindTo<features::IDescriptorMatcher>();
#else
    matcher = xpcf::ComponentFactory::createInstance<SolARDescriptorMatcherRadiusOpencv>()->bindTo<features::IDescriptorMatcher>();

#endif

    overlay = xpcf::ComponentFactory::createInstance<SolARSideBySideOverlayOpencv>()->bindTo<display::ISideBySideOverlay>();
    overlay2d = xpcf::ComponentFactory::createInstance<SolAR2DOverlayOpencv>()->bindTo<display::I2DOverlay>();

    viewer = xpcf::ComponentFactory::createInstance<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    matchesFilterGeometric = xpcf::ComponentFactory::createInstance<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
    fundamentalFinder = xpcf::ComponentFactory::createInstance<SolARFundamentalMatrixEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
    fundamentalDecomposer = xpcf::ComponentFactory::createInstance<SolARSVDFundamentalMatrixDecomposerOpencv>()->bindTo<solver::pose::I2DTO3DTransformDecomposer>();
    mapper = xpcf::ComponentFactory::createInstance<SolARSVDTriangulationOpencv>()->bindTo<solver::map::ITriangulator>();
    mapFilter = xpcf::ComponentFactory::createInstance<SolARMapFilterOpencv>()->bindTo<solver::map::IMapFilter>();

    poseGraph = xpcf::ComponentFactory::createInstance<SolARMapperOpencv>()->bindTo<solver::map::IMapper>();
    PnP = xpcf::ComponentFactory::createInstance<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinder>();

    corr2D3DFinder = xpcf::ComponentFactory::createInstance<SolAR2D3DCorrespondencesFinderOpencv>()->bindTo<solver::pose::I2D3DCorrespondencesFinder>();

    LOG_INFO(" Parse file");
    ParseConfigFile(configFile.c_str());

#ifdef USE_FREE
    keypointsDetector->setType(features::KeypointDetectorType::ORB);
#else
    keypointsDetector->setType(features::KeypointDetectorType::SURF);
#endif
    // load camera parameters from yml input file
    camera->loadCameraParameters(calibCameraSource);
    PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
    K = camera->getIntrinsicsParameters();
    dist = camera->getDistorsionParameters();
    // Open camera or file
    if (streamSource == "camera")
    {
        camera->start(0);
    }
    else
    {
        camera->start(streamSource);
        //std::cout << "	# loading frames: ";
        //// this part to load image from video:
    }
    
    LOG_INFO(" Loading static view:");
    
    static_views.resize(static_views_no);
    
    for (int k = 0; k < static_views_no; ++k)
    {
        std::stringstream buff;
        buff << std::setfill('0') << std::setw(5) << k;
        std::string path_temp = streamSource + buff.str() + ".png";
        
        LOG_INFO(" temp: {}",path_temp);
        
        imageLoader->loadImage(path_temp, static_views[k]);

        LOG_INFO("     ->img size: {}, {}" ,static_views[k]->getWidth(), static_views[k]->getHeight());

        viewer->display("__view", static_views[k]);
    }

        frame_counter = frame_begin;
        LOG_INFO(" Loading static view:........done");
}

bool fullTriangulation(const std::vector<SRef<Point2Df>> &pt2d_1,
                       const std::vector<SRef<Point2Df>> &pt2d_2,
                       const std::vector<DescriptorMatch> &matches,
                       const std::pair<int, int> &working_views,
                       const Transform3Df &p1,
                       const std::vector<Transform3Df> &p2,
                       const CamCalibration &cam,
                       const CamDistortion &dist,
                       Transform3Df &corrected_pose,
                       std::vector<SRef<CloudPoint>> &cloud)
{

    if (p2.size() != 4)
    {
        LOG_ERROR(" number of decomposed poses supposed to be 4.." );

        return false;
    }

    std::vector<SRef<CloudPoint>> pcloud;
    std::vector<SRef<CloudPoint>> pcloud1;
    std::vector<bool> tmp_status;

    //check if points are triangulated --in front-- of cameras for all 4 ambiguations
    for (int i = 0; i < 4; i++)
    {
        corrected_pose = p2[i];
        pcloud.clear();
        pcloud1.clear();

        double reproj_error1 = mapper->triangulate(pt2d_1, pt2d_2, matches, working_views, p1, corrected_pose, cam, dist, pcloud);
        double reproj_error2 = mapper->triangulate(pt2d_2, pt2d_1, matches, working_views, corrected_pose, p1, cam, dist, pcloud1);
        if (mapFilter->checkFrontCameraPoints(pcloud, corrected_pose, tmp_status) && mapFilter->checkFrontCameraPoints(pcloud1, p1, tmp_status) && reproj_error1 < 100.0 && reproj_error2 < 100.0)
        {
            // points are in front of camera and reproj error is acceptable
            break;
        }
        if (i == 3)
        {
            return false;
        }
    }
    LOG_INFO("pcloud basic: {}" , pcloud.size());
    cloud.reserve(pcloud.size());

    for (unsigned int k = 0; k < pcloud.size(); k++)
    {
        cloud.push_back(pcloud[k]);
    }

    mapFilter->filterPointCloud(pcloud, tmp_status, cloud);
    LOG_INFO("cloud filtred: {}" , cloud.size());

    return true;
}

SRef<Frame> createAndInitFrame(SRef<Image> &img)
{
    SRef<Frame> output_frame = xpcf::utils::make_shared<Frame>();

    std::vector<SRef<Keypoint>> keyPoints;
    SRef<DescriptorBuffer> descriptors;

    keypointsDetector->detect(img, keyPoints);
    descriptorExtractor->extract(img, keyPoints, descriptors);

    output_frame->InitKeyPointsAndDescriptors(keyPoints, descriptors);

    return output_frame;
}

void getMatchedKeyPoints(std::vector<SRef<Keypoint>> &keyPoints1, std::vector<SRef<Keypoint>> &keyPoints2, std::vector<DescriptorMatch> &matches, std::vector<SRef<Point2Df>> &matchedKeyPoints1, std::vector<SRef<Point2Df>> &matchedKeyPoints2)
{
    matchedKeyPoints1.reserve(matches.size()); // allocate memory

    for (int i = 0; i < matches.size(); i++)
    {
        matchedKeyPoints1.push_back(xpcf::utils::make_shared<Point2Df>(keyPoints1[matches[i].getIndexInDescriptorA()]->getX(), keyPoints1[matches[i].getIndexInDescriptorA()]->getY()));
        matchedKeyPoints2.push_back(xpcf::utils::make_shared<Point2Df>(keyPoints2[matches[i].getIndexInDescriptorB()]->getX(), keyPoints2[matches[i].getIndexInDescriptorB()]->getY()));
    }
}

void getPoint2DFromKeyPoint(std::vector<SRef<Keypoint>> &keyPoints, std::vector<SRef<Point2Df>> &to2D)
{
    to2D.reserve(keyPoints.size());
    for (int i = 0; i < keyPoints.size(); i++)
    {
        to2D.push_back(xpcf::utils::make_shared<Point2Df>(keyPoints[i]->getX(), keyPoints[i]->getY()));
    }
}


bool addFrameToMapAsKeyFrame(SRef<Frame> &frame, SRef<Image> &view, int newIndex)
{
    LOG_INFO( "    #debug adding keyframes: ");
    SRef<Keyframe> referenceKeyFrame = frame->getReferenceKeyFrame();

    viewer->display("reference keyFrame", referenceKeyFrame->m_view);
    cv::waitKey(0);

    Transform3Df poseFrame = frame->m_pose;
    Transform3Df poseKeyFrame = referenceKeyFrame->m_pose;

    LOG_INFO("        #pose frame: " );
    for (int ii = 0; ii < 3; ++ii)
    {
        for (int jj = 0; jj < 3; ++jj)
        {
              LOG_INFO("        {}",poseFrame(ii, jj)  );
        }
    }

    LOG_INFO("        #pose kframe: " );
    for (int ii = 0; ii < 3; ++ii)
    {
        for (int jj = 0; jj < 3; ++jj)
        {
            LOG_INFO("        {}",poseKeyFrame(ii, jj));
        }
    }

    std::vector<SRef<Point2Df>> pointsFrame;
    std::vector<SRef<Point2Df>> pointsKeyFrame;

    std::vector<SRef<Keypoint>> kp1, kp2;
    kp1 = referenceKeyFrame->getKeyPoints();
    kp2 = frame->getKeyPoints();

    // triangulate all points from keyFrame?
    getMatchedKeyPoints(kp1, kp2, frame->getUnknownMatchesWithReferenceKeyFrame(), pointsKeyFrame, pointsFrame);

    LOG_INFO("    ->match points: {}",pointsFrame.size());

    std::vector<SRef<CloudPoint>> newMapPoints;
    std::pair<int, int> corres(referenceKeyFrame->m_idx, newIndex);

    // Triangulate new points
    mapper->triangulate(pointsFrame, pointsKeyFrame, frame->getUnknownMatchesWithReferenceKeyFrame(), corres, frame->m_pose, referenceKeyFrame->m_pose,
                        K, dist, newMapPoints);

    LOG_INFO("     ->new 3d points: {}",newMapPoints.size());                

    // check point cloud
    std::vector<bool> tmp_status;

    if (!mapFilter->checkFrontCameraPoints(newMapPoints, frame->m_pose, tmp_status))
    {
        // not good triangulation : do not add key frame
        LOG_INFO(" not good triangulation : do not add key frame ");
        return false;
    }

    //filter point cloud
    std::vector<SRef<CloudPoint>> filteredPoints;
    mapFilter->filterPointCloud(newMapPoints, tmp_status, filteredPoints);

    LOG_INFO(" filtered point size: {}" ,filteredPoints.size());

    referenceKeyFrame->addVisibleMapPoints(filteredPoints);
    poseGraph->getMap()->addCloudPoints(filteredPoints);

    SRef<Keyframe> newKeyFrame = xpcf::utils::make_shared<Keyframe>(view, frame->getDescriptors(), newIndex, frame->m_pose, frame->getKeyPoints());

    newKeyFrame->addVisibleMapPoints(frame->getCommonMapPointsWithReferenceKeyFrame());
    newKeyFrame->addVisibleMapPoints(filteredPoints);

    // update visibility of common points
    std::vector<SRef<CloudPoint>> &commonPoints = frame->getCommonMapPointsWithReferenceKeyFrame();
    std::vector<DescriptorMatch> &knownMatches = frame->getKnownMatchesWithReferenceKeyFrame();

    for (int i = 0; i < commonPoints.size(); i++)
    {
        commonPoints[i]->m_visibility[newIndex] = knownMatches[i].getIndexInDescriptorB(); // Clean needed : Try to do this somewhere else
    }

    LOG_INFO(" add new keyframe  with : {} points" ,filteredPoints.size());
    
    poseGraph->addNewKeyFrame(newKeyFrame);

    // update viewer
    viewerGL.AddKeyFrameCameraPose(newKeyFrame->m_pose);

    return true;
}

bool init_mapping(SRef<Image> &view_1, SRef<Image> &view_2, const std::string &path_cloud = std::string())
{

    SRef<Frame> frame1 = createAndInitFrame(view_1);
    SRef<Frame> frame2 = createAndInitFrame(view_2);

    LOG_INFO("   frame 1: {} ", frame1->getKeyPoints().size());
    LOG_INFO("   frame 2: {} " , frame2->getKeyPoints().size()); 

    std::vector<DescriptorMatch> matches;    
    std::vector<SRef<Point2Df>> matched_Keypoints1,matched_Keypoints2;
    std::vector<SRef<Keypoint>> kp1, kp2;

    SRef<DescriptorBuffer> d1 = frame1->getDescriptors();
    SRef<DescriptorBuffer> d2 = frame2->getDescriptors();

    matcher->match(d1, d2, matches);

    kp1 = frame1->getKeyPoints();
    kp2 = frame2->getKeyPoints();

    getMatchedKeyPoints(kp1, kp2, matches, matched_Keypoints1,matched_Keypoints2);
    int vizPoints0 = matches.size();

    // Draw the matches in a dedicated image
    overlay->drawMatchesLines(view_1, view_2, viewerImage1, matched_Keypoints1, matched_Keypoints2, vizPoints0);
    std::vector<DescriptorMatch> ggmatches;

    matchesFilterGeometric->filter(matches, ggmatches, frame1->getKeyPoints(), frame2->getKeyPoints());
    
    LOG_INFO("Output geometric matches: {} ", ggmatches.size());

    std::vector<SRef<Point2Df>> ggmatchedKeypoints1;
    std::vector<SRef<Point2Df>> ggmatchedKeypoints2;

    kp1 = frame1->getKeyPoints();
    kp2 = frame2->getKeyPoints();

    getMatchedKeyPoints(kp1, kp2, ggmatches, ggmatchedKeypoints1, ggmatchedKeypoints2);

    int vizPoints2 = int(ggmatches.size());

    overlay->drawMatchesLines(view_1, view_2, viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2, vizPoints2);

    viewer->display("original matches", viewerImage1, 27, 1280, 480);
    viewer->display("filtred matches (epipolar)", viewerImage3, 27, 1280, 480);

    fundamentalFinder->find(ggmatchedKeypoints1, ggmatchedKeypoints2, F);

    std::vector<Transform3Df> poses;

    fundamentalDecomposer->decompose(F, K, dist, poses);

    Transform3Df pose_canonique;  pose_canonique.setIdentity();

    std::pair<int, int> working_view = std::make_pair(0, 1);

    Transform3Df pose_final;
    std::vector<SRef<CloudPoint>> tempCloud;
    
    if (fullTriangulation(ggmatchedKeypoints1, ggmatchedKeypoints2, ggmatches, working_view, pose_canonique, poses,
                          K, dist, pose_final, tempCloud))
    {

        keyframe_poses.push_back(pose_canonique);
        keyframe_poses.push_back(pose_final);

        SRef<Keyframe> kframe1 = xpcf::utils::make_shared<Keyframe>(view_1, frame1->getDescriptors(), 0, pose_canonique, frame1->getKeyPoints());
        SRef<Keyframe> kframe2 = xpcf::utils::make_shared<Keyframe>(view_2, frame2->getDescriptors(), 1, pose_final, frame2->getKeyPoints());

        kframe1->addVisibleMapPoints(tempCloud);
        kframe2->addVisibleMapPoints(tempCloud);
        poseGraph->initMap(kframe1, kframe2, tempCloud, ggmatches);
        Point3Df gravity;
        float maxDist;
        poseGraph->getMap()->computeGravity(gravity, maxDist);
        nbFrameSinceKeyFrame = 0;

        // update viewer

        viewer3D.resetview(math_vector_3f(gravity.getX(), gravity.getY(), gravity.getZ()), maxDist);
        viewer3D.rotate_180();

        //     viewerGL.AddKeyFrameCameraPose(pose_canonique);
        //    viewerGL.AddKeyFrameCameraPose(pose_final);
        return true;
    }
    else
    {
        LOG_ERROR("can't find good baseline, select another pair of images for triangulation..");

        return false;
    }
}

bool tracking(SRef<Image> &view)
{
    nbFrameSinceKeyFrame++;
    SRef<Frame> newFrame = createAndInitFrame(view);
    poseGraph->associateReferenceKeyFrameToFrame(newFrame);
    newFrame->setNumberOfFramesSinceLastKeyFrame(nbFrameSinceKeyFrame);
    std::vector<DescriptorMatch> new_matches, new_matches_filtred;

    SRef<Keyframe> referenceKeyFrame = newFrame->getReferenceKeyFrame();
    SRef<DescriptorBuffer> d1 = referenceKeyFrame->getDescriptors();
    SRef<DescriptorBuffer> d2 = newFrame->getDescriptors();

    matcher->match(d1, d2, new_matches);
    matchesFilterGeometric->filter(new_matches, new_matches_filtred, referenceKeyFrame->getKeyPoints(), newFrame->getKeyPoints());

    std::vector<SRef<Keypoint>> kp1, kp2;
    std::vector<SRef<Point2Df>> current_kp1;
    std::vector<SRef<Point2Df>> current_kp2;

    kp1 = referenceKeyFrame->getKeyPoints();
    kp2 = newFrame->getKeyPoints();
    getMatchedKeyPoints(kp1, kp2, new_matches_filtred, current_kp1, current_kp2);

    overlay->drawMatchesLines(referenceKeyFrame->m_view, view, currentMatchImage, current_kp1, current_kp2, current_kp1.size());
    viewer->display("current matches", currentMatchImage, &escape_key, 1280, 480);

    std::vector<SRef<Point2Df>> pt2d;
    std::vector<SRef<Point3Df>> pt3d;
    std::vector<SRef<CloudPoint>> foundPoints;
    std::vector<DescriptorMatch> foundMatches;
    std::vector<DescriptorMatch> remainingMatches;

    corr2D3DFinder->find(referenceKeyFrame->getVisibleMapPoints(), referenceKeyFrame->m_idx, new_matches_filtred, newFrame->getKeyPoints(), foundPoints, pt3d, pt2d, foundMatches, remainingMatches);

    std::vector<SRef<Point2Df>> imagePoints_inliers;
    std::vector<SRef<Point3Df>> worldPoints_inliers;

    Transform3Df pose_current;

    if (PnP->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, pose_current) == FrameworkReturnCode::_SUCCESS)
    {
        LOG_INFO(" pnp inliers size: {} / {}",worldPoints_inliers.size(), pt3d.size());

        newFrame->m_pose = pose_current.inverse();
        frame_poses.push_back(newFrame->m_pose);
        // triangulate with the first keyframe !

        std::vector<SRef<CloudPoint>>cloud_t;
        double reproj_error = mapper->triangulate(current_kp1, current_kp2, new_matches_filtred,std::make_pair<int,int>(0,2),
                                                  referenceKeyFrame->m_pose, pose_current, K, dist, cloud_t);

        cloud_current = xpcf::utils::make_shared<std::vector<SRef<CloudPoint>>>() ;
        cloud_current->insert(cloud_current->end(), cloud_t.begin()  , cloud_t.end()) ;
        /*
        cloud_current->resize(cloud_t.size());
        for(int k = 0; k < cloud_t.size(); ++k){
            std::vector<int>visibility = std::vector<int>(50, -1);
           (*cloud_current)[k] = xpcf::utils::make_shared<CloudPoint>(cloud_t[k]->getX(),cloud_t[k]->getY(),cloud_t[k]->getZ(),
                                                                      0,0,0,0.0,visibility);
        }*/

        LOG_INFO(" cloud current size: {}", cloud_current->size());
        
        return true;
        
        }else{
           // std::cout<<"new keyframe creation.."<<std::endl;
            return false;
    }
}

void ogl_callback_idle_static()
{
    if (exit_)
    {
        exit(0);
    }

    if (triangulation_first)
    {
        std::string path_cloud = output_debug_folder_path+"old_points.txt";

        if (init_mapping(static_views[3], static_views[4], path_cloud))
        {

            triangulation_first = false;
            LOG_INFO("Init Mapping done");
        }
    }

    if (processing && !triangulation_first)
    {
        viewer->display("Frame to track", static_views[frame_counter]);
        tracking(static_views[frame_counter]);
        ++frame_counter;

        if (frame_counter >= static_views.size() - 1)
        {
            frame_counter = frame_begin;
            frame_poses.clear();
        }
    }
}

void ogl_callback_resize(int _w, int _h)
{
    w = _w;
    h = _h;
}

void ogl_callback_mouse_motion(int x, int y)
{
    y = h - y;
    viewer3D.mouse_move(x, y);
}

void ogl_callback_mouse_function(int button, int state, int x, int y)
{
    y = h - y;
    int zoom = 10;
    Mouse::button b = Mouse::NONE;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        b = Mouse::ROTATE;
        viewer3D.mouse(x, y, b);
    }
    else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        b = Mouse::MOVEXY;
        viewer3D.mouse(x, y, b);
    }
    else if ((button & 3) == 3)
    {
        viewer3D.mouse_wheel(zoom);
    }
    else if ((button & 4) == 4)
    {
        viewer3D.mouse_wheel(-zoom);
    }
}


void ogl_callback_draw()
{
    if (poseGraph->getMap()->getPointCloud()->size() > 0)
    {
        glEnable(GL_NORMALIZE);
        glEnable(GL_DEPTH_TEST);

        viewer3D.set_viewport(0, 0, w, h);
        viewer3D.setup();
        viewer3D.use_light(false);

        glClearColor(1, 1, 1, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_CULL_FACE);

        float color_cloud0[3] = {1.0,0.0,0.0};
        float color_cloud1[3] = {0.0,0.0,1.0};

        float color_keyframe[3] = {0.0,0.0,1.0};
        float color_frame[3] = {0.0,1.0,0.0};

        float radius_cloud = 1.25;
        float radius_camera = 1.0;

        SRef<std::vector<SRef<CloudPoint>>> cloud_temp = poseGraph->getMap()->getPointCloud();

        drawing_cloud(cloud_temp, radius_cloud, color_cloud0);
        drawing_cameras(keyframe_poses, radius_camera, color_keyframe);

        if(frame_poses.size()>0){
            drawing_cameras(frame_poses, radius_camera, color_frame);
            drawing_cloud(cloud_current, radius_cloud, color_cloud1);
        }

        glutSwapBuffers();
        glutPostRedisplay();
    }
}

int printHelp()
{
    std::cout << "You should add a SlamConfig.txt file such as: " << std::endl;
    std::cout << "        > SolARSample slamConfig.txt  " << std::endl;

    return 1;
}

int main(int argc, char *argv[])
{

    LOG_ADD_LOG_TO_CONSOLE();

    std::string configFile;
    output_debug_folder_path = ""; //default debug folder path
    
    //Test input parameters
    if (argc >= 2)
    {
        configFile = std::string(argv[1]);
    }

    if(!is_file_exist(configFile.c_str())){
        printHelp();
        return -1;
    }

    init(configFile);
    
    //opengl initialization and run
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(1280,1024);
    glutCreateWindow("Sample-Slam");

    //glut callback function to overide behavior
    glutDisplayFunc(ogl_callback_draw);
    glutKeyboardFunc(ogl_callback_keyBoard);
    glutMouseFunc(ogl_callback_mouse_function);
    glutMotionFunc(ogl_callback_mouse_motion);
    glutReshapeFunc(ogl_callback_resize);
    glutIdleFunc(ogl_callback_idle_static);

    glutMainLoop();

    return 0;
}
