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


void drawCamera(rigid_motion<float>&camPose, cv::Vec3f&color, float scale, bool check) {
    double diameter = 0.01f * scale;
    float offset = 0.075f *scale;
    GLfloat line_width = 1.f *scale;
    math_vector_3f origin[4], pos[4];

    if (check) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (i == j)camPose.m_rotation(i, j) = 1.f;
                else camPose.m_rotation(i, j) = 0.f;
            }
        }
    }

    origin[0] = math_vector_3f(offset, offset, 2.f*offset);
    origin[1] = math_vector_3f(-offset, offset, 2.f*offset);
    origin[2] = math_vector_3f(-offset, -offset, 2.f*offset);
    origin[3] = math_vector_3f(offset, -offset, 2.f*offset);

    for (int c = 0; c < 4; ++c)
        pos[c] = camPose.apply(origin[c]);



    GLUquadric * point = gluNewQuadric();
    for (int i = 0; i < 4; ++i) {
        glPopMatrix();
        glPushMatrix();
        glColor3f(color[0], color[1], color[2]);
        glTranslatef(pos[i][0], pos[i][1], pos[i][2]);
        gluSphere(point, diameter*0.5, 30, 30);
        glPopMatrix();
    }
    gluDeleteQuadric(point);

    for (int i = 0; i < 4; ++i) {
        glLineWidth(line_width);
        glColor3f(color[0], color[1], color[2]);
        glBegin(GL_LINES);
        glVertex3f(camPose.m_translation[0], camPose.m_translation[1], camPose.m_translation[2]);
        glVertex3f(pos[i][0], pos[i][1], pos[i][2]);
        glEnd();
    }


    glLineWidth(line_width);
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINES);
    glVertex3f(pos[0][0], pos[0][1], pos[0][2]);
    glVertex3f(pos[1][0], pos[1][1], pos[1][2]);
    glEnd();

    glLineWidth(line_width);
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINES);
    glVertex3f(pos[1][0], pos[1][1], pos[1][2]);
    glVertex3f(pos[2][0], pos[2][1], pos[2][2]);
    glEnd();

    glLineWidth(line_width);
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINES);
    glVertex3f(pos[2][0], pos[2][1], pos[2][2]);
    glVertex3f(pos[3][0], pos[3][1], pos[3][2]);
    glEnd();

    glLineWidth(line_width);
    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_LINES);
    glVertex3f(pos[3][0], pos[3][1], pos[3][2]);
    glVertex3f(pos[0][0], pos[0][1], pos[0][2]);
    glEnd();
}
void solarPoseToRigidMotion(SRef<Pose>&m, rigid_motion<float>&rm, float scale) {
    rm.m_rotation(0, 0) = m->m_poseTransform(0, 0);
    rm.m_rotation(0, 1) = m->m_poseTransform(0, 1);
    rm.m_rotation(0, 2) = m->m_poseTransform(0, 2);

    rm.m_rotation(1, 0) = m->m_poseTransform(1, 0);
    rm.m_rotation(1, 1) = m->m_poseTransform(1, 1);
    rm.m_rotation(1, 2) = m->m_poseTransform(1, 2);

    rm.m_rotation(2, 0) = m->m_poseTransform(2, 0);
    rm.m_rotation(2, 1) = m->m_poseTransform(2, 1);
    rm.m_rotation(2, 2) = m->m_poseTransform(2, 2);

    rm.m_translation[0] = m->m_poseTransform(0, 3) * scale;
    rm.m_translation[1] = m->m_poseTransform(1, 3) * scale;
    rm.m_translation[2] = m->m_poseTransform(2, 3) * scale;
}
void resize(int _w, int _h) {
    w = _w;
    h = _h;
}
void mm(int x, int y)
{
    y = h - y;
    g_camera.mouse_move(x, y);

}
void mb(int button, int state, int x, int y)
{
    y = h - y;
    int zoom = 3;
    Mouse::button b = Mouse::NONE;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::ROTATE;
        g_camera.mouse(x, y, b);

    }
    else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::MOVEXY;
        g_camera.mouse(x, y, b);
    }
    else if ((button & 3) == 3) {

        g_camera.mouse_wheel(zoom);
    }
    else if ((button & 4) == 4) {

        g_camera.mouse_wheel(-zoom);
    }
}
void draw() {
   if(drawing){
        glEnable(GL_NORMALIZE);
        glEnable(GL_DEPTH_TEST);

        g_camera.set_viewport(0, 0, w, h);
        g_camera.setup();
        g_camera.use_light(false);

        glClearColor(1, 1, 1, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_CULL_FACE);
        glPushMatrix();
    //	glPointSize(3.25f);
        glBegin(GL_POINTS);
        for (unsigned int i = 0; i <gcloud.size(); ++i) {
            glColor3f(0.0, 0.0, 1.0);
            cv::Vec3f v = cv::Vec3f(gcloud[i]->getX(), gcloud[i]->getY(), gcloud[i]->getZ());
    //		v *= sc;
            glVertex3f(v[0], v[1], v[2]);
        }
        glEnd();

        // draw  camera pose !

        glPushMatrix();
        if(processing){
            double sc = 1.0;
            rigid_motion<float>cam_temp, cam_canonique, cam_current;
            solarPoseToRigidMotion(pose_current, cam_current, sc);
            drawCamera(cam_current, cv::Vec3f(0.0, 0.0, 1.0), 4 * sc, false);
        }
        /*
        double sc = 1.0;
        rigid_motion<float>cam_temp, cam_canonique, cam_current;

        solarPoseToRigidMotion(pose_final, cam_temp, sc);
        solarPoseToRigidMotion(pose_canonique, cam_canonique, sc);

        drawCamera(cam_temp, cv::Vec3f(1.0, 0.0, 0.0), 4 * sc, false);
        drawCamera(cam_canonique, cv::Vec3f(0.0, 1.0, 0.0), 4 * sc, false);
        */

        glPopMatrix();
        glutSwapBuffers();
        glutPostRedisplay();
   }
}
void fillPoseCanonique(SRef<Pose>&pcano){

    pcano = sptrnms::make_shared<Pose>();

    pcano->m_poseTransform(0,0) = 1.0;
    pcano->m_poseTransform(0,1) = 0.0;
    pcano->m_poseTransform(0,2) = 0.0;
    pcano->m_poseTransform(0,3) = 0.0;

    pcano->m_data[0][0] = 1.0;
    pcano->m_data[0][1] = 0.0;
    pcano->m_data[0][2] = 0.0;
    pcano->m_data[0][3] = 0.0;


    pcano->m_poseTransform(1,0) = 0.0;
    pcano->m_poseTransform(1,1) = 1.0;
    pcano->m_poseTransform(1,2) = 0.0;
    pcano->m_poseTransform(1,3) = 0.0;

    pcano->m_data[1][0] = 0.0;
    pcano->m_data[1][1] = 1.0;
    pcano->m_data[1][2] = 0.0;
    pcano->m_data[1][3] = 0.0;


    pcano->m_poseTransform(2,0) = 0.0;
    pcano->m_poseTransform(2,1) = 0.0;
    pcano->m_poseTransform(2,2) = 1.0;
    pcano->m_poseTransform(2,3) = 0.0;

    pcano->m_data[2][0] = 0.0;
    pcano->m_data[2][1] = 0.0;
    pcano->m_data[2][2] = 1.0;
    pcano->m_data[2][3] = 0.0;



    pcano->m_poseTransform(3,0) = 0.0;
    pcano->m_poseTransform(3,1) = 0.0;
    pcano->m_poseTransform(3,2) = 0.0;
    pcano->m_poseTransform(3,3) = 1.0;

    pcano->m_data[3][0] = 0.0;
    pcano->m_data[3][1] = 0.0;
    pcano->m_data[3][2] = 0.0;
    pcano->m_data[3][3] = 1.0;



}
void keyBord(unsigned char key,
             int x, int y){
    switch (key) {
    case 's': {
        saving_images = !saving_images;
        std::cout << "image saved: " << views.size() << std::endl;
        break;

    }
    case 't': {
        triangulation_first = !triangulation_first;
        std::cout << "tringulaiton started: " << triangulation_first << std::endl;
        break;
    }
    case 'd': {
        drawing = !drawing;
        std::cout << "drawing started: " << drawing << std::endl;
        break;
    }
    case 'p': {
        processing = !processing;
        std::cout << "processing started: " << processing << std::endl;
        break;
    }

    default:
        break;
    }
}
void init(){
    // component creation

       xpcf::ComponentFactory::createComponent<SolARCameraOpencv>(gen(input::devices::ICamera::UUID), camera);
       xpcf::ComponentFactory::createComponent<SolARImageLoaderOpencv>(gen(image::IImageLoader::UUID ), imageLoader);

       xpcf::ComponentFactory::createComponent<SolARKeypointDetectorNonFreeOpencv>(gen(features::IKeypointDetector::UUID ), keypointsDetector);
       xpcf::ComponentFactory::createComponent<SolARDescriptorsExtractorSURF64Opencv>(gen(features::IDescriptorsExtractor::UUID ), descriptorExtractor);

   //    xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherKNNOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);

       xpcf::ComponentFactory::createComponent<SolARDescriptorMatcherRadiusOpencv>(gen(features::IDescriptorMatcher::UUID), matcher);

       xpcf::ComponentFactory::createComponent<SolARSideBySideOverlayOpencv>(gen(display::ISideBySideOverlay::UUID ), overlay);
       xpcf::ComponentFactory::createComponent<SolAR2DOverlayOpencv>(gen(display::I2DOverlay::UUID ), overlay2d);

       xpcf::ComponentFactory::createComponent<SolARImageViewerOpencv>(gen(display::IImageViewer::UUID ), viewer);
       xpcf::ComponentFactory::createComponent<SolARGeometricMatchesFilterOpencv>(gen(features::IMatchesFilter::UUID ), matchesFilterGeometric);
       xpcf::ComponentFactory::createComponent<SolARFundamentalMatrixEstimationOpencv>(gen(solver::pose::I2DTransformFinder::UUID ), fundamentalFinder);
       xpcf::ComponentFactory::createComponent<SolARSVDFundamentalMatrixDecomposerOpencv>(gen(solver::pose::I2DTO3DTransformDecomposer::UUID ), fundamentalDecomposer);
       xpcf::ComponentFactory::createComponent<SolARSVDTriangulationOpencv>(gen(solver::map::ITriangulator::UUID ), mapper);
       xpcf::ComponentFactory::createComponent<SolARMapperOpencv>(gen(solver::map::IMapper::UUID), poseGraph);
       xpcf::ComponentFactory::createComponent<SolARPoseEstimationPnpOpencv>(gen(solver::pose::I3DTransformFinder::UUID), PnP);

       keypointsDetector->setType(features::KeypointDetectorType::SURF);       
       // load camera parameters from yml input file
       std::string cameraParameters = std::string("D:/AmineSolar/source/slam/build-SolARTriangulationSample/mycamera_calibration0.yml");

       camera->loadCameraParameters(cameraParameters);
       PnP->setCameraParameters(camera->getIntrinsicsParameters(), camera->getDistorsionParameters());
       K = camera->getIntrinsicsParameters();
       dist = camera->getDistorsionParameters();

       camera->start(0);
}
bool processFrame(SRef<Image>&img,
                  std::vector< SRef<Keypoint>>&kpts,
                  SRef<DescriptorBuffer>&desc){
    kpts.clear();
    Sizei ss1 = img->getSize();
    keypointsDetector->detect(img, kpts);
    descriptorExtractor->extract(img, kpts, desc);
    return true;

}
bool debug_reprojection(){
    init();
    cv::namedWindow("debug_window",0);
    cv::Mat view = cv::imread("D:/AmineSolar/mySLAM/my_validdata_2/0002.png");
    if (!view.empty()) {
        std::cout << " --------------------<debug 2d-3d reprojection>" << std::endl;
        std::cout << "	#loading 2d-3d correspondances:" << std::endl;
        int corr_no = 1817;
        std::vector<SRef<Point2Df>>pt2d_temp;
        std::vector<SRef<Point3Df>>pt3d_temp;
        pt3d_temp.resize(corr_no); pt2d_temp.resize(corr_no);
        std::string corr_path = "D:/corr_cvSlam.txt";
        std::ifstream oxCorr(corr_path);
        std::string dummy;
        for (int i = 0; i < corr_no; ++i) {
            float v[5];
            for (int j = 0; j < 5; ++j) {
                oxCorr >> dummy;
                v[j] = std::stof(dummy);
            }
            pt3d_temp[i] = sptrnms::make_shared<Point3Df>(v[0], v[1], v[2]);
            pt2d_temp[i] = sptrnms::make_shared<Point2Df>(v[3], v[4]);
     //       std::cout << " pt3d: " << pt3d_temp[i]->getX() << " "<<pt3d_temp[i]->getY()<<" "<<
      //                    pt3d_temp[i]->getZ()<<"   pt2d: " << pt2d_temp[i]->getX()<<" "<< pt2d_temp[i]->getY() << std::endl;
       //     cv::waitKey(0);
        }
        std::string thirdImagePath = "D:/AmineSolar/source/slam/build-SolARTriangulationSample/0002.png";
        if (imageLoader->loadImage(thirdImagePath,view_current) != FrameworkReturnCode::_SUCCESS)
        {
           LOG_ERROR("Cannot load image with path {}", std::string());
           return -1;
        }

        SRef<Pose> pose_pnp;
        PnP->estimate(pt2d_temp,pt3d_temp,pose_current);

        std::cout<<" pose solar from pnp: "<<std::endl;

        for(int ii = 0; ii < 4; ++ii){
            for(int jj = 0; jj < 4; ++jj){
                std::cout<<pose_current->m_data[ii][jj]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl<<std::endl;
        std::cout<<"  number of correspondances: "<<pt2d_temp.size()<<"  "<<pt3d_temp.size()<<std::endl;
        PnP->reproject(view_current,pose_current,K,dist,pt2d_temp,pt3d_temp);
        viewer->display("2d-3d corr", view_current, &escape_key,640,480);

        cv::waitKey(0);

        /*
        std::pair<int, int>working_pair = std::make_pair<int, int>(0, 1);
        if (vo->computePoseFromPnP(view, working_pair, rvec, t, R, pt3d_temp, pt2d_temp, paramPnP)) {
            current_pose = cv::Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0),
                R(1, 0), R(1, 1), R(1, 2), t(1),
                R(2, 0), R(2, 1), R(2, 2), t(2));

            std::cout << "		# pose: " << t(0) << " " << t(1) << " " << t(2) << std::endl;
            std::cout << "		# tracking: good" << std::endl;
            cv::waitKey(0);
            return true;
        }
        else {
            std::cout << "		# tracking: bad" << std::endl;
            return false;
        }
        */
    }

}
void computeGravity(std::vector<SRef<CloudPoint>>&cloud,cv::Vec3f&grav, float& maxDist){
   grav =  cv::Vec3f(0, 0, 0);
   maxDist  =0.f;

    cv::Vec3f vr_temp(0, 0, 0);
    int count = 0;
    for (int i = 0; i < cloud.size(); ++i) {
        vr_temp = cv::Vec3f(cloud[i]->getX(), cloud[i]->getX(), cloud[i]->getY());
        grav = grav + vr_temp;
        count++;
    }
    maxDist = 0;
    if (count > 0) {
        grav= (1.f / (float)count)*gravity;
        for (int i = 0; i < cloud.size(); ++i) {
            vr_temp = cv::Vec3f(cloud[i]->getX(),cloud[i]->getY(), cloud[i]->getZ());
            cv::Vec3f temp = vr_temp - grav;
            maxDist = MAX(maxDist, (float)norm(vr_temp - grav));
        }
    }
}
bool init_mapping(SRef<Image>&view_1,SRef<Image>&view_2, bool verbose){
 processFrame(views[0], keypoints1, descriptors1);
 processFrame(views[1], keypoints2, descriptors2);

 std::cout<<"--< Descriptors matching: "<<std::endl;
 matcher->match(descriptors1, descriptors2, matches);
 std::cout<<"     #original matches: "<<matches.size()<<std::endl;
 if(verbose){
     int vizPoints0 = int(matches.size());
    matchedKeypoints1.clear();
    matchedKeypoints2.clear();
    for( int i = 0; i < matches.size(); i++ ){
     matchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(keypoints1[ matches[i].getIndexInDescriptorA()]->getX(),keypoints1[ matches[i].getIndexInDescriptorA()]->getY()));
     matchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(keypoints2[ matches[i].getIndexInDescriptorB()]->getX(),keypoints2[ matches[i].getIndexInDescriptorB()]->getY()));
     }
    // Draw the matches in a dedicated image
     overlay->drawMatchesLines(views[0], views[1], viewerImage1, matchedKeypoints1, matchedKeypoints2, vizPoints0);
    int vizPoints1 = int(gmatches.size());
    overlay->drawMatchesLines(views[0],views[1], viewerImage2, gmatchedKeypoints1, gmatchedKeypoints2,vizPoints1);
 }
 matchesFilterGeometric->filter(matches,ggmatches,keypoints1, keypoints2);

 if(verbose){
    std::cout<<"    #filtred matches: "<<ggmatches.size()<<std::endl;
    ggmatchedKeypoints1.clear();
    ggmatchedKeypoints2.clear();
    for( int i = 0; i < ggmatches.size(); i++ ){
       ggmatchedKeypoints1.push_back(xpcf::utils::make_shared<Point2Df>(keypoints1[ggmatches[i].getIndexInDescriptorA()]->getX(),keypoints1[ ggmatches[i].getIndexInDescriptorA()]->getY()));
       ggmatchedKeypoints2.push_back(xpcf::utils::make_shared<Point2Df>(keypoints2[ggmatches[i].getIndexInDescriptorB()]->getX(),keypoints2[ ggmatches[i].getIndexInDescriptorB()]->getY()));
    }
    int vizPoints2 = int(ggmatches.size());
    overlay->drawMatchesLines(views[0], views[1], viewerImage3, ggmatchedKeypoints1, ggmatchedKeypoints2,vizPoints2);
    viewer->display("original matches", viewerImage1, &escape_key,1280,480);
    viewer->display("filtred matches (epipolar)", viewerImage3, &escape_key,1280,480);
    }
   std::cout<<"--<Triangulation: "<<std::endl;
   std::cout<<"    #Fundamental calculation"<<std::endl;
   fundamentalFinder->find(ggmatchedKeypoints1, ggmatchedKeypoints2,F);
   std::cout<<"    #Fundamental decomposition"<<std::endl;
   fundamentalDecomposer->decompose(F,K,dist,poses);
   fillPoseCanonique(pose_canonique);

   std::cout<<"    #Full triangulation"<<std::endl;
   std::cout<<"    #pt2d1 size: "<<ggmatchedKeypoints1.size()<<std::endl;
   std::cout<<"    #pt2d2 size: "<<ggmatchedKeypoints2.size()<<std::endl;
   std::cout<<"    #matches size: "<<ggmatches.size()<<std::endl;

   std::pair<int,int>working_view = std::make_pair(0,1);
   std::cout<<"   #full triangulation+filtering"<<std::endl;
     if(mapper->triangulateFull(ggmatchedKeypoints1,ggmatchedKeypoints2,ggmatches, working_view,pose_canonique,poses,K,dist,pose_final,gcloud)){
         std::cout<<"   #final cloud size: "<<gcloud.size()<<std::endl;
         computeGravity(gcloud,gravity, maxDist);
         std::cout<<"gravity center: "<<gravity<<std::endl;
         g_camera.resetview(math_vector_3f(gravity[0], gravity[1], gravity[2]), maxDist);
         g_camera.rotate_180();
         kframe1 = xpcf::utils::make_shared<Keyframe>(view_1,descriptors1,0,pose_canonique,keypoints1);
         kframe2 = xpcf::utils::make_shared<Keyframe>(view_2,descriptors2,1,pose_final,keypoints2);
         std::cout<<"   #map init"<<std::endl;
         poseGraph->initMap(kframe1,kframe2,gcloud,ggmatches);
         std::cout<<"--<Pose graph: "<<std::endl;
         std::cout<<"     # kframe(t): "<<kframe1->m_idx<<std::endl;
         std::cout<<"     # kframe(t+1): "<<kframe2->m_idx<<std::endl;
         return true;
     }
     else{
         std::cerr<<"can't find good baseline, select another pair of images for triangulation.."<<std::endl;
         return false;
     }

}
bool tracking(SRef<Image>&view, const int kframe_idx, bool verbose){
    std::vector<DescriptorMatch>new_matches, new_matches_filtred;
    processFrame(view,keypoints3,descriptors3);
    matcher->match(descriptors2, descriptors3, new_matches);
    matchesFilterGeometric->filter(new_matches,new_matches_filtred,keypoints2, keypoints3);

    std::vector<SRef<Point2Df>>pt2d;
    std::vector<SRef<Point3Df>>pt3d;
    poseGraph->find2D3DCorrespondances(kframe_idx,new_matches_filtred,keypoints3,pt2d,pt3d);
    if(PnP->estimate(pt2d,pt3d,pose_current) == FrameworkReturnCode::_SUCCESS){
        if(verbose){
            PnP->reproject(view,pose_current,K,dist,pt2d,pt3d);
        }
        return true;
    }else{
        return false;
    }
}
void idle(){
    camera->getNextImage(view_current);
    viewer->display("view current", view_current, &escape_key);
    cv::waitKey(1);
    if(saving_images){
        views.push_back(view_current);
        saving_images=false;
    }
    if (triangulation_first&& views.size()>1) {
        init_mapping(views[0],views[1], true);
        triangulation_first = false;
    }
    if(processing){
       std::cout<<"TRACKING THREAD"<<std::endl;
       tracking(view_current,1 /*kframe_idx*/,true);
    }
}
int main(int argc, char* argv[]){
//    debug_reprojection();
    init();
    glutInit(&argc, argv);
    //	capture_data();
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutCreateWindow("window");
    glutDisplayFunc(draw);
    glutKeyboardFunc(keyBord);
    glutMouseFunc(mb);
    glutMotionFunc(mm);
    glutReshapeFunc(resize);
    glutIdleFunc(idle);
    glutMainLoop();


    /*
    glutInit(&argc, argv);
    if (argc == 4) {
        std::string firstImagePath = std::string(argv[1]);
        std::string secondImagePath = std::string(argv[2]);
        std::string cameraParameters = std::string(argv[3]);
        init(cameraParameters);
        run(firstImagePath, secondImagePath);
    }
    //	capture_data();
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutCreateWindow("window");
    glutDisplayFunc(draw);
    glutMouseFunc(mb);
    glutMotionFunc(mm);
    glutReshapeFunc(resize);
    glutIdleFunc(idle);
    glutMainLoop();
    */

}


