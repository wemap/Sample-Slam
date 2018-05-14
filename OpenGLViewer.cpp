#include "OpenGLViewer.h"
#include "freeglut.h"
#include "gl_camera.h"

OpenGLViewer * OpenGLViewer::m_instance = NULL ;


OpenGLViewer::OpenGLViewer()
{
    m_instance = this ;
    m_pointCloud = NULL ;
}

OpenGLViewer::~OpenGLViewer()
{

}


void OpenGLViewer::InitViewer(int resolutionX, int resolutionY)
{
    m_resolutionX = resolutionX ;
    m_resolutionY = resolutionY ;

    char *myargv [1];
    int myargc=1;
    myargv [0]=strdup ("OpenGLViewer");
    glutInit(&myargc, myargv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    glutInitWindowSize(resolutionX, resolutionY);

    glutCreateWindow("OpenGLViewer");
    glutDisplayFunc(Render);
    glutKeyboardFunc(KeyBoard);
    glutMouseFunc(MouseState);
    glutMotionFunc(MouseMotion);
    glutReshapeFunc(ResizeWindow);
    glutIdleFunc(MainLoop);
    glutMainLoop();

}


void OpenGLViewer::SetRealCameraPose(Transform3Df & m)
{
    m_realCameraPose = m ;
}
void OpenGLViewer::SetPointCloudToDisplay(SRef<std::vector<SRef<CloudPoint>>> pointCloud)
{
    m_pointCloud = pointCloud ;
}


void OpenGLViewer::OnMainLoop()
{
   // std::cout << "main loop "  << std::endl;
   callBackIdle() ;
}




void OpenGLViewer::OnRender()
{
   // std::cout << " render " << std::endl  ;

    bool drawing = (m_pointCloud != NULL) ;
    if(drawing){
         glEnable(GL_NORMALIZE);
         glEnable(GL_DEPTH_TEST);

         m_glcamera.set_viewport(0, 0, m_resolutionX, m_resolutionY);
         m_glcamera.setup();
         m_glcamera.use_light(false);

         glClearColor(1, 1, 1, 1);
         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
         glDisable(GL_CULL_FACE);
         glPushMatrix();
     //	glPointSize(3.25f);

         glBegin(GL_POINTS);
         for (unsigned int i = 0; i < (*m_pointCloud).size(); ++i) {
             glColor3f(0.0, 0.0, 1.0);
             cv::Vec3f v = cv::Vec3f((*m_pointCloud)[i]->getX(), (*m_pointCloud)[i]->getY(), (*m_pointCloud)[i]->getZ());
     //		v *= sc;
             glVertex3f(v[0], v[1], v[2]);
         }
         glEnd();

         // draw  camera pose !

         glPushMatrix();
         DrawPhysicalCamera(m_realCameraPose, cv::Vec3f(1.0, 0.0, 0.0), 1, false);
         glPopMatrix();
         glutSwapBuffers();
         glutPostRedisplay();
    }
}


void OpenGLViewer::OnResizeWindow(int _w, int _h)
{
    m_resolutionX = _w;
    m_resolutionY = _h;
}

void OpenGLViewer::OnKeyBoard(unsigned char key, int x, int y)
{
   callbackKeyBoard(key) ;
}


void OpenGLViewer::OnMouseMotion(int x, int y)
{
    y = m_resolutionY - y;
    m_glcamera.mouse_move(x, y);

}
void OpenGLViewer::OnMouseState(int button, int state, int x, int y)
{
    y = m_resolutionY - y;
    int zoom = 3;
    Mouse::button b = Mouse::NONE;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::ROTATE;
        m_glcamera.mouse(x, y, b);

    }
    else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::MOVEXY;
        m_glcamera.mouse(x, y, b);
    }
    else if ((button & 3) == 3) {

        m_glcamera.mouse_wheel(zoom);
    }
    else if ((button & 4) == 4) {

        m_glcamera.mouse_wheel(-zoom);
    }
}



void OpenGLViewer::DrawPhysicalCamera(Transform3Df & m, cv::Vec3f&color, float scale, bool check) {

    rigid_motion<float> camPose ;
    camPose.m_rotation(0, 0) = m(0, 0);
    camPose.m_rotation(0, 1) = m(0, 1);
    camPose.m_rotation(0, 2) = m(0, 2);

    camPose.m_rotation(1, 0) = m(1, 0);
    camPose.m_rotation(1, 1) = m(1, 1);
    camPose.m_rotation(1, 2) = m(1, 2);

    camPose.m_rotation(2, 0) = m(2, 0);
    camPose.m_rotation(2, 1) = m(2, 1);
    camPose.m_rotation(2, 2) = m(2, 2);

    camPose.m_translation[0] = m(0, 3) * scale;
    camPose.m_translation[1] = m(1, 3) * scale;
    camPose.m_translation[2] = m(2, 3) * scale;


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


