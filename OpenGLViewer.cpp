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

	m_glcamera.resetview(math_vector_3f(0, 0, 0), 1.0f);

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

void OpenGLViewer::AddKeyFrameCameraPose(const Transform3Df & m)
{
	m_keyFramesPoses.push_back(m); 
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
	glEnable(GL_NORMALIZE);
	glEnable(GL_DEPTH_TEST);

	m_glcamera.set_viewport(0, 0, m_resolutionX, m_resolutionY);
	m_glcamera.setup();
	m_glcamera.use_light(false);

	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_CULL_FACE);

//	DrawAxis();

    bool drawing = (m_pointCloud != NULL) ;
    if(drawing)
	{
         glPushMatrix();
        glPointSize(2.25f);
         glBegin(GL_POINTS);
         for (unsigned int i = 0; i < (*m_pointCloud).size(); ++i) {
             glColor3f(0.0, 0.0, 1.0);
             cv::Vec3f v = cv::Vec3f((*m_pointCloud)[i]->getX(), (*m_pointCloud)[i]->getY(), (*m_pointCloud)[i]->getZ());
             glVertex3f(v[0], v[1], v[2]);
         }
         glEnd();

         // draw  camera pose !
         glPushMatrix();
         cv::Vec3f tmpVect=cv::Vec3f(1.0, 0.0, 0.0);
         DrawPhysicalCamera(m_realCameraPose, tmpVect, 1);
         glPopMatrix();
		 for (int i = 0; i < m_keyFramesPoses.size(); i++)
		 {
			 glPushMatrix(); 
			 if (i == 0)
			 {
                 DrawPhysicalCamera(m_keyFramesPoses[i], tmpVect, 1);
			 }
			 else
			 {
                 DrawPhysicalCamera(m_keyFramesPoses[i], tmpVect, 1);
			 }
			 
			 glPopMatrix(); 
		 } 
    }
	glutSwapBuffers();
	glutPostRedisplay();
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


void OpenGLViewer::DrawAxis()
{
	glPushMatrix(); 

	// 0
	glColor3f(1, 1, 0);
	glutSolidSphere(0.05, 10, 10);

	// z 
	glColor3f(0, 0, 1);
	glTranslatef(0, 0, 1); 
	glutSolidSphere(0.05, 10, 10);
	
	// x 
	glPopMatrix(); 
	glPushMatrix(); 
	glColor3f(1, 0, 0);
	glTranslatef(1, 0, 0); 
	glutSolidSphere(0.05, 10, 10);

	// y 
	glPopMatrix();
	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslatef(0, 1, 0);
	glutSolidSphere(0.05, 10, 10);


	// zAxis
	glPopMatrix();
	glPushMatrix();
	glColor3f(0, 0, 1); 
	glutSolidCone(0.1, 1, 10, 10);

	// x Axis
	glPopMatrix();
	glPushMatrix();
	glRotatef(90, 0, 1 , 0);
	glColor3f(1, 0, 0);
	glutSolidCone(0.1, 1, 10, 10);

	// y axis 
	glPopMatrix();
	glPushMatrix();
	glRotatef(-90, 1, 0, 0);
	glColor3f(0, 1, 0);
	glutSolidCone(0.1, 1, 10, 10);


	glPopMatrix(); 
}


void OpenGLViewer::DrawPhysicalCamera(Transform3Df & m, cv::Vec3f&color, float scale) {

    rigid_motion<float> camPose = ConvertTransformToRigidMotion(m, scale);
    
	// Compute frustum corners according to camera transform
	math_vector_3f  transformedCorners[5];
	float offsetCorners = 0.075f *scale;
	transformedCorners[0] = camPose.apply( math_vector_3f(offsetCorners, offsetCorners, 2.f*offsetCorners));
	transformedCorners[1] = camPose.apply(math_vector_3f(-offsetCorners, offsetCorners, 2.f*offsetCorners));
	transformedCorners[2] = camPose.apply(math_vector_3f(-offsetCorners, -offsetCorners, 2.f*offsetCorners));
	transformedCorners[3] = camPose.apply(math_vector_3f(offsetCorners, -offsetCorners, 2.f*offsetCorners));
	transformedCorners[4] = camPose.apply(math_vector_3f(0, 0, 0));


	// draw a sphere at each corner of the frustum
	double cornerDiameter = 0.02f * scale;
	glColor3f(color[0], color[1], color[2]);
	for (int i = 0; i < 5; ++i)
	{
		glPushMatrix();
		glTranslatef(transformedCorners[i][0], transformedCorners[i][1], transformedCorners[i][2]);
		glutSolidSphere(cornerDiameter*0.5, 30, 30);
		glPopMatrix();
	}

	// draw frustum lines
	float line_width = 1.0f *scale;
	glLineWidth(line_width);
    for (int i = 0; i < 4; ++i) 
	{ 
        glBegin(GL_LINES);
        glVertex3f(camPose.m_translation[0], camPose.m_translation[1], camPose.m_translation[2]);
        glVertex3f(transformedCorners[i][0], transformedCorners[i][1], transformedCorners[i][2]);
        glEnd();
    }

    glBegin(GL_LINE_STRIP);
    glVertex3f(transformedCorners[0][0], transformedCorners[0][1], transformedCorners[0][2]);
    glVertex3f(transformedCorners[1][0], transformedCorners[1][1], transformedCorners[1][2]);
	glVertex3f(transformedCorners[2][0], transformedCorners[2][1], transformedCorners[2][2]);
	glVertex3f(transformedCorners[3][0], transformedCorners[3][1], transformedCorners[3][2]);
	glVertex3f(transformedCorners[0][0], transformedCorners[0][1], transformedCorners[0][2]);
    glEnd();
}


rigid_motion<float>  OpenGLViewer::ConvertTransformToRigidMotion(Transform3Df & m, float scalePosition)
{
	rigid_motion<float> camPose; 
	camPose.m_rotation(0, 0) = m(0, 0);
	camPose.m_rotation(0, 1) = m(0, 1);
	camPose.m_rotation(0, 2) = m(0, 2);

	camPose.m_rotation(1, 0) = m(1, 0);
	camPose.m_rotation(1, 1) = m(1, 1);
	camPose.m_rotation(1, 2) = m(1, 2);

	camPose.m_rotation(2, 0) = m(2, 0);
	camPose.m_rotation(2, 1) = m(2, 1);
	camPose.m_rotation(2, 2) = m(2, 2);

	camPose.m_translation[0] = m(0, 3) * scalePosition;
	camPose.m_translation[1] = m(1, 3) * scalePosition;
	camPose.m_translation[2] = m(2, 3) * scalePosition;
	return camPose; 
}


