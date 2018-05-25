#ifndef OPENGLVIEWER_H
#define OPENGLVIEWER_H

#include "gl_camera.h"
#include "datastructure/MathDefinitions.h"
#include "SolARSVDTriangulationOpencv.h"
#include <functional>

using namespace SolAR;
using namespace SolAR::datastructure;

class OpenGLViewer
{
public :

    /// \brief defualt constructor
    OpenGLViewer() ;

    /// \brief destructor
    ~OpenGLViewer() ;

    /// \brief init viewer with resolution
    void InitViewer(int resolutionX , int resolutionY) ;


    void SetRealCameraPose(Transform3Df & m) ;
	void AddKeyFrameCameraPose(const Transform3Df & m); 
    void SetPointCloudToDisplay(SRef<std::vector<SRef<CloudPoint>>>  pointCloud) ;


    void OnMainLoop() ;
    void OnRender() ;
    void OnResizeWindow(int _w , int _h) ;
    void OnKeyBoard(unsigned char key, int x, int y) ;
    void OnMouseMotion(int x, int y);
    void OnMouseState(int button, int state, int x, int y);



    // to do : this must become private
    std::function<void(unsigned char)>  callbackKeyBoard ;
    std::function<void()>  callBackIdle ;
    gl_camera       m_glcamera;

protected:

    /// \brief  one instance of viewer
    static OpenGLViewer * m_instance ;



private :

    int             m_resolutionX ;
    int             m_resolutionY ;

    SRef<std::vector<SRef<CloudPoint>>>  m_pointCloud ;
	std::vector<Transform3Df> m_keyFramesPoses; 
    Transform3Df    m_realCameraPose ;



    static void MainLoop()
    {
        m_instance->OnMainLoop();
    }
    static void Render()
    {
        m_instance->OnRender();
    }
    static void ResizeWindow(int _w , int _h)
    {
        m_instance->OnResizeWindow(_w, _h);
    }
    static void KeyBoard(unsigned char key, int x, int y)
    {
        m_instance->OnKeyBoard(key, x , y);
    }

    static void MouseMotion(int x, int y)
    {
        m_instance->OnMouseMotion(x,  y);
    }
    static void MouseState(int button, int state, int x, int y)
    {
        m_instance->OnMouseState(button, state, x , y);
    }

    void DrawPhysicalCamera(Transform3Df & m, cv::Vec3f&color, float scale, bool check) ;

} ;



#endif // OPENGLVIEWER_H
