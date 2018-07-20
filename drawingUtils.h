#ifndef _DRAWING_UTILS_H_
#define _DRAWING_UTILS_H_

#include "constants.h"

void DrawPointCloud(Transform3Df relative_pose)
{
    std::string window_name = "cloud_points";
    cv::namedWindow(window_name, 0);
    cv::resizeWindow(window_name, 640, 480);
    cv::Mat reprojected(cv::Size(640, 480), CV_8UC3);
    reprojected.setTo(0);

    std::vector<cv::Point3f> worldCVPoints;
    std::set<cv::Point3f> worldCVPointsSet;
    std::vector<cv::Point2f> projected3D;

    std::vector<SRef<CloudPoint>> Map = *(poseGraph->getMap()->getPointCloud());

    for (auto point : Map)
    {
        worldCVPoints.push_back(cv::Point3f(point->getX(), point->getY(), point->getZ()));
    }

    if (worldCVPoints.size() != 0)
    {

        CamCalibration intrinsic_param;
        CamDistortion distorsion_param;

        intrinsic_param = camera->getIntrinsicsParameters();
        distorsion_param = camera->getDistorsionParameters();

        cv::Mat m_camMatrix;
        cv::Mat m_camDistorsion;
        m_camMatrix.create(3, 3, CV_32FC1);
        m_camDistorsion.create(5, 1, CV_32FC1);

        m_camDistorsion.at<float>(0, 0) = dist(0);
        m_camDistorsion.at<float>(1, 0) = dist(1);
        m_camDistorsion.at<float>(2, 0) = dist(2);
        m_camDistorsion.at<float>(3, 0) = dist(3);
        m_camDistorsion.at<float>(4, 0) = dist(4);

        m_camMatrix.at<float>(0, 0) = intrinsic_param(0, 0);
        m_camMatrix.at<float>(0, 1) = intrinsic_param(0, 1);
        m_camMatrix.at<float>(0, 2) = intrinsic_param(0, 2);
        m_camMatrix.at<float>(1, 0) = intrinsic_param(1, 0);
        m_camMatrix.at<float>(1, 1) = intrinsic_param(1, 1);
        m_camMatrix.at<float>(1, 2) = intrinsic_param(1, 2);
        m_camMatrix.at<float>(2, 0) = intrinsic_param(2, 0);
        m_camMatrix.at<float>(2, 1) = intrinsic_param(2, 1);
        m_camMatrix.at<float>(2, 2) = intrinsic_param(2, 2);

        // Rotation and Translation from input pose
        cv::Mat Rvec;
        Rvec.create(3, 3, CV_32FC1);
        cv::Mat Tvec;
        Tvec.create(3, 1, CV_32FC1);

        Rvec.at<float>(0, 0) = relative_pose(0, 0);
        Rvec.at<float>(0, 1) = relative_pose(0, 1);
        Rvec.at<float>(0, 2) = relative_pose(0, 2);

        Rvec.at<float>(1, 0) = relative_pose(1, 0);
        Rvec.at<float>(1, 1) = relative_pose(1, 1);
        Rvec.at<float>(1, 2) = relative_pose(1, 2);

        Rvec.at<float>(2, 0) = relative_pose(2, 0);
        Rvec.at<float>(2, 1) = relative_pose(2, 1);
        Rvec.at<float>(2, 2) = relative_pose(2, 2);

        Tvec.at<float>(0, 0) = relative_pose(0, 3);
        Tvec.at<float>(1, 0) = relative_pose(1, 3);
        Tvec.at<float>(2, 0) = relative_pose(2, 3);

        cv::Mat rodrig;
        cv::Rodrigues(Rvec, rodrig);
   
        cv::projectPoints(worldCVPoints, rodrig, Tvec, m_camMatrix, m_camDistorsion, projected3D);

        for (int i = 0; i < projected3D.size(); i++)
        {
            cv::circle(reprojected, projected3D[i], 2.0, cv::Scalar(0, 0, 255), 2.0, 8, 0);
        }

        cv::imshow("cloud_points", reprojected);
    }
}
void DrawPnpMatches(const std::vector<SRef<Point2Df>> &imagePoints, const std::vector<SRef<Point3Df>> &worldPoints, Transform3Df relative_pose)
{

    std::vector<cv::Point2f> imageCVPoints;
    std::vector<cv::Point3f> worldCVPoints;

    for (int i = 0; i < imagePoints.size(); ++i)
    {
        Point2Df point2D = *(imagePoints.at(i));
        Point3Df point3D = *(worldPoints.at(i));
        imageCVPoints.push_back(cv::Point2f(point2D.getX(), point2D.getY()));
        worldCVPoints.push_back(cv::Point3f(point3D.getX(), point3D.getY(), point3D.getZ()));
    }

    std::string window_name = "reprojectedPoints";
    cv::namedWindow(window_name, 0);
    cv::resizeWindow(window_name, 640, 480);
    cv::Mat reprojected(cv::Size(640, 480), CV_8UC3);
    reprojected.setTo(0);

    std::vector<cv::Point2f> projected3D;

    if (worldCVPoints.size() != 0)
    {

        CamCalibration intrinsic_param;
        CamDistortion distorsion_param;

        intrinsic_param = camera->getIntrinsicsParameters();
        distorsion_param = camera->getDistorsionParameters();

        cv::Mat m_camMatrix;
        cv::Mat m_camDistorsion;
        m_camMatrix.create(3, 3, CV_32FC1);
        m_camDistorsion.create(5, 1, CV_32FC1);

        m_camDistorsion.at<float>(0, 0) = dist(0);
        m_camDistorsion.at<float>(1, 0) = dist(1);
        m_camDistorsion.at<float>(2, 0) = dist(2);
        m_camDistorsion.at<float>(3, 0) = dist(3);
        m_camDistorsion.at<float>(4, 0) = dist(4);

        m_camMatrix.at<float>(0, 0) = intrinsic_param(0, 0);
        m_camMatrix.at<float>(0, 1) = intrinsic_param(0, 1);
        m_camMatrix.at<float>(0, 2) = intrinsic_param(0, 2);
        m_camMatrix.at<float>(1, 0) = intrinsic_param(1, 0);
        m_camMatrix.at<float>(1, 1) = intrinsic_param(1, 1);
        m_camMatrix.at<float>(1, 2) = intrinsic_param(1, 2);
        m_camMatrix.at<float>(2, 0) = intrinsic_param(2, 0);
        m_camMatrix.at<float>(2, 1) = intrinsic_param(2, 1);
        m_camMatrix.at<float>(2, 2) = intrinsic_param(2, 2);

        // Rotation and Translation from input pose
        cv::Mat Rvec;
        Rvec.create(3, 3, CV_32FC1);
        cv::Mat Tvec;
        Tvec.create(3, 1, CV_32FC1);

        Rvec.at<float>(0, 0) = relative_pose(0, 0);
        Rvec.at<float>(0, 1) = relative_pose(0, 1);
        Rvec.at<float>(0, 2) = relative_pose(0, 2);

        Rvec.at<float>(1, 0) = relative_pose(1, 0);
        Rvec.at<float>(1, 1) = relative_pose(1, 1);
        Rvec.at<float>(1, 2) = relative_pose(1, 2);

        Rvec.at<float>(2, 0) = relative_pose(2, 0);
        Rvec.at<float>(2, 1) = relative_pose(2, 1);
        Rvec.at<float>(2, 2) = relative_pose(2, 2);

        Tvec.at<float>(0, 0) = relative_pose(0, 3);
        Tvec.at<float>(1, 0) = relative_pose(1, 3);
        Tvec.at<float>(2, 0) = relative_pose(2, 3);


        //draw points
        cv::Mat rodrig;
        cv::Rodrigues(Rvec, rodrig);
        cv::projectPoints(worldCVPoints, rodrig, Tvec, m_camMatrix, m_camDistorsion, projected3D);

        for (int i = 0; i < projected3D.size(); i++)
        {
            cv::circle(reprojected, imageCVPoints[i], 2.0, cv::Scalar(0, 255, 0), 2.0, 8, 0);
            cv::circle(reprojected, projected3D[i], 2.0, cv::Scalar(0, 0, 255), 2.0, 8, 0);
        }

        cv::imshow("reprojectedPoints", reprojected);
    }
}

rigid_motion<float> converting_rigidMotion(Transform3Df &m, float scalePosition)
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

void drawing_pose(Transform3Df &m, float radius, float *color)
{

    rigid_motion<float> camPose = converting_rigidMotion(m, radius);

    // Compute frustum corners according to camera transform
    math_vector_3f transformedCorners[5];
    float offsetCorners = 0.075f * radius;
    transformedCorners[0] = camPose.apply(math_vector_3f(offsetCorners, offsetCorners, 2.f * offsetCorners));
    transformedCorners[1] = camPose.apply(math_vector_3f(-offsetCorners, offsetCorners, 2.f * offsetCorners));
    transformedCorners[2] = camPose.apply(math_vector_3f(-offsetCorners, -offsetCorners, 2.f * offsetCorners));
    transformedCorners[3] = camPose.apply(math_vector_3f(offsetCorners, -offsetCorners, 2.f * offsetCorners));
    transformedCorners[4] = camPose.apply(math_vector_3f(0, 0, 0));

    // draw a sphere at each corner of the frustum
    double cornerDiameter = 0.02f * radius;
    glColor3f(color[0], color[1], color[2]);

    for (int i = 0; i < 5; ++i)
    {
        glPushMatrix();
        glTranslatef(transformedCorners[i][0], transformedCorners[i][1], transformedCorners[i][2]);
        glutSolidSphere(cornerDiameter * 0.5, 30, 30);
        glPopMatrix();
    }

    // draw frustum lines
    float line_width = 1.0f * radius;

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

void drawing_cameras(std::vector<Transform3Df> &poses, float radius, float *color)
{
    for (unsigned int k = 0; k < poses.size(); ++k)
    {
        drawing_pose(poses[k], radius, color);
    }
}

void drawing_cloud(SRef<std::vector<SRef<CloudPoint>>> &cloud_temp, float radius, float *color)
{
    GLUquadric *quad = gluNewQuadric();
    glPushMatrix();
    glBegin(GL_POINTS);
    glPointSize(radius);
    Point3Df vr_temp(0, 0, 0);
    for (unsigned int i = 0; i < cloud_temp->size(); ++i)
    {
        vr_temp = Point3Df((*cloud_temp)[i]->getX(), (*cloud_temp)[i]->getY(), (*cloud_temp)[i]->getZ());
        glColor3f(color[0], color[1], color[2]);
        glVertex3f(vr_temp.getX(), vr_temp.getY(), vr_temp.getZ());
    }
    glEnd();
    glPopMatrix();
    gluDeleteQuadric(quad);
}



#endif //_DRAWING_UTILS_H_