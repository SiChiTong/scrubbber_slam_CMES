//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_CAMERA_H
#define HAMO_LINUX_CAMERA_H

#include <Eigen/Geometry>

#include "algorithm/matrix44.h"
#include "algorithm/viewport.h"

namespace HAMO {

class Camera {
   public:
    Camera() : position_(0.0f, 0.0f, 50.0f), theta_(0.0f), phi_(0.0f) {}

    void SetPosition(const V3d &position);
    void SetHeight(double height);

    V3d GetPosition() const;

    void MoveCamera(const V3d &dir, double distance, const float theta = 0);

    void SetProjectionMatrix(const Matrix4x4f &projMatrix);

    void SetViewMatrix(const Matrix4x4f &viewMatrix);

    Matrix4x4f GetViewMatrix();

    Matrix4x4f GetProjectionMatrix() const;

    Matrix4x4f GetViewProjMatrix() const;

    void SetViewport(const Viewport &viewport);

    Viewport GetViewport() const;

    void Rotate(double dx, double dy);

    Eigen::Quaternionf Rotation() const;

    float GetTheta();

    void ResetCamera();

   protected:
    Matrix4x4f view_matrix_;
    Matrix4x4f inv_view_mat_;

    Matrix4x4f proj_matrix_;
    Matrix4x4f inv_proj_mat_;

    Matrix4x4f p_matrix_;
    Viewport viewport_;
    V3d position_;

    double theta_ = 0;
    double phi_ = 0;
};

//------------------------------

class StereoCamera : public Camera {
   public:
    void SetPerspective(float fovy, float aspect, float zn, float zf);
};

class PlanCamera : public Camera {
   public:
    void SetOrtho(float aspect, float zn, float zf);

   private:
};

}  // namespace HAMO

#endif  // HAMO_LINUX_CAMERA_H
