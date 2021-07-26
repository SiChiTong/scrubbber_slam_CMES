//
// Created by gaoxiang on 2019/12/18.
//

#include "renderGL/camera.h"

#include <glog/logging.h>

namespace HAMO {

void Camera::SetViewMatrix(const Matrix4x4f &viewMatrix) {
    view_matrix_ = viewMatrix;
    p_matrix_ = view_matrix_ * proj_matrix_;
}

Matrix4x4f Camera::GetViewMatrix() {
    Eigen::Vector3f offset = Rotation() * Eigen::Vector3f(0.0f, 0.0f, position_[2]);

    V3d eye = V3d(position_.x, position_.y, 0.0f) + V3d(offset[0], offset[1], offset[2]);
    
    view_matrix_ =
        Matrix4x4f ::LookAt(V3f(eye.x, eye.y, eye.z), V3f(position_.x, position_.y, 0.0f), V3f(0.0f, 1.0f, 0.0f));

    return view_matrix_;
}

void Camera::SetProjectionMatrix(const Matrix4x4f &projMatrix) {
    proj_matrix_ = projMatrix;
    p_matrix_ = view_matrix_ * proj_matrix_;
}

Matrix4x4f Camera::GetProjectionMatrix() const { return proj_matrix_; }

Matrix4x4f Camera::GetViewProjMatrix() const { return p_matrix_; }

Viewport Camera::GetViewport() const { return viewport_; }

void Camera::SetViewport(const Viewport &viewport) { viewport_ = viewport; }

void Camera::SetPosition(const V3d &position) {
    position_ = position;
    view_matrix_ = Matrix4x4f::LookAt(V3f(position_.x, position_.y, position_.z),
                                      V3f(position_.x, position_.y, position_.z - 1.0f), V3f(0.0f, 1.0f, 0.0f));

    p_matrix_ = view_matrix_ * proj_matrix_;
}

void Camera::SetHeight(double height) {
    position_[2] = height;
    SetPosition(position_);
}

void Camera::MoveCamera(const V3d &dir, double distance, const float theta) {
    V3d vDir = Normalize(dir);
    V3d vTmp = vDir * distance;

    Eigen::Vector3d delta_p = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) * vTmp.ToEigen();

    position_ += V3d(delta_p);

    SetPosition(position_);
}

V3d Camera::GetPosition() const { return position_; }

void Camera::Rotate(double dx, double dy) {
    theta_ = dx;
    phi_ = dy;
}

Eigen::Quaternionf Camera::Rotation() const {
    return Eigen::AngleAxisf(theta_, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(phi_, Eigen::Vector3f::UnitX());
}

float Camera::GetTheta() { return theta_; }

void Camera::ResetCamera() {
    position_.clear();
    position_[2] = 50.0f;
    theta_ = 0;
    phi_ = 0;
}

void StereoCamera::SetPerspective(float fovy, float aspect, float zn, float zf) {
    proj_matrix_ = Matrix4x4f::Perspective(Mathf::ToRadians(fovy), aspect, zn, zf);
    p_matrix_ = view_matrix_ * proj_matrix_;
}

void PlanCamera::SetOrtho(float aspect, float zn, float zf) {
    proj_matrix_ = Matrix4x4f::Ortho(-aspect, aspect, -1.0, 1.0f, zn, zf);
    p_matrix_ = view_matrix_ * proj_matrix_;
}

}  // namespace HAMO
