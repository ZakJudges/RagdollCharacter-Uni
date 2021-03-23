#include "Camera.h"
#include <maths/math_utils.h>
#include <system/platform.h>


Camera::Camera(gef::Platform& platform) : platform_(platform)
{
}

void Camera::Init(gef::Vector4 eye, gef::Vector4 lookat, gef::Vector4 up, float fov, float near, float far)
{
	camera_eye_ = eye;
	camera_lookat_ = lookat;
	camera_up_ = up;
	camera_fov_ = fov;
	near_plane_ = near;
	far_plane_ = far;

	GetViewMatrix();
	GetProjectionMatrix();
}

gef::Matrix44 Camera::GetViewMatrix()
{
	view_matrix_.LookAt(camera_eye_, camera_lookat_, camera_up_);

	return view_matrix_;
}

gef::Matrix44 Camera::GetProjectionMatrix()
{
	projection_matrix_ = platform_.PerspectiveProjectionFov(camera_fov_, (float)platform_.width() / (float)platform_.height(), near_plane_, far_plane_);

	return projection_matrix_;

}


