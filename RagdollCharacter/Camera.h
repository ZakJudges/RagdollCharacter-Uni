#pragma once

#include <maths/vector4.h>
#include <maths/vector2.h>
#include <maths/matrix44.h>


namespace gef
{
	class Platform;
}

class Camera
{
public:
	Camera(gef::Platform& platform);
	void Init(gef::Vector4 eye, gef::Vector4 lookat, gef::Vector4 up, float fov, float near_plane, float far_plane);
	gef::Matrix44 GetViewMatrix();
	gef::Matrix44 GetProjectionMatrix();
	inline gef::Vector4 GetCameraEye() { return camera_eye_; }
	inline gef::Vector4 GetCameraLookat() { return camera_lookat_; }
	inline void SetCameraLookat(gef::Vector4 lookat) { camera_lookat_ = lookat; }
	inline void SetCameraEye(gef::Vector4 new_eye) { camera_eye_ = new_eye; }
private:
	gef::Matrix44 view_matrix_;
	gef::Matrix44 projection_matrix_;
	gef::Vector4 camera_eye_;
	gef::Vector4 camera_lookat_;
	gef::Vector4 camera_up_;
	float camera_fov_;
	float near_plane_;
	float far_plane_;

	gef::Platform& platform_;
};