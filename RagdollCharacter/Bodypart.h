#pragma once

#include <graphics/mesh_instance.h>
#include <maths/vector4.h>
#include <maths/quaternion.h>

class Bodypart : public gef::MeshInstance
{
private:
	float width_;
	float height_;
	float depth_;
	int child_joint_;
	gef::Vector4 end_point_;
	gef::Vector4 pos0_;
	gef::Vector4 pos1_;
	gef::Quaternion rot0_;
	gef::Quaternion rot1_;
	//	Transform for this bodypart from an animation.
	gef::Matrix44 joint_transform_;
	//	Transform for this bodypart from the ragdoll.
	gef::Matrix44 rigid_body_transform_;
	
public:
	Bodypart();
	inline float GetWidth() { return width_; }
	inline float GetHeight() { return height_; }
	inline float GetDepth() { return depth_; }
	inline gef::Vector4 GetEndPoint() { return end_point_; }
	inline gef::Matrix44 GetTransform() { return joint_transform_; }
	inline int GetChildJoint() { return child_joint_; }
	inline gef::Quaternion GetLastRotation() { return rot0_; }
	inline gef::Quaternion GetCurrentRotation() { return rot1_; }
	inline gef::Vector4 GetLastPosition() { return pos0_; }
	inline gef::Vector4 GetCurrentPosition() { return pos1_; }
	inline gef::Matrix44 GetRigidBodyTransform() { return rigid_body_transform_; }

	void SetChildJoint(int joint);
	void SetWidth(float width);
	void SetHeight(float height);
	void SetDepth(float depth);
	void SetEndPoint(gef::Vector4 end_point);
	void SetTransform(gef::Matrix44& transform, gef::Matrix44& child_transform);
	void SetTransform(gef::Matrix44& transform);
	void SetRigidBodyTransform(gef::Matrix44& transform);
};