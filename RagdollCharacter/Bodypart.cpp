#include "Bodypart.h"
#include <system/debug_log.h>
#include <cmath>
#include <maths/math_utils.h>

Bodypart::Bodypart()
{
	SetDepth(0.03f);
	SetHeight(0.03f);
	SetWidth(0.03f);
	pos0_ = gef::Vector4(0.0f, 0.0f, 0.0f);
	pos1_ = gef::Vector4(0.0f, 0.0f, 0.0f);
	rot0_.Identity();
	rot1_.Identity();

	rigid_body_transform_.SetIdentity();
}

void Bodypart::SetWidth(float width)
{
	width_ = width;
}

void Bodypart::SetHeight(float height)
{
	height_ = height;
}

void Bodypart::SetDepth(float depth)
{
	depth_ = depth;
}

void Bodypart::SetEndPoint(gef::Vector4 end_point)
{
	end_point_ = end_point;
}

//	Called when updating this bodypart to match the mesh.
//		Need to know the transform of the child joint in the skeleton to calculate the offset of the bodypart from the joint in the skeleton.
void Bodypart::SetTransform(gef::Matrix44& transform, gef::Matrix44& child_transform)
{
	joint_transform_ = transform;
	end_point_ = child_transform.GetTranslation();

	//	Set the position/rotation for the last timestep.
	pos0_ = pos1_;
	rot0_ = rot1_;

	//	Update the transform of this bodypart.
	gef::Matrix44 new_transform = joint_transform_;
	gef::Vector4 mid = (new_transform.GetTranslation() + end_point_) / 2;
	new_transform.SetTranslation(mid);
	set_transform(new_transform);
	
	//	Set the position/rotation for the current timestep.
	pos1_ = mid;
	gef::Matrix44 rotation;
	rotation = new_transform;
	rot1_.SetFromMatrix(rotation);
}

//	Called when updating this bodypart to match the ragdoll.
//		Do not need to know child transform due to no offset of the bodypart.
void Bodypart::SetTransform(gef::Matrix44& transform)
{
	joint_transform_ = transform;

	//	Set the position/rotation for the last timestep.
	pos0_ = pos1_;
	rot0_ = rot1_;

	//	Update the transform of this bodypart.
	gef::Matrix44 new_transform = joint_transform_;
	pos1_ = new_transform.GetTranslation();
	new_transform.SetTranslation(pos1_);
	set_transform(new_transform);

	//	Set the rotation for the current timestep.
	gef::Matrix44 rotation;
	rotation = new_transform;
	rot1_.SetFromMatrix(rotation);
}

void Bodypart::SetRigidBodyTransform(gef::Matrix44& transform)
{
	rigid_body_transform_ = transform;
}

void Bodypart::SetChildJoint(int joint)
{
	child_joint_ = joint;
}
