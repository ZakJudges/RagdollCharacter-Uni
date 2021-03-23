#include "RagdollSkeleton.h"
#include <graphics/mesh_instance.h>
#include <graphics/skinned_mesh_instance.h>
#include <system/platform.h>
#include <system/debug_log.h>

#include <maths/math_utils.h>
RagdollSkeleton::RagdollSkeleton(gef::Platform& platform, AnimatedMesh* animated_mesh) : platform_(platform), animated_mesh_(animated_mesh)
{	
	active_ = false;
	last_state_ = active_;
	//	Set the joint indices from the original skeleton.
	//		*Must* match the order of the Bodyparts enumerated typ AND the BodypartIndex enumerated type.
	int indices[24] = 
	{
		i_HIPS,
		i_SPINE_BOTTOM,
		i_SPINE_MID,
		i_SPINE_TOP,
		i_HEAD,
		i_LEFT_UPPER_LEG,
		i_LEFT_LOWER_LEG,
		i_LEFT_FOOT,
		i_RIGHT_UPPER_LEG,
		i_RIGHT_LOWER_LEG,
		i_RIGHT_FOOT,
		i_LEFT_SHOULDER,
		i_LEFT_UPPER_ARM,
		i_LEFT_LOWER_ARM,
		i_RIGHT_SHOULDER,
		i_RIGHT_UPPER_ARM,
		i_RIGHT_LOWER_ARM,
		i_LEFT_FOOT_END,
		i_RIGHT_FOOT_END,
		i_LEFT_LOWER_ARM_END,
		i_RIGHT_LOWER_ARM_END,
		i_HEAD_END,
		i_LEFT_LOWER_LEG_END ,
		i_RIGHT_LOWER_LEG_END,
	};
	for (int i = 0; i < INDEX_COUNT; i++)
	{
		bodypart_index_[i] = indices[i];
	}

	primitive_builder_ = new PrimitiveBuilder(platform_);

	//	Create the mesh instance objects for the ragdoll skeleton.
	for (int i = 0; i < BODYPART_COUNT; i++)
	{
		Bodypart* mesh = new Bodypart();
		gef::Matrix44 transform;
		transform.SetIdentity();
		mesh->set_transform(transform);
		bodyparts_.push_back(mesh);
	}
	
	MapBodyparts();
	
	counter_ = 0.0f;
	wait_timer_ = 0.0f;
	follow_strength_ = 1.0f;
}

//	Set the dimensions/meshes for the bodyparts.
void RagdollSkeleton::Init()
{
	for (int i = 0; i < BODYPART_COUNT; i++)
	{
		bodyparts_[i]->SetTransform(animated_mesh_->GetBoneMatrix(GetIndex(i)), animated_mesh_->GetBoneMatrix(bodyparts_[i]->GetChildJoint()));

		Bodypart* bodypart = bodyparts_[i];
		gef::Vector4 length = bodypart->GetTransform().GetTranslation() - bodypart->GetEndPoint();
		float l = length.Length();

		//	Decrease the length of the feet.
		if (i == LEFT_FOOT || i == RIGHT_FOOT)
		{
			l *= 0.75f;
		}
		//	Decrease the length of the head.
		if (i == HEAD)
		{
			l *= 0.65f;
		}

		//	Set the length of the body part (leave spaces between).
		bodypart->SetWidth((l * 0.5f));

		if (i <11)
		{
			//	Swap the width and height values for the non-rotated bodyparts.
			float temp_height = bodyparts_[i]->GetHeight();
			bodyparts_[i]->SetHeight(bodyparts_[i]->GetWidth());
			bodyparts_[i]->SetWidth(temp_height);
		}

		//	Create the box mesh with the given dimensions of the bodypart.
		bodyparts_[i]->set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(bodyparts_[i]->GetWidth(), bodyparts_[i]->GetHeight(), bodyparts_[i]->GetDepth())));
	}
}

void RagdollSkeleton::Update(float delta_time, gef::Matrix44& world_transform)
{
	//	Calculate how closely the skeleton will follow the ragdolls movements.
	float follow_change = counter_;
	if (follow_change > 1.0f)
	{
		follow_change = 1.0f;
	}
	follow_strength_ = 1.0f - follow_change;

	for (int i = 0; i < BODYPART_COUNT; i++)
	{
		if (active_)
		{
			//	Blend between the ragdoll pose and the pose from the animation:
			//		Blend positions.
			gef::Matrix44 rigid_body_t, mesh_t;
			rigid_body_t = bodyparts_[i]->GetRigidBodyTransform();
			mesh_t = animated_mesh_->GetBoneMatrix(GetIndex(i));
			gef::Vector4 mesh_position = animated_mesh_->GetBoneMatrix(bodyparts_[i]->GetChildJoint()).GetTranslation();
			//	Offset the position from the skinned mesh skeleton so that it can be compared to the rigid body.
			mesh_position = (mesh_t.GetTranslation() + mesh_position) / 2.0f;
			gef::Vector4 final_position;
			final_position.Lerp(mesh_position, rigid_body_t.GetTranslation(), follow_strength_);
			//		Blend rotations.
			gef::Quaternion r_body_rotation, mesh_rotation;
			r_body_rotation.SetFromMatrix(rigid_body_t);
			mesh_rotation.SetFromMatrix(mesh_t);
			gef::Quaternion final_rotation;
			final_rotation.Slerp(mesh_rotation, r_body_rotation, follow_strength_);
			//		Calculate new transform.
			gef::Matrix44 transform;
			transform.SetIdentity();
			transform.Rotation(final_rotation);
			transform.SetTranslation(final_position);

			bodyparts_[i]->SetTransform(transform);
		}
		else
		{
			//	Copy animation exactly.
			bodyparts_[i]->SetTransform(animated_mesh_->GetBoneMatrix(GetIndex(i)), animated_mesh_->GetBoneMatrix(bodyparts_[i]->GetChildJoint()));
			//	When switching to velocity tracking, require the most up to date transform to prevent jittering when triggering the active ragdoll.
			UpdateRigidBody(bodyparts_[i]->GetTransform(), i);
		}
	}

	//	Ragdoll position reset, so reset the counters as well.
	if (last_state_ != active_)
	{
		last_state_ = active_;
		counter_ = 0.0f;
		wait_timer_ = 0.0f;
	}

	if (active_)
	{	
		//	Increment the timer that counts how ling since the force was applied.
		if (wait_timer_ <= start_blend_time_threshold_)
		{
			wait_timer_ += delta_time;
		}

		//	Limit the counter to 2.5 seconds.
		if (counter_ >= 2.5f)
		{
			counter_ = 2.5f;
		}
		else
		{
			if (wait_timer_ > start_blend_time_threshold_)
			{
				//Wait for a set amount of time and then start blending.
				//	This is to prevent sudden changes in the target pose (the target pose is less likely to change after this time has passed).
				counter_ += (delta_time * blend_speed_);
			}	
		}
	}
}

void RagdollSkeleton::UpdateRigidBody(gef::Matrix44& transform, int i)
{
	bodyparts_[i]->SetRigidBodyTransform(transform);
}

void RagdollSkeleton::MapBodyparts()
{
	//	For each bodypart, set the parent joint and the end point of the joint.
	bodyparts_[HIPS]->SetChildJoint(i_SPINE_BOTTOM);
	bodyparts_[SPINE_BOTTOM]->SetChildJoint(i_SPINE_MID);
	bodyparts_[SPINE_MID]->SetChildJoint(i_SPINE_TOP);
	bodyparts_[SPINE_TOP]->SetChildJoint(i_HEAD);
	bodyparts_[HEAD]->SetChildJoint(i_HEAD_END);
	bodyparts_[LEFT_SHOULDER]->SetChildJoint(i_LEFT_UPPER_ARM);
	bodyparts_[LEFT_UPPER_ARM]->SetChildJoint(i_LEFT_LOWER_ARM);
	bodyparts_[LEFT_LOWER_ARM]->SetChildJoint(i_LEFT_LOWER_ARM_END);
	bodyparts_[RIGHT_SHOULDER]->SetChildJoint(i_RIGHT_UPPER_ARM);
	bodyparts_[RIGHT_UPPER_ARM]->SetChildJoint(i_RIGHT_LOWER_ARM);
	bodyparts_[RIGHT_LOWER_ARM]->SetChildJoint(i_RIGHT_LOWER_ARM_END);
	bodyparts_[LEFT_UPPER_LEG]->SetChildJoint(i_LEFT_LOWER_LEG);
	bodyparts_[LEFT_LOWER_LEG]->SetChildJoint(i_LEFT_LOWER_LEG_END);
	bodyparts_[LEFT_FOOT]->SetChildJoint(i_LEFT_FOOT_END);
	bodyparts_[RIGHT_UPPER_LEG]->SetChildJoint(i_RIGHT_LOWER_LEG);
	bodyparts_[RIGHT_LOWER_LEG]->SetChildJoint(i_RIGHT_LOWER_LEG_END);
	bodyparts_[RIGHT_FOOT]->SetChildJoint(i_RIGHT_FOOT_END);

	//	Set the dimensions of the bodypart (excluding length between joints)
	bodyparts_[HIPS]->SetHeight(0.15f);
	bodyparts_[HIPS]->SetDepth(0.1f);
	bodyparts_[SPINE_BOTTOM]->SetHeight(0.12f);
	bodyparts_[SPINE_BOTTOM]->SetDepth(0.1f);
	bodyparts_[SPINE_MID]->SetHeight(0.1f);
	bodyparts_[SPINE_MID]->SetDepth(0.095f);
	bodyparts_[SPINE_TOP]->SetHeight(0.1f);
	bodyparts_[SPINE_TOP]->SetDepth(0.095f);
	bodyparts_[LEFT_UPPER_ARM]->SetHeight(0.04f);
	bodyparts_[LEFT_UPPER_ARM]->SetDepth(0.04f);
	bodyparts_[RIGHT_UPPER_ARM]->SetHeight(0.04f);
	bodyparts_[RIGHT_UPPER_ARM]->SetDepth(0.04f);
	bodyparts_[LEFT_UPPER_LEG]->SetHeight(0.06f);
	bodyparts_[LEFT_UPPER_LEG]->SetDepth(0.06f);
	bodyparts_[LEFT_LOWER_LEG]->SetHeight(0.04f);
	bodyparts_[LEFT_LOWER_LEG]->SetDepth(0.04f);
	bodyparts_[RIGHT_UPPER_LEG]->SetHeight(0.06f);
	bodyparts_[RIGHT_UPPER_LEG]->SetDepth(0.06f);
	bodyparts_[RIGHT_LOWER_LEG]->SetHeight(0.04f);
	bodyparts_[RIGHT_LOWER_LEG]->SetDepth(0.04f);
	bodyparts_[RIGHT_FOOT]->SetDepth(0.08f);
	bodyparts_[LEFT_FOOT]->SetDepth(0.08f);
	bodyparts_[HEAD]->SetHeight(0.08f);
	bodyparts_[HEAD]->SetDepth(0.08f);
	bodyparts_[RIGHT_SHOULDER]->SetHeight(0.05f);
	bodyparts_[RIGHT_SHOULDER]->SetDepth(0.05f);
	bodyparts_[LEFT_SHOULDER]->SetHeight(0.05f);
	bodyparts_[LEFT_SHOULDER]->SetDepth(0.05f);
}

void RagdollSkeleton::SetActive(bool active)
{
	active_ = active;
}

//	Check to finalise the target pose.
//		If following animation exactly and at least 2.5 seconds have passed since the application of the force.
bool RagdollSkeleton::IsInFinalPose()
{
	if (follow_strength_ <= 0.0f && counter_ >= 2.5f)
	{
		return true;
	}
	return false;
}

float RagdollSkeleton::GetFollowStrength()
{
	return follow_strength_;
}

RagdollSkeleton::~RagdollSkeleton()
{
	//	Cleanup.
	for (int y = 0; y < bodyparts_.size(); y++)
	{
		if (bodyparts_.at(y))
		{
			delete bodyparts_.at(y);
			bodyparts_.at(y) = nullptr;
		}
	}

	if (primitive_builder_)
	{
		delete primitive_builder_;
		primitive_builder_ = nullptr;
	}

	if (animated_mesh_)
	{
		delete animated_mesh_;
		animated_mesh_ = nullptr;
	}

}