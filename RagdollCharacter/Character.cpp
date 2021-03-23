#include "Character.h"
#include "AnimatedMesh.h"
#include <system/platform.h>
#include <graphics/mesh.h>
#include <graphics/mesh_instance.h>
#include <graphics/renderer_3d.h>
#include <graphics/skinned_mesh_instance.h>

#include <maths/math_utils.h>
#include <system/debug_log.h>

#include "RagdollSkeleton.h"
#include "Ragdoll.h"

Character::Character(gef::Platform& platform, btDiscreteDynamicsWorld* world) : platform_(platform), world_(world)
{
	ragdoll_ = NULL;
	active_ragdoll_ = NULL;
	animated_mesh_ = NULL;
	primitive_renderer_ = NULL;
	primitive_builder_ = NULL;
	mesh_toggle_ = false;
	skeleton_toggle_ = false;
	ragdoll_toggle_ = false;
	active_ragdoll_toggle_ = true;
	ragdoll_skeleton_ = NULL;
	changed_pose_ = true;
	rot_y_angle_ = 0.0f;
	world_transform_.SetIdentity();
	debug_lines_ = true;
	is_face_down_ = false;
}

void Character::Init()
{
	primitive_builder_ = new PrimitiveBuilder(platform_);
	primitive_renderer_ = new PrimitiveRenderer(platform_);

	//	Initialise world transform.
	gef::Matrix44 rotation_y;
	rotation_y.SetIdentity();
	rotation_y.RotationY(gef::DegToRad(rot_y_angle_));
	world_transform_ = rotation_y;
	world_transform_.SetTranslation(gef::Vector4(0.0f, 0.0f, 0.0f));

	//	Initialise the animated mesh.
	animated_mesh_ = new AnimatedMesh(platform_);
	animated_mesh_->Init(world_transform_);

	//	Initialise the ragdoll skeleton based on the animated mesh.
	ragdoll_skeleton_ = new RagdollSkeleton(platform_, animated_mesh_);
	ragdoll_skeleton_->Init();
	
	//	Pass the ragdoll skeleton to the ragdoll for initialisation.
	ragdoll_ = new Ragdoll(world_, ragdoll_skeleton_, true);
	ragdoll_->Init();

	//	Initialise the active ragdoll and disable collisions between the ragdolls.
	active_ragdoll_ = new Ragdoll(world_, ragdoll_skeleton_, true);
	active_ragdoll_->Init();
	active_ragdoll_->SetCollisionResponses(ragdoll_->GetRigidBodies(), false);

	//	Initialise the mesh instances that will show the resultant motion of the system.
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		gef::MeshInstance* ragdoll_mesh = new gef::MeshInstance();
		ragdoll_mesh->set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(ragdoll_skeleton_->GetWidthAt(i), ragdoll_skeleton_->GetHeightAt(i), ragdoll_skeleton_->GetDepthAt(i))));
		ragdoll_mesh->set_transform(ragdoll_->GetTransformAt(i));
		ragdoll_mesh_.push_back(ragdoll_mesh);

		gef::MeshInstance* active_ragdoll_mesh = new gef::MeshInstance();
		active_ragdoll_mesh->set_mesh(primitive_builder_->CreateBoxMesh(gef::Vector4(ragdoll_skeleton_->GetWidthAt(i), ragdoll_skeleton_->GetHeightAt(i), ragdoll_skeleton_->GetDepthAt(i))));
		active_ragdoll_mesh->set_transform(active_ragdoll_->GetTransformAt(i));
		active_ragdoll_mesh_.push_back(active_ragdoll_mesh);
	}
}

void Character::Update(float delta_time)
{
	delta_time_ = delta_time;

	if (primitive_renderer_)
	{
		primitive_renderer_->Reset();
	}

	//	Update the mesh that the ragdoll skeleton is based on.
	if (animated_mesh_)
	{
		animated_mesh_->Update(delta_time_, world_transform_);
	}

	//	Update the ragdoll skeleton.
	if (ragdoll_skeleton_)
	{
		ragdoll_skeleton_->Update(delta_time_, world_transform_);

	}

	//	Update the ragdolls.
	if (ragdoll_ && active_ragdoll_)
	{
		//if (ragdoll_skeleton_->IsInFinalPose())
		//{
		//	active_ragdoll_->ZeroVel();
		//}
		//else
		//{
			ragdoll_->Update(delta_time_);
			active_ragdoll_->Update(delta_time_);
		//}

		//	Determine the target pose based on whether or not the ragdoll is face-up or face-down.
		bool is_face_down = ragdoll_->IsFaceDown();
		bool changed_pose = false;
	    if (is_face_down != is_face_down_)
		{
			//	The ragdoll has switched from being face up to face down or vice versa in the last frame.
			is_face_down_ = is_face_down;
			changed_pose = true;
		}
		//	If the ragdoll's orientation has changed then update the pose.
		if (is_face_down && changed_pose)
		{
			animated_mesh_->SetTargetPose(is_face_down);
		}
		else if (!is_face_down && changed_pose)
		{
			animated_mesh_->SetTargetPose(is_face_down);
		}

		//	Reset the ragdoll poses.
		if (changed_pose_)
		{
			ragdoll_->SetPose();
			active_ragdoll_->SetPose();
			changed_pose_ = false;
		}

		
		//	Move the target pose to the ragdoll's location and orientation - if it is not yet resting.
		if (!ragdoll_skeleton_->IsInFinalPose())
		{
			//	Determine the pose to drive towards.
			gef::Vector4 location, start, end;
			bool landing_detected = ragdoll_->GetLandingLocation(is_face_down, location, start, end);
			if (landing_detected)
			{
				//	Rotate the animated mesh to match the ragdolls rotation about the y-axis.
				//	Build the new world transform for the character.
				gef::Matrix44 rotation;
				rotation.SetIdentity();
				float angle = ragdoll_->GetYRotation(location);
				if (is_face_down)
				{
					//	The poses for face down and face up have oppposite orientations, so flip the face down pose, to make direction match ragdoll.
					angle -= gef::DegToRad(180.0f);
				}

				//	Get the rotation of the ragdoll about the y-axis and apply the rotation to the target pose.
				rotation.RotationY(angle);
				rot_y_angle_ = angle;
				world_transform_ = rotation;
				world_transform_.SetTranslation(location);

				//	Draw the landing location of the ragdoll.
				if (debug_lines_ && primitive_renderer_)
				{
					//primitive_renderer_->AddLine(start, end, gef::Colour(1.0f, 0.0f, 0.0f));
				}
			}
		}
	}
	
	//	Update the transforms of the meshes that make up the ragdoll character.
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		ragdoll_mesh_[i]->set_transform(ragdoll_->GetTransformAt(i));
		active_ragdoll_mesh_[i]->set_transform(active_ragdoll_->GetTransformAt(i));
	}
}

void Character::Render(gef::Renderer3D* renderer)
{
	//	Render the animated mesh.
	renderer->set_override_material(nullptr);
	if (animated_mesh_->GetAnimatedMesh())
	{
		if (mesh_toggle_)
		{
			renderer->DrawSkinnedMesh(*animated_mesh_->GetAnimatedMesh(), animated_mesh_->GetAnimatedMesh()->bone_matrices());
		}
	}

	//	Render the ragdoll's skeleton.
	if(skeleton_toggle_ && ragdoll_skeleton_)
	{
		renderer->set_override_material(&primitive_builder_->blue_material());
		for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
		{
			gef::MeshInstance* mesh = ragdoll_skeleton_->GetMeshInstance(i);
			renderer->DrawMesh(*mesh);
		}
	}
	
	//	Render the ragdoll.
	if (ragdoll_toggle_ && ragdoll_)
	{
		renderer->set_override_material(nullptr);
		for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
		{
			if (ragdoll_mesh_[i])
			{
				renderer->DrawMesh(*ragdoll_mesh_[i]);
			}
		}
	}

	if (active_ragdoll_toggle_ && active_ragdoll_)
	{
		renderer->set_override_material(&primitive_builder_->red_material());
		for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
		{
			if (active_ragdoll_mesh_[i])
			{
				renderer->DrawMesh(*active_ragdoll_mesh_[i]);
			}
		}
	}
	
	//	Draw trajectory ray.
	if (debug_lines_)
	{
		if (primitive_renderer_)
		{
			//primitive_renderer_->Render(*renderer);
		}
	}
}

void Character::ApplyForce(gef::Vector4& direction)
{
	//	Force has been applied, so start ragdoll simulation.
	if (ragdoll_->IsFollowing())
	{
		//	Start simulating physics and get ready to land.
		ragdoll_->SetFollow(false);
		ragdoll_->ApplyForce(direction);
		//	Set the skeleton to produce a pose based on the animation and the ragdoll.
		ragdoll_skeleton_->SetActive(true);
		//	Update the pose of the animated mesh (the target pose)
		animated_mesh_->SetLanding(true);
		animated_mesh_->PlayStandUp(false);
	}
	//	Reset if the force has already been applied.
	else
	{
		//	Stop simulating normal ragdoll physics and revert to an animation.
		ragdoll_->SetFollow(true);
		//	Update changed pose flag so that ragdoll rigid body transforms can be reset.
		changed_pose_ = true;
		//	Update the pose of the animated mesh (default animation)
		animated_mesh_->PlayStandUp(false);
		animated_mesh_->SetLanding(false);
		//	Reset position of the character in the world.
		//world_transform_.SetIdentity();
		//	Set the skeleton to produce a pose based only on the animation.
		ragdoll_skeleton_->SetActive(false);
		rot_y_angle_ = 0.0f;
	}
}

void Character::Rotate(bool isClockwise)
{
	//	Only rotate if the ragdoll is in idle pose.
	if (ragdoll_->IsFollowing())
	{
		gef::Vector4 position_store = world_transform_.GetTranslation();

		float rotation_speed = 200.0f;
		if (!isClockwise)
		{
			rotation_speed *= -1.0f;
		}
		rot_y_angle_ += (rotation_speed * delta_time_);

		gef::Matrix44 rotation;
		rotation.SetIdentity();
		rotation.RotationY(gef::DegToRad(rot_y_angle_));

		world_transform_ = rotation;
		world_transform_.SetTranslation(position_store);

		changed_pose_ = true;
	}
}

void Character::StandUp()
{
	animated_mesh_->PlayStandUp(true);
}

bool Character::ToggleMesh()
{
	if (mesh_toggle_)
	{
		mesh_toggle_ = false;
	}
	else
	{
		mesh_toggle_ = true;
	}

	return mesh_toggle_;
}

bool Character::ToggleSkeleton()
{
	if (skeleton_toggle_)
	{
		skeleton_toggle_ = false;
	}
	else
	{
		skeleton_toggle_ = true;
	}

	return skeleton_toggle_;
}

void Character::ToggleRagdoll()
{
	if (ragdoll_toggle_)
	{
		ragdoll_toggle_ = false;
	}
	else
	{
		ragdoll_toggle_ = true;
	}
}

void Character::ToggleActiveRagdoll()
{
	if (active_ragdoll_toggle_)
	{
		active_ragdoll_toggle_ = false;
	}
	else
	{
		active_ragdoll_toggle_ = true;
	}
}

void Character::Reset()
{
	rot_y_angle_ = 0.0f;
	world_transform_.SetIdentity();
	changed_pose_ = true;
	ragdoll_->SetFollow(true);
	animated_mesh_->PlayStandUp(false);
	animated_mesh_->SetLanding(false);
	ragdoll_skeleton_->SetActive(false);
}

float Character::GetFollowStrength()
{
	return ragdoll_skeleton_->GetFollowStrength();
}

float Character::ChangeAnimationSpeed(bool increased)
{
	return animated_mesh_->ChangeAnimationSpeed(increased);
}

