#include "AnimatedMesh.h"

#include <system/platform.h>
#include <graphics/mesh.h>
#include <graphics/scene.h>
#include <graphics/skinned_mesh_instance.h>
#include <animation/animation.h>
#include <animation/skeleton.h>

#include <system/debug_log.h>
#include <maths/math_utils.h>

AnimatedMesh::AnimatedMesh(gef::Platform& platform) : platform_(platform)
{
	animated_mesh_scene_ = NULL;
	animated_mesh_ = NULL;
	jump_anim_ = NULL;
	landing_front_ = NULL;
	landing_back_ = NULL;
	idle_ = NULL;
	pose_ = false;
	play_stand_up_ = false;
}

void AnimatedMesh::Init(gef::Matrix44 transform)
{
	world_transform_ = transform;
	//	Animated mesh:
	//	Read mesh data from the scene file.
	animated_mesh_scene_ = new gef::Scene();
	animated_mesh_scene_->ReadSceneFromFile(platform_, "AnimatedMesh/ybot.scn");

	//	The data stored in the scene file is going to be rendered, so create the materials from the material data present in the scene file.
	animated_mesh_scene_->CreateMaterials(platform_);

	//	Get the animated mesh skeleton from the scene file:
	skeleton_ = GetFirstSkeleton(animated_mesh_scene_);

	if (skeleton_)
	{
		animated_mesh_ = new gef::SkinnedMeshInstance(*skeleton_);
		animation_player_.Init(animated_mesh_->bind_pose());
		//	Create a mesh from the front of the mesh data in the scene file.
		animated_mesh_->set_mesh(GetFirstMesh(animated_mesh_scene_));
		animated_mesh_->UpdateBoneMatrices(animation_player_.pose());

		//	Load the animations for the animated mesh:
		idle_ = LoadAnimation("AnimatedMesh/ybot@idle.scn", "mixamo.com");
		landing_front_ = LoadAnimation("AnimatedMesh/ybot@down_stand_up.scn", "mixamo.com");
		landing_back_ = LoadAnimation("AnimatedMesh/ybot@up_stand_up.scn", "mixamo.com");

		//	Initialise the animation player.
		animation_player_.set_playback_speed(1.25f);
		animation_player_.set_clip(idle_);
		animation_player_.set_looping(true);
		animation_player_.set_anim_time(0.0f);
	}

	//	Build the transformation matrix of the animated mesh.
	animated_mesh_->set_transform(world_transform_);
}

void AnimatedMesh::Update(float delta_time, gef::Matrix44 transform)
{
	world_transform_ = transform;
	animated_mesh_->set_transform(world_transform_);

	//	Update animated mesh:
	if (animated_mesh_)
	{
		//	Set the pose in the animation player to that of the pose from the animation.
		if (pose_)
		{
			if (!play_stand_up_)
			{
				//	Consider one frame of motion.
				animation_player_.set_anim_time(0.0f);
				animation_player_.Update(0.0f, animated_mesh_->bind_pose());
			}
			else
			{
				//	Play stand up animation.
				animation_player_.set_looping(false);
				animation_player_.Update(delta_time, animated_mesh_->bind_pose());
			}
		}
		else
		{
			//	Loop a specific animation.
			animation_player_.set_looping(true);
			animation_player_.Update(delta_time, animated_mesh_->bind_pose());
		}

		//	Update the bone matrices of the skinned mesh instance used for rendering the animated mesh,
		//		So that they are the same as the ones from the animation.
		animated_mesh_->UpdateBoneMatrices(animation_player_.pose());
	}
}

//	Get the world transform of a joint in the skeleton.
gef::Matrix44 AnimatedMesh::GetBoneMatrix(int joint_num)
{
	return animation_player_.pose().global_pose().at(joint_num) * world_transform_;
}

//	Switch from idle to pose targeting.
void AnimatedMesh::SetLanding(bool landing)
{
	if (landing)
	{
		if (landing_back_)
		{
			animation_player_.set_clip(landing_back_);
			pose_ = true;
		}
	}
	else
	{
		if (idle_)
		{
			animation_player_.set_clip(idle_);
			pose_ = false;
		}
	}
}

void AnimatedMesh::SetTargetPose(bool facedown)
{
	if (pose_)
	{
		//	If the ragdoll is landing, choose which clip is the target pose based on whether the ragdoll is face down or face up.
		if (facedown)
		{
			if (landing_front_)
			{
				animation_player_.set_clip(landing_front_);
			}
		}
		else
		{
			if (landing_back_)
			{
				animation_player_.set_clip(landing_back_);
			}
		}
	}
}
	
void AnimatedMesh::PlayStandUp(bool stand_up)
{
	play_stand_up_ = stand_up;
}

float AnimatedMesh::ChangeAnimationSpeed(bool increased)
{
	if (increased)
	{
		//	Increase animation speed.
		if (anim_speed_ < 2.0f)
		{
			anim_speed_ += 0.1f;
		}
	}
	else
	{
		//	Decrease animation speed.
		if (anim_speed_ > 0.0f)
		{
			anim_speed_ -= 0.1f;
		}
	}

	animation_player_.set_playback_speed(anim_speed_);

	return anim_speed_;
}

gef::Mesh* AnimatedMesh::GetFirstMesh(gef::Scene* scene)
{
	gef::Mesh* mesh = NULL;

	if (scene)
	{
		//	Check if there is any mesh data in the scene file, 
		//		if there is then create a mesh from it.
		if (scene->mesh_data.size() > 0)
		{
			mesh = scene->CreateMesh(platform_, scene->mesh_data.front());
		}
	}

	return mesh;
}

gef::Skeleton* AnimatedMesh::GetFirstSkeleton(gef::Scene* scene)
{
	gef::Skeleton* skeleton = NULL;

	if (scene)
	{
		//	Check to see if there are any skeletons in the scene file,
		//		if so, get the bind pose and create an array of matrices to store the bone transformations.
		if (scene->skeletons.size() > 0)
		{
			skeleton = scene->skeletons.front();
		}
	}

	return skeleton;
}

gef::Animation* AnimatedMesh::LoadAnimation(const char* anim_scene_filename, const char* anim_name)
{
	gef::Animation* anim = NULL;

	gef::Scene anim_scene;
	if (anim_scene.ReadSceneFromFile(platform_, anim_scene_filename))
	{
		// if the animation name is specified then try and find the named anim
		// otherwise return the first animation if there is one
		std::map<gef::StringId, gef::Animation*>::const_iterator anim_node_iter;
		if (anim_name)
			anim_node_iter = anim_scene.animations.find(gef::GetStringId(anim_name));
		else
			anim_node_iter = anim_scene.animations.begin();

		if (anim_node_iter != anim_scene.animations.end())
			anim = new gef::Animation(*anim_node_iter->second);
	}

	return anim;
}

AnimatedMesh::~AnimatedMesh()
{
	//	Cleanup.
	if (animated_mesh_scene_)
	{
		delete animated_mesh_scene_;
		animated_mesh_scene_ = nullptr;
	}
	if (animated_mesh_)
	{
		delete animated_mesh_;
		animated_mesh_ = nullptr;
	}
	if (skeleton_)
	{
		delete skeleton_;
		skeleton_ = nullptr;
	}
}