#pragma once

#include "motion_clip_player.h"

namespace gef
{
	class Scene;
	class SkinnedMeshInstance;
	class Animation;
	class Platform;
	class Mesh;
	class Skeleton;
	class Animation;
	
}

class AnimatedMesh
{
private:
	gef::Scene* animated_mesh_scene_;
	gef::SkinnedMeshInstance* animated_mesh_;
	gef::Animation* jump_anim_;
	MotionClipPlayer animation_player_;
	gef::Platform& platform_;
	gef::Skeleton* skeleton_;
	gef::Matrix44 world_transform_;

	gef::Animation* landing_front_;
	gef::Animation* landing_back_;
	gef::Animation* idle_;

	float anim_speed_;

	bool pose_;
	bool play_stand_up_;

public:
	AnimatedMesh(gef::Platform& platform);
	~AnimatedMesh();
	void Init(gef::Matrix44 transform);
	void Update(float delta_time, gef::Matrix44 world_transform);
	void SetLanding(bool landing);
	void SetTargetPose(bool facedown);
	void PlayStandUp(bool stand_up);
	float ChangeAnimationSpeed(bool increase);
	inline gef::SkinnedMeshInstance* GetAnimatedMesh() { return animated_mesh_; }
	gef::Matrix44 GetBoneMatrix(int joint_num);

private:
	gef::Mesh* GetFirstMesh(gef::Scene* scene);
	gef::Skeleton* GetFirstSkeleton(gef::Scene* scene);
	gef::Animation* LoadAnimation(const char* anim_scene_filename, const char* anim_name);
	

};