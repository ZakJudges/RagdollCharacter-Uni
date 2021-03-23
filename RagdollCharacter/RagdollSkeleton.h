#pragma once
#include <vector>
#include <maths/matrix44.h>
#include <maths/vector4.h>
#include "primitive_builder.h"
#include "Bodypart.h"
#include "AnimatedMesh.h"
#include <maths/quaternion.h>

namespace gef
{
	class MeshInstance;
	class Platform;
}

class RagdollSkeleton
{
public:
	RagdollSkeleton(gef::Platform& platform, AnimatedMesh* animated_mesh);
	~RagdollSkeleton();
	void Init();
	void Update(float delta_time, gef::Matrix44& world_transform);
	void UpdateRigidBody(gef::Matrix44& transform, int i);
	void SetActive(bool active);

	inline int GetIndex(int bodypart) { return bodypart_index_[bodypart]; }
	inline gef::MeshInstance* GetMeshInstance(int bodypart) { return (gef::MeshInstance*)bodyparts_[bodypart]; }
	inline float GetWidthAt(int i) { return bodyparts_[i]->GetWidth(); }
	inline float GetHeightAt(int i) { return bodyparts_[i]->GetHeight(); }
	inline float GetDepthAt(int i) { return bodyparts_[i]->GetDepth(); }
	inline gef::Matrix44 GetTransformAt(int i) { return bodyparts_[i]->transform(); }
	inline gef::Quaternion GetLastRotationAt(int i) { return bodyparts_[i]->GetLastRotation(); }
	inline gef::Quaternion GetCurrentRotationAt(int i) { return bodyparts_[i]->GetCurrentRotation(); }
	inline gef::Vector4 GetLastPositionAt(int i) { return bodyparts_[i]->GetLastPosition(); }
	inline gef::Vector4  GetCurrentPositionAt(int i) { return bodyparts_[i]->GetCurrentPosition(); }
	bool IsInFinalPose();
	float GetFollowStrength();

public:
	enum BodyParts
	{
		HIPS = 0,
		SPINE_BOTTOM,
		SPINE_MID,
		SPINE_TOP,
		HEAD,
		LEFT_UPPER_LEG,
		LEFT_LOWER_LEG,
		LEFT_FOOT,
		RIGHT_UPPER_LEG,
		RIGHT_LOWER_LEG,
		RIGHT_FOOT,
		LEFT_SHOULDER,
		LEFT_UPPER_ARM,
		LEFT_LOWER_ARM,
		RIGHT_SHOULDER,
		RIGHT_UPPER_ARM,
		RIGHT_LOWER_ARM,
		BODYPART_COUNT
	};

private:
	std::vector<Bodypart*> bodyparts_;
	int bodypart_index_[24];
	enum BodyPartIndex
	{
		i_HIPS = 0,
		i_SPINE_BOTTOM = 1,
		i_SPINE_MID = 2,
		i_SPINE_TOP = 3,
		i_HEAD = 4,
		i_LEFT_UPPER_LEG = 62,
		i_LEFT_LOWER_LEG = 63,
		i_LEFT_FOOT = 65,
		i_RIGHT_UPPER_LEG = 57,
		i_RIGHT_LOWER_LEG = 58,
		i_RIGHT_FOOT = 60,
		i_LEFT_SHOULDER = 9,
		i_LEFT_UPPER_ARM = 10,
		i_LEFT_LOWER_ARM = 11,
		i_RIGHT_SHOULDER = 33,
		i_RIGHT_UPPER_ARM = 34,
		i_RIGHT_LOWER_ARM = 35,
		i_LEFT_FOOT_END = 66,
		i_RIGHT_FOOT_END = 61,
		i_LEFT_LOWER_ARM_END = 12,
		i_RIGHT_LOWER_ARM_END = 36,
		i_HEAD_END = 6,
		i_LEFT_LOWER_LEG_END = 64,
		i_RIGHT_LOWER_LEG_END = 59,
		INDEX_COUNT = 24
	};

	gef::Platform& platform_;
	PrimitiveBuilder* primitive_builder_;

	//	The mesh that this skeleton is based on.
	AnimatedMesh* animated_mesh_;

	bool active_;
	bool last_state_;
	float counter_;
	float follow_strength_;

	//	Blend Control settings.
	const float blend_speed_ = 1.5f;
	const float start_blend_time_threshold_ = 0.5f;
	float wait_timer_;
private:
	void MapBodyparts();

};


