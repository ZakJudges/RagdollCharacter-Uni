#pragma once

#include <vector>
#include <maths/vector4.h>
#include <maths/matrix44.h>
#include <maths/quaternion.h>
#include "primitive_renderer.h"
#include "primitive_builder.h"

#include "btBulletDynamicsCommon.h"

class Ragdoll;
class AnimatedMesh;
class RagdollSkeleton;

namespace gef
{
	class Platform;
	class Mesh;
	class MeshInstance;
	class Renderer3D;
}

class Character
{
public:
	Character(gef::Platform& platform, btDiscreteDynamicsWorld* world);
	void Init();
	void Render(gef::Renderer3D* renderer);
	void Update(float delta_time);
	void Rotate(bool isClockwise);
	bool ToggleMesh();
	bool ToggleSkeleton();
	void ToggleRagdoll();
	void ToggleActiveRagdoll();
	void StandUp();
	void Reset();
	float ChangeAnimationSpeed(bool increased);
	inline gef::Vector4 GetPosition() { return world_transform_.GetTranslation(); }
	float GetFollowStrength();

	void ApplyForce(gef::Vector4& direction);
private:
	AnimatedMesh* animated_mesh_;
	gef::Platform& platform_;
	std::vector<gef::MeshInstance*> capsules_;
	PrimitiveRenderer* primitive_renderer_;
	PrimitiveBuilder* primitive_builder_;

	bool mesh_toggle_;
	bool skeleton_toggle_;
	bool ragdoll_toggle_;
	bool active_ragdoll_toggle_;

	bool changed_pose_;

	RagdollSkeleton* ragdoll_skeleton_;
	Ragdoll* ragdoll_;
	Ragdoll* active_ragdoll_;

	std::vector<gef::MeshInstance*> ragdoll_mesh_;
	std::vector<gef::MeshInstance*> active_ragdoll_mesh_;

	btDiscreteDynamicsWorld* world_;

	gef::Matrix44 world_transform_;

	float delta_time_;
	float rot_y_angle_;
	bool debug_lines_;
	bool is_face_down_;
	
};