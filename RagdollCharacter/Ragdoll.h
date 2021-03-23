#pragma once
#include "Physics.h"
#include "RagdollSkeleton.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"

#include <vector>
#include <maths/vector4.h>
#include <maths/matrix44.h>

namespace gef
{
	class MeshInstance;
}

class Ragdoll
{
public:
		Ragdoll(btDynamicsWorld* world, RagdollSkeleton* skeleton, bool follow);
		~Ragdoll();
		void Init(btCollisionShape* collision_shapes[]);
		void ZeroVel();
		void Init();
		void SetPose();
		void SetFollow(bool follow);
		bool GetLandingLocation(bool use_root, gef::Vector4& location, gef::Vector4& start_point, gef::Vector4& endpoint);
		float GetYRotation(gef::Vector4& origin);
		bool IsFaceDown();
		gef::Matrix44 GetTransformAt(int i);
		void Update(float delta_time);
		void SetCollisionResponses(btRigidBody* bodies[], bool enalbed);
		inline btRigidBody** GetRigidBodies() { return rigid_bodies_; }
		inline bool IsFollowing() { return follow_; }
		void ApplyForce(gef::Vector4& direction);
private:
	void CreateRagdollConstraints();
	void SetCollisionResponses();
	btVector3 GetTrajectory();
	btVector3 GetXZPlaneMax();
	void ToggleGravity(bool on);
private:

	struct RigidBodyDimensions
	{
		float width;
		float height;
		float depth;
	};

	enum RagdollConstraints
	{
		HIPS_TO_SPINE_BOTTOM = 0,
		SPINE_BOTTOM_TO_MID,
		SPINE_MID_TO_TOP,
		SPINE_TOP_TO_HEAD,

		SPINE_TOP_TO_LEFT_SHOULDER,
		LEFT_SHOULDER_TO_LEFT_UPPER_ARM,
		LEFT_UPPER_ARM_TO_LOWER,

		SPINE_TOP_TO_RIGHT_SHOULDER,
		RIGHT_SHOULDER_TO_RIGHT_UPPER_ARM,
		RIGHT_UPPER_ARM_TO_LOWER,

		HIPS_TO_LEFT_UPPER_LEG,
		LEFT_UPPER_LEG_TO_LOWER,
		LEFT_LOWER_LEG_TO_FOOT,

		HIPS_TO_RIGHT_UPPER_LEG,
		RIGHT_UPPER_LEG_TO_LOWER,
		RIGHT_LOWER_LEG_TO_FOOT,

		CONSTRAINT_COUNT
	};


	btDynamicsWorld* world_;
	RagdollSkeleton* skeleton_;
	btCollisionShape* shapes_[RagdollSkeleton::BODYPART_COUNT];
	btRigidBody* rigid_bodies_[RagdollSkeleton::BODYPART_COUNT];
	btTypedConstraint* constraints_[CONSTRAINT_COUNT];
	bool follow_;
};