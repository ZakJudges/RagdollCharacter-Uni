#pragma once
#include <LinearMath/btAlignedObjectArray.h>
#include <maths/vector4.h>
#include <maths/matrix44.h>

class Physics
{
public:
	Physics();
	~Physics();
	void Init();
	void Update(float delta_time);
	void CreateStaticObject(gef::Vector4& half_extents, gef::Matrix44& transform);
private:
	class btDefaultCollisionConfiguration* collision_configuration_;
	class btCollisionDispatcher* dispatcher_;
	class btBroadphaseInterface* overlapping_pair_cache_;
	class btSequentialImpulseConstraintSolver* solver_;
	class btDiscreteDynamicsWorld* dynamics_world_;
	btAlignedObjectArray<class btCollisionShape*> collision_shapes_;
public:
	inline btDiscreteDynamicsWorld* GetWorld() { return dynamics_world_; }
	bool ToggleGravity();
private:
	bool gravity_toggle_;

};