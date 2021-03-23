#include "Physics.h"
#include "btBulletDynamicsCommon.h"

#include <maths/quaternion.h>

Physics::Physics() :
	collision_configuration_(NULL),
	dispatcher_(NULL),
	overlapping_pair_cache_(NULL),
	solver_(NULL),
	dynamics_world_(NULL)
{
	gravity_toggle_ = true;
}

void Physics::Init()
{
	collision_configuration_ = new btDefaultCollisionConfiguration();
	dispatcher_ = new btCollisionDispatcher(collision_configuration_);
	overlapping_pair_cache_ = new btDbvtBroadphase();
	solver_ = new btSequentialImpulseConstraintSolver;
	dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, overlapping_pair_cache_, solver_, collision_configuration_);
	
	dynamics_world_->setGravity(btVector3(0, -10, 0));
}

void Physics::CreateStaticObject(gef::Vector4& half_extents, gef::Matrix44& transform)
{
	btCollisionShape* shape = new btBoxShape(btVector3(half_extents.x(), half_extents.y(), half_extents.z()));
	collision_shapes_.push_back(shape);

	btTransform b_transform;
	gef::Quaternion rotation;
	rotation.SetFromMatrix(transform);
	btQuaternion b_rotation(rotation.x, rotation.y, rotation.z, rotation.w);
	b_transform.setRotation(b_rotation);
	b_transform.setOrigin(btVector3(transform.GetTranslation().x(), transform.GetTranslation().y(), transform.GetTranslation().z()));

	btScalar mass = 0.;
	btVector3 local_inertia(0, 0, 0);

	btDefaultMotionState* motion_state = new btDefaultMotionState(b_transform);
	btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, local_inertia);
	btRigidBody* body = new btRigidBody(rb_info);
	body->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);

	dynamics_world_->addRigidBody(body);

}

void Physics::Update(float delta_time)
{
	dynamics_world_->stepSimulation(delta_time, 4);
}

bool Physics::ToggleGravity()
{
	if (gravity_toggle_)
	{
		gravity_toggle_ = false;
		dynamics_world_->setGravity(btVector3(0, 0, 0));
	}
	else
	{
		gravity_toggle_ = true;
		dynamics_world_->setGravity(btVector3(0, -10, 0));
	}

	return gravity_toggle_;
}

Physics::~Physics()
{

	for (int i = 0; i < collision_shapes_.size(); i++)
	{
		if (collision_shapes_[i])
		{
			delete collision_shapes_[i];
			collision_shapes_[i] = nullptr;
		}
	}

	if (collision_configuration_)
	{
		delete collision_configuration_;
		collision_configuration_ = nullptr;
	}

	if (dispatcher_)
	{
		delete dispatcher_;
		dispatcher_ = nullptr;
	}

	if (overlapping_pair_cache_)
	{
		delete overlapping_pair_cache_;
		overlapping_pair_cache_ = nullptr;
	}

	if (solver_)
	{
		delete solver_;
		solver_ = nullptr;
	}

	if (dynamics_world_)
	{
		delete dynamics_world_;
		dynamics_world_ = nullptr;
	}
}
