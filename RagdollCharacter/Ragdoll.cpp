#include "Ragdoll.h"
#include <system/debug_log.h>
#include <graphics/mesh_instance.h>

#include <maths/math_utils.h>
#include <cmath>

Ragdoll::Ragdoll(btDynamicsWorld* world, RagdollSkeleton* skeleton, bool follow)
{
	world_ = world;
	skeleton_ = skeleton;
	follow_ = follow;
}

void Ragdoll::Init(btCollisionShape* collision_shapes[])
{
	
	//	Create the rigid bodies.
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		gef::Matrix44 initial_transform = skeleton_->GetTransformAt(i);

		//	First, create the box shapes for the rigid bodies.

		shapes_[i] = collision_shapes[i];

		//	Get the initial position and rotation for each bodypart.
		btVector3 position(initial_transform.GetTranslation().x(), initial_transform.GetTranslation().y(), initial_transform.GetTranslation().z());
		gef::Quaternion initial_rotation;

		initial_rotation.SetFromMatrix(initial_transform);
		btQuaternion rotation(initial_rotation.x, initial_rotation.y, initial_rotation.z);

		btTransform transform;
		transform.setIdentity();
		transform.setRotation(rotation);
		transform.setOrigin(position);

		float mass = 1.0f;
		bool isDynamic = (mass != 0.0f);
		btVector3 local_inertia(0, 0, 0);
		if (isDynamic)
		{
			shapes_[i]->calculateLocalInertia(mass, local_inertia);
		}
		
		btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shapes_[i], local_inertia);
		btRigidBody* body = new btRigidBody(rb_info);
		world_->addRigidBody(body);
		rigid_bodies_[i] = body;
		rigid_bodies_[i]->setDeactivationTime(3.0f);
		rigid_bodies_[i]->setSleepingThresholds(1.6f, 2.5f);
	}

	CreateRagdollConstraints();

	SetFollow(follow_);
}

void Ragdoll::Init()
{

	//	Create the rigid bodies.
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		gef::Matrix44 initial_transform = skeleton_->GetTransformAt(i);

		//	First, create the box shapes for the rigid bodies, using the dimensions of the bodyparts from the ragdoll skeleton.
		shapes_[i] = new btBoxShape(btVector3(skeleton_->GetWidthAt(i), skeleton_->GetHeightAt(i), skeleton_->GetDepthAt(i)));

		//	Get the initial position and rotation for each bodypart.
		btVector3 position(initial_transform.GetTranslation().x(), initial_transform.GetTranslation().y(), initial_transform.GetTranslation().z());
		gef::Quaternion initial_rotation;

		initial_rotation.SetFromMatrix(initial_transform);
		btQuaternion rotation(initial_rotation.x, initial_rotation.y, initial_rotation.z);

		btTransform transform;
		transform.setIdentity();
		transform.setRotation(rotation);
		transform.setOrigin(position);

		float mass = 1.0f;
		bool isDynamic = (mass != 0.0f);
		btVector3 local_inertia(0, 0, 0);
		if (isDynamic)
		{
			shapes_[i]->calculateLocalInertia(mass, local_inertia);
		}

		btDefaultMotionState* motion_state = new btDefaultMotionState(transform);
		btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shapes_[i], local_inertia);
		btRigidBody* body = new btRigidBody(rb_info);
		world_->addRigidBody(body);
		rigid_bodies_[i] = body;
		rigid_bodies_[i]->setActivationState(DISABLE_DEACTIVATION);
		rigid_bodies_[i]->setSleepingThresholds(1.6f, 2.5f);
		
	}

	CreateRagdollConstraints();
	SetFollow(follow_);
}

void Ragdoll::SetPose()
{
	//	Loop through each rigid body and set its position to its corresponding body part from the ragdoll skeleton.
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		gef::Vector4 position = skeleton_->GetTransformAt(i).GetTranslation();
		gef::Quaternion rotation;
		rotation.SetFromMatrix(skeleton_->GetTransformAt(i));

		btVector3 origin(position.x(), position.y(), position.z());
		btQuaternion orientation(rotation.x, rotation.y, rotation.z, rotation.w);

		btTransform transform;
		transform.setRotation(orientation);
		transform.setOrigin(origin);

		rigid_bodies_[i]->setWorldTransform(transform);
		rigid_bodies_[i]->setLinearVelocity(btVector3(0, 0, 0));
		rigid_bodies_[i]->setAngularVelocity(btVector3(0, 0, 0));
	}
}

void Ragdoll::ZeroVel()
{
	if (follow_)
	{
		for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
		{
			rigid_bodies_[i]->setAngularVelocity(btVector3(0, 0, 0));
			rigid_bodies_[i]->setLinearVelocity(btVector3(0, 0, 0));
		}
	}
}

void Ragdoll::Update(float delta_time)
{
	//	Apply velocities to target an animation.
	if (follow_)
	{
		//	Set linear and angular velocities here.
		for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
		{
			//	The transform of this rigid body.
			btTransform rigid_body_transform;
			rigid_bodies_[i]->getMotionState()->getWorldTransform(rigid_body_transform);

			//	Calculate the linear velocity for this timestep from the animation:
			gef::Vector4 last_pos = skeleton_->GetLastPositionAt(i);
			gef::Vector4 current_pos = skeleton_->GetCurrentPositionAt(i);
			btVector3 last_position(last_pos.x(), last_pos.y(), last_pos.z());
			btVector3 current_position(current_pos.x(), current_pos.y(), current_pos.z());
			btVector3 linear_velocity = (current_position - last_position) / delta_time;

			//	Calculate the difference in position between this rigid body and the skeleton joint at the end of the last time step.
			btVector3 position_difference = last_position - rigid_body_transform.getOrigin();

			//	Calculate the angular velocity for this timestep from the animation:
			gef::Quaternion last_rot = skeleton_->GetLastRotationAt(i);
			gef::Quaternion current_rot = skeleton_->GetCurrentRotationAt(i);
			btQuaternion last_rotation(last_rot.x, last_rot.y, last_rot.z, last_rot.w);
			last_rotation.normalize();
			btQuaternion current_rotation(current_rot.x, current_rot.y, current_rot.z, current_rot.w);
			current_rotation.normalize();
			if (current_rotation.dot(last_rotation) < 0.0f)
			{
				//	Current rotation differs from last rotation by more than 90 degrees, so invert the rotation to reduce spinning.
				current_rotation = -current_rotation;
			}
			btQuaternion delta_rotation = current_rotation * last_rotation.inverse();
			delta_rotation.normalize();
			float angle = delta_rotation.getAngleShortestPath();
			btVector3 axis = delta_rotation.getAxis();
			axis.normalize();
			btVector3 skeleton_angular_velocity = axis * angle / delta_time;
			
			//	Calculate the difference in rotation between this rigid body and the joint in the skeleton at the end of the last timestep.
			btQuaternion ragdoll_rotation = rigid_body_transform.getRotation();
			btQuaternion rotation_difference = last_rotation * ragdoll_rotation.inverse();
			rotation_difference.normalize();
			btVector3 diff_axis = rotation_difference.getAxis();
			float diff_angle = rotation_difference.getAngleShortestPath();
			//	Decompose the difference in rotation into an axis and an angle.
			btVector3 difference_axis_angle = diff_axis * diff_angle;

			//	Set the velocities for this rigid body (taking into account the differences in position and rotation).
			rigid_bodies_[i]->setAngularVelocity(skeleton_angular_velocity + difference_axis_angle);
			rigid_bodies_[i]->setLinearVelocity(linear_velocity + position_difference);
		}
	}
	else
	{
		//	Simulating normal ragdoll physics.

		//	Pass the rigid body transforms back to the ragdoll skeleton.
		for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
		{
			skeleton_->UpdateRigidBody(GetTransformAt(i), i);
		}
	}
}

void Ragdoll::ApplyForce(gef::Vector4& direction)
{	
	btTransform affected_bodypart;

	btVector3 force(direction.x(), 0.0f, direction.z());
	force *= 4.0f;
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		rigid_bodies_[i]->getMotionState()->getWorldTransform(affected_bodypart);
		rigid_bodies_[i]->applyImpulse(force, btVector3(0, 0, 0));
	}
}

gef::Matrix44 Ragdoll::GetTransformAt(int i)
{
	gef::Matrix44 transform, translation_matrix;
	transform.SetIdentity();
	translation_matrix.SetIdentity();

	btTransform b_transform;
	rigid_bodies_[i]->getMotionState()->getWorldTransform(b_transform);

	gef::Vector4 translation(b_transform.getOrigin().x(), b_transform.getOrigin().y(), b_transform.getOrigin().z());
	translation_matrix.SetTranslation(translation);

	btQuaternion b_rotation = b_transform.getRotation();
	gef::Quaternion rotation(b_rotation.getX(), b_rotation.getY(), b_rotation.getZ(), b_rotation.getW());
	transform.Rotation(rotation);
	transform = transform * translation_matrix;

	return transform;
}

//	Disable self collision.
void Ragdoll::SetCollisionResponses()
{
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		for (int j = 0; j < RagdollSkeleton::BODYPART_COUNT; j++)
		{
			if (j != i)
			{
				//	If the ith rigid body is not the jth rigid body then disable collisions.
				rigid_bodies_[i]->setIgnoreCollisionCheck(rigid_bodies_[j], true);
			}
		}
	}
}

//	Disable collisions with the rigid bodies passed into this function.
void Ragdoll::SetCollisionResponses(btRigidBody* other_bodies[], bool enabled)
{
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		for (int j = 0; j < RagdollSkeleton::BODYPART_COUNT; j++)
		{
			rigid_bodies_[i]->setIgnoreCollisionCheck(other_bodies[j], !enabled);
		}
	}
}

btVector3 Ragdoll::GetTrajectory()
{
	return rigid_bodies_[RagdollSkeleton::HIPS]->getLinearVelocity();
}

bool Ragdoll::GetLandingLocation(bool use_root, gef::Vector4& location, gef::Vector4& start_point, gef::Vector4& end_point)
{
	if (!follow_)
	{
		btVector3 start, end, hit_point;
		if (use_root)
		{
			//	ray test from hip (should be used for most animation poses - it assumes the world position to be at the hips.)
			btTransform t;
			rigid_bodies_[RagdollSkeleton::HIPS]->getMotionState()->getWorldTransform(t);
			start = t.getOrigin();
		}
		else
		{
			//	Ray test from between feet (should be used for specific animation poses only - it assumes the world position to be between the feet.)
			btTransform left, right;
			rigid_bodies_[RagdollSkeleton::LEFT_FOOT]->getMotionState()->getWorldTransform(left);
			rigid_bodies_[RagdollSkeleton::RIGHT_FOOT]->getMotionState()->getWorldTransform(right);
			start = btVector3((left.getOrigin() + right.getOrigin()) / 2.0f);
		}

		end = start;
		end.setY(-2.0f);

		//	If a landing location is determined then return true so that the ragdoll skeleton can be placed at the landing location.
		btCollisionWorld::AllHitsRayResultCallback ray_result(start, end);
		world_->rayTest(start, end, ray_result);

		bool hit = ray_result.hasHit();
		if (hit)
		{
			//	Ray hit something:
			//		Loop through the hits and check if a static object was hit.
			for (int i = 0 ; i < ray_result.m_collisionObjects.size(); i++)
			{
				hit = ray_result.m_collisionObjects[i]->isStaticObject();
				hit_point = ray_result.m_hitPointWorld[i];
				hit_point.setY(-0.25f);	//	Set the collision point to be leveled at the ground.
				//-0.25
				break;	//	Want the first static object only.
			}
		}

		//	GEF conversions.
		start_point = gef::Vector4(start.x(), start.y(), start.z());
		end_point = gef::Vector4(end.x(), end.y(), end.z());
		location = gef::Vector4(hit_point.x(), hit_point.y(), hit_point.z());

		return hit;
	}
	//	Not simulating 'normal' physics on the ragdoll, so do not find landing location.
	return false;
}

//	Get the position of the ragdoll's head on the XZ plane.
//		Used to determine the ragdoll's rotation about the y-axis on the XZ plane.
btVector3 Ragdoll::GetXZPlaneMax()
{
	btTransform head;
	rigid_bodies_[RagdollSkeleton::HEAD]->getMotionState()->getWorldTransform(head);

	btVector3 pos = head.getOrigin();

	return btVector3(pos.x(), 0.0f, pos.z());
}

//	Get the ragdoll's rotation about the y-axis
//		Direction determined from the ragdolls feet to head.
float Ragdoll::GetYRotation(gef::Vector4& origin)
{
	btVector3 position(origin.x(), origin.y(), origin.z());
	btVector3 direction = GetXZPlaneMax() - position;
	float r = direction.length();

	//	Get the anticlockwise angle to the x-axis with trigonometry.
	float angle = acos(direction.x() / r);
	if (direction.z() > 0.0f)
	{
		//	Get the clockwise angle to the x axis, since the ragdoll is pointing backwards.
		angle = gef::DegToRad(360.0f) - angle;
	}
	//	Rotation about the y-axis starts at an anticlockwise rotation of 90 degrees to the x-axis, so offset the angle.
	angle -= gef::DegToRad(90.0f);

	return angle;
}

//	Determine if the ragdoll is face-up or face-down.
bool Ragdoll::IsFaceDown()
{
	btTransform hips;
	rigid_bodies_[RagdollSkeleton::HIPS]->getMotionState()->getWorldTransform(hips);

	//	Get the forward vector of the root of the ragdoll.
	btVector3 forward = hips.getBasis().getColumn(2);

	btVector3 up(0.0f, 1.0f, 0.0f);

	//	If the dot product is negative then the ragdoll forward vector is greater than 90 degrees from the world up vector.
	//		If the dot product is positive then the directions are within 90 degrees (hence closer to facing upwards than downwards)
	if (forward.dot(up) > 0.0f)
	{
		return false;
	}

	return true;
}

void Ragdoll::SetFollow(bool follow)
{
	if (follow)
	{
		//		Note: Gravity is always on when simulating normal ragdoll behaviour, 
		//			so turning gravity off for the active ragdoll ensures that gravity is not applied twice.
		ToggleGravity(false);
	}
	else
	{
		ToggleGravity(true);
	}
	follow_ = follow;
}

void Ragdoll::ToggleGravity(bool on)
{
	for (int i = 0; i < RagdollSkeleton::BODYPART_COUNT; i++)
	{
		if (on)
		{
			rigid_bodies_[i]->setGravity(btVector3(0, -10, 0));
		}
		else
		{
			rigid_bodies_[i]->setGravity(btVector3(0, 0, 0));
		}
	}
}
		
//	Constrain the movements of the rigid bodies.
void Ragdoll::CreateRagdollConstraints()
{
	btTransform local_a, local_b;
	local_a.setIdentity();
	local_b.setIdentity();
	btVector3 upper_limit;
	btVector3 lower_limit;
	local_a.getBasis().setEulerZYX(0, 0, 0);
	local_b.getBasis().setEulerZYX(0, 0, 0);

	//	SPINE TOP HEAD
	local_a.setOrigin(btVector3(0.0f, 0.11f, 0.0f));
	local_b.setOrigin(btVector3(0.0f, -0.11f, 0.0f));
	btConeTwistConstraint* spine_head = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::SPINE_TOP], *rigid_bodies_[RagdollSkeleton::HEAD], local_a, local_b);
	spine_head->setLimit(gef::DegToRad(5.0f), gef::DegToRad(15.0f), gef::DegToRad(35.0f));
	constraints_[SPINE_TOP_TO_HEAD] = spine_head;
	world_->addConstraint(spine_head, true);
	//	SPINE MID TOP
	local_a.setOrigin(btVector3(0.0f, 0.06f, 0.0f));
	local_b.setOrigin(btVector3(0.0f, -0.06f, 0.0f));
	btConeTwistConstraint* spine_mid_top = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::SPINE_MID], *rigid_bodies_[RagdollSkeleton::SPINE_TOP], local_a, local_b);
	spine_mid_top->setLimit(gef::DegToRad(5.0f), gef::DegToRad(5.0f), gef::DegToRad(15.0f));
	constraints_[SPINE_MID_TO_TOP] = spine_mid_top;
	world_->addConstraint(spine_mid_top, true);
	//	SPINE BOTTOM MID
	btConeTwistConstraint* spine_bottom_mid = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::SPINE_BOTTOM], *rigid_bodies_[RagdollSkeleton::SPINE_MID], local_a, local_b);
	spine_bottom_mid->setLimit(gef::DegToRad(1.0f), gef::DegToRad(1.0f), gef::DegToRad(15.0f));
	constraints_[SPINE_BOTTOM_TO_MID] = spine_bottom_mid;
	world_->addConstraint(spine_bottom_mid, true);
	//	HIPS SPINE
	btConeTwistConstraint* hip_spine = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::HIPS], *rigid_bodies_[RagdollSkeleton::SPINE_BOTTOM], local_a, local_b);
	hip_spine->setLimit(gef::DegToRad(3.0f), gef::DegToRad(3.0f), gef::DegToRad(5.0f));
	constraints_[HIPS_TO_SPINE_BOTTOM] = hip_spine;
	world_->addConstraint(hip_spine, true);

	//	SPINE TOP RIGHT SHOULDER
	local_a.setOrigin(btVector3(-0.05f, 0.0f, 0.0f));
	//local_a.getBasis().setEulerZYX(0, 0, 0);
	local_b.setOrigin(btVector3(0.08f, 0.0f, 0.0f));
	//local_b.getBasis().setEulerZYX(0, 90, 0);
	btConeTwistConstraint* spine_right_shoulder = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::SPINE_TOP], *rigid_bodies_[RagdollSkeleton::RIGHT_SHOULDER], local_a, local_b);
	spine_right_shoulder->setLimit(gef::DegToRad(40.0f), gef::DegToRad(15.0f), gef::DegToRad(20.0f));
	constraints_[SPINE_TOP_TO_RIGHT_SHOULDER] = spine_right_shoulder;
	world_->addConstraint(spine_right_shoulder, true);
	//	RIGHT SHOULDER TO ARM
	local_a.setOrigin(btVector3(-0.05f, 0.0f, 0.0f));
	local_b.setOrigin(btVector3(0.15f, 0.0f, 0.0f));
	btConeTwistConstraint* right_shoulder_arm = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::RIGHT_SHOULDER], *rigid_bodies_[RagdollSkeleton::RIGHT_UPPER_ARM], local_a, local_b);
	right_shoulder_arm->setLimit(gef::DegToRad(60.0f), gef::DegToRad(45.0f), gef::DegToRad(15.0f));
	constraints_[RIGHT_SHOULDER_TO_RIGHT_UPPER_ARM] = right_shoulder_arm;
	world_->addConstraint(right_shoulder_arm, true);
	//	RIGHT UPPER ARM TO LOWER
	local_a.setOrigin(btVector3(-0.15f, 0.0f, 0.0f));
	local_b.setOrigin(btVector3(0.135f, 0.0f, 0.0f));
	local_b.getBasis().setEulerZYX(0.0f, gef::DegToRad(0.0f), 0.0f);
	btConeTwistConstraint* right_arm_up_low = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::RIGHT_UPPER_ARM], *rigid_bodies_[RagdollSkeleton::RIGHT_LOWER_ARM], local_a, local_b);
	right_arm_up_low->setLimit(gef::DegToRad(20.0f), gef::DegToRad(70.0f), gef::DegToRad(20.0f));
	constraints_[RIGHT_UPPER_ARM_TO_LOWER] = right_arm_up_low;
	world_->addConstraint(right_arm_up_low, true);
	local_b.getBasis().setEulerZYX(0.0f, 0.0f, 0.0f);
	


	//	SPINE TOP LEFT SHOULDER
	local_a.setOrigin(btVector3(0.05f, 0.0f, 0.0f));
	//local_a.getBasis().setEulerZYX(0, 0, 0);
	local_b.setOrigin(btVector3(-0.08f, 0.0f, 0.0f));
	//local_b.getBasis().setEulerZYX(0, 90, 0);
	btConeTwistConstraint* spine_left_shoulder = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::SPINE_TOP], *rigid_bodies_[RagdollSkeleton::LEFT_SHOULDER], local_a, local_b);
	spine_left_shoulder->setLimit(gef::DegToRad(40.0f), gef::DegToRad(15.0f), gef::DegToRad(20.0f));
	constraints_[SPINE_TOP_TO_LEFT_SHOULDER] = spine_left_shoulder;
	world_->addConstraint(spine_left_shoulder, true);
	//	LEFT SHOULDER TO ARM
	local_a.setOrigin(btVector3(0.05f, 0.0f, 0.0f));
	local_b.setOrigin(btVector3(-0.15f, 0.0f, 0.0f));
	btConeTwistConstraint* left_shoulder_arm = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::LEFT_SHOULDER], *rigid_bodies_[RagdollSkeleton::LEFT_UPPER_ARM], local_a, local_b);
	left_shoulder_arm->setLimit(gef::DegToRad(60.0f), gef::DegToRad(60.0f), gef::DegToRad(15.0f));
	constraints_[LEFT_SHOULDER_TO_LEFT_UPPER_ARM] = left_shoulder_arm;
	world_->addConstraint(left_shoulder_arm, true);
	//	LEFT UPPER ARM TO LOWER
	local_a.setOrigin(btVector3(0.15f, 0.0f, 0.0f));
	local_b.setOrigin(btVector3(-0.135f, 0.0f, 0.0f));
	local_b.getBasis().setEulerZYX(0.0f, gef::DegToRad(0.0f), 0.0f);
	btConeTwistConstraint* left_arm_up_low = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::LEFT_UPPER_ARM], *rigid_bodies_[RagdollSkeleton::LEFT_LOWER_ARM], local_a, local_b);
	left_arm_up_low->setLimit(gef::DegToRad(10.0f), gef::DegToRad(70.0f), gef::DegToRad(20.0f));
	constraints_[LEFT_UPPER_ARM_TO_LOWER] = left_arm_up_low;
	world_->addConstraint(left_arm_up_low, true);
	local_b.getBasis().setEulerZYX(0.0f, 0.0f, 0.0f);


	//	HIP LEFT LEG
	local_a.setOrigin(btVector3(0.1f, -0.05f, 0));
	local_b.setOrigin(btVector3(0.0f, 0.26f, 0));
	btConeTwistConstraint* hip_left_leg = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::HIPS], *rigid_bodies_[RagdollSkeleton::LEFT_UPPER_LEG], local_a, local_b);
	hip_left_leg->setLimit(gef::DegToRad(10.0f), gef::DegToRad(5.0f), gef::DegToRad(30.0f));
	constraints_[HIPS_TO_LEFT_UPPER_LEG] = hip_left_leg;
	world_->addConstraint(hip_left_leg, true);
	//	LEFT UPPER LEG TO LOWER
	local_a.setOrigin(btVector3(0.0f, -0.26f, 0));
	local_b.setOrigin(btVector3(0.0f, 0.15f, 0));
	btConeTwistConstraint* left_leg_up_low = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::LEFT_UPPER_LEG], *rigid_bodies_[RagdollSkeleton::LEFT_LOWER_LEG], local_a, local_b);
	left_leg_up_low->setLimit(gef::DegToRad(15.0f), gef::DegToRad(10.0f), gef::DegToRad(30.0f));
	constraints_[LEFT_UPPER_LEG_TO_LOWER] = left_leg_up_low;
	world_->addConstraint(left_leg_up_low, true);
	//	LEFT LEG TO FOOT
	local_a.setOrigin(btVector3(0.0f, -0.15f, 0));
	local_b.setOrigin(btVector3(0.0f, 0.15f, -0.1f));
	btConeTwistConstraint* left_leg_foot = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::LEFT_LOWER_LEG], *rigid_bodies_[RagdollSkeleton::LEFT_FOOT], local_a, local_b);
	left_leg_foot->setLimit(gef::DegToRad(6.0f), gef::DegToRad(6.0f), gef::DegToRad(3.0f));
	constraints_[LEFT_LOWER_LEG_TO_FOOT] = left_leg_foot;
	world_->addConstraint(left_leg_foot, true);

	//	HIP RIGHT LEG
	local_a.setOrigin(btVector3(-0.1f, -0.05f, 0));
	local_b.setOrigin(btVector3(0.0f, 0.26f, 0));
	btConeTwistConstraint* hip_right_leg = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::HIPS], *rigid_bodies_[RagdollSkeleton::RIGHT_UPPER_LEG], local_a, local_b);
	hip_right_leg->setLimit(gef::DegToRad(10.0f), gef::DegToRad(5.0f), gef::DegToRad(30.0f));
	constraints_[HIPS_TO_RIGHT_UPPER_LEG] = hip_right_leg;
	world_->addConstraint(hip_right_leg, true);
	//RiGHT UPPER LEG TO LOWER
	local_a.setOrigin(btVector3(0.0f, -0.26f, 0));
	local_b.setOrigin(btVector3(0.0f, 0.15f, 0));
	btConeTwistConstraint* right_leg_up_low = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::RIGHT_UPPER_LEG], *rigid_bodies_[RagdollSkeleton::RIGHT_LOWER_LEG], local_a, local_b);
	right_leg_up_low->setLimit(gef::DegToRad(15.0f), gef::DegToRad(10.0f), gef::DegToRad(30.0f));
	constraints_[RIGHT_UPPER_LEG_TO_LOWER] = right_leg_up_low;
	world_->addConstraint(right_leg_up_low, true);
	//	RIGHT LEG TO FOOT
	local_a.setOrigin(btVector3(0.0f, -0.15f, 0));
	local_b.setOrigin(btVector3(0.0f, 0.15f, -0.1f));
	btConeTwistConstraint* right_leg_foot = new btConeTwistConstraint(*rigid_bodies_[RagdollSkeleton::RIGHT_LOWER_LEG], *rigid_bodies_[RagdollSkeleton::RIGHT_FOOT], local_a, local_b);
	right_leg_foot->setLimit(gef::DegToRad(6.0f), gef::DegToRad(6.0f), gef::DegToRad(3.0f));
	constraints_[RIGHT_LOWER_LEG_TO_FOOT] = right_leg_foot;
	world_->addConstraint(right_leg_foot, true);
}

Ragdoll::~Ragdoll()
{
	//	Cleanup.
	if (world_)
	{
		delete world_;
		world_ = nullptr;
	}

	for (int y = 0; y < RagdollSkeleton::BODYPART_COUNT; y++)
	{
		if (shapes_[y])
		{
			delete shapes_[y];
			shapes_[y] = nullptr;
		}
		if (rigid_bodies_[y])
		{
			if (rigid_bodies_[y]->getMotionState())
			{
				delete rigid_bodies_[y]->getMotionState();
			}
			world_->removeRigidBody(rigid_bodies_[y]);
			delete rigid_bodies_[y];
			rigid_bodies_[y] = nullptr;
		}
	}
	for(int i = 0 ; i < CONSTRAINT_COUNT; i++)
	{
		if (constraints_[i])
		{
			world_->removeConstraint(constraints_[i]);
			delete constraints_[i];
			constraints_[i] = nullptr;
		}
	}
}