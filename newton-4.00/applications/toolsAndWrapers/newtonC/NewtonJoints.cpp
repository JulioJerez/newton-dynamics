/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "newtonStdafx.h"
#include "Newton.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"


int NewtonJointIsActive(const NewtonJoint* const jointPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const joint = (dgConstraint*)jointPtr;
	//return joint->IsActive() ? 1 : 0;
	ndAssert(0);
	return 0;
}


/*!
  Store a user defined data value with the joint.

  @param *joint pointer to the joint.
  @param *userData pointer to the user defined user data value.

  @return Nothing.

  The application can store a user defined value with the Joint. This value can be the pointer to a structure containing some application data for special effect.
  if the application allocate some resource to store the user data, the application can register a joint destructor to get rid of the allocated resource when the Joint is destroyed

  See also: ::NewtonJointSetDestructor
*/
void NewtonJointSetUserData(const NewtonJoint* const joint, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//contraint->SetUserData(userData);
	ndAssert(0);
}

/*!
  Retrieve a user defined data value stored with the joint.

  @param *joint pointer to the joint.

  @return The user defined data.

  The application can store a user defined value with a joint. This value can be the pointer
  to a structure to store some game play data for special effect.

  See also: ::NewtonJointSetUserData
*/
void* NewtonJointGetUserData(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//return contraint->GetUserData();
	ndAssert(0);
	return 0;
}

/*!
  Get creation parameters for this joint.

  @param joint is the pointer to a convex collision primitive.
  @param *jointInfo pointer to a collision information record.

  This function can be used by the application for writing file format and for serialization.

  See also: ::// See also:
*/
void NewtonJointGetInfo(const NewtonJoint* const joint, NewtonJointRecord* const jointInfo)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//contraint->GetInfo((dgConstraintInfo*)jointInfo);
	ndAssert(0);
}

/*!
  Get the first body connected by this joint.

  @param *joint is the pointer to a convex collision primitive.


  See also: ::// See also:
*/
NewtonBody* NewtonJointGetBody0(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//dgBody* const body = contraint->GetBody0();
	//dgWorld* const world = body->GetWorld();
	//return (world->GetSentinelBody() != body) ? (NewtonBody*)body : NULL;
	ndAssert(0);
	return 0;
}


/*!
  Get the second body connected by this joint.

  @param *joint is the pointer to a convex collision primitive.

  See also: ::// See also:
*/
NewtonBody* NewtonJointGetBody1(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//dgBody* const body = contraint->GetBody1();
	//dgWorld* const world = body->GetWorld();
	//return (world->GetSentinelBody() != body) ? (NewtonBody*)body : NULL;
	ndAssert(0);
	return 0;
}


/*!
  Enable or disable collision between the two bodies linked by this joint. The default state is collision disable when the joint is created.

  @param *joint pointer to the joint.
  @param state collision state, zero mean disable collision, non zero enable collision between linked bodies.

  @return nothing.

  usually when two bodies are linked by a joint, the application wants collision between this two bodies to be disabled.
  This is the default behavior of joints when they are created, however when this behavior is not desired the application can change
  it by setting collision on. If the application decides to enable collision between jointed bodies, the application should make sure the
  collision geometry do not collide in the work space of the joint.

  if the joint is destroyed the collision state of the two bodies linked by this joint is determined by the material pair assigned to each body.

  See also: ::NewtonJointGetCollisionState, ::NewtonBodySetJointRecursiveCollision
*/
void NewtonJointSetCollisionState(const NewtonJoint* const joint, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//return contraint->SetCollidable(state ? true : false);
	ndAssert(0);
}

/*!
  Get the collision state of the two bodies linked by the joint.

  @param *joint pointer to the joint.

  @return the collision state.

  usually when two bodies are linked by a joint, the application wants collision between this two bodies to be disabled.
  This is the default behavior of joints when they are created, however when this behavior is not desired the application can change
  it by setting collision on. If the application decides to enable collision between jointed bodies, the application should make sure the
  collision geometry do not collide in the work space of the joint.

  See also: ::NewtonJointSetCollisionState
*/
int NewtonJointGetCollisionState(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//return contraint->IsCollidable() ? 1 : 0;
	ndAssert(0);
	return 0;
}


/*!
  Set the strength coefficient to be applied to the joint reaction forces.

  @param *joint pointer to the joint.
  @param stiffness stiffness coefficient, a value between 0, and 1.0, the default value for most joint is 0.9

  @return nothing.

  Constraint keep bodies together by calculating the exact force necessary to cancel the relative acceleration between one or
  more common points fixed in the two bodies. The problem is that when the bodies drift apart due to numerical integration inaccuracies,
  the reaction force work to pull eliminated the error but at the expense of adding extra energy to the system, does violating the rule
  that constraint forces must be work less. This is a inevitable situation and the only think we can do is to minimize the effect of the
  extra energy by dampening the force by some amount. In essence the stiffness coefficient tell Newton calculate the precise reaction force
  by only apply a fraction of it to the joint point. And value of 1.0 will apply the exact force, and a value of zero will apply only
  10 percent.

  The stiffness is set to a all around value that work well for most situation, however the application can play with these
  parameter to make finals adjustment. A high value will make the joint stronger but more prompt to vibration of instability; a low
  value will make the joint more stable but weaker.

  See also: ::NewtonJointGetStiffness
*/
void NewtonJointSetStiffness(const NewtonJoint* const joint, dFloat stiffness)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//contraint->SetStiffness(dgFloat32(1.0f) - stiffness);
	ndAssert(0);
}

/*!
  Get the strength coefficient bing applied to the joint reaction forces.

  @param *joint pointer to the joint.

  @return stiffness coefficient.

  Constraint keep bodies together by calculating the exact force necessary to cancel the relative acceleration between one or
  more common points fixed in the two bodies. The problem is that when the bodies drift apart due to numerical integration inaccuracies,
  the reaction force work to pull eliminated the error but at the expense of adding extra energy to the system, does violating the rule
  that constraint forces must be work less. This is a inevitable situation and the only think we can do is to minimize the effect of the
  extra energy by dampening the force by some amount. In essence the stiffness coefficient tell Newton calculate the precise reaction force
  by only apply a fraction of it to the joint point. And value of 1.0 will apply the exact force, and a value of zero will apply only
  10 percent.

  The stiffness is set to a all around value that work well for most situation, however the application can play with these
  parameter to make finals adjustment. A high value will make the joint stronger but more prompt to vibration of instability; a low
  value will make the joint more stable but weaker.

  See also: ::NewtonJointSetStiffness
*/
dFloat NewtonJointGetStiffness(const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//return dgFloat32(1.0f) - contraint->GetStiffness();
	ndAssert(0);
	return 0;
}

/*!
  Register a destructor callback to be called when the joint is about to be destroyed.

  @param *joint pointer to the joint.
  @param destructor pointer to the joint destructor callback.

  @return nothing.

  If application stores any resource with the joint, or the application wants to be notified when the
  joint is about to be destroyed. The application can register a destructor call back with the joint.

  See also: ::NewtonJointSetUserData
*/
void NewtonJointSetDestructor(const NewtonJoint* const joint, NewtonConstraintDestructor destructor)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgConstraint* const contraint = (dgConstraint*)joint;
	//contraint->SetDestructorCallback((OnConstraintDestroy)destructor);
	ndAssert(0);
}


/*!
  destroy a joint.

  @param *newtonWorld is the pointer to the body.
  @param *joint pointer to joint to be destroyed

  @return nothing

  The application can call this function when it wants to destroy a joint. This function can be used by the application to simulate
  breakable joints

  See also: ::NewtonConstraintCreateSlider
*/
void NewtonDestroyJoint(const NewtonWorld* const newtonWorld, const NewtonJoint* const joint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//world->DestroyJoint((dgConstraint*)joint);
	ndAssert(0);
}

