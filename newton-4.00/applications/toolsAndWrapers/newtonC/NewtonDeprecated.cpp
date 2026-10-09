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


int NewtonGetParallelSolverOnLargeIsland(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	return 0;
}

int NewtonGetBroadphaseAlgorithm(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	return 0;
}


void NewtonResetBroadphase(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
}

dFloat NewtonGetContactMergeTolerance(const NewtonWorld* const)
{
	TRACE_FUNCTION(__FUNCTION__);
	return ndFloat32(0.0f);
}

void NewtonSetContactMergeTolerance(const NewtonWorld* const, dFloat)
{
	TRACE_FUNCTION(__FUNCTION__);
}

/*!
  Return the AABB of the body on this island

  @param island Pointer to simulation island.
  @param bodyIndex index to the body in current island.
  @param p0 - fixme
  @param p1 - fixme

  This function can only be called from an island update callback.

  See also: ::NewtonSetIslandUpdateEvent
*/
void NewtonIslandGetBodyAABB(const void* const, int, dFloat* const, dFloat* const)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
	ndAssert(0);
}

/*!
  Set a function callback to be call on each island update.

  @param *newtonWorld Pointer to the Newton world.
  @param islandUpdate callback function.

  @return Nothing.

  The application can set a function callback to be called just after the array
  of all bodies making an island of connected bodies are collected. This
  function will be called just before the array is accepted for contact
  resolution and integration.

  The callback function must return an integer 0 or 1 to either skip or process
  the bodies in that particular island.

  Applications can leverage this function to implement an game physics LOD. For
  example the application can determine the AABB of the island and check it
  against the view frustum. If the entire island AABB is invisible, then the
  application can suspend its simulation, even if it is not in equilibrium.

  Other possible applications are to implement of a visual debugger, or freeze
  entire islands for application specific reasons.

  The application must not create, modify, or destroy bodies inside the callback
  or risk putting the engine into an undefined state (ie it will crash, if you
  are lucky).

  See also: ::NewtonIslandGetBody
*/
void NewtonSetIslandUpdateEvent(const NewtonWorld* const, NewtonIslandUpdate)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
	ndAssert(0);
}

/*!
  Retrieve body by index from island.

  @param island Pointer to simulation island.
  @param bodyIndex Index of body on current island.

  @return requested body. fixme: does it return NULL on error?

  This function can only be called from an island update callback.

  See also: ::NewtonSetIslandUpdateEvent
*/
NewtonBody* NewtonIslandGetBody(const void* const island, int bodyIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
	ndAssert(0);
	return nullptr;
}


/*!
  this function block all other threads from executing the same subsequent code simultaneously.

  @param *newtonWorld Pointer to the Newton world.
  @param threadIndex thread index from whe thsi function is called, zero if call form outsize a newton update

  this function should use to present racing conditions when when a call back ins executed form a mutithreaded loop.
  In general most call back are thread safe when they do not write to object outside the scope of the call back.
  this means for example that the application can modify values of object pointed by the arguments and or call that function
  that are allowed to be call for such callback.
  There are cases, however, when the application need to collect data for the client logic, example of such case are collecting
  information to display debug information, of collecting data for feedback.
  In these situations it is possible the the same critical code could be execute at the same time but several thread causing unpredictable side effect.
  so it is necessary to block all of the thread from executing any pieces of critical code.

  Not calling function *NewtonWorldCriticalSectionUnlock* will result on the engine going into an infinite loop.

  it is important that the critical section wrapped by functions *NewtonWorldCriticalSectionLock* and
  *NewtonWorldCriticalSectionUnlock* be keep small if the application is using the multi threaded functionality of the engine
  no doing so will lead to serialization of the parallel treads since only one thread can run the a critical section at a time.

  @return Nothing.

  See also: ::NewtonWorldCriticalSectionUnlock
*/
void NewtonWorldCriticalSectionLock(const NewtonWorld* const, int)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
	ndAssert(0);
}

/*!
  this function block all other threads from executing the same subsequent code simultaneously.

  @param *newtonWorld Pointer to the Newton world.


  this function should use to present racing conditions when when a call back ins executed form a multi threaded loop.
  In general most call back are thread safe when they do not write to object outside the scope of the call back.
  this means for example that the application can modify values of object pointed by the arguments and or call that function
  that are allowed to be call for such callback.
  There are cases, however, when the application need to collect data for the client logic, example of such case are collecting
  information to display debug information, of collecting data for feedback.
  In these situations it is possible the the same critical code could be execute at the same time but several thread causing unpredictable side effect.
  so it is necessary to block all of the thread from executing any pieces of critical code.

  it is important that the critical section wrapped by functions *NewtonWorldCriticalSectionLock* and
  *NewtonWorldCriticalSectionUnlock* be keep small if the application is using the multi threaded functionality of the engine
  no doing so will lead to serialization of the parallel treads since only one thread can run the a critical section at a time.

  @return Nothing.

  See also: ::NewtonWorldCriticalSectionLock
*/
void NewtonWorldCriticalSectionUnlock(const NewtonWorld* const)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
	ndAssert(0);
}

int NewtonAtomicAdd(int* const ptr, int value)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
	ndAssert(0);
	return value;
}

void NewtonDispachThreadJob(const NewtonWorld* const, NewtonJobTask, void* const, const char* const)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);
}

void NewtonLoadPlugins(const NewtonWorld* const, const char* const)
{
	TRACE_FUNCTION(__FUNCTION__);
}

void NewtonUnloadPlugins(const NewtonWorld* const)
{
	TRACE_FUNCTION(__FUNCTION__);
}

void* NewtonCollisionAggregateCreate(NewtonWorld* const worldPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgWorld* const world = (dgWorld*)worldPtr;
	//return world->CreateAggreGate();
	ndAssert(0);
	return 0;
}

void NewtonCollisionAggregateDestroy(void* const aggregatePtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)aggregatePtr;
	//aggregate->m_broadPhase->GetWorld()->DestroyAggregate(aggregate);
	ndAssert(0);
}

void NewtonCollisionAggregateAddBody(void* const aggregatePtr, const NewtonBody* const body)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)aggregatePtr;
	//aggregate->AddBody((dgBody*)body);
	ndAssert(0);
}

void NewtonCollisionAggregateRemoveBody(void* const aggregatePtr, const NewtonBody* const body)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)aggregatePtr;
	//aggregate->RemoveBody((dgBody*)body);
	ndAssert(0);
}

int NewtonCollisionAggregateGetSelfCollision(void* const aggregatePtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)aggregatePtr;
	//return aggregate->GetSelfCollision() ? true : false;
	ndAssert(0);
	return 0;
}

void NewtonCollisionAggregateSetSelfCollision(void* const aggregatePtr, int state)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)aggregatePtr;
	//aggregate->SetSelfCollision(state ? true : false);
	ndAssert(0);
}

/*!
  set a function call back to be call during the face query of a collision tree.

  @param *staticCollision is the pointer to the static collision (a CollisionTree of a HeightFieldCollision)
  @param *userCallback pointer to an event function to call before Newton evaluates the polygons colliding with a body. This parameter can be NULL.

  because debug display display report all the faces of a collision primitive, it could get slow on very large static collision.
  this function can be used for debugging purpose to just report only faces intersection the collision AABB of the collision shape colliding with the polyginal mesh collision.

  this function is not recommended to use for production code only for debug purpose.

  See also: ::NewtonTreeCollisionGetFaceAttribute, ::NewtonTreeCollisionSetFaceAttribute
*/
void NewtonStaticCollisionSetDebugCallback(const NewtonCollision* const staticCollision, NewtonTreeCollisionCallback userCallback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated funtion: %s\n", __FUNCDNAME__));
}

void NewtonSelectBroadphaseAlgorithm(const NewtonWorld* const newtonWorld, int algorithmType)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated funtion: %s\n", __FUNCDNAME__));
}

/*!
  Enable/disable multi-threaded constraint resolution for large islands
  (disabled by default).

  @param *newtonWorld Pointer to the Newton world.
  @param mode 1: enabled  0: disabled (default)

  @return Nothing

  Multi threaded mode is not always faster. Among the reasons are

  1 - Significant software cost to set up threads, as well as instruction overhead.
  2 - Different systems have different cost for running separate threads in a shared memory environment.
  3 - Parallel algorithms often have decreased converge rate. This can be as
	  high as half of the of the sequential version. Consequently, the parallel
	  solver requires a higher number of interactions to achieve similar convergence.

  It is recommended this option is enabled on system with more than two cores,
  since the performance gain in a dual core system are marginally better. Your
  mileage may vary.

  At the very least the application must test the option to verify the performance gains.

  This option has no impact on other subsystems of the engine.

  See also: ::NewtonGetThreadsCount, ::NewtonSetThreadsCount
*/
void NewtonSetParallelSolverOnLargeIsland(const NewtonWorld* const, int)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndTrace(("deprecated funtion: %s\n", __FUNCDNAME__));
}
