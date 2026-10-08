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


/*!
  Return to number of contact in this contact joint.

  @param *contactJoint pointer to corrent contact joint.

  @return number of contacts.

  See also: ::NewtonContactJointGetFirstContact, ::NewtonContactJointGetNextContact, ::NewtonContactJointRemoveContact
*/
int NewtonContactJointGetContactCount(const NewtonJoint* const contactJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//
	//if ((joint->GetId() == dgConstraint::m_contactConstraint) && joint->GetCount()) {
	//	return joint->GetCount();
	//}
	//else {
	//	return 0;
	//}
	ndAssert(0);
	return 0;
}


/*!
  Return to pointer to the first contact from the contact array of the contact joint.

  @param *contactJoint pointer to a contact joint.

  @return a pointer to the first contact from the contact array, NULL if no contacts exist

  See also: ::NewtonContactJointGetNextContact, ::NewtonContactGetMaterial, ::NewtonContactJointRemoveContact
*/
void* NewtonContactJointGetFirstContact(const NewtonJoint* const contactJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//if ((joint->GetId() == dgConstraint::m_contactConstraint) && joint->GetCount() && joint->GetMaxDOF()) {
	//	return joint->GetFirst();
	//}
	//else {
	//	return NULL;
	//}
	ndAssert(0);
	return 0;
}

/*!
  Return a pointer to the next contact from the contact array of the contact joint.

  @param *contactJoint pointer to a contact joint.
  @param *contact pointer to current contact.

  @return a pointer to the next contact in the contact array,  NULL if no contacts exist.

  See also: ::NewtonContactJointGetFirstContact, ::NewtonContactGetMaterial, ::NewtonContactJointRemoveContact
*/
void* NewtonContactJointGetNextContact(const NewtonJoint* const contactJoint, void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//
	//if ((joint->GetId() == dgConstraint::m_contactConstraint) && joint->GetCount()) {
	//	dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//	return node->GetNext();
	//}
	//else {
	//	return NULL;
	//}
	ndAssert(0);
	return 0;
}


/*!
  Return to the next contact from the contact array of the contact joint.

  @param *contactJoint pointer to corrent contact joint.
  @param *contact pointer to current contact.

  @return first contact contact array of the joint contact exist, NULL otherwise

  See also: ::NewtonBodyGetFirstContactJoint, ::NewtonBodyGetNextContactJoint, ::NewtonContactJointGetFirstContact, ::NewtonContactJointGetNextContact
*/
void NewtonContactJointRemoveContact(const NewtonJoint* const contactJoint, void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//
	//if ((joint->GetId() == dgConstraint::m_contactConstraint) && joint->GetCount()) {
	//	dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//
	//	dgAssert(joint->GetBody0());
	//	dgAssert(joint->GetBody1());
	//	dgWorld* const world = joint->GetBody0()->GetWorld();
	//	world->GlobalLock();
	//	joint->Remove(node);
	//	joint->GetBody0()->SetSleepState(false);
	//	joint->GetBody1()->SetSleepState(false);
	//	world->GlobalUnlock();
	//}
	ndAssert(0);
}

dFloat NewtonContactJointGetClosestDistance(const NewtonJoint* const contactJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//return joint->GetClosestDistance();
	ndAssert(0);
	return 0;
}

void NewtonContactJointResetSelftJointCollision(const NewtonJoint* const contactJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//joint->ResetSkeletonSelftCollision();
	ndAssert(0);
}

void NewtonContactJointResetIntraJointCollision(const NewtonJoint* const contactJoint)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgContact* const joint = (dgContact*)contactJoint;
	//joint->ResetSkeletonIntraCollision();
	ndAssert(0);
}

/*!
  Return to the next contact from the contact array of the contact joint.

  @param *contact pointer to current contact.

  @return first contact contact array of the joint contact exist, NULL otherwise

  See also: ::NewtonContactJointGetFirstContact, ::NewtonContactJointGetNextContact
*/
NewtonMaterial* NewtonContactGetMaterial(const void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//dgContactMaterial& contactMaterial = node->GetInfo();
	//return (NewtonMaterial*)&contactMaterial;
	ndAssert(0);
	return 0;
}

NewtonCollision* NewtonContactGetCollision0(const void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//dgContactMaterial& contactMaterial = node->GetInfo();
	//return (NewtonCollision*)contactMaterial.m_collision0;

	ndAssert(0);
	return 0;
}

NewtonCollision* NewtonContactGetCollision1(const void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//dgContactMaterial& contactMaterial = node->GetInfo();
	//return (NewtonCollision*)contactMaterial.m_collision1;
	ndAssert(0);
	return 0;
}

void* NewtonContactGetCollisionID0(const void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//dgContactMaterial& contactMaterial = node->GetInfo();
	//return (void*)contactMaterial.m_shapeId0;
	ndAssert(0);
	return 0;
}

void* NewtonContactGetCollisionID1(const void* const contact)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgList<dgContactMaterial>::dgListNode* const node = (dgList<dgContactMaterial>::dgListNode*) contact;
	//dgContactMaterial& contactMaterial = node->GetInfo();
	//return (NewtonCollision*)contactMaterial.m_shapeId1;
	ndAssert(0);
	return 0;
}
