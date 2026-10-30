//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************
#include <stdafx.h>

#include "MousePick.h"


static dFloat pickedParam;
static dVector pickedForce;
static dVector rayLocalNormal;
static dVector rayWorldNormal;
static dVector rayWorldOrigin;
static dVector attachmentPoint;

static bool isPickedBodyDynamics;
static NewtonBody* pickedBody;
static NewtonApplyForceAndTorque chainForceCallBack; 

#define SHOW_RAYS_ON_STATI_BODIES


// implement a ray cast pre-filter
static unsigned RayCastPrefilter (const NewtonBody* body,  const NewtonCollision* collision, void* userData)
{
	// alway cast the primitive 
	return 1;
}

// implement a ray cast filter
static dFloat RayCastFilter (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
#ifndef SHOW_RAYS_ON_STATI_BODIES
	if (mass > 0.0f) {
#endif
		if (intersetParam <  pickedParam) {
			isPickedBodyDynamics = mass > 0.0f;
			pickedParam = intersetParam;
			pickedBody = (NewtonBody*)body;
			rayLocalNormal = dVector (normal[0], normal[1], normal[2]);
		}
#ifndef SHOW_RAYS_ON_STATI_BODIES
	}
#endif

	return intersetParam;
}


static void PhysicsApplyPickForce (const NewtonBody* body)
{

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dVector veloc;
	dVector omega;
	dMatrix matrix;

	// apply the thew body forces
	if (chainForceCallBack)
	{
		chainForceCallBack(body);
	}


	// add the mouse pick penalty force and torque
	NewtonBodyGetVelocity(body, &veloc[0]);

	NewtonBodyGetOmega(body, &omega[0]);
	NewtonBodyGetVelocity(body, &veloc[0]);
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dVector force (pickedForce.Scale (mass * 100.0f));
	dVector dampForce (veloc.Scale (10.0f * mass));
	force -= dampForce;

	NewtonBodyGetMatrix(body, &matrix[0][0]);

	// get the center of mass for bodies with offset
	dVector com;
	NewtonBodyGetCentreOfMass (body, &com[0]);

	dVector point (matrix.RotateVector (attachmentPoint - com));
	dVector torque (point * force);

	dVector torqueDamp (omega.Scale (mass * 0.1f));

	NewtonBodyAddForce (body, &force.m_x);
	NewtonBodyAddTorque (body, &torque.m_x);
}


bool MousePick (
	NewtonWorld* nWorld, 
	MOUSE_POINT mouse1, 
	dInt32 mouseLeftKey1,
	dFloat witdh,
	dFloat length)
{
	static int mouseLeftKey0;
	static MOUSE_POINT mouse0;
	static int saveAutoFreeMode;
	static bool mousePickMode = false;

	dMatrix matrix;
	if (mouseLeftKey1) {
		if (!mouseLeftKey0) {
			dVector p0 (ScreenToWorld(mouse1.x, mouse1.y, 0));
			dVector p1 (ScreenToWorld(mouse1.x, mouse1.y, 1));

			pickedBody = NULL;
			pickedParam = 1.1f;
			NewtonWorldRayCast(nWorld, &p0[0], &p1[0], RayCastFilter, NULL, RayCastPrefilter);
			if (pickedBody) {
				mousePickMode = true;
				NewtonBodyGetMatrix(pickedBody, &matrix[0][0]);
				dVector p (p0 + (p1 - p0).Scale (pickedParam));

				dVector localCenterOfMass;

				// calculate local point relative to center of mass
				attachmentPoint = matrix.UntransformVector (p);


				// convert normal to local space
				rayLocalNormal = matrix.UnrotateVector(rayLocalNormal);

				// save the transform call back
				chainForceCallBack = NewtonBodyGetForceAndTorqueCallback (pickedBody);

				// set a new call back
				NewtonBodySetForceAndTorqueCallback (pickedBody, PhysicsApplyPickForce);
				saveAutoFreeMode = NewtonBodyGetAutoFreeze (pickedBody);
				NewtonBodySetAutoFreeze (pickedBody, 0);
				NewtonWorldUnfreezeBody (nWorld, pickedBody);
			}
		}

		if (mousePickMode) {
			// init pick mode
			dMatrix matrix;
			NewtonBodyGetMatrix(pickedBody, &matrix[0][0]);
			dVector p0 (ScreenToWorld(mouse1.x, mouse1.y, 0));
			dVector p1 (ScreenToWorld(mouse1.x, mouse1.y, 1));
			dVector p2 (matrix.TransformVector (attachmentPoint));

			dVector p (p0 + (p1 - p0).Scale (pickedParam));
			pickedForce = p - p2;

			// rotate normal to global space
			rayWorldNormal = matrix.RotateVector(rayLocalNormal);
			rayWorldOrigin = p2;

			// show the pick points
			ShowMousePicking (p, p2, witdh);
			ShowMousePicking (rayWorldOrigin, rayWorldOrigin + rayWorldNormal.Scale (length), witdh);
		}
	} else {
		mousePickMode = false;
		if (pickedBody) {
			NewtonWorldUnfreezeBody (nWorld, pickedBody);
			NewtonBodySetAutoFreeze (pickedBody, saveAutoFreeMode);
			NewtonBodySetForceAndTorqueCallback (pickedBody, chainForceCallBack);
			pickedBody = NULL;
			chainForceCallBack = NULL;
		}
	}

	mouse0 = mouse1;
	mouseLeftKey0 = mouseLeftKey1;

	bool retState = mousePickMode;

#ifdef SHOW_RAYS_ON_STATI_BODIES
	retState = isPickedBodyDynamics;
#endif

	return retState;
}

