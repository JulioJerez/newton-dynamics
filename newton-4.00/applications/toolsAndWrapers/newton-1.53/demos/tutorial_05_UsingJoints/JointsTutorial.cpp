//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// JointsTutorial.cpp: implementation of the JointsTutorial class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "tutorial.h"
#include "Materials.h"
#include "JointsTutorial.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// create a rope of boxes
void AddRope (NewtonWorld* nWorld, dSceneNode* root)
{
	int i;
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* link0;
	NewtonBody* link1;
	NewtonJoint* joint;
	NewtonCollision* collision;
	RenderPrimitive* visualObject;

	dVector size (2.0f, 0.25f, 0.25f);

	//dMatrix location (GetIdentityMatrix());
	dMatrix location (dgRollMatrix(3.1426f * 0.5f));
	location.m_posit.m_y =  9.0f; 
	location.m_posit.m_z = -5.0f; 

	// create a collision primitive to be shared by all links
	collision = NewtonCreateCapsule (nWorld, size.m_y, size.m_x, NULL);

	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = 2.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	link0 = NULL;
	for (i = 0; i < 7; i ++) {
		// create the a graphic character (use a visualObject as our body
		visualObject = new CapsulePrimitive (root, location, size.m_y, size.m_x);

		//create the rigid body
		link1 = NewtonCreateBody (nWorld, collision);

		// add some damping to each link
		NewtonBodySetLinearDamping (link1, 0.2f);
		dVector angularDamp (0.2f, 0.2f, 0.2f);
		NewtonBodySetAngularDamping (link1, &angularDamp.m_x);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link1, woodID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link1, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link1, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link1, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link1, PhysicsApplyForceAndTorque);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0]);

		dVector pivot (location.m_posit);
		pivot.m_y += (size.m_x - size.m_y) * 0.5f;


		// connect these two bodies by a ball and socket joint
		joint = NewtonConstraintCreateBall (nWorld, &pivot.m_x, link1, link0);


		// override the stiffness, 
		// adjust this parameter to see foe the joint become more or less strong and stable 
		NewtonJointSetStiffness (joint, 0.9f);
		//dFloat stiffness = NewtonJointGetStiffness (joint);

		// set a twist angle to prevent the links to spin unrealistically
		dVector pin (location.m_front.Scale (-1.0f));
		NewtonBallSetConeLimits (joint, &pin.m_x, 0.0f, 10.0f * 3.1416f / 180.0f);

		link0 = link1;
		location.m_posit.m_y -= (size.m_x - size.m_y);
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

}




static unsigned DoubleDoorUserCallback (const NewtonJoint* hinge, NewtonHingeSliderUpdateDesc* desc)
{
	dFloat angle;
	const dFloat angleLimit = 90.0f;

	angle = NewtonHingeGetJointAngle (hinge);
	if (angle > ((angleLimit / 180.0f) * 3.1416f)) {
		// if the joint angle is large than the predefine interval, stop the hinge
		desc->m_accel = NewtonHingeCalculateStopAlpha (hinge, desc, (angleLimit / 180.0f) * 3.1416f);
		return 1;
	} else if (angle < ((-angleLimit / 180.0f) * 3.1416f)) {
		// if the joint angle is large than the predefine interval, stop the hinge
		desc->m_accel = NewtonHingeCalculateStopAlpha (hinge, desc, (-angleLimit / 180.0f) * 3.1416f);
		return 1;
	}

	// no action need it if the joint angle is with the limits
	return 0;
}


void AddDoubleSwingDoors (NewtonWorld* nWorld, dSceneNode* root)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* link0;
	NewtonBody* link1;
	BoxPrimitive* visualObject;
	NewtonJoint* joint;
	NewtonCollision* collision;

	dVector size (2.0f, 5.0f, 0.5f);

	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = -2.0f; 
	location.m_posit.m_y =  0.0f; 
	location.m_posit.m_z = -2.0f; 

	// create a collision primitive to be shared by all links
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = 2.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// make first wing
	{
		// create the a graphic character (use a visualObject as our body
		visualObject = new BoxPrimitive (root, location, size);

		//create the rigid body
		link1 = NewtonCreateBody (nWorld, collision);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link1, woodID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link1, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link1, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link1, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link1, PhysicsApplyForceAndTorque);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0]);

		dVector pivot (location.m_posit);
		dVector pin (location.m_up);
		pivot.m_x += size.m_x * 0.5f;

		// connect these two bodies by a ball and sockect joint
		joint = NewtonConstraintCreateHinge (nWorld, &pivot.m_x, &pin.m_x, link1, NULL);

		// set limit to the door
		NewtonHingeSetUserCallback (joint, DoubleDoorUserCallback);

	}


	// make second wing
	{

		location.m_posit.m_x -= size.m_x;

		// create the a graphic character (use a visualObject as our body
		visualObject = new BoxPrimitive (root, location, size);

		//create the rigid body
		link0 = NewtonCreateBody (nWorld, collision);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link0, woodID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link0, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link0, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link0, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link0, PhysicsApplyForceAndTorque);

		// set the mass matrix
		NewtonBodySetMassMatrix (link0, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link0, &location[0][0]);
		PhysicsSetTransform (link0, &location[0][0]);

		dVector pivot (location.m_posit);
		dVector pin (location.m_up);
		pivot.m_x += size.m_x * 0.5f;

		// connect these two bodies by a ball and socket joint
		joint = NewtonConstraintCreateHinge (nWorld, &pivot.m_x, &pin.m_x, link0, link1);

		// set limit to the door
		NewtonHingeSetUserCallback (joint, DoubleDoorUserCallback);
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);
}



struct JointControlStruct
{
	dFloat m_min;
	dFloat m_max;
};

static void DestroyJointControlStruct(const NewtonJoint* slider)
{
	JointControlStruct* limits;
	limits = (JointControlStruct*) NewtonJointGetUserData (slider);
	delete limits; 
}

static unsigned SliderUpdateEvent (const NewtonJoint* slider, NewtonHingeSliderUpdateDesc* desc)
{
	dFloat distance;
	JointControlStruct* limits;

	limits = (JointControlStruct*) NewtonJointGetUserData (slider);

	distance = NewtonSliderGetJointPosit (slider);
	if (distance < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc->m_accel = NewtonSliderCalculateStopAccel (slider, desc, limits->m_min);
		return 1;
	} else if (distance > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc->m_accel = NewtonSliderCalculateStopAccel (slider, desc, limits->m_max);
		return 1;
	}

	// no action need it if the joint angle is with the limits
	return 0;
}

static unsigned CorkScrewUpdateEvent (const NewtonJoint* screw, NewtonHingeSliderUpdateDesc* desc)
{
	unsigned retCode; 
	dFloat distance;
	dFloat omega;
	JointControlStruct* limits;

	// no action need it if the joint angle is with the limits
	retCode = 0;

	limits = (JointControlStruct*) NewtonJointGetUserData (screw);

	// the first entry in NewtonHingeSliderUpdateDesc control the screw linear acceleration 
	distance = NewtonCorkscrewGetJointPosit (screw);
	if (distance < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc[0].m_accel = NewtonCorkscrewCalculateStopAccel (screw, &desc[0], limits->m_min);
		retCode |= 1;
	} else if (distance > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc[0].m_accel = NewtonCorkscrewCalculateStopAccel (screw, &desc[0], limits->m_max);
		retCode |= 1;
	}

	// the second entry in NewtonHingeSliderUpdateDesc control the screw angular acceleration. 
	// Make s small screw motor by setting the angular acceleration of the screw axis
	// We are not going to limit the angular rotation of the screw, but is we did we should or return code with 2,
	omega = NewtonCorkscrewGetJointOmega (screw);
	desc[1].m_accel = 2.5f - 0.2f * omega;

	// or with 0x10 to tell newton this axis is active
	retCode |= 2;
	
	// return the code
	return retCode;

}


static unsigned UniversalUpdateEvent (const NewtonJoint* screw, NewtonHingeSliderUpdateDesc* desc)
{
	unsigned retCode; 
	dFloat angle;
	dFloat omega;
	JointControlStruct* limits;

	// no action need it if the joint angle is with the limits
	retCode = 0;

// change this to #if 0 put a double limit on the universal joint
#if 1
	// the first entry in is the axis fixes of the spinning beat
	// apply a simple motor to this object
	omega = NewtonUniversalGetJointOmega0 (screw);
	desc[0].m_accel = -4.5f - 0.2f * omega;
	// or with 0x10 to tell newton this axis is active
	retCode |= 1;

#else
	// put a top limit on the joint
	limits = (JointControlStruct*) NewtonJointGetUserData (screw);
	angle = NewtonUniversalGetJointAngle0 (screw);
	if (angle < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc[0].m_accel = NewtonUniversalCalculateStopAlpha0 (screw, &desc[0], limits->m_min);
		retCode |= 1;
	} else if (angle > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc[0].m_accel = NewtonUniversalCalculateStopAlpha0 (screw, &desc[0], limits->m_max);
		retCode |= 1;
	}
#endif



	limits = (JointControlStruct*) NewtonJointGetUserData (screw);

	// the first entry in NewtonHingeSliderUpdateDesc control the screw linear acceleration 
	angle = NewtonUniversalGetJointAngle1 (screw);
	if (angle < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc[1].m_accel = NewtonUniversalCalculateStopAlpha1 (screw, &desc[0], limits->m_min);
		retCode |= 2;
	} else if (angle > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc[1].m_accel = NewtonUniversalCalculateStopAlpha1 (screw, &desc[0], limits->m_max);
		retCode |= 2;
	}
	
	// return the code
	return retCode;

}


void AddRollingBeats (NewtonWorld* nWorld, dSceneNode* root)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* bar;
	NewtonJoint* joint;
	NewtonCollision* collision;


	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x =  5.0f; 
	location.m_posit.m_y = -1.0f; 
	location.m_posit.m_z = -2.0f; 
	dVector size (10.0f, 0.25f, 0.25f);

	bar = NULL;
	// /////////////////////////////////////////////////////////////////////////////////////             
	//
	// create a bar and attach it to the world with a hinge with limits
	//
	// ////////////////////////////////////////////////////////////////////////////////////
	{
		RenderPrimitive* visualObject;

		// create the a graphic character (use a visualObject as our body
		visualObject = new CylinderPrimitive (root, location, size.m_y, size.m_x);

		// create a collision primitive to be shared by all links
		collision = NewtonCreateCylinder (nWorld, size.m_y, size.m_x, NULL); 

		// create the bar body
		bar = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(bar, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (bar, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (bar, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (bar, PhysicsApplyForceAndTorque);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (bar, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid nody and the graphic body
		NewtonBodySetMatrix (bar, &location[0][0]);
		PhysicsSetTransform (bar, &location[0][0]);

		dVector pin (0.0f, 1.0f, 0.0f);
		dVector pivot (location.m_posit);
		pivot.m_x -= size.m_x * 0.5f;
		joint = NewtonConstraintCreateHinge (nWorld, &pivot.m_x, &pin.m_x, bar, NULL);
		NewtonHingeSetUserCallback (joint, DoubleDoorUserCallback);
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a sliding visualObject with limits
		//
		NewtonBody* beat;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 2.0f, 2.0f);

		beatLocation.m_posit.m_x += size.m_x * 0.25f;

		// create the a graphic character (use a visualObject as our body
		visualObject = new BoxPrimitive (root, beatLocation, beatSize);

		// create a collision primitive to be shared by all links
		collision = NewtonCreateBox (nWorld, beatSize.m_x, beatSize.m_y, beatSize.m_z, NULL); 

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

			// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat, PhysicsApplyForceAndTorque);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0]);

		// set the pivot relative for the first bar
		dVector pivot (beatLocation.m_posit); 
		dVector pin (beatLocation.m_front);
		joint = NewtonConstraintCreateSlider (nWorld, &pivot.m_x, &pin.m_x, beat, bar);

		// assign a control function for this joint
		NewtonHingeSetUserCallback (joint, SliderUpdateEvent);

		JointControlStruct *limits;
		limits = new JointControlStruct;

		// calculate the minimum and maximum limit for this joints
		limits->m_min = ((location.m_posit.m_x - beatLocation.m_posit.m_x) - size.m_x * 0.5f);
		limits->m_max = ((location.m_posit.m_x - beatLocation.m_posit.m_x) + size.m_x * 0.5f);

		// store local private data with this joint 
		NewtonJointSetUserData (joint, limits);

		// tell this joint to destroy its local private data when destroyed
		NewtonJointSetDestructor (joint, DestroyJointControlStruct);			
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a corkscrew visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x -= size.m_x * 0.25f;

		// create the a graphic character (use a visualObject as our body
		//visualObject = new BoxPrimitive (beatLocation, beatSize);
		visualObject = new ChamferCylinderPrimitive (root, beatLocation, beatSize.m_y, beatSize.m_x);

		// create a collision primitive to be shared by all links
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, NULL); 

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

			// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat, PhysicsApplyForceAndTorque);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0]);

		// set the pivot relative for the first bar
		dVector pivot (beatLocation.m_posit); 
		dVector pin (beatLocation.m_front);
		joint = NewtonConstraintCreateCorkscrew (nWorld, &pivot.m_x, &pin.m_x, beat, bar);
		
		// assign a control function for this joint
		NewtonCorkscrewSetUserCallback (joint, CorkScrewUpdateEvent);

		JointControlStruct *limits;
		limits = new JointControlStruct;

		// calculate the minimum and maximum limit for this joints
		limits->m_min = ((location.m_posit.m_x - beatLocation.m_posit.m_x) - size.m_x * 0.5f);
		limits->m_max = ((location.m_posit.m_x - beatLocation.m_posit.m_x) + size.m_x * 0.5f);

		// store local private data with this joint 
		NewtonJointSetUserData (joint, limits);

		// tell this joint to destroy its local private data when destroyed
		NewtonJointSetDestructor (joint, DestroyJointControlStruct);			
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a universal joint visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x -= size.m_x * 0.5f;

		// create the a graphic character (use a visualObject as our body
		//visualObject = new BoxPrimitive (beatLocation, beatSize);
		visualObject = new ChamferCylinderPrimitive (root, beatLocation, beatSize.m_y, beatSize.m_x);

		// create a collision primitive to be shared by all links
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, NULL); 

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

			// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat, PhysicsApplyForceAndTorque);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0]);

		// set the pivot relative for the first bar
		dVector pivot (beatLocation.m_posit); 
		dVector pin0 (beatLocation.m_front);
		dVector pin1 (beatLocation.m_up);
		joint = NewtonConstraintCreateUniversal (nWorld, &pivot.m_x, &pin0.m_x, &pin1.m_x, beat, bar);
		
		// assign a control function for this joint
		NewtonCorkscrewSetUserCallback (joint, UniversalUpdateEvent);

		JointControlStruct *limits;
		limits = new JointControlStruct;

		// calculate the minimum and maximum limit for this joints
		limits->m_min = -30.0f * 3.1416/ 180.0f;
		limits->m_max = 30.0f * 3.1416/ 180.0f;

		// store local private data with this joint 
		NewtonJointSetUserData (joint, limits);

		// tell this joint to destroy its local private data when destroyed
		NewtonJointSetDestructor (joint, DestroyJointControlStruct);			
	}
}
