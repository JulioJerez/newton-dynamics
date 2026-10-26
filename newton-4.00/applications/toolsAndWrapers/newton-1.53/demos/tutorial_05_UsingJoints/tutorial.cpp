//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

#include <stdafx.h>
#include "tutorial.h"
#include "HiResTimer.h"
#include "Materials.h"
#include "JointsTutorial.h"
#include "LevelPrimitive.h"
#include "RenderPrimitive.h"


#define GRAVITY	-10.0f
CHiResTimer timer;


static NewtonWorld* nWorld;
static RenderPrimitive* player;
static dSceneNode* nGrapphicWorld;

static dVector cameraDir (0.0f, 0.0f, -1.0f);
static dVector cameraEyepoint (0.0f, 5.0f, 0.0f);

static bool forceMode;
static dVector cameraTorqueVector (0.0f, 0.0f, 0.0f); 



static void CleanUp ();
static void  Keyboard();
static void*  PhysicsAlloc (int sizeInBytes);
static void  PhysicsFree (void *ptr, int sizeInBytes);



static void PopulateLevel ();
static void AddBallCharacter ();
static void  BallCharacterApplyForceAndTorque (const NewtonBody* body);

//********************************************************************
// 
//	Newton Tutorial 2 Using Callbacks
//
//********************************************************************
int main(int argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutorial 5: Using Joints", DrawScene);

	// create the physics scene
	InitScene();
	
	// Execute opengl main loop for ever
	glutMainLoop();   

	return 0;
} 

// memory allocation for Newton
void* PhysicsAlloc (int sizeInBytes)
{
	return malloc (sizeInBytes);
}

// memory de-allocation for Newton
void PhysicsFree (void *ptr, int sizeInBytes)
{
	free (ptr);
}

// destroy the world and every thing in it
void CleanUp ()
{
	// clear all resources use with materials
	CleanUpMaterials (nWorld);

	// destroy the Newton world
	NewtonDestroy (nWorld);
}

// rigid body destructor
void PhysicsBodyDestructor (const NewtonBody* body)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete primitive;
}

// set the transformation of a rigid body
void PhysicsApplyForceAndTorque (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);
}

// add force and torque to rigid body
void PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& mat = *((dMatrix*)matrix);
	primitive->SetMatrix (mat);
}


void PopulateLevel ()
{
	dFloat Ixx = 0.0f;
	dFloat Iyy = 0.0f;
	dFloat Izz = 0.0f;
	dFloat mass = 1.0f; 
	NewtonCollision* collision;

	// set the initial size
	dVector size (1.0f, 1.0f, 1.0f);

	// create the collision 
//	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
	collision = NewtonCreateCapsule (nWorld, size.m_x * 0.5f, size.m_y * 2.0f, NULL); 
//	collision = NewtonCreateCylinder (nWorld, size.m_x * 0.5f, size.m_y * 2.0f, NULL); 
//	collision = NewtonCreateChamferCylinder (nWorld, size.m_x * 0.5f, size.m_y * 2.0f, NULL); 


	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the center of mass to the lower bottom for a special effect
	origin.m_x = -size.m_y;
		
	// populate the world with some boxes 
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = -10.0f; 


	for (int k = 0; k < 4; k ++) { 
		location.m_posit.m_z = -2.0f; 
		for (int j = 0; j < 4; j ++) { 
			location.m_posit.m_y = 10.0f; 
			for (int i = 0; i < 1; i ++) {
				RenderPrimitive* box;
				NewtonBody* boxBody;

				// create a graphic box
//				box = new BoxPrimitive (location, size);
				box = new CapsulePrimitive (nGrapphicWorld, location, size.m_x * 0.5f, size.m_y * 2.0f);
//				box = new CylinderPrimitive (nGrapphicWorld, location, size.m_x * 0.5f, size.m_y * 2.0f);
//				box = new ChamferCylinderPrimitive (nGrapphicWorld, location, size.m_x * 0.5f, size.m_y * 2.0f);

				//create the rigid body
				boxBody = NewtonCreateBody (nWorld, collision);

				// set the center of mass to the lower bottom for a special effect
				NewtonBodySetCentreOfMass (boxBody, &origin[0]);

				//dVector omega(5, 0, 0);
				//NewtonBodySetOmega (boxBody, &omega[0]);

				// Set Material Id for this object
				if (dRand() & 1) {
					NewtonBodySetMaterialGroupID (boxBody, metalID);
				} else {
					NewtonBodySetMaterialGroupID (boxBody, woodID);
				}

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (boxBody, box);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyForceAndTorque);

				// set the mass matrix
				NewtonBodySetMassMatrix (boxBody, mass, Ixx, Iyy, Izz);

				// set the matrix for both the rigid body and the graphic body
				NewtonBodySetMatrix (boxBody, &location[0][0]);
				PhysicsSetTransform (boxBody, &location[0][0]);

				location.m_posit.m_y += size.m_y * 2.0f;
			}
			location.m_posit.m_z -= size.m_z * 4.0f; 	
		}
		location.m_posit.m_x += size.m_x * 4.0f; 
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);
}


// create physics scene
void InitScene()
{
	LevelPrimitive *level;

	// /////////////////////////////////////////////////////////////////////
	//
	// create the newton world
	nWorld = NewtonCreate (PhysicsAlloc, PhysicsFree);

	// Set the termination function
	atexit(CleanUp); 

	// /////////////////////////////////////////////////////////////////////
	//
	// set up all material and  material interactions
	SetupMaterials (nWorld);


	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	nGrapphicWorld = new SkyBoxPrimitive (NULL);


	// /////////////////////////////////////////////////////////////////////
	//
	// Create a ball controllable by the player
	AddBallCharacter ();


	// /////////////////////////////////////////////////////////////////////
	//
	// add some joint configurations
	AddRope (nWorld, nGrapphicWorld);
	AddRollingBeats (nWorld, nGrapphicWorld);
	AddDoubleSwingDoors (nWorld, nGrapphicWorld);

	// add some object to the background
	PopulateLevel ();

	// /////////////////////////////////////////////////////////////////////
	//
	// Load level geometry and add it to the display list 
	level = new LevelPrimitive (nGrapphicWorld, "level1.dg", nWorld, NULL);

	// assign the concrete id to the level geometry
	NewtonBodySetMaterialGroupID (level->m_level, levelID);

}


// DrawScene()
void DrawScene ()
{
	dFloat timeStep;


	// get the time step
	timeStep = timer.GetElapsedSeconds();

	// update the Newton physics world
	NewtonUpdate (nWorld, timeStep);

	// read the keyboard
	Keyboard ();

	// move the camera
	dVector target (cameraEyepoint + cameraDir);
	SetCamera (cameraEyepoint, target);

	// render the scene
	glEnable (GL_LIGHTING);
	glPushMatrix();	
	nGrapphicWorld->Render ();
	glPopMatrix();


	dVector color (1.0, 1.0f, 1.0f);
	Print (color, 4, 4, "fps %f", 1.0f / (timeStep + 1.0e-6f));
} 


//	Keyboard handler. 
void  Keyboard()
{
	// check for termination
	if (dGetKeyState (VK_ESCAPE) & 0x8000) {
		exit(0);
	}

	// read the mouse position and set the camera direction
	static MOUSE_POINT mouse0;
	static dFloat yawAngle = 90.0f * 3.1416 / 180.0f;
	static dFloat rollAngle = 0.0f;

	MOUSE_POINT mouse1;
	GetCursorPos(mouse1);

	if (dGetKeyState (VK_LBUTTON) & 0x8000) {
		if (mouse1.x > (mouse0.x + 1)) {
			yawAngle += 5.0f * 3.1416 / 180.0f;
			if (yawAngle > (360.0f * 3.1416 / 180.0f)) {
				yawAngle -= (360.0f * 3.1416 / 180.0f);
			}
		} else if (mouse1.x < (mouse0.x - 1)) {
			yawAngle -= 5.0f * 3.1416 / 180.0f;
			if (yawAngle < 0.0f) {
				yawAngle += (360.0f * 3.1416 / 180.0f);
			}
		}

		if (mouse1.y > (mouse0.y + 1)) {
			rollAngle += 2.0f * 3.1416 / 180.0f;
			if (rollAngle > (80.0f * 3.1416 / 180.0f)) {
				rollAngle = 80.0f * 3.1416 / 180.0f;
			}
		} else if (mouse1.y < (mouse0.y - 1)) {
			rollAngle -= 2.0f * 3.1416 / 180.0f;
			if (rollAngle < -(80.0f * 3.1416 / 180.0f)) {
				rollAngle = -80.0f * 3.1416 / 180.0f;
			}
		}
		dMatrix cameraDirMat (dgRollMatrix(rollAngle) * dgYawMatrix(yawAngle));
		cameraDir = cameraDirMat.m_front;
	}

	mouse0 = mouse1;


	cameraTorqueVector = dVector (0.0, 0.0, 0.0); 

	// camera control
	if (dGetKeyState ('W') & 0x8000) {
		cameraTorqueVector.m_z = -1.0f;
	} else if (dGetKeyState ('S') & 0x8000) {
		cameraTorqueVector.m_z = 1.0f;
	}

	if (dGetKeyState ('D') & 0x8000) {
		cameraTorqueVector.m_x = 1.0f;
	} else if (dGetKeyState ('A') & 0x8000) {
		cameraTorqueVector.m_x = -1.0f;
	}

	forceMode = false;
	if (dGetKeyState (' ') & 0x8000) {
		forceMode = true;
	}

	if (player) {
		cameraEyepoint = player->GetMatrix().m_posit;
		cameraEyepoint.m_y += 1.0f;

		cameraEyepoint -= cameraDir.Scale (4.0f);
	}
} 




// set the transformation of a rigid body
void BallCharacterApplyForceAndTorque (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);

	// calculate a rotation matrix from the camera heading
	dMatrix matrix (GetIdentityMatrix()); 
	dVector lateralDir (cameraDir * dVector (0.0f, 1.0f, 0.0f));
	matrix.m_right = lateralDir.Scale (1.0f / (lateralDir % lateralDir));
	matrix.m_front = matrix.m_up * matrix.m_right;


	if (forceMode) {
		// this is the flight mode
		dVector force;
		dVector velocity;

		// rotate the force direction to align with the camera
		dVector forceDir (matrix.RotateVector (cameraTorqueVector));
 		forceDir = dgYawMatrix(-3.1416f * 0.5f).RotateVector (forceDir);
		forceDir.m_y = 1.0f;

		
		NewtonBodyGetForce(body, &force.m_x);
		NewtonBodyGetVelocity(body, &velocity.m_x);

		force += (forceDir.Scale (mass * 30.0f) - forceDir.Scale (5.0f * (velocity % forceDir))); 
		NewtonBodySetForce (body, &force.m_x);
	} else {
		// this is the rolling mode
		dVector omega;

		// rotate the torque direction to align with the camera
		dVector torqueDir (matrix.RotateVector (cameraTorqueVector));
		NewtonBodyGetOmega(body, &omega.m_x);
		dVector torque (torqueDir.Scale (Ixx * 75.0f) - torqueDir.Scale (5.0f * Ixx * (omega % torqueDir))); 
		NewtonBodySetTorque (body, &torque.m_x);
	}
}



void AddBallCharacter ()
{
	NewtonBody* sphBody;
	SpherePrimitive* sph;
	NewtonCollision* collision;
	const dFloat radius = 1.0f;

	// create the collision 
	collision = NewtonCreateSphere (nWorld, radius, radius, radius, NULL); 
		
	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = 10; 

	// create the graphic object
	sph = new SpherePrimitive (nGrapphicWorld, location, radius, radius, radius);

	// save as player
	player = sph;

	//create the rigid body
	sphBody = NewtonCreateBody (nWorld, collision);

	// disable auto freeze management for the player
	NewtonBodySetAutoFreeze (sphBody, 0);

	// keep the player always active 
	NewtonWorldUnfreezeBody (nWorld, sphBody);

	// Set Material Id for this object
	NewtonBodySetMaterialGroupID (sphBody, characterID);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (sphBody, sph);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (sphBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (sphBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (sphBody, BallCharacterApplyForceAndTorque);

	// set the mass matrix
	NewtonBodySetMassMatrix (sphBody, 10.0f, 10.0f, 10.0f, 10.0f);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (sphBody, &location[0][0]);
	PhysicsSetTransform (sphBody, &location[0][0]);

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);
}



