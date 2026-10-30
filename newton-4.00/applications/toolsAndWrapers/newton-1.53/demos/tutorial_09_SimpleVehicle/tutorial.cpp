//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************
#include <stdafx.h>
#include "tutorial.h"
#include "Materials.h"
#include "MousePick.h"
#include "NewtonProp.h"
#include "HiResTimer.h"
#include "NewtonTractor.h"
#include "NewtonRaceCar.h"
#include "RenderPrimitive.h"
#include "LevelPrimitive.h"

CHiResTimer timer;

static NewtonWorld* nWorld;
static dSceneNode* nGrapphicWorld;


#define CAMERA_SPEED	10.0f

static NewtonVehicle* player;
static NewtonVehicle* raceCarVehicle;
static NewtonVehicle* dirTractorVehicle;

static dFloat cameraZoon = 6.0f;

static dVector cameraDir (1.0f, 0.0f, 0.0f);
static dVector cameraEyepoint (0, 50.0f, -5.0f);
//static dVector cameraEyepoint (10.0f, 50.0f, -25.0f);
static void CalculateTrackingCamera (dFloat rollAngle, dFloat yawAngle);


bool debugLinesMode = false;
//bool debugLinesMode = true;
bool hideVehicle = false;

dInt32 hightMap_debugCount;
dVector hightMap_debugLines[1024][2];


static void  CleanUp ();
static void  Keyboard();
static void* PhysicsAlloc (dInt32 sizeInBytes);
static void  BodyLeaveWorkd (const NewtonBody* body);
static void  PhysicsFree (void *ptr, dInt32 sizeInBytes);

static void  DebugShowCollision ();
static void  DebugShowBodyCollision (const NewtonBody* body);
static void DebugShowGeometryCollision (const NewtonBody* body, dInt32 vertexCount, const dFloat* faceVertec, dInt32 id);







//********************************************************************
// 
//	Newton Tutorial 2 Using Callbacks
//
//********************************************************************
dInt32 main(dInt32 argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutorial 9: implementing a simple vehicle", DrawScene);

	// create the physics scene
	InitScene();
	
	// Execute opengl main loop for ever
	glutMainLoop();   

	return 0;
} 

// memory allocation for Newton
void* PhysicsAlloc (dInt32 sizeInBytes)
{
	return malloc (sizeInBytes);
}

// memory de-allocation for Newton
void PhysicsFree (void *ptr, dInt32 sizeInBytes)
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

	// destroy the graphic world;
	delete nGrapphicWorld;
}


static dFloat RayCastPlacement (const NewtonBody* body, const dFloat* normal, dInt32 collisionID, void* userData, dFloat intersetParam)
{
	dFloat* paramPtr;

	paramPtr = (dFloat*)userData;
	paramPtr[0] = intersetParam;
	return intersetParam;
}


// find the vertical intersection with the ground
static dFloat FindFloor (dFloat x, dFloat z)
{
//return 0.0f;

	dFloat parameter;
	dVector p0 (x, 1000.0f, z); 
	dVector p1 (x, -1000.0f, z); 

	parameter = 1.2f;
	NewtonWorldRayCast (nWorld, &p0[0], &p1[0], RayCastPlacement, &parameter, NULL);
	_ASSERTE (parameter < 1.0f);

	return 1000.0f - 2000.0f * parameter;
}


// callback to handle when the car leave the world
void BodyLeaveWorkd (const NewtonBody* body)
{
	NewtonVehicle* car;

	// get the user data
	car = (NewtonVehicle*) NewtonBodyGetUserData(body);

// uncomment this for destroy and recreate the vehicle
//#define DESTROY_AND_RECREATE

	#ifdef DESTROY_AND_RECREATE
		// destroy this vehicle for the scene
		NewtonDestroyBody(nWorld, body);

		// if this is the car spawn other car
		if (car == raceCarVehicle) {
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z);

			raceCarVehicle = new NewtonRaceCar (nGrapphicWorld, "f1.dg", nWorld, matrix);
			//this is the player, Set auto freeze off
			NewtonBodySetAutoFreeze (raceCarVehicle->GetRigidBody(), 0);
		}  

		
		if (car == player) {
			player = raceCarVehicle;
		}

	#else 
		// do not destroy vehicle just reposition it

		dMatrix matrix (GetIdentityMatrix());
		matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z) + 4.0f;

		// important to rest the vehicle tires before moving the car
		NewtonBodySetMatrixRecursive(body, &matrix[0][0]);
	#endif

}



void DebugShowGeometryCollision (const NewtonBody* body, dInt32 vertexCount, const dFloat* faceVertec, dInt32 id)
{
	dInt32 i;

	i = vertexCount - 1;
	dVector p0 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
	for (i = 0; i < vertexCount; i ++) {
		dVector p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
//		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
//		glVertex3f (p1.m_x, p1.m_y, p1.m_z);
		p0 = p1;
	}

}


void  DebugShowBodyCollision (const NewtonBody* body)
{
	// render the origin center of mass of the body

	dVector origin;
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodyGetCentreOfMass (body, &origin[0]);
	origin = matrix.TransformVector (origin);

	float size = 0.1f;
	glVertex3f(origin.m_x - size, origin.m_y + size, origin.m_z + size);
	glVertex3f(origin.m_x + size, origin.m_y + size, origin.m_z + size);

	glVertex3f(origin.m_x - size, origin.m_y - size, origin.m_z + size);
	glVertex3f(origin.m_x + size, origin.m_y - size, origin.m_z + size);

	glVertex3f(origin.m_x - size, origin.m_y - size, origin.m_z - size);
	glVertex3f(origin.m_x + size, origin.m_y - size, origin.m_z - size);
	
	glVertex3f(origin.m_x - size, origin.m_y + size, origin.m_z - size);
	glVertex3f(origin.m_x + size, origin.m_y + size, origin.m_z - size);

	glVertex3f(origin.m_x + size, origin.m_y - size, origin.m_z + size);
	glVertex3f(origin.m_x + size, origin.m_y + size, origin.m_z + size);
						 				  
	glVertex3f(origin.m_x - size, origin.m_y - size, origin.m_z + size);
	glVertex3f(origin.m_x - size, origin.m_y + size, origin.m_z + size);
						 				  
	glVertex3f(origin.m_x - size, origin.m_y - size, origin.m_z - size);
	glVertex3f(origin.m_x - size, origin.m_y + size, origin.m_z - size);
						 				  
	glVertex3f(origin.m_x + size, origin.m_y - size, origin.m_z - size);
	glVertex3f(origin.m_x + size, origin.m_y + size, origin.m_z - size);

	glVertex3f(origin.m_x + size, origin.m_y + size, origin.m_z - size);
	glVertex3f(origin.m_x + size, origin.m_y + size, origin.m_z + size);
						 									    
	glVertex3f(origin.m_x + size, origin.m_y - size, origin.m_z - size);
	glVertex3f(origin.m_x + size, origin.m_y - size, origin.m_z + size);
						 									    
	glVertex3f(origin.m_x - size, origin.m_y - size, origin.m_z - size);
	glVertex3f(origin.m_x - size, origin.m_y - size, origin.m_z + size);
						 									    
	glVertex3f(origin.m_x - size, origin.m_y + size, origin.m_z - size);
	glVertex3f(origin.m_x - size, origin.m_y + size, origin.m_z + size);



	// render the collision is wire frame
	NewtonBodyForEachPolygonDo (body, DebugShowGeometryCollision);
}


void  DebugShowCollision ()
{
	dInt32 i;
	glColor3f(1.0f, 1.0f, 1.0f);         

	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glPushMatrix();

	glBegin(GL_LINES);
	NewtonWorldForEachBodyDo (nWorld, DebugShowBodyCollision);
	for (i = 0; i < hightMap_debugCount; i ++) {
		glVertex3f (hightMap_debugLines[i][0].m_x, hightMap_debugLines[i][0].m_y, hightMap_debugLines[i][0].m_z);
		glVertex3f (hightMap_debugLines[i][1].m_x, hightMap_debugLines[i][1].m_y, hightMap_debugLines[i][1].m_z);
		
	}
	glEnd();

	glPopMatrix();

	hightMap_debugCount = 0;
}


//static dInt32 debug_count;
//static dVector m_debug[1024 * 4][2];

// DrawScene()
void DrawScene ()
{
	dFloat timeStep;


	// get the time step
	timeStep = timer.GetElapsedSeconds();


	// update the Newton physics world
	NewtonUpdate (nWorld, timeStep);

	// move the camera
	dVector target (cameraEyepoint + cameraDir);
	SetCamera (cameraEyepoint, target);


	// render the scene
	glEnable (GL_LIGHTING);
	glPushMatrix();	
	nGrapphicWorld->Render ();
	glPopMatrix();

	// read the keyboard
	Keyboard ();

	// if debug display is on, scene all rigid bodies and display the collision geometry in wire frame
	if (debugLinesMode) {
		DebugShowCollision ();
	}

	dVector color (1.0, 1.0f, 1.0f);
	Print (color, 4, 4, "fps %f", 1.0f / (timeStep + 1.0e-6f));
	Print (color, 4, 24, "f1  : show collition mesh");
	Print (color, 4, 44, "f2  : hide model");
	Print (color, 4, 64, "f3  : toggle vehicle");
	Print (color, 4, 84, "+ - : zoom camera");

	if (player) {
		Print (color, 4, 104, "VehecleSpeed  (mile/hour) %5.2f", player->GetSpeed() * 2.24f);
	}
} 


static void CalculateTrackingCamera (dFloat rollAngle, dFloat yawAngle)
{
	// build a rotation matrix by doing a grand smith projection
	dVector front (1.0f, 0.0f, 0.0f);
	if (player) {
		front = player->GetMatrix().m_front;
	}
	front.m_y = 0.0f;
	dMatrix matrix (GetIdentityMatrix()); 

	matrix.m_front = front.Scale (1.0f / dSqrt (front % front));
	matrix.m_right = matrix.m_front * matrix.m_up;

	dMatrix cameraDirMatrix (dgRollMatrix(rollAngle) * dgYawMatrix(yawAngle) * matrix);
	cameraDir = cameraDirMatrix.m_front;

	cameraEyepoint = dVector (-10.0f, 2.0f, -10.0f);
	if (player) {
		cameraEyepoint = player->GetMatrix().m_posit;
	}
	cameraEyepoint.m_y += 1.0f;

	cameraEyepoint -= cameraDir.Scale (cameraZoon);

}

//	Keyboard handler. 
void Keyboard()
{
	// check for termination
	if (dGetKeyState (VK_ESCAPE) & 0x8000) {
		exit(0);
	}

	// show debug information when press F1
	static unsigned prevF1Key = ~(dGetKeyState (VK_F1) & 0x8000);
	unsigned f1Key = dGetKeyState (VK_F1) & 0x8000;
	if (prevF1Key & f1Key) {
		// toggle debug line mode 
		debugLinesMode = ! debugLinesMode;
	}
	prevF1Key = ~f1Key;




	static unsigned prevF2Key = ~(dGetKeyState (VK_F2) & 0x8000);
	unsigned f2Key = dGetKeyState (VK_F2) & 0x8000;
	if (prevF2Key & f2Key) {
		// toggle debug line mode 
		hideVehicle = ! hideVehicle;
	}
	prevF2Key = ~f2Key;


	static unsigned prevF3Key = ~(dGetKeyState (VK_F3) & 0x8000);
	unsigned f3Key = dGetKeyState (VK_F3) & 0x8000;
	if ((prevF3Key & f3Key) && player) {
		// toggle debug line mode 
		player = (player == raceCarVehicle) ? dirTractorVehicle : raceCarVehicle;
	}
	prevF3Key = ~f3Key;


	// read the mouse position and set the camera direction
	static dFloat yawAngle = 0.0f * 3.1416 / 180.0f;
	static dFloat rollAngle = 0.0f;

	MOUSE_POINT mouse1;
	dInt32 mouseLeftKey;
	static MOUSE_POINT mouse0;
	
	GetCursorPos(mouse1);
	mouseLeftKey = dGetKeyState (VK_LBUTTON) & 0x8000;

	// this section control the camera object picking
	mouseLeftKey = dGetKeyState (VK_LBUTTON) & 0x8000;
	if (!MousePick (nWorld, mouse1, mouseLeftKey, 0.125f, 1.0f)) {
		// we are not in mouse pick mode, then we are in camera tracking mode
		if (mouseLeftKey) {
			// when click left mouse button the first time, we reset the camera
			// convert the mouse x position to delta yaw angle
			if (mouse1.x > (mouse0.x + 1)) {
				yawAngle += 2.0f * 3.1416 / 180.0f;
				if (yawAngle > (360.0f * 3.1416 / 180.0f)) {
					yawAngle -= (360.0f * 3.1416 / 180.0f);
				}
			} else if (mouse1.x < (mouse0.x - 1)) {
				yawAngle -= 2.0f * 3.1416 / 180.0f;
				if (yawAngle < 0.0f) {
					yawAngle += (360.0f * 3.1416 / 180.0f);
				}
			}

			// convert the mouse y position to delta roll angle
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
		}
		CalculateTrackingCamera(rollAngle, yawAngle);
	}

	// save mouse position and left mouse key state for next frame
	mouse0 = mouse1;

	// ******************************************************************************************************
	//
	// vehicle controls
	//
	// steering	
	if (player) {
		if (dGetKeyState ('D') & 0x8000) {
			player->SetSteering (-1.0f);
		} else if (dGetKeyState ('A') & 0x8000) {
			player->SetSteering (1.0f);
		} else {
			player->SetSteering (0.0f);
		}
		
		// forward torque
		if (dGetKeyState ('W') & 0x8000) {
			player->SetTireTorque(1.0f);
		} else if (dGetKeyState ('S') & 0x8000) {
			player->SetTireTorque(-1.0f);
		} else {
			player->SetTireTorque(0.0f);
		}

		 // hand brakes torque
		if (dGetKeyState (' ') & 0x8000) {
			player->SetApplyHandBrakes(1.0f);
		} else {
			player->SetApplyHandBrakes(0.0f);
		}

//player->SetApplyHandBrakes(1.0f);



		
		if (dGetKeyState (VK_SUBTRACT) & 0x8000) {
			// toggle debug line mode 
			cameraZoon *= 1.025f;
			if (cameraZoon > 12.0f) {
				cameraZoon = 12.0f;
			}
		} else if (dGetKeyState (VK_ADD) & 0x8000) {
			cameraZoon *= 0.975f;
			if (cameraZoon < 3.0f) {
				cameraZoon = 3.0f;
			}
		}
	}


	// apply controls
	if (player) {
		char controlKeys[] = "1234RFTGYHUJ";
		for (dInt32 i = 0; i < 10; i ++) {
			if (dGetKeyState (controlKeys[i * 2]) & 0x8000) {
				player->SetControl(i, 1.0f);
			} else if (dGetKeyState (controlKeys[i * 2 + 1]) & 0x8000) {
				player->SetControl(i, -1.0f);
			}
		}
	}
} 





enum PrimitiveType
{
	_BOX_PRIMITIVE,
	_CONE_PRIMITIVE,
	_SPHERE_PRIMITIVE,
	_CYLINDER_PRIMITIVE,
	_CAPSULE_PRIMITIVE,
	_CHAMFER_CYLINDER_PRIMITIVE,
	_RANDOM_COMVEX_HULL_PRIMITIVE,
};

dFloat RandomVariable(dFloat amp)
{
	return amp * (dFloat (dRand() + dRand()) / dRAND_MAX - 1.0f) * 0.5f;
}


// set the transformation of a rigid body
void PhysicsApplyGravityForce (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);
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


// set the transform and all child object transform
void PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

#ifdef USE_HULL_MODIFEIER
	NewtonCollision* collision;
	// set the transformation matrix for this rigid body
	dMatrix mat (*((dMatrix*)matrix));

	// Warp the graphical part to match the collision geometry

	dMatrix collisionScaleMatrix;
	collision = NewtonBodyGetCollision (body);

	// animate the Scale angle
	primitive->m_modifierScaleAngleX = dMod (primitive->m_modifierScaleAngleX + 0.002f, 6.2832f);
	primitive->m_modifierScaleAngleY = dMod (primitive->m_modifierScaleAngleY + 0.002f, 6.2832f);
	primitive->m_modifierScaleAngleZ = dMod (primitive->m_modifierScaleAngleZ + 0.002f, 6.2832f);

	primitive->m_modifierSkewAngleY = dMod (primitive->m_modifierSkewAngleY + 0.002f, 6.2832f);

	// get the Modifier scale values matrix 
	NewtonConvexHullModifierGetMatrix (collision, &collisionScaleMatrix[0][0]);
	collisionScaleMatrix[0][0] = 0.5f * (1.0f + 0.5f * dSin (primitive->m_modifierScaleAngleX));
	collisionScaleMatrix[1][1] = 0.5f * (1.0f + 0.5f * dSin (primitive->m_modifierScaleAngleY));
	collisionScaleMatrix[2][2] = 0.5f * (1.0f + 0.5f * dSin (primitive->m_modifierScaleAngleZ));

	// also skew the shape
	collisionScaleMatrix[0][1] = 0.5f * (1 + dSin (primitive->m_modifierSkewAngleY));
//	collisionScaleMatrix[0][2] = 0.5f (1 + dSin (primitive->m_modifierSkewAngleY));
//	collisionScaleMatrix[0][1] = 1.0f;

	// copy the matrix back into the collision
	NewtonConvexHullModifierSetMatrix (collision, &collisionScaleMatrix[0][0]);

	// apply the Modifier to the geometry too
	mat = collisionScaleMatrix * mat;
#else
	dMatrix& mat = *((dMatrix*)matrix);
#endif

	primitive->SetMatrix (mat);

}


void AddPrimitiveObject (dFloat mass, const dMatrix& srcMatrix, const dVector& size, PrimitiveType type)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* blockBoxBody = NULL;
	RenderPrimitive* blockBox = NULL;
	NewtonCollision* collision = NULL;

	// set realistic mass and inertia matrix for each block

	dMatrix matrix (srcMatrix);
	switch (type) 
	{

		case _SPHERE_PRIMITIVE:
		{
			//dVector size1 (size.m_x, size.m_y + RandomVariable(size.m_y * 2.0f), size.m_z + RandomVariable(size.m_z * 2.0f));  
			dVector size1 (size);  

			// create the collision 
			collision = NewtonCreateSphere (nWorld, size1.m_x, size1.m_y, size1.m_z, NULL); 

			// create a graphic box
			blockBox = new SpherePrimitive (nGrapphicWorld, matrix, size1.m_x, size1.m_y, size1.m_z);
			//blockBox = new EllipsePrimitive (matrix, nWorld, collision, g_ballTexture);
			
			break;
		}

		case _BOX_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

			// create a graphic box
			blockBox = new BoxPrimitive (nGrapphicWorld, matrix, size);
			break;
		}

		case _CONE_PRIMITIVE:
		{
			dFloat h = size.m_x;
			dFloat r = size.m_y;

			matrix = dgRollMatrix (3.0) * matrix;
			
			// create the collision 
			collision = NewtonCreateCone (nWorld, r, h, NULL); 

			// create a graphic box
			blockBox = new ConePrimitive (nGrapphicWorld, matrix, r, h);
			break;
		}

		case _CYLINDER_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateCylinder (nWorld, size.m_y, size.m_x, NULL); 


			// create a graphic box
			blockBox = new CylinderPrimitive (nGrapphicWorld, matrix, size.m_y, size.m_x);
			break;
		}

		case _CAPSULE_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateCapsule (nWorld, size.m_y, size.m_x, NULL); 

			// create a graphic box
			blockBox = new CapsulePrimitive (nGrapphicWorld, matrix, size.m_y, size.m_x);
			break;
		}

		case _CHAMFER_CYLINDER_PRIMITIVE:
		{
			// create the collision 
			matrix = dgRollMatrix (3.1416f * 0.5f) * matrix;
			collision = NewtonCreateChamferCylinder (nWorld, size.m_x * 0.5f, size.m_y, NULL); 

			// create a graphic box
			blockBox = new ChamferCylinderPrimitive (nGrapphicWorld, matrix, size.m_x * 0.5f, size.m_y);
			break;
		}


		case _RANDOM_COMVEX_HULL_PRIMITIVE:
		{
			#define SAMPLE_COUNT 5000

			// Create a clouds of random point around the origin
			dVector cloud [SAMPLE_COUNT];
			int i;
			// make sure that at least the top and bottom are present
			cloud [0] = dVector ( size.m_x * 0.5f, -0.3f,  0.0f);
			cloud [1] = dVector ( size.m_x * 0.5f,  0.3f,  0.3f);
			cloud [2] = dVector ( size.m_x * 0.5f,  0.3f, -0.3f);
			cloud [3] = dVector (-size.m_x * 0.5f, -0.3f,  0.0f);
			cloud [4] = dVector (-size.m_x * 0.5f,  0.3f,  0.3f);
			cloud [5] = dVector (-size.m_x * 0.5f,  0.3f, -0.3f);

			// populate the cloud with pseudo Gaussian random points
			for (i = 6; i < SAMPLE_COUNT; i ++) {
				cloud [i].m_x = RandomVariable(size.m_x);
				cloud [i].m_y = RandomVariable(size.m_y * 2.0f);
				cloud [i].m_z = RandomVariable(size.m_z * 2.0f);
			}
			
			collision = NewtonCreateConvexHull (nWorld, SAMPLE_COUNT, &cloud[0].m_x, sizeof (dVector), NULL); 
			
			// create a graphic box
			blockBox = new ConvexHullPrimitive (nGrapphicWorld, matrix, nWorld, collision);
			break;
		}
	}

	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];


	// add a Collision Transform modifier to show of a collision Geometry feature 
#ifdef USE_HULL_MODIFEIER	
	NewtonCollision* collisionModifier;
	collisionModifier = NewtonCreateConvexHullModifier (nWorld, collision);
	NewtonReleaseCollision (nWorld, collision);		
	collision = collisionModifier;
	dMatrix scaleMatrix (GetIdentityMatrix());
	// set the modifier matrix to identity
	NewtonConvexHullModifierSetMatrix (collision, &scaleMatrix[0][0]);
#endif

	//create the rigid body
	blockBoxBody = NewtonCreateBody (nWorld, collision);
  
	// activate Corilies
//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (blockBoxBody, blockBox);

	// Set auto freeze off
//	NewtonBodySetAutoFreeze (blockBoxBody, sleepMode);

	// set a call back to track the auto active state of this body
//	NewtonBodySetAutoactiveCallback (blockBoxBody, TrackActivesBodies);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (blockBoxBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (blockBoxBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (blockBoxBody, PhysicsApplyGravityForce);

	// set the body to be wood
	NewtonBodySetMaterialGroupID (blockBoxBody, woodID);

	// set the correct center of gravity for this body
	NewtonBodySetCentreOfMass (blockBoxBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassMatrix (blockBoxBody, mass, Ixx, Iyy, Izz);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (blockBoxBody, &matrix[0][0]);
	PhysicsSetTransform (blockBoxBody, &matrix[0][0]);

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

	// Set the damping coefficient to the minimum allowed by the engine
	dFloat damp[3];
	damp[0] = 0.0f;
	damp[1] = 0.0f;
	damp[2] = 0.0f;
//	NewtonBodySetLinearDamping (blockBoxBody, 0.0f);
//	NewtonBodySetAngularDamping (blockBoxBody, damp);

	damp[0] = NewtonBodyGetLinearDamping (blockBoxBody);
	NewtonBodyGetAngularDamping (blockBoxBody, damp);

	//dVector veloc (0, -100, 0);
	//NewtonBodySetVelocity (blockBoxBody, &veloc[0]);
	NewtonBodySetContinuousCollisionMode (blockBoxBody, 1);
}




void AddCylinder(const dMatrix& location)
{
	dInt32 i;
	dInt32 j;
	dFloat x;
	dFloat y;
	dFloat z;
	dFloat mass;
  
	dVector size (1.0f, 0.5f, 0.5f);

	mass = 50.0f;
	dMatrix matrix (dgRollMatrix (0.5f * 3.1414) * location);
	dVector origin (location.m_posit);
	origin.m_x += 25.0f;
	origin.m_z -= 2.0f;

	x = origin.m_x;
	for (i = 0; i < 4; i ++) {
		x += 10;
		z = origin.m_z;
		y = FindFloor (x, z) + size.m_x * 0.5f;

		// find a position above the floor
		matrix.m_posit.m_x = x;
		matrix.m_posit.m_y = y;
		matrix.m_posit.m_z = z;
		for (j = 0; j < 4; j ++) {
			AddPrimitiveObject (mass, matrix, size, _CYLINDER_PRIMITIVE);
			matrix.m_posit.m_y += size.m_x;
		}
	}
}


// create physics scene
void InitScene()
{
	int i;
	i = 0;

	LevelPrimitive *level;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the newton world
	nWorld = NewtonCreate (PhysicsAlloc, PhysicsFree);

	// set the frame rate to 60
//	NewtonSetMinimumFrameRate (nWorld, 60.0f);
//	NewtonSetMinimumFrameRate (nWorld, 90.0f);
//	NewtonSetMinimumFrameRate (nWorld, 180.0f);
	
	// add a notification call back for when the car leave the world
	NewtonSetBodyLeaveWorldEvent (nWorld, BodyLeaveWorkd); 

	// Set the termination function
	atexit(CleanUp); 


	// need materials for this projects
	SetupMaterials (nWorld);


	// add a sky box to the scene
	nGrapphicWorld = new SkyBoxPrimitive (NULL);

	// /////////////////////////////////////////////////////////////////////
	//
	// load a height field mesh
//	level = new LevelPrimitive(nGrapphicWorld, "newtontrack.dg", nWorld, NULL);
	level = new LevelPrimitive(nGrapphicWorld, "newtontrack_flat.dg", nWorld, NULL);



	// set the material group id to the terrain
	NewtonBodySetMaterialGroupID (level->GetRigidBody(), levelID);

	// set the camera origin 10 meter above the ground
	cameraEyepoint.m_y = FindFloor (cameraEyepoint.m_x, cameraEyepoint.m_z) + 3.0f;

	// /////////////////////////////////////////////////////////////////////
	//
	// Add a race car to the scene
	dMatrix matrix (GetIdentityMatrix());
	matrix.m_posit = cameraEyepoint + cameraDir.Scale (5.0f);

	matrix.m_posit.m_z = -8.0f;  
	matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z) + 1.0f;
//	new NewtonProp (nGrapphicWorld, "prop_01.dg", nWorld, matrix);


	matrix.m_posit.m_y += 4.0f;
//	new NewtonProp (nGrapphicWorld, "subaru.dg", nWorld, matrix);
	
	matrix.m_posit.m_x -= 14.0f;	
	matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z);
	matrix.m_posit.m_y += 2.5f;

	raceCarVehicle = new NewtonRaceCar (nGrapphicWorld, "f1.dg", nWorld, matrix);
	//this is the player, Set auto freeze off
	NewtonBodySetAutoFreeze (raceCarVehicle->GetRigidBody(), 0);

	// set the player to point to this vehicle
	player = raceCarVehicle;


	// add another race car
	matrix.m_posit.m_x += 30.0f;
	matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z);
//	new NewtonRaceCar (nGrapphicWorld, "f1.dg", nWorld, matrix);
	matrix.m_posit.m_x -= 30.0f;
	//this is the player, Set auto freeze off
//	NewtonBodySetAutoFreeze (raceCarVehicle->GetRigidBody(), 0);
  
	// Add a tractor 
	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_z -= 10.0f;
	matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z);
//	dirTractorVehicle = new NewtonTractor (nGrapphicWorld, "tractor.dg", nWorld, matrix);
	//this is the player, Set auto freeze off
//	NewtonBodySetAutoFreeze (dirTractorVehicle->GetRigidBody(), 0);

	matrix.m_posit.m_x -= 10.0f;
	matrix.m_posit.m_z += 10.0f;
	// set the player to point to this vehicle
//	player = dirTractorVehicle;


//	AddCylinder(matrix);

	matrix.m_posit.m_x += 15.0f;
	for (i = 0; i < 20; i ++) {	
		// add some props 
		matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z);
//		new NewtonProp (nGrapphicWorld, "prop_03.dg", nWorld, matrix);
		matrix.m_posit.m_x += 2.0f;
	}


	matrix.m_posit.m_x -= 100.0f;
	matrix.m_posit.m_z -= 5.0f;
	for (i = 0; i < 20; i ++) {	
		// add some props 
		matrix.m_posit.m_y = FindFloor (matrix.m_posit.m_x, matrix.m_posit.m_z);
//		new NewtonProp (nGrapphicWorld, "prop_02.dg", nWorld, matrix);
		matrix.m_posit.m_x += 8.0f;
	}

}






