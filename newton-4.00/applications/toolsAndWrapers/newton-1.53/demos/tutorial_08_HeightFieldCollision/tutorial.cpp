//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

#include <stdafx.h>
#include "tutorial.h"
#include "MousePick.h"
#include "HiResTimer.h"
#include "Materials.h"
#include "LevelPrimitive.h"
#include "RenderPrimitive.h"
#include "HeightFieldCollision.h"

CHiResTimer timer;

//#define USE_HULL_MODIFIER
#define USE_BOUYANCY_FORCE
#define USE_CONTINUE_COLLISION


static NewtonWorld* nWorld;
static dSceneNode* nGrapphicWorld;

#define OBJEST_COUNT	2
//#define OBJEST_COUNT	1


#define GRAVITY		   -50.0f
#define CAMERA_SPEED	24.0f

#define SPACING			60.0f

static dVector cameraDir (1.0f, 0.0f, 0.0f);
static dVector cameraEyepoint (400, 50.0f, 400);
//static dVector cameraEyepoint (400, 1.0f, 400);

bool hideObjects = false;
//bool hideObjects = true;

bool debugLinesMode = false;
//bool debugLinesMode = true;


//bool sleepMode = false;
bool sleepMode = true;

int hightMap_debugCount;
dVector hightMap_debugLines[1024 * 4][2];


static void  CleanUp ();
static void  Keyboard();
static void* PhysicsAlloc (int sizeInBytes);
static void  PhysicsFree (void *ptr, int sizeInBytes);
static void  PhysicsBodyDestructor (const NewtonBody* body);
static void  PhysicsApplyGravityForce (const NewtonBody* body);
static void  PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix);

static void AddBodies();
static dFloat RayCastPlacement (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam);
static dFloat FindFloor (dFloat x, dFloat z);

static void DebugShowCollision ();
static void DebugShowBodyCollision (const NewtonBody* body);
static void DebugShowGeometryCollision (const NewtonBody* body, int vertexCount, const dFloat* faceVertec, int id);


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


//********************************************************************
// 
//	Newton Tutorial 2 Using Callbacks
//
//********************************************************************
int main(int argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutorial 8: implementing height map mesh", DrawScene);
	

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


#ifdef USE_BOUYANCY_FORCE
static int PhysicsBouyancyPlane (const int collisionID, void *context, const dFloat* globalSpaceMatrix, dFloat* globalSpacePlane)
{
	globalSpacePlane[0] = 0.0f;
	globalSpacePlane[1] = 1.0f;
	globalSpacePlane[2] = 0.0f;
	globalSpacePlane[3] = -60.0f;

	return 1;	
}
#endif


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


/*
static int xxx = 0;
xxx ++;
dMatrix matrix;
NewtonBodyGetMatrix(body, &matrix[0][0]);
if (xxx  == 100){
dVector impulse (0, 0, 4);
NewtonAddBodyImpulse(body, &impulse[0], &matrix.m_posit[0]);
}

if (xxx == 400){
//if (xxx == 130){
dVector impulse (-0.15,0,-10.50);
NewtonAddBodyImpulse(body, &impulse[0], &matrix.m_posit[0]);
}

if (xxx > 130)
{
	xxx *= 1;
}
*/

	#ifdef USE_BOUYANCY_FORCE
		dVector gravity (0.0f, GRAVITY, 0.0f);
//		NewtonBodyAddBuoyancyForce (body, 1.0f, 0.8f, 0.8f, &gravity[0], PhysicsBouyancyPlane, (void*) body);
		NewtonBodyAddBuoyancyForce (body, 1.0f, 0.8f, 0.8f, &gravity[0], PhysicsBouyancyPlane, (void*) body);
	#endif
}




// set the transform and all child object transform
void PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

#ifdef USE_HULL_MODIFIER
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
//	collisionScaleMatrix[2][2] = 1.0f + 2.0f * (1.0f - dCos (primitive->m_modifierScaleAngleX)); 


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



static void DebugShowGeometryCollision (const NewtonBody* body, int vertexCount, const dFloat* faceVertec, int id)
{
	int i;

	i = vertexCount - 1;
	dVector p0 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
	for (i = 0; i < vertexCount; i ++) {
		dVector p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p1.m_x, p1.m_y, p1.m_z);
 		p0 = p1;
	}
}


static void  DebugShowBodyCollision (const NewtonBody* body)
{
	NewtonBodyForEachPolygonDo (body, DebugShowGeometryCollision);
}


static void  DebugShowCollision ()
{
	int i;


	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 0.0f);
	NewtonWorldForEachBodyDo (nWorld, DebugShowBodyCollision);

	glColor3f(1.0f, 1.0f, 1.0f);
	for (i = 0; i < hightMap_debugCount; i ++) {
		glVertex3f (hightMap_debugLines[i][0].m_x, hightMap_debugLines[i][0].m_y, hightMap_debugLines[i][0].m_z);
		glVertex3f (hightMap_debugLines[i][1].m_x, hightMap_debugLines[i][1].m_y, hightMap_debugLines[i][1].m_z);
		
	}
	glEnd();

	hightMap_debugCount = 0;
}


// DrawScene()
void DrawScene ()
{
	dFloat timeStep;
	unsigned physicsTime;

	// get the time step
	timeStep = timer.GetElapsedSeconds();

	// update the Newton physics world
	physicsTime = timer.GetTimeInMiliseconds();
	NewtonUpdate (nWorld, timeStep);
	physicsTime = timer.GetTimeInMiliseconds() - physicsTime;


	// move the camera
	dVector target (cameraEyepoint + cameraDir);
	SetCamera (cameraEyepoint, target);

	// render the scene
	if (!hideObjects) {
		glEnable (GL_LIGHTING);
		glPushMatrix();	
		nGrapphicWorld->Render ();
		glPopMatrix();
	}

	// read the keyboard
	Keyboard ();

	// if debug display is on, scan all rigid bodies and display the collision geometry in wire frame
	if (debugLinesMode) {
		DebugShowCollision ();
	}


	dVector color (1.0, 1.0f, 1.0f);
	Print (color, 4, 4, "fps %f  physics(ms) %d", 1.0f / (timeStep + 1.0e-6f), physicsTime);
	Print (color, 4, 34, "f1 - show collition mesh");
	Print (color, 4, 64, "f2 - hide model");
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
		// clear debugLines counter
		debugLinesMode = ! debugLinesMode;
	}
	prevF1Key = ~f1Key;


	static unsigned prevF2Key = ~(dGetKeyState (VK_F2) & 0x8000);
	unsigned f2Key = dGetKeyState (VK_F2) & 0x8000;
	if (prevF2Key & f2Key) {
		// toggle debug line mode 
		hideObjects = ! hideObjects;
	}
	prevF2Key = ~f2Key;


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
	if (!MousePick (nWorld, mouse1, mouseLeftKey, 0.25f, 5.0f)) {
		// we are not in mouse pick mode, then we are in camera tracking mode
		if (mouseLeftKey) {
			// when click left mouse button the first time, we reset the camera
			// convert the mouse x position to delta yaw angle
			if (mouse1.x > (mouse0.x + 1)) {
				yawAngle += 1.0f * 3.1416 / 180.0f;
				if (yawAngle > (360.0f * 3.1416 / 180.0f)) {
					yawAngle -= (360.0f * 3.1416 / 180.0f);
				}
			} else if (mouse1.x < (mouse0.x - 1)) {
				yawAngle -= 1.0f * 3.1416 / 180.0f;
				if (yawAngle < 0.0f) {
					yawAngle += (360.0f * 3.1416 / 180.0f);
				}
			}

			if (mouse1.y > (mouse0.y + 1)) {
				rollAngle += 1.0f * 3.1416 / 180.0f;
				if (rollAngle > (80.0f * 3.1416 / 180.0f)) {
					rollAngle = 80.0f * 3.1416 / 180.0f;
				}
			} else if (mouse1.y < (mouse0.y - 1)) {
				rollAngle -= 1.0f * 3.1416 / 180.0f;
				if (rollAngle < -(80.0f * 3.1416 / 180.0f)) {
					rollAngle = -80.0f * 3.1416 / 180.0f;
				}
			}
			dMatrix cameraDirMat (dgRollMatrix(rollAngle) * dgYawMatrix(yawAngle));
			cameraDir = cameraDirMat.m_front;
		}
	}

	// save mouse position and left mouse key state for next frame
	mouse0 = mouse1;


	// camera control
	if (dGetKeyState ('W') & 0x8000) {
		cameraEyepoint += cameraDir.Scale (CAMERA_SPEED / 60.0f);
	} else if (dGetKeyState ('S') & 0x8000) {
		cameraEyepoint -= cameraDir.Scale (CAMERA_SPEED / 60.0f);
	}

	if (dGetKeyState ('D') & 0x8000) {
		dVector up (0.0f, 1.0f, 0.0f);
		dVector right (cameraDir * up);
		cameraEyepoint += right.Scale (CAMERA_SPEED / 60.0f);
	} else if (dGetKeyState ('A') & 0x8000) {
		dVector up (0.0f, 1.0f, 0.0f);
		dVector right (cameraDir * up);
		cameraEyepoint -= right.Scale (CAMERA_SPEED / 60.0f);
	}
} 


// create physics scene
void InitScene()
{
	int defaulftID;
	HeightFieldCollision *level;

	// /////////////////////////////////////////////////////////////////////
	//
	// create the newton world
	nWorld = NewtonCreate (PhysicsAlloc, PhysicsFree);

	// Set the termination function
	atexit(CleanUp); 

	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	nGrapphicWorld = new SkyBoxPrimitive (NULL);

	
	defaulftID = NewtonMaterialGetDefaultGroupID(nWorld);
	NewtonMaterialSetDefaultSoftness (nWorld, defaulftID, defaulftID, 0.05f);
	NewtonMaterialSetDefaultElasticity (nWorld, defaulftID, defaulftID, 0.1f);
	NewtonMaterialSetDefaultFriction (nWorld, defaulftID, defaulftID, 0.9f, 0.5f);


	// need materials for this projects
	SetupMaterials (nWorld);

	// /////////////////////////////////////////////////////////////////////
	//
	// load a height field mesh
	level = new HeightFieldCollision (nGrapphicWorld, nWorld, PhysicsBodyDestructor);

	// set the material group id to the terrain
	NewtonBodySetMaterialGroupID (level->GetRigidBody(), levelID);


	// /////////////////////////////////////////////////////////////////////
	//
	// add some rigid bodies over the height map
	AddBodies ();
}


dFloat  RayCastPlacement (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam)
{
	dFloat* paramPtr;

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f) {
		// if the ray hit a static body, considered it the floors
		paramPtr = (dFloat*)userData;
		paramPtr[0] = intersetParam;
		return intersetParam;
	}
	// else continue the search
	return 1.0;
}

dFloat FindFloor (dFloat x, dFloat z)
{
	dFloat parameter;
	
	// shot a vertical ray from a high altitude and collected the intersection parameter.
	dVector p0 (x, 1000.0f, z); 
	dVector p1 (x, -1000.0f, z); 

	parameter = 1.2f;
	NewtonWorldRayCast (nWorld, &p0[0], &p1[0], RayCastPlacement, &parameter, NULL);
//	_ASSERTE (parameter < 1.0f);

	// the intersection is the interpolated value
	return 1000.0f - 2000.0f * parameter;
}


dFloat RandomVariable(dFloat amp)
{
	return amp * (dFloat (dRand() + dRand()) / dRAND_MAX - 1.0f) * 0.5f;
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

//float scale = 0.04f; 
//for(i = 0; i < SAMPLE_COUNT; i++) 
//{ 
	//cloud[i].m_x *= scale; 
	//cloud[i].m_y *= scale; 
	//cloud[i].m_z *= scale; 
//} 
			
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

//Ixx = 2.0f;
//Iyy = 2.0f;
//Izz = 2.0f;
  
	// add a Collision Transform modifier to show of a collision Geometry feature 
#ifdef USE_HULL_MODIFIER	
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
  
	// activate Cori lies
//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (blockBoxBody, blockBox);

	// Set auto freeze off
	NewtonBodySetAutoFreeze (blockBoxBody, sleepMode);

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
	NewtonBodySetLinearDamping (blockBoxBody, 0.0f);
	NewtonBodySetAngularDamping (blockBoxBody, damp);

	damp[0] = NewtonBodyGetLinearDamping (blockBoxBody);
	NewtonBodyGetAngularDamping (blockBoxBody, damp);


	//dVector veloc (0, -100, 0);
	//NewtonBodySetVelocity (blockBoxBody, &veloc[0]);
#ifdef USE_CONTINUE_COLLISION
	NewtonBodySetContinuousCollisionMode (blockBoxBody, 1);
#endif

/*
dMatrix m (GetIdentityMatrix());
dVector p0;
dVector p1;
NewtonCollisionCalculateAABB (collision, &matrix[0][0], &p0[0], &p1[0]);

NewtonBodyGetAABB (blockBoxBody, &p0[0], &p1[0]);	

dFloat volume;
dFloat volume1 = (4.0 * 3.1416/3.0) * size.m_x * size.m_x * size.m_x;
volume = NewtonConvexCollisionCalculateVolume (collision);
*/

}


class CrossPrimitive: public RenderPrimitive 
{
	public:
	CrossPrimitive (const dMatrix& matrix, const dVector& size, NewtonCollision** collisionArray)
		:RenderPrimitive  (nGrapphicWorld, matrix) 
	{
		m_childMatrix[0] = GetIdentityMatrix();
		m_childMatrix[1] = dgYawMatrix (3.1416f * 0.5f);
		m_childMatrix[2] = dgRollMatrix (3.1416f * 0.5f);

		m_childBox[0] = new BoxPrimitive (this, m_childMatrix[0], size); 
		m_childBox[1] = new BoxPrimitive (this, m_childMatrix[1], size); 
		m_childBox[2] = new BoxPrimitive (this, m_childMatrix[2], size); 

		// save the collision array
		m_collisionArray[0] = collisionArray[0];
		m_collisionArray[1] = collisionArray[1];
		m_collisionArray[2] = collisionArray[2];

	}


	// set the transform and all child object transform
	static void PhysicsSetTransform (
		const NewtonBody* body, 
		const dFloat* matrix)
	{
		CrossPrimitive* primitive;

		// get the graphic object form the rigid body
		primitive = (CrossPrimitive*) NewtonBodyGetUserData (body);

		const dMatrix& mat = (*((dMatrix*)matrix));
		primitive->SetMatrix (mat);


	#ifdef USE_HULL_MODIFIER
		dInt32 i;
		RenderPrimitive* box;
		NewtonCollision* collision;

		for (i = 0; i < 3; i ++) {
			// Warp the graphical part to match the collision geometry
			
			box = primitive->m_childBox[i];
			collision = primitive->m_collisionArray[i];

			// animate the Scale angle
			box->m_modifierScaleAngleX = dMod (box->m_modifierScaleAngleX + 0.002f, 6.2832f);
			box->m_modifierScaleAngleY = dMod (box->m_modifierScaleAngleY + 0.002f, 6.2832f);
			box->m_modifierScaleAngleZ = dMod (box->m_modifierScaleAngleZ + 0.002f, 6.2832f);

			box->m_modifierSkewAngleY = dMod (box->m_modifierSkewAngleY + 0.002f, 6.2832f);

			// get the Modifier scale values matrix 
			dMatrix collisionScaleMatrix (GetIdentityMatrix());
			NewtonConvexHullModifierGetMatrix (collision, &collisionScaleMatrix[0][0]);
			collisionScaleMatrix[0][0] = 0.5f * (1.0f + 0.5f * dSin (box->m_modifierScaleAngleX));
			collisionScaleMatrix[1][1] = 0.5f * (1.0f + 0.5f * dSin (box->m_modifierScaleAngleY));
			collisionScaleMatrix[2][2] = 0.5f * (1.0f + 0.5f * dSin (box->m_modifierScaleAngleZ));

			// also skew the shape
			collisionScaleMatrix[0][1] = 0.5f * (1 + dSin (primitive->m_modifierSkewAngleY));

			// copy the matrix back into the collision
			collisionScaleMatrix = collisionScaleMatrix * primitive->m_childMatrix[i];
  			NewtonConvexHullModifierSetMatrix (collision, &collisionScaleMatrix[0][0]);

			// apply the Modifier to the geometry too
			box->SetMatrix(collisionScaleMatrix);
		}
		#endif

	}

	void Render() const
	{
		RenderPrimitive::Render(); 
	}

	dMatrix m_childMatrix[3];
	RenderPrimitive* m_childBox[3]; 
	NewtonCollision* m_collisionArray[3];
};


void AddCrosses()
{
	int i;
	int j;
	dInt32 k;
	dFloat mass;

	dFloat x;
	dFloat z;
	NewtonBody* body;
	RenderPrimitive* obj;
	NewtonCollision* collision;
	NewtonCollision* collisionArray[3];

	dVector size (18.0f, 2.0f, 2.0f);
//	dVector size (10.0f, 10.0f, 10.0f);

	// create the collision 
	#ifdef USE_HULL_MODIFIER	
		// if using modifiers the offset must be NULL, the modifier will do the offset
		collisionArray[0] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
		collisionArray[1] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
		collisionArray[2] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

	#else
		// not using modifier we need to set the offset matrix into the primitive

		dMatrix ident (GetIdentityMatrix());


		//collisionArray[0] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
		collisionArray[0] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, &ident[0][0]); 
		dMatrix yawMatrix (dgYawMatrix (3.1416f * 0.5f)); 


		collisionArray[1] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, &yawMatrix[0][0]); 
		dMatrix rollMatrix (dgRollMatrix (3.1416f * 0.5f)); 


		collisionArray[2] = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, &rollMatrix[0][0]); 
	#endif


	mass = 100.0f;
//	Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
//	Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
//	Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;
//	Ixx = Ixx + Iyy + Izz;

	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 140.0f;
			matrix.m_posit.m_z = z;


			NewtonCollision* tmpCollArray[3];
							
			for (k = 0; k < 3; k ++) {
				#ifdef USE_HULL_MODIFIER	
					tmpCollArray[k] = NewtonCreateConvexHullModifier (nWorld, collisionArray[k]);
					dMatrix scaleMatrix (GetIdentityMatrix());
					NewtonConvexHullModifierSetMatrix (tmpCollArray[k], &scaleMatrix[0][0]);
				#else
					tmpCollArray[k] = collisionArray[k];
				#endif
			}

			// create a graphic box
			obj = new CrossPrimitive (matrix, size, tmpCollArray);

			//create the rigid body
			collision = NewtonCreateCompoundCollision (nWorld, 3, tmpCollArray);
			#ifdef USE_HULL_MODIFIER	
			for (k = 0; k < 3; k ++) {
				NewtonReleaseCollision (nWorld, tmpCollArray[k]);
			}
			#endif


			dVector inertia;
			dVector origin ;

			// calculate the inertia
			NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	


			body = NewtonCreateBody (nWorld, collision);
			NewtonReleaseCollision (nWorld, collision);

			// set to continue collision
#ifdef USE_CONTINUE_COLLISION
			NewtonBodySetContinuousCollisionMode (body, 1);
#endif


			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData (body, obj);

			// set a destructor for this rigid body
			NewtonBodySetDestructorCallback (body, PhysicsBodyDestructor);

			// set the transform call back function
			NewtonBodySetTransformCallback (body, CrossPrimitive::PhysicsSetTransform);
			

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);

			// set the mass matrix
			NewtonBodySetMassMatrix (body, mass, mass * inertia.m_x, mass * inertia.m_y, mass * inertia.m_z);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (body, &matrix[0][0]);
			PhysicsSetTransform (body, &matrix[0][0]);
		}
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collisionArray[0]);
	NewtonReleaseCollision (nWorld, collisionArray[1]);
	NewtonReleaseCollision (nWorld, collisionArray[2]);

}



void AddSpheres()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;

//	dVector size (0.2f, 0.2f, 0.2f);
//	dVector size (1.0f, 1.0f, 1.0f);
	dVector size (4.0f, 4.0f, 4.0f);
//	dVector size (10.0f, 10.0f, 10.0f);

	mass = 100.0f;

//mass = 2.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));
//	dVector origin (cameraEyepoint + cameraDir.Scale (2.0f));
//origin.m_z -= 0.5;

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 30.0f;
//			matrix.m_posit.m_y = FindFloor (x, z) + 2.0f;
			matrix.m_posit.m_z = z;

			AddPrimitiveObject (mass, matrix, size, _SPHERE_PRIMITIVE);
		}
	}
 }


void AddBoxes()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;
  
//	dVector size (1.0f, 1.0f, 1.0f);
	dVector size (8.0f, 5.0f, 7.0f);
//	dVector size (10.0f, 10.0f, 10.0f);
//	dVector size (0.2f, 0.2f, 0.2f);
//	dVector size (0.03f, 0.03f, 0.03f);
//	dVector size (0.035f, 0.035f, 0.035f);

	mass = 100.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 20.0f;
			matrix.m_posit.m_z = z;

			AddPrimitiveObject (mass, matrix, size, _BOX_PRIMITIVE);
		}
	}
}


void AddCones()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;
  
	dVector size (7.0f, 5.0f, 5.0f);

	mass = 50.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 40.0f;
			matrix.m_posit.m_z = z;

			AddPrimitiveObject (mass, matrix, size, _CONE_PRIMITIVE);
		}
	}
}


void AddCylinder()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;
  
	dVector size (8.0f, 4.0f, 4.0f);

	mass = 50.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 60.0f;
			matrix.m_posit.m_z = z;

			AddPrimitiveObject (mass, matrix, size, _CYLINDER_PRIMITIVE);
		}
	}
}

void AddCapsules()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;
  
	dVector size (10.0f, 3.0f, 3.0f);

	mass = 50.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 80.0f;
			matrix.m_posit.m_z = z;

			AddPrimitiveObject (mass, matrix, size, _CAPSULE_PRIMITIVE);
		}
	}
}


void AddChamferCylinder()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;
  
	dVector size (10.0f, 5.0f, 5.0f);

	mass = 50.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));


	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * SPACING;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * SPACING;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 100.0f;
			matrix.m_posit.m_z = z;


			AddPrimitiveObject (mass, matrix, size, _CHAMFER_CYLINDER_PRIMITIVE);
		}
	}
}


void AddRandomHull()
{
	int i;
	int j;
	dFloat mass;

	dFloat x;
	dFloat z;
  
	dVector size (10.0f, 5.0f, 5.0f);

	mass = 50.0f;
	dMatrix matrix (GetIdentityMatrix());
	dVector origin (cameraEyepoint + cameraDir.Scale (80.0f));

	for (i = 0; i < OBJEST_COUNT; i ++) {
		x = origin.m_x + (i - OBJEST_COUNT / 2) * size.m_z * 6.0f;
		for (j = 0; j < OBJEST_COUNT; j ++) {
			z = origin.m_z + (j - OBJEST_COUNT / 2) * size.m_z * 6.0f;

			// find a position above the floor
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (x, z) + 120.0f;
			matrix.m_posit.m_z = z;

			AddPrimitiveObject (mass, matrix, size, _RANDOM_COMVEX_HULL_PRIMITIVE);
		}
	}
}


void AddBodies()
{
	AddBoxes();
	AddCones();
	AddSpheres();
	AddCylinder();
	AddCapsules();
	AddCrosses();
	AddRandomHull();
	AddChamferCylinder();
} 




