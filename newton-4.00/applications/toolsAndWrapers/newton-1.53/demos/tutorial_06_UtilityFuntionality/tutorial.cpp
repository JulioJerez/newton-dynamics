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
//#include "customJoints/CustomHinge.h"
//#include "customJoints/CustomRigid.h"

CHiResTimer timer;


static NewtonWorld* nWorld;
static dSceneNode* nGrapphicWorld;
static NewtonJoint* jointAssy5;
//static CustomHinge* jointAssy5;

static NewtonJoint* jointAssy6;
static NewtonJoint* jointAssy7;
static NewtonJoint* jointAssy8;
//static CustomHinge* jointAssy8;

//static CustomRigid * jointAssyRigid;
//static CustomRigid * jointAssyRigid2;
static NewtonBody* jointAssyRigid;
static NewtonBody* jointAssyRigid2;
static NewtonJoint* joint;
static NewtonJoint* joint2;
static NewtonJoint* joint3;
static NewtonJoint* joint4;

#define GRAVITY		-10.0f
#define CAMERA_SPEED 8.0f

static dVector cameraDir (1.0f, 0.0f, 0.0f);
static dVector cameraEyepoint (0.0f, 5.0f, 0.0f);

static int g_activeBodies = 0;
bool debugLinesMode = false;
//bool debugLinesMode = true;

int hightMap_debugCount;
dVector hightMap_debugLines[1024][2];


// set this to 0 for more realistic game (the stack will never go to sleep mode)
//#define AUTO_SLEEP_MODE 1
#define AUTO_SLEEP_MODE 0
static bool sleepMode = AUTO_SLEEP_MODE;


static void   CleanUp ();
static void   Keyboard();
static dFloat FindFloor (dFloat x, dFloat z);
static void*  PhysicsAlloc (int sizeInBytes);
static void   PhysicsFree (void *ptr, int sizeInBytes);
static void   PhysicsBodyDestructor (const NewtonBody* body);
static void   PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix);
static void   PhysicsApplyGravityForce (const NewtonBody* body);
static dFloat RayCastPlacement (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam);

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

void CreateBuildingStruture_1 (dVector& location);
void CreateBuildingStruture_2 ();
void CreateBuildingStruture_3 ();
void CreateBuildingStruture_4 ();
void CompoundCollisionStruture ();
NewtonBody* AddBlockObject (dFloat mass, const dMatrix& matrix, const dVector& size, PrimitiveType type);
NewtonBody* AddBlockObject2 (dFloat mass, const dMatrix& matrix, const dVector& size, PrimitiveType type);




//********************************************************************
// 
//	Newton Tutorial 7 Using ray cast and other utility functions
//
//********************************************************************
int main(int argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutorial 7: Using Utility functions", DrawScene);

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
void PhysicsApplyGravityForce (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
  //printf("Mass = %.3f       Inertia = %.3f %.3f %.3f \n",mass,Ixx,Iyy,Izz);
	dVector force (0.0f, mass*GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);

  //dFloat * fForce5 = new dFloat[3];
  //NewtonHingeGetJointForce(jointAssy5,fForce5);
  //printf("Force for link jointAssy5 = %.3f %.3f %.3f \n", fForce5[0],fForce5[1],fForce5[2]);
  //dFloat * fForce6 = new dFloat[3];
  //NewtonHingeGetJointForce(jointAssy6,fForce6);
  //printf("Force for link jointAssy6 = %.3f %.3f %.3f \n", fForce6[0],fForce6[1],fForce6[2]);
  //dFloat * fForce7 = new dFloat[3];
  //NewtonHingeGetJointForce(jointAssy7,fForce7);
  //printf("Force for link jointAssy7 = %.3f %.3f %.3f \n", fForce7[0],fForce7[1],fForce7[2]);
  //dFloat * fForce8 = new dFloat[3];
  //NewtonHingeGetJointForce(jointAssy8,fForce8);
  //printf("Force for link jointAssy8 = %.3f %.3f %.3f \n", fForce8[0],fForce8[1],fForce8[2]);

  //dFloat * fForce5 = new dFloat[3];
  //NewtonHingeGetJointForce(joint,fForce5);
  //printf("Force for link jointAssy5 = %.3f %.3f %.3f \n", fForce5[0],fForce5[1],fForce5[2]);
  //dFloat * fForce6 = new dFloat[3];
  //NewtonHingeGetJointForce(joint2,fForce6);
  //printf("Force for link jointAssy6 = %.3f %.3f %.3f \n", fForce6[0],fForce6[1],fForce6[2]);
  //dFloat * fForce7 = new dFloat[3];
  //NewtonHingeGetJointForce(joint3,fForce7);
  //printf("Force for link jointAssy7 = %.3f %.3f %.3f \n", fForce7[0],fForce7[1],fForce7[2]);
  //dFloat * fForce8 = new dFloat[3];
  //NewtonHingeGetJointForce(joint4,fForce8);
  //printf("Force for link jointAssy8 = %.3f %.3f %.3f \n", fForce8[0],fForce8[1],fForce8[2]);
}


// add force and torque to rigid body
void PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix mat (*((dMatrix*)matrix));

// get the collision Matrix
//dMatrix collisionScaleMatrix;
//NewtonCollision* collision;
//collision = NewtonBodyGetCollision (body);
//NewtonConvexHullModifierGetMatrix (collision, &collisionScaleMatrix[0][0]);
//mat = collisionScaleMatrix * mat;

	primitive->SetMatrix (mat);
}

// count the number of blocks actives in the scene
static void TrackActivesBodies (const NewtonBody* body, unsigned state)
{
	g_activeBodies += (state == 1) ? 1 : -1;
}

// change auto sleep mode for this body and activate the body
static void  ToggleAutoSleep (const NewtonBody* body)
{
	NewtonBodySetAutoFreeze (body, sleepMode);
	NewtonWorldUnfreezeBody (nWorld, body);
}


void DebugShowGeometryCollision (const NewtonBody* body, int vertexCount, const dFloat* faceVertec, int id)
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


void  DebugShowBodyCollision (const NewtonBody* body)
{
	NewtonBodyForEachPolygonDo (body, DebugShowGeometryCollision);
}


void  DebugShowCollision ()
{
	
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glBegin(GL_LINES);

	glColor3f(1.0f, 1.0f, 0.0f);

	dVector aabbP0 (-100.0f, -100.0f, -100.0f);
	dVector aabbP1 (100.0f, 100.0f, 100.0f);
	_ASSERT(0);
	//NewtonWorldForEachBodyInAABBDo (nWorld, &aabbP0[0], &aabbP1[0], DebugShowBodyCollision);

	glColor3f(1.0f, 1.0f, 1.0f);
	for (int i = 0; i < hightMap_debugCount; i ++) {
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


	// get the time step
	timeStep = timer.GetElapsedSeconds();
  //printf("timeStep = %.3f \n",timeStep);

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
	Print (color, 4,  4, "fps %f     active bodies %d", 1.0f / (timeStep + 1.0e-6f), g_activeBodies);
	Print (color, 4, 34, "f1 - auto sleep %s      f2 - show collision boxes", sleepMode ? "ON" : "OFF");
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
		// pick any static rigid body
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

	// shot a vertical ray from a high altitude and collect the intersection parameter.
	dVector p0 (x, 1000.0f, z); 
	dVector p1 (x, -1000.0f, z); 

	parameter = 1.2f;
	_ASSERT(0);
	//NewtonWorldRayCast (nWorld, &p0[0], &p1[0], RayCastPlacement, &parameter, NULL);
	//_ASSERTE (parameter < 1.0f);

	// the intersection is the interpolated value
	return 1000.0f - 2000.0f * parameter;
}



//	Keyboard handler. 
void Keyboard()
{
	// check for termination
	if (dGetKeyState (VK_ESCAPE) & 0x8000) {
		exit(0);
	}


	static unsigned prevF1Key = ~(dGetKeyState (VK_F1) & 0x8000);
	unsigned f1Key = dGetKeyState (VK_F1) & 0x8000;
	if (prevF1Key & f1Key) {
		// toggle auto sleep mode when press F1
		sleepMode = ! sleepMode;
		_ASSERT(0);
		//NewtonWorldForEachBodyDo (nWorld, ToggleAutoSleep);
	}
	prevF1Key = ~f1Key;

	// show debug information when press F2
	static unsigned prevF2Key = ~(dGetKeyState (VK_F2) & 0x8000);
	unsigned f2Key = dGetKeyState (VK_F2) & 0x8000;
	if (prevF2Key & f2Key) {
		// toggle debug line mode 
		debugLinesMode = ! debugLinesMode;
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

	_ASSERT(0);
	//if (!MousePick (nWorld, mouse1, mouseLeftKey, 0.125f, 1.0f)) {
	//	// we are not in mouse pick mode, then we are in camera tracking mode
	//	if (mouseLeftKey) {
	//		// when click left mouse button the first time, we reset the camera
	//		// convert the mouse x position to delta yaw angle
	//		if (mouse1.x > (mouse0.x + 1)) {
	//			yawAngle += 1.0f * 3.1416 / 180.0f;
	//			if (yawAngle > (360.0f * 3.1416 / 180.0f)) {
	//				yawAngle -= (360.0f * 3.1416 / 180.0f);
	//			}
	//		} else if (mouse1.x < (mouse0.x - 1)) {
	//			yawAngle -= 1.0f * 3.1416 / 180.0f;
	//			if (yawAngle < 0.0f) {
	//				yawAngle += (360.0f * 3.1416 / 180.0f);
	//			}
	//		}
	//
	//		if (mouse1.y > (mouse0.y + 1)) {
	//			rollAngle += 1.0f * 3.1416 / 180.0f;
	//			if (rollAngle > (80.0f * 3.1416 / 180.0f)) {
	//				rollAngle = 80.0f * 3.1416 / 180.0f;
	//			}
	//		} else if (mouse1.y < (mouse0.y - 1)) {
	//			rollAngle -= 1.0f * 3.1416 / 180.0f;
	//			if (rollAngle < -(80.0f * 3.1416 / 180.0f)) {
	//				rollAngle = -80.0f * 3.1416 / 180.0f;
	//			}
	//		}
	//		dMatrix cameraDirMat (dgRollMatrix(rollAngle) * dgYawMatrix(yawAngle));
	//		cameraDir = cameraDirMat.m_front;
	//	}
	//}
	//
	//// save mouse position and left mouse key state for next frame
	//mouse0 = mouse1;
	//
	//
	//
	//// camera control
	//if (dGetKeyState ('W') & 0x8000) {
	//	cameraEyepoint += cameraDir.Scale (CAMERA_SPEED / 60.0f);
	//} else if (dGetKeyState ('S') & 0x8000) {
	//	cameraEyepoint -= cameraDir.Scale (CAMERA_SPEED / 60.0f);
	//}
	//
	//if (dGetKeyState ('D') & 0x8000) {
	//	dVector up (0.0f, 1.0f, 0.0f);
	//	dVector right (cameraDir * up);
	//	cameraEyepoint += right.Scale (CAMERA_SPEED / 60.0f);
	//} else if (dGetKeyState ('A') & 0x8000) {
	//	dVector up (0.0f, 1.0f, 0.0f);
	//	dVector right (cameraDir * up);
	//	cameraEyepoint -= right.Scale (CAMERA_SPEED / 60.0f);
	//}
} 


static NewtonCollision* floorBoxCollisiom;

// create physics scene
void InitScene()
{
	BoxPrimitive* blockFloor;
	NewtonBody* blockFloorBody;
	NewtonCollision* collision;

	// /////////////////////////////////////////////////////////////////////
	//
	// create the newton world
	nWorld = NewtonCreate (PhysicsAlloc, PhysicsFree);

	// Set the adaptive solve for compromise trade off speed/stability combination
  // NewtonSetSolverModel (nWorld, 20);

	// used the adaptive friction model
//	NewtonSetFrictionModel (nWorld, 1);

	// Set the termination function
	atexit(CleanUp); 


	// set the size of the world
	dVector minWorldBox (-100.0f, -100.0f, -100.0f); 
	dVector maxWorldBox ( 100.0f,  500.0f,  100.0f);
	NewtonSetWorldSize (nWorld, &minWorldBox.m_x, &maxWorldBox.m_x); 

	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	nGrapphicWorld = new SkyBoxPrimitive (NULL);

	//int defaulftID;
	//defaulftID = NewtonMaterialGetDefaultGroupID(nWorld);
	//NewtonMaterialSetDefaultSoftness (nWorld, defaulftID, defaulftID, 1.0f);
	//NewtonMaterialSetDefaultElasticity (nWorld, defaulftID, defaulftID, 1.0f);
	//NewtonMaterialSetDefaultFriction (nWorld, defaulftID, defaulftID, 1.0f, 0.9f);

	SetupMaterials (nWorld);

	// /////////////////////////////////////////////////////////////////////
	//
	// create a floor to hold the block world
//	dVector floorSize (100.0f, 2.0f, 100.0f);
	dVector floorSize (100.0f, 0.1f, 100.0f);
	dMatrix location (GetIdentityMatrix());
  location.m_posit.m_y = -10.0; //-1000.0f; 
	
	// create a box for floor 
	blockFloor = new BoxPrimitive (nGrapphicWorld, location, floorSize, g_ballTexture);

	// create the the floor collision, and body with default values
	collision = NewtonCreateBox (nWorld, floorSize.m_x, floorSize.m_y, floorSize.m_z, NULL); 
	blockFloorBody = NewtonCreateBody (nWorld, collision);

  floorBoxCollisiom = collision;

	NewtonBodySetMaterialGroupID (blockFloorBody, woodID);
	NewtonReleaseCollision (nWorld, collision);

	// set the transformation for this rigid body
	NewtonBodySetMatrix (blockFloorBody, &location[0][0]);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (blockFloorBody, blockFloor);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (blockFloorBody, PhysicsBodyDestructor);

	// position the camera relative to the building struct
	cameraEyepoint.m_y = FindFloor (cameraEyepoint.m_x, cameraEyepoint.m_z) + 10.0f;

	// /////////////////////////////////////////////////////////////////////
	//
	// build a standard block stack of 20 * 3 boxes for a total of 60
	dVector posit (cameraEyepoint.m_x + 15.0f, 0.0f, cameraEyepoint.m_z + 4.0f);
	CreateBuildingStruture_4();//CreateBuildingStruture_4 () ; // CreateBuildingStruture_3

	posit.m_x += 20.0f;
//	CreateBuildingStruture_1 (posit);

	posit.m_z -= 20.0f;
//	CreateBuildingStruture_1 (posit);

	posit.m_x -= 20.0f;
//	CreateBuildingStruture_1 (posit);

	posit.m_z += 40.0f;
//	CreateBuildingStruture_1 (posit);

	posit.m_x += 20.0f;
//	CreateBuildingStruture_1 (posit);
  
//	CreateBuildingStruture_2 ();
//	CompoundCollisionStruture ();
}


dFloat RandomVariable(dFloat amp)
{
	return amp * (dFloat (dRand() + dRand()) / dRAND_MAX - 1.0f) * 0.5f;
}


NewtonBody* AddBlockObject (dFloat mass, const dMatrix& srcMatrix, const dVector& size, PrimitiveType type)
{
	dFloat Ixx = 0.0f;
	dFloat Iyy = 0.0f;
	dFloat Izz = 0.0f;
	NewtonBody* blockBoxBody = NULL;
	RenderPrimitive* blockBox = NULL;
	NewtonCollision* collision = NULL;

	// set realistic mass and inertia matrix for each block

	dMatrix matrix (srcMatrix);
	switch (type) 
	{
		case _SPHERE_PRIMITIVE:
		{
			dFloat r = size.m_x * 0.5f;
		
			// create the collision 
			collision = NewtonCreateSphere (nWorld, r, r, r, NULL); 

			// create a graphic box
			blockBox = new SpherePrimitive (nGrapphicWorld, matrix, r, r, r);
			
			break;
		}

		case _BOX_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

			// uncomment this to make the collision unique, this should be used only when it is really necessary
			//NewtonCollisionMakeUnique (nWorld, collision);

			// create a graphic box
			blockBox = new BoxPrimitive (nGrapphicWorld, matrix, size);
			break;
		}

		case _CONE_PRIMITIVE:
		{
			dFloat h = size.m_x;
			dFloat r = size.m_y;

			//matrix = dgRollMatrix (3.0) * matrix;
			
			// create the collision 
			collision = NewtonCreateCone (nWorld, r, h, NULL); 

			// create a graphic box
			blockBox = new ConePrimitive (nGrapphicWorld, matrix, r, h);
			break;
		}

		case _CYLINDER_PRIMITIVE:
		{
			//matrix = dgRollMatrix (3.1416f * 0.5f) * matrix;

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

			int count;
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
			count = 5;

			// populate the cloud with pseudo Gaussian random points
			for (i = 6; i < SAMPLE_COUNT; i ++) {
				cloud [i].m_x = RandomVariable(size.m_x);
				cloud [i].m_y = RandomVariable(size.m_y * 2.0f);
				cloud [i].m_z = RandomVariable(size.m_z * 2.0f);
				count ++;
			}

			collision = NewtonCreateConvexHull (nWorld, count, &cloud[0].m_x, sizeof (dVector), NULL); 
			
			// create a graphic box
			blockBox = new ConvexHullPrimitive (nGrapphicWorld, matrix, nWorld, collision);
			break;
		}
	}

	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	
  printf("Origin = %.3f %.3f %.3f \n",origin[0],origin[1],origin[2]);

	Ixx = mass * inertia[0]; //size[0]/10;
	Iyy = mass * inertia[1]; //size[1]/10;
	Izz = mass * inertia[2]; //size[2]/10;

	//create the rigid body
	blockBoxBody = NewtonCreateBody (nWorld, collision ); //collision

	// set the correct center of gravity for this body
	NewtonBodySetCentreOfMass (blockBoxBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassMatrix (blockBoxBody, mass, Ixx, Iyy, Izz);

  
	// activate 
//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (blockBoxBody, blockBox);

	// assign the wood id
	NewtonBodySetMaterialGroupID (blockBoxBody, woodID); //woodID

	// Set auto freeze off
	NewtonBodySetAutoFreeze (blockBoxBody, sleepMode);

	// set a call back to catch the auto active state of this body
	_ASSERT(0);
	//NewtonBodySetAutoactiveCallback (blockBoxBody, TrackActivesBodies);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (blockBoxBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (blockBoxBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (blockBoxBody, PhysicsApplyGravityForce);


	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (blockBoxBody, &matrix[0][0]);
  printf("matrix = %.3f %.3f %.3f \n",matrix[3][0],matrix[3][1],matrix[3][2]);


	PhysicsSetTransform (blockBoxBody, &matrix[0][0]);
  dFloat fDamping = 1.0;
  dFloat fAngularDamping[3];
  fAngularDamping[0]=1.0;
  fAngularDamping[1]=1.0; 
  fAngularDamping[2]=1.0; 

  NewtonBodySetLinearDamping(blockBoxBody, fDamping);
  NewtonBodySetAngularDamping(blockBoxBody, fAngularDamping);

//dFloat xxx[4];
//xxx[3] = NewtonBodyGetLinearDamping (blockBoxBody);
//NewtonBodyGetAngularDamping (blockBoxBody, xxx);

  dFloat tempMass;
  dFloat tempIxx;
  dFloat tempIyy;
  dFloat tempIzz;
  NewtonBodyGetMassMatrix (blockBoxBody, &tempMass, &tempIxx, &tempIyy, &tempIzz);
  printf("Mass = %.3f       Inertia = %.3f %.3f %.3f \n",tempMass,tempIxx,tempIyy,tempIzz);



/*
int xxx;
dMatrix matrixB (GetIdentityMatrix());
dVector velocA(0, -1000, 0);
dVector omegaA(0, 0, 0);
dVector velocB(0, 0, 0);
dVector omegaB(0, 0, 0);
dFloat timeOfImpact;
dVector contacts[20];
dVector normals[20];
dFloat depths[20];
xxx = NewtonCollisionCollideContinue (nWorld, 20, 0.01f, 
									  collision, &matrix[0][0], &velocA[0], &omegaA[0], 
									  floorBoxCollisiom, &matrixB[0][0], &velocB[0], &omegaB[0],  
									  &timeOfImpact, &contacts[0][0], &normals[0][0], depths);
*/


	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

	return blockBoxBody;
}


NewtonBody* AddBlockObject2 (dFloat mass, const dMatrix& srcMatrix, const dVector& size, PrimitiveType type)
{
	dFloat Ixx = 0.0f;
	dFloat Iyy = 0.0f;
	dFloat Izz = 0.0f;
	NewtonBody* blockBoxBody = NULL;
	RenderPrimitive* blockBox = NULL;
	NewtonCollision* collision = NULL;

	// set realistic mass and inertia matrix for each block

	dMatrix matrix (srcMatrix);
	switch (type) 
	{
		case _SPHERE_PRIMITIVE:
		{
			dFloat r = size.m_x * 0.5f;
		
			// create the collision 
			collision = NewtonCreateSphere (nWorld, r, r, r, NULL); 

			// create a graphic box
			blockBox = new SpherePrimitive (nGrapphicWorld, matrix, r, r, r);
			
			break;
		}

		case _BOX_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

			// uncomment this to make the collision unique, this should be used only when it is really necessary
			//NewtonCollisionMakeUnique (nWorld, collision);

			// create a graphic box
			blockBox = new BoxPrimitive (nGrapphicWorld, matrix, size);
			break;
		}

		case _CONE_PRIMITIVE:
		{
			dFloat h = size.m_x;
			dFloat r = size.m_y;

			//matrix = dgRollMatrix (3.0) * matrix;
			
			// create the collision 
			collision = NewtonCreateCone (nWorld, r, h, NULL); 

			// create a graphic box
			blockBox = new ConePrimitive (nGrapphicWorld, matrix, r, h);
			break;
		}

		case _CYLINDER_PRIMITIVE:
		{
			//matrix = dgRollMatrix (3.1416f * 0.5f) * matrix;

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

			int count;
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
			count = 5;

			// populate the cloud with pseudo Gaussian random points
			for (i = 6; i < SAMPLE_COUNT; i ++) {
				cloud [i].m_x = RandomVariable(size.m_x);
				cloud [i].m_y = RandomVariable(size.m_y * 2.0f);
				cloud [i].m_z = RandomVariable(size.m_z * 2.0f);
				count ++;
			}

			collision = NewtonCreateConvexHull (nWorld, count, &cloud[0].m_x, sizeof (dVector), NULL); 
			
			// create a graphic box
			blockBox = new ConvexHullPrimitive (nGrapphicWorld, matrix, nWorld, collision);
			break;
		}
	}

	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	
  printf("Origin = %.3f %.3f %.3f \n",origin[0],origin[1],origin[2]);

	Ixx = size[0]/10; //size[0]/10;
	Iyy = size[1]/10; //size[1]/10;
	Izz = size[2]/10; //size[2]/10;

	//create the rigid body
	blockBoxBody = NewtonCreateBody (nWorld, collision ); //collision

	// set the correct center of gravity for this body
	NewtonBodySetCentreOfMass (blockBoxBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassMatrix (blockBoxBody, mass, Ixx, Iyy, Izz);

  
	// activate 
//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (blockBoxBody, blockBox);

	// assign the wood id
	NewtonBodySetMaterialGroupID (blockBoxBody, woodID); //woodID

	// Set auto freeze off
	NewtonBodySetAutoFreeze (blockBoxBody, 0); //sleepMode

	// set a call back to catch the auto active state of this body
	_ASSERT(0);
	//NewtonBodySetAutoactiveCallback (blockBoxBody, TrackActivesBodies);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (blockBoxBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (blockBoxBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (blockBoxBody, PhysicsApplyGravityForce);


	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (blockBoxBody, &matrix[0][0]);
  //printf("matrix = %.3f %.3f %.3f \n",matrix[3][0],matrix[3][1],matrix[3][2]);


	PhysicsSetTransform (blockBoxBody, &matrix[0][0]);
  dFloat fDamping = 0.8;
  dFloat fAngularDamping[3];
  fAngularDamping[0]=0.5;
  fAngularDamping[1]=0.5; 
  fAngularDamping[2]=0.5; 

  NewtonBodySetLinearDamping(blockBoxBody, fDamping);
  NewtonBodySetAngularDamping(blockBoxBody, fAngularDamping);

//dFloat xxx[4];
//xxx[3] = NewtonBodyGetLinearDamping (blockBoxBody);
//NewtonBodyGetAngularDamping (blockBoxBody, xxx);


/*
int xxx;
dMatrix matrixB (GetIdentityMatrix());
dVector velocA(0, -1000, 0);
dVector omegaA(0, 0, 0);
dVector velocB(0, 0, 0);
dVector omegaB(0, 0, 0);
dFloat timeOfImpact;
dVector contacts[20];
dVector normals[20];
dFloat depths[20];
xxx = NewtonCollisionCollideContinue (nWorld, 20, 0.01f, 
									  collision, &matrix[0][0], &velocA[0], &omegaA[0], 
									  floorBoxCollisiom, &matrixB[0][0], &velocB[0], &omegaB[0],  
									  &timeOfImpact, &contacts[0][0], &normals[0][0], depths);
*/


	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

	return blockBoxBody;
}


NewtonBody* AddBlockObject (dFloat mass, const dMatrix& srcMatrix, const dVector& size, PrimitiveType type, int nMaterialSet)
{
	dFloat Ixx = 0.0f;
	dFloat Iyy = 0.0f;
	dFloat Izz = 0.0f;
	NewtonBody* blockBoxBody = NULL;
	RenderPrimitive* blockBox = NULL;
	NewtonCollision* collision = NULL;

	// set realistic mass and inertia matrix for each block

	dMatrix matrix (srcMatrix);
	switch (type) 
	{
		case _SPHERE_PRIMITIVE:
		{
			dFloat r = size.m_x * 0.5f;
		
			// create the collision 
			collision = NewtonCreateSphere (nWorld, r, r, r, NULL); 

			// create a graphic box
			blockBox = new SpherePrimitive (nGrapphicWorld, matrix, r, r, r);
			
			break;
		}

		case _BOX_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

			// uncomment this to make the collision unique, this should be used only when it is really necessary
			//NewtonCollisionMakeUnique (nWorld, collision);

			// create a graphic box
			blockBox = new BoxPrimitive (nGrapphicWorld, matrix, size);
			break;
		}

		case _CONE_PRIMITIVE:
		{
			dFloat h = size.m_x;
			dFloat r = size.m_y;

			//matrix = dgRollMatrix (3.0) * matrix;
			
			// create the collision 
			collision = NewtonCreateCone (nWorld, r, h, NULL); 

			// create a graphic box
			blockBox = new ConePrimitive (nGrapphicWorld, matrix, r, h);
			break;
		}

		case _CYLINDER_PRIMITIVE:
		{
			//matrix = dgRollMatrix (3.1416f * 0.5f) * matrix;

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

			int count;
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
			count = 5;

			// populate the cloud with pseudo Gaussian random points
			for (i = 6; i < SAMPLE_COUNT; i ++) {
				cloud [i].m_x = RandomVariable(size.m_x);
				cloud [i].m_y = RandomVariable(size.m_y * 2.0f);
				cloud [i].m_z = RandomVariable(size.m_z * 2.0f);
				count ++;
			}

			collision = NewtonCreateConvexHull (nWorld, count, &cloud[0].m_x, sizeof (dVector), NULL); 
			
			// create a graphic box
			blockBox = new ConvexHullPrimitive (nGrapphicWorld, matrix, nWorld, collision);
			break;
		}
	}

	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);
  //printf("Origin = %.3f %.3f %.3f \n",origin[0],origin[1],origin[2]);

	Ixx = mass * inertia[0]; //size[0]/10;
	Iyy = mass * inertia[1]; //size[1]/10;
	Izz = mass * inertia[2]; //size[2]/10;

	//create the rigid body
	blockBoxBody = NewtonCreateBody (nWorld, collision ); //collision

	// set the correct center of gravity for this body
	NewtonBodySetCentreOfMass (blockBoxBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassMatrix (blockBoxBody, mass, Ixx, Iyy, Izz);

  
	// activate 
//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (blockBoxBody, blockBox);

	// assign the wood id
	NewtonBodySetMaterialGroupID (blockBoxBody, nMaterialSet); //woodID

	// Set auto freeze off
	NewtonBodySetAutoFreeze (blockBoxBody, sleepMode);

	// set a call back to catch the auto active state of this body
	_ASSERT(0);
	//NewtonBodySetAutoactiveCallback (blockBoxBody, TrackActivesBodies);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (blockBoxBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (blockBoxBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (blockBoxBody, PhysicsApplyGravityForce);


	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (blockBoxBody, &matrix[0][0]);
  //printf("matrix = %.3f %.3f %.3f \n",matrix[3][0],matrix[3][1],matrix[3][2]);

	PhysicsSetTransform (blockBoxBody, &matrix[0][0]);

//dFloat xxx[4];
//xxx[3] = NewtonBodyGetLinearDamping (blockBoxBody);
//NewtonBodyGetAngularDamping (blockBoxBody, xxx);


/*
int xxx;
dMatrix matrixB (GetIdentityMatrix());
dVector velocA(0, -1000, 0);
dVector omegaA(0, 0, 0);
dVector velocB(0, 0, 0);
dVector omegaB(0, 0, 0);
dFloat timeOfImpact;
dVector contacts[20];
dVector normals[20];
dFloat depths[20];
xxx = NewtonCollisionCollideContinue (nWorld, 20, 0.01f, 
									  collision, &matrix[0][0], &velocA[0], &omegaA[0], 
									  floorBoxCollisiom, &matrixB[0][0], &velocB[0], &omegaB[0],  
									  &timeOfImpact, &contacts[0][0], &normals[0][0], depths);
*/


	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

	return blockBoxBody;
}


PrimitiveType GetRandType()
{
//return _SPHERE_PRIMITIVE;
//return _BOX_PRIMITIVE;
//return _CONE_PRIMITIVE;
//return _CYLINDER_PRIMITIVE;
//return _CAPSULE_PRIMITIVE;
//return _CHAMFER_CYLINDER_PRIMITIVE;
//return _RANDOM_COMVEX_HULL_PRIMITIVE;			  

	static int index = -1;
	PrimitiveType probability[] = {_BOX_PRIMITIVE, 
//								   _SPHERE_PRIMITIVE, 
								   _CONE_PRIMITIVE,
								   _RANDOM_COMVEX_HULL_PRIMITIVE,
								   _CYLINDER_PRIMITIVE,
								   _RANDOM_COMVEX_HULL_PRIMITIVE,
//								   _CHAMFER_CYLINDER_PRIMITIVE, 
//								   _CAPSULE_PRIMITIVE,
	};

	
	
//	index = (dRand () >> 5) % sizeof (probability) / sizeof (int);
	index = (index + 1) % (sizeof (probability) / sizeof (int));
	return probability[index];
}


void CreateBuildingStruture_1 (dVector& location)
{
	int i;
	dFloat plankMass;
	dFloat columnMass;
	NewtonBody* body;
	// /////////////////////////////////////////////////////////////////////
	//
	// Build a parking lot type structure
	 
	dVector columnBoxSize (3.0f, 1.0f, 1.0f);
//	dVector columnBoxSize (3.0f, 3.0f, 1.0f);
	dVector plankBoxSize (6.0f, 1.0f, 6.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// get the floor position in from o the camera
//	baseMatrix.m_posit.m_x = cameraEyepoint.m_x + 20.0f;
//	baseMatrix.m_posit.m_z = cameraEyepoint.m_z + 4.0f;
	baseMatrix.m_posit = location;
	baseMatrix.m_posit.m_y = FindFloor (baseMatrix.m_posit.m_x, baseMatrix.m_posit.m_z); 
	baseMatrix.m_posit.m_y += columnBoxSize.m_x / 2.0f;
	

	// set realistic (extremes in this case for 24 bits precision) masses for the different components
	// note how the newton engine handle different masses ratios without compromising stability, 
	// we recommend the application keep this ration under 100 for contacts and 50 for joints 
	columnMass = 1.0f;
	//plankMass = 200.0f;
	plankMass = 50.0f;

	dMatrix columAlignment (dgRollMatrix(3.1416f * 0.5f));
//dMatrix columAlignment (dgRollMatrix(3.1416f * 0.0f));
//	for (i = 0; i < 10; i ++) { 
	for (i = 0; i < 10; i ++) { 
		dMatrix matrix(columAlignment * baseMatrix);
		// add the 4 column
		matrix.m_posit.m_x -=  (columnBoxSize.m_z - plankBoxSize.m_x) * 0.5f;
		matrix.m_posit.m_z -=  (columnBoxSize.m_z - plankBoxSize.m_z) * 0.5f;

//matrix.m_posit.m_y += 0.2;
		body = AddBlockObject (columnMass, matrix, columnBoxSize, GetRandType());
//NewtonBodySetContinuousCollisionMode (body, 1);
//dVector veloc (100, -200, 0);
//NewtonBodySetVelocity (body, &veloc[0]);
 
  		matrix.m_posit.m_x += columnBoxSize.m_z - plankBoxSize.m_x;
		body = AddBlockObject (columnMass, matrix, columnBoxSize, GetRandType());

		matrix.m_posit.m_z += columnBoxSize.m_z - plankBoxSize.m_z;		
		body = AddBlockObject (columnMass, matrix, columnBoxSize, GetRandType());
	
		matrix.m_posit.m_x -= columnBoxSize.m_z - plankBoxSize.m_x;
		body = AddBlockObject (columnMass, matrix, columnBoxSize, GetRandType());
  
		// add a plank
		baseMatrix.m_posit.m_y += (columnBoxSize.m_x + plankBoxSize.m_y) * 0.5f * 1.0f;
		body = AddBlockObject (plankMass, baseMatrix, plankBoxSize, _BOX_PRIMITIVE);


		// set up for another level
		baseMatrix.m_posit.m_y += (columnBoxSize.m_x + plankBoxSize.m_y) * 0.5f * 1.0f;
	}

}


void CreateBuildingStruture_2 ()
{
	int i;
	int j;
	dFloat gap;
	dFloat mass;

	// /////////////////////////////////////////////////////////////////////
	//
	// build a standard block stack of 20 * 3 boxes for a total of 60

	// set the Jenga rectangle using the golden ratio for base (1 * 0.5f, 1.62 * 0.5f, 2.0f)
	dVector blockBoxSize (0.8f, 0.5f, 0.8f * 3.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = cameraEyepoint.m_x + 12.0f;

	baseMatrix.m_posit.m_y = FindFloor (baseMatrix.m_posit.m_x, baseMatrix.m_posit.m_z); 
	baseMatrix.m_posit.m_y += blockBoxSize.m_y / 2.0f;


	// set realistic mass and inertia matrix for each block
	mass = 5.0f;

	// create a 90 degree rotation matrix
	dMatrix rotMatrix (dgYawMatrix (3.1416f * 0.5f));

	// separate a bit the block alone the horizontal direction
	gap = 0.01f;
	for (i = 0; i < 20; i ++) { 
		dMatrix matrix(baseMatrix);
		matrix.m_posit -= matrix.m_front.Scale (blockBoxSize.m_x - gap); 

		for (j = 0; j < 3; j ++) { 
			AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
			matrix.m_posit += matrix.m_front.Scale (blockBoxSize.m_x + gap);  
		}

		baseMatrix = rotMatrix * baseMatrix;
		baseMatrix.m_posit += matrix.m_up.Scale (blockBoxSize.m_y * 0.99f);
	}
}



void CreateBuildingStruture_3 ()
{
//	int i;
//	int j;
	dFloat gap;
	dFloat mass;

	// /////////////////////////////////////////////////////////////////////
	//
	// build a standard block stack of 20 * 3 boxes for a total of 60

	// set the Jenga rectangle using the golden ratio for base (1 * 0.5f, 1.62 * 0.5f, 2.0f)
	dVector blockBoxSize (0.8f, 0.5f, 0.8f * 3.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = cameraEyepoint.m_x + 12.0f;

	baseMatrix.m_posit.m_y = FindFloor (baseMatrix.m_posit.m_x, baseMatrix.m_posit.m_z); 
	baseMatrix.m_posit.m_y += blockBoxSize.m_y / 2.0f;


	// set realistic mass and inertia matrix for each block
	mass = 50.0f;

	// create a 90 degree rotation matrix
	dMatrix rotMatrix (dgYawMatrix (3.1416f * 0.5f));

	// separate a bit the block alone the horizontal direction
	gap = 0.01f;

  dMatrix matrix(baseMatrix);
  NewtonBody * firstBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot = new dFloat[3]; 
  pivot[0] = 0.0f;
  pivot[1] = 0.0f;
  pivot[2] = -2.4f;
  dFloat * pin = new dFloat[3]; 
  pin[0] = 1.0f;
  pin[1] = 0.0f;
  pin[2] = 0.0f;
  joint = NewtonConstraintCreateHinge (nWorld, pivot, pin, firstBody, NULL); //NewtonJoint* 



  matrix.m_posit.m_z += 3.6f;
  NewtonBody * secondBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot2 = new dFloat[3]; 
  pivot2[0] = 0.0f;
  pivot2[1] = 0.0f;
  pivot2[2] = 1.2f;
  dFloat * pin2 = new dFloat[3]; 
  pin2[0] = 1.0f;
  pin2[1] = 0.0f;
  pin2[2] = 0.0f;
  joint2 = NewtonConstraintCreateHinge (nWorld, pivot2, pin2, secondBody,firstBody); //NewtonJoint* 


  matrix.m_posit.m_y += 0.5f;
  NewtonBody * thirdBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot3 = new dFloat[3]; 
  pivot3[0] = 0.0f;
  pivot3[1] = 0.5f;
  pivot3[2] = 4.8f;
  dFloat * pin3 = new dFloat[3]; 
  pin3[0] = 1.0f;
  pin3[1] = 0.0f;
  pin3[2] = 0.0f;
  joint3 = NewtonConstraintCreateHinge (nWorld, pivot3, pin3, thirdBody, secondBody); //NewtonJoint* 

  matrix.m_posit.m_z -= 3.6f;
  NewtonBody * fourthBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot4 = new dFloat[3]; 
  pivot4[0] = 0.0f;
  pivot4[1] = 0.5f;
  pivot4[2] = 1.2f;
  dFloat * pin4 = new dFloat[3]; 
  pin4[0] = 1.0f;
  pin4[1] = 0.0f;
  pin4[2] = 0.0f;
  joint4 = NewtonConstraintCreateHinge (nWorld, pivot4, pin4, fourthBody, thirdBody); //NewtonJoint* 

  dFloat * pivot5 = new dFloat[3]; 
  pivot5[0] = 0.0f;
  pivot5[1] = 1.0f;
  pivot5[2] = -2.4f;
  dFloat * pin5 = new dFloat[3]; 
  pin5[0] = 1.0f;
  pin5[1] = 0.0f;
  pin5[2] = 0.0f;
  NewtonJoint* joint5 = NewtonConstraintCreateHinge (nWorld, pivot5, pin5, fourthBody, NULL); 


  matrix.m_posit.m_y += 0.5f;
  NewtonBody * fifthBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot6 = new dFloat[3]; 
  pivot6[0] = 0.0f;
  pivot6[1] = 1.5f;
  pivot6[2] = -2.4f;
  dFloat * pin6 = new dFloat[3]; 
  pin6[0] = 1.0f;
  pin6[1] = 0.0f;
  pin6[2] = 0.0f;
  NewtonJoint* joint6 = NewtonConstraintCreateHinge (nWorld, pivot6, pin6, fifthBody, NULL); 


  matrix.m_posit.m_z += 3.6f;
  NewtonBody * sixthBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot7 = new dFloat[3]; 
  pivot7[0] = 0.0f;
  pivot7[1] = 1.5f;
  pivot7[2] = 1.2f;
  dFloat * pin7 = new dFloat[3]; 
  pin7[0] = 1.0f;
  pin7[1] = 0.0f;
  pin7[2] = 0.0f;
  NewtonJoint* joint7 = NewtonConstraintCreateHinge (nWorld, pivot7, pin7, fifthBody,sixthBody); 


  dFloat * pivot8 = new dFloat[3]; 
  pivot8[0] = 0.0f;
  pivot8[1] = 1.5f;
  pivot8[2] = 4.8f;
  dFloat * pin8 = new dFloat[3]; 
  pin8[0] = 1.0f;
  pin8[1] = 0.0f;
  pin8[2] = 0.0f;
  NewtonJoint* joint8 = NewtonConstraintCreateHinge (nWorld, pivot8, pin8, sixthBody, fourthBody);



  //NewtonUserJointSetRowStiffness(joint,1.0);
  //NewtonUserJointSetRowStiffness(joint2,1.0);
  //NewtonUserJointSetRowStiffness(joint3,1.0);
  //NewtonUserJointSetRowStiffness(joint5,1.0);
  //NewtonUserJointSetRowStiffness(joint4,1.0);
  //NewtonUserJointSetRowStiffness(joint6,1.0);
  //NewtonUserJointSetRowStiffness(joint7,1.0);
  //NewtonUserJointSetRowStiffness(joint8,1.0);


  NewtonJointSetStiffness(joint,1.0);
  NewtonJointSetStiffness(joint2,1.0);
  NewtonJointSetStiffness(joint3,1.0);
  NewtonJointSetStiffness(joint4,1.0);
  NewtonJointSetStiffness(joint5,1.0);
  NewtonJointSetStiffness(joint6,1.0);
  NewtonJointSetStiffness(joint7,1.0);
  NewtonJointSetStiffness(joint8,1.0);



  matrix.m_posit.m_x += 2.0f;
  matrix.m_posit.m_y -= 1.0f;
  matrix.m_posit.m_z -= 3.6f;
  NewtonBody * seventhBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot9 = new dFloat[3];
  pivot9[0] = 2.0f;
  pivot9[1] = 0.0f;
  pivot9[2] = -2.4f;
  dFloat * pin9 = new dFloat[3];
  pin9[0] = 1.0f;
  pin9[1] = 0.0f;
  pin9[2] = 0.0f;
  NewtonJoint* joint9 = NewtonConstraintCreateHinge (nWorld, pivot9, pin9, seventhBody, NULL); // NewtonConstraintCreateHinge  , pin9



  matrix.m_posit.m_z += 3.6f;
  NewtonBody * eigthBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot10 = new dFloat[3]; 
  pivot10[0] = 2.0f;
  pivot10[1] = 0.0f;
  pivot10[2] = 1.2f;
  dFloat * pin10 = new dFloat[3]; 
  pin10[0] = 1.0f;
  pin10[1] = 0.0f;
  pin10[2] = 0.0f;
  NewtonJoint* joint10 = NewtonConstraintCreateHinge (nWorld, pivot10, pin10, eigthBody,seventhBody); // NewtonConstraintCreateHinge  , pin10


  dFloat * pivot11 = new dFloat[3]; 
  pivot11[0] = 2.0f;
  pivot11[1] = 0.0f;
  pivot11[2] = 4.8f;
  dFloat * pin11 = new dFloat[3]; 
  pin11[0] = 1.0f;
  pin11[1] = 0.0f;
  pin11[2] = 0.0f;
  NewtonJoint* joint11 = NewtonConstraintCreateHinge (nWorld, pivot11, pin11, eigthBody, secondBody); //, pin11

  NewtonJointSetStiffness(joint9,1);
  NewtonJointSetStiffness(joint10,1);
  NewtonJointSetStiffness(joint11,1);


  matrix.m_posit.m_x += 2.0f;
  //matrix.m_posit.m_y -= 1.0f;
  matrix.m_posit.m_z -= 3.6f;
  NewtonBody * ninethBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot12 = new dFloat[3];
  pivot12[0] = 4.0f;
  pivot12[1] = 0.0f;
  pivot12[2] = -2.4f;
  dFloat * pin12 = new dFloat[3];
  pin12[0] = 1.0f;
  pin12[1] = 0.0f;
  pin12[2] = 0.0f;
  NewtonJoint* joint12 = NewtonConstraintCreateHinge (nWorld, pivot12, pin12, ninethBody, NULL); // NewtonConstraintCreateHinge  , pin9



  matrix.m_posit.m_z += 3.6f;
  NewtonBody * tenthBody = AddBlockObject (mass, matrix, blockBoxSize, _BOX_PRIMITIVE);
  dFloat * pivot13 = new dFloat[3]; 
  pivot13[0] = 4.0f;
  pivot13[1] = 0.0f;
  pivot13[2] = 1.2f;
  dFloat * pin13 = new dFloat[3]; 
  pin13[0] = 1.0f;
  pin13[1] = 0.0f;
  pin13[2] = 0.0f;
  NewtonJoint* joint13 = NewtonConstraintCreateHinge (nWorld, pivot13, pin13, tenthBody,ninethBody); // NewtonConstraintCreateHinge  , pin10


  dFloat * pivot14 = new dFloat[3]; 
  pivot14[0] = 4.0f;
  pivot14[1] = 0.0f;
  pivot14[2] = 4.8f;
  dFloat * pin14 = new dFloat[3]; 
  pin14[0] = 1.0f;
  pin14[1] = 0.0f;
  pin14[2] = 0.0f;
  NewtonJoint* joint14 = NewtonConstraintCreateHinge (nWorld, pivot14, pin14, tenthBody, eigthBody); //, pin11

  NewtonJointSetStiffness(joint12,1);
  NewtonJointSetStiffness(joint13,1);
  NewtonJointSetStiffness(joint14,1);



  _ASSERT(0);
  //NewtonBodySetJointRecursiveCollision(firstBody,0);
  //NewtonBodySetJointRecursiveCollision(secondBody,0);
  //NewtonBodySetJointRecursiveCollision(thirdBody,0);
  //NewtonBodySetJointRecursiveCollision(fourthBody,0);
  //NewtonBodySetJointRecursiveCollision(fifthBody,0);
  //NewtonBodySetJointRecursiveCollision(sixthBody,0);
  //NewtonBodySetJointRecursiveCollision(seventhBody,0);
  //NewtonBodySetJointRecursiveCollision(eigthBody,0);
  //NewtonBodySetJointRecursiveCollision(ninethBody,0);
  //NewtonBodySetJointRecursiveCollision(tenthBody,0);
}







void CompoundCollisionStruture ()
{
	int i;
	int j;
	int count;
	int segments;
	int hullCount;
	dFloat A;
	dFloat C;
	dFloat alpha;
	dFloat beta0;
	dFloat beta1;
	dVector array[200];
	NewtonCollision* collision;
	NewtonCollision* conhexHullArray[100];
  
	A = 0.8f;
	C = 3.0f;

	segments = 16;
	hullCount = 0;
	// create the upper half of a change
	for (i = 0; i < segments; i ++) {
		beta0 = 2.0f * dFloat (i) * 3.1416 / segments; 
		beta1 = 2.0f * dFloat (i + 1) * 3.1416 / segments; 
		count = 0;
		for (j = 0; j < segments; j ++) {
			alpha = 2.0f * dFloat (j) * 3.1416 / segments; 
			array[count].m_x = A * dSin (alpha);
			array[count].m_y = (C + A * dCos(alpha)) * dCos (beta0);
			array[count].m_z = (C + A * dCos(alpha)) * dSin (beta0);
			count ++;

			array[count].m_x = A * dSin (alpha);
			array[count].m_y = (C + A * dCos(alpha)) * dCos (beta1);
			array[count].m_z = (C + A * dCos(alpha)) * dSin (beta1);
			count ++;
		}
		conhexHullArray[hullCount] = NewtonCreateConvexHull (nWorld, count, &array[0].m_x, sizeof (dVector), NULL);
		hullCount ++;
	}


	// test collision between two compound objects
#if 0
	NewtonCollision* collisionA;
	NewtonCollision* collisionB;
//	collisionB = NewtonCreateBox (nWorld, 3.5f, 3.5f, 3.5f, NULL);
	collisionB = NewtonCreateCompoundCollision (nWorld, hullCount, conhexHullArray); 
	collisionA = NewtonCreateCompoundCollision (nWorld, hullCount, conhexHullArray);

	dMatrix matrixA (GetIdentityMatrix());
	dMatrix matrixB (GetIdentityMatrix());
	matrixA.m_posit.m_y += 10.0f;
	matrixB.m_posit.m_z += 0.0f;

	dVector contacts[20];
	dVector normals[20];
	dFloat depths[20];
//	count = NewtonCollisionCollide (nWorld, 10, 
//									collisionA, &matrixA[0][0], 
//									collisionB, &matrixB[0][0],
//									&contacts[0][0], &normals[0][0], &depths[0]);


	dVector velocA(0, -1000, 0);
	dVector omegaA(0, 0, 0);
	dVector velocB(0, 0, 0);
	dVector omegaB(0, 0, 0);
	dFloat timeOfImpact;

	count = NewtonCollisionCollideContinue (nWorld, 1, 0.01f, 
										  collisionA, &matrixA[0][0], &velocA[0], &omegaA[0], 
										  collisionB, &matrixB[0][0], &velocB[0], &omegaB[0],  
										  &timeOfImpact, &contacts[0][0], &normals[0][0], depths);

	NewtonReleaseCollision (nWorld, collisionA);
	NewtonReleaseCollision (nWorld, collisionB);
#endif


	dMatrix baseMatrix (GetIdentityMatrix());
	baseMatrix.m_posit.m_x = cameraEyepoint.m_x + 20.0f;
	baseMatrix.m_posit.m_z = cameraEyepoint.m_z + 4.0f;
//	baseMatrix.m_posit = location;
	baseMatrix.m_posit.m_y = FindFloor (baseMatrix.m_posit.m_x, baseMatrix.m_posit.m_z); 
	baseMatrix.m_posit.m_y += 5.0f;

	dFloat mass;
	dVector origin;
	dVector inertia;

	mass = 10;
	for (i = 0; i < 1; i ++) {
		NewtonBody* blockBoxBody;
		RenderPrimitive* blockBox;
		baseMatrix = dgYawMatrix (0.5f * 3.1416) * baseMatrix;

		dMatrix matrix (baseMatrix);
		blockBox = new TorusPrimitive (nGrapphicWorld, matrix, A, C);

		// create the compound collision
		_ASSERT(0);
		//collision = NewtonCreateCompoundCollision (nWorld, hullCount, conhexHullArray);
		//
 		////create the rigid body
		//blockBoxBody = NewtonCreateBody (nWorld, collision);

//NewtonBodySetContinuousCollisionMode (blockBoxBody, 1);
//dVector veloc (0, -500, 0);
//NewtonBodySetVelocity (blockBoxBody, &veloc[0]);
  
		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (blockBoxBody, blockBox);

		// assign the wood id
		NewtonBodySetMaterialGroupID (blockBoxBody, woodID);

		// Set auto freeze off
		NewtonBodySetAutoFreeze (blockBoxBody, sleepMode);

		// set a call back to catch the auto active state of this body
		_ASSERT(0);
		//NewtonBodySetAutoactiveCallback (blockBoxBody, TrackActivesBodies);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (blockBoxBody, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (blockBoxBody, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (blockBoxBody, PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	
		NewtonBodySetMassMatrix (blockBoxBody, mass, inertia[0] * mass, inertia[1] * mass, inertia[2] * mass);

		// set the center of mass 
		_ASSERT(0);
		//NewtonBodyGetCentreOfMass (blockBoxBody, &origin[0]);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (blockBoxBody, &matrix[0][0]);
		PhysicsSetTransform (blockBoxBody, &matrix[0][0]);

		// release the collision geometry when not need it
		NewtonReleaseCollision (nWorld, collision);

		baseMatrix.m_posit.m_y += (C + A); 
	}

	// release all collision parts
	for (i = 0; i < hullCount; i ++) {
		NewtonReleaseCollision (nWorld, conhexHullArray[i]);
	}
 }



 void CreateBuildingStruture_4 ()
 {
	 //int i;
	 //int j;
	 dFloat gap;
	 dFloat mass;

	 // /////////////////////////////////////////////////////////////////////
	 //
	 // build a standard block stack of 20 * 3 boxes for a total of 60

	 // set the Jenga rectangle using the golden ratio for base (1 * 0.5f, 1.62 * 0.5f, 2.0f)
	 dVector blockBoxSize8 (0.03850f, 0.133580f, 0.1350f);
	 dVector blockBoxSize7 (0.204302f, 0.172838f, 0.160245f);
	 dVector blockBoxSize6 (0.473000f, 0.250530f, 0.1900f);
	 dVector blockBoxSize5 (0.632000f, 0.369000f, 0.469151f);

	 //dVector blockBoxSize8 (100.0f, 100.0f, 100.0f);
	 //dVector blockBoxSize7 (100.0f, 100.0f, 100.0f);
	 //dVector blockBoxSize6 (100.0f, 100.0f, 100.0f);
	 //dVector blockBoxSize5 (100.0f, 100.0f, 100.0f);

	 // create the stack
	 dMatrix baseMatrix (GetIdentityMatrix());
	 dMatrix matAssy5 = baseMatrix;
	 matAssy5.m_posit = dVector(2.892221f, 0.008445f, 2.101425f);
	 dMatrix matAssy6 = baseMatrix;
	 matAssy6.m_posit = dVector(3.444721f, -0.014763f, 2.2360f);
	 dMatrix matAssy7 = baseMatrix;
	 matAssy7.m_posit = dVector(3.64757f, -0.008636f, 2.236059f);
	 dMatrix matAssy8 = baseMatrix;
	 matAssy8.m_posit = dVector(3.768971f, -0.019055f, 2.2360f);

	 //dMatrix baseMatrix (GetIdentityMatrix());
	 // dMatrix matAssy5 = baseMatrix;
	 // matAssy5.m_posit = dVector(392.221f, 0.0f, 101.425f);
	 // dMatrix matAssy6 = baseMatrix;
	 // matAssy6.m_posit = dVector(944.721f, 0.0f, 236.0f);
	 // dMatrix matAssy7 = baseMatrix;
	 // matAssy7.m_posit = dVector(1147.57f, 0.0f, 236.059f);
	 // dMatrix matAssy8 = baseMatrix;
	 // matAssy8.m_posit = dVector(1268.971f, -0.0f, 236.0f);


	 // for the elevation of the floor at the stack position
	 baseMatrix.m_posit.m_x = cameraEyepoint.m_x + 12.0f;

	 baseMatrix.m_posit.m_y = FindFloor (baseMatrix.m_posit.m_x, baseMatrix.m_posit.m_z); 
	 //baseMatrix.m_posit.m_y += blockBoxSize.m_y / 2.0f;


	 // set realistic mass and inertia matrix for each block
	 mass = 1.0f;

	 // create a 90 degree rotation matrix
	 dMatrix rotMatrix (dgYawMatrix (3.1416f * 0.5f));

	 // separate a bit the block alone the horizontal direction
	 gap = 0.01f;

	 dMatrix matrix(baseMatrix);
	 NewtonBody * Assy5 = AddBlockObject (mass, matAssy5, blockBoxSize5, _BOX_PRIMITIVE);
	 dFloat * pivot = new dFloat[3]; 
	 pivot[0] = 2.798221f;
	 pivot[1] = -0.079555f;
	 pivot[2] = 2.0910f;
	 dFloat * pin = new dFloat[3]; 
	 pin[0] = 0.0f;
	 pin[1] = 1.0f;
	 pin[2] = 0.0f;
	 _ASSERT(0);
	 //jointAssy5 = new CustomHinge (pivot, pin, Assy5, NULL); //NewtonJoint* 
	 

	 NewtonBody * Assy6 = AddBlockObject (mass, matAssy6, blockBoxSize6, _BOX_PRIMITIVE);
	 dFloat * pivot2 = new dFloat[3]; 
	 pivot2[0] = 3.249219f;
	 pivot2[1] = -0.019026f;
	 pivot2[2] = 2.235000f;
	 dFloat * pin2 = new dFloat[3]; 
	 pin2[0] = 1.0f;
	 pin2[1] = 0.0f;
	 pin2[2] = 0.0f;
	 //jointAssy6 = NewtonConstraintCreateHinge (nWorld, pivot2, pin2, Assy6,Assy5); //NewtonJoint* 
	 _ASSERT(0);
	 //jointAssyRigid2 = new CustomRigid(pivot2,Assy6,Assy5);

/*
	 NewtonBody * Assy7 = AddBlockObject (mass, matAssy7, blockBoxSize7, _BOX_PRIMITIVE);
	 dFloat * pivot3 = new dFloat[3]; 
	 pivot3[0] = 3.618219f;
	 pivot3[1] = 0.110498f;
	 pivot3[2] = 2.236000f;
	 dFloat * pin3 = new dFloat[3]; 
	 pin3[0] = 0.0f;
	 pin3[1] = 1.0f;
	 pin3[2] = 0.0f;
	 //jointAssy7 = NewtonConstraintCreateHinge (nWorld, pivot3, pin3, Assy7, Assy6); //NewtonJoint* 
	 jointAssyRigid = new CustomRigid(pivot3,Assy7, Assy6);


	 NewtonBody * Assy8 = AddBlockObject (mass, matAssy8, blockBoxSize8, _BOX_PRIMITIVE);
	 dFloat * pivot4 = new dFloat[3]; 
	 pivot4[0] = 3.768955f;
	 pivot4[1] = -0.019055f;
	 pivot4[2] = 2.236000f;
	 dFloat * pin4 = new dFloat[3]; 
	 pin4[0] = 1.0f;
	 pin4[1] = 0.0f;
	 pin4[2] = 0.0f;
//	 jointAssy8 = NewtonConstraintCreateHinge (nWorld, pivot4, pin4, Assy8, Assy7); //NewtonJoint* 
	 jointAssy8 = new CustomHinge (pivot4, pin4, Assy8, Assy7); //NewtonJoint* 



	 //NewtonUserJointSetRowStiffness(joint,1.0);
	 //NewtonUserJointSetRowStiffness(joint2,1.0);
	 //NewtonUserJointSetRowStiffness(joint3,1.0);
	 //NewtonUserJointSetRowStiffness(joint5,1.0);
	 //NewtonUserJointSetRowStiffness(joint4,1.0);
	 //NewtonUserJointSetRowStiffness(joint6,1.0);
	 //NewtonUserJointSetRowStiffness(joint7,1.0);
	 //NewtonUserJointSetRowStiffness(joint8,1.0);
	 //NewtonJointSetStiffness(jointAssy5,1.0f);
	 //NewtonJointSetStiffness(jointAssy6,1.0f);
	 //NewtonJointSetStiffness(jointAssy7,1.0f);
	 //NewtonJointSetStiffness(jointAssy8,1.0f);

	 NewtonBodySetJointRecursiveCollision(Assy8,0);
	 NewtonBodySetJointRecursiveCollision(Assy7,0);
	 NewtonBodySetJointRecursiveCollision(Assy6,0);
	 NewtonBodySetJointRecursiveCollision(Assy5,0);

	 dFloat tempMass;
	 dFloat tempIxx;
	 dFloat tempIyy;
	 dFloat tempIzz;

	 NewtonBodyGetMassMatrix (Assy5, &tempMass, &tempIxx, &tempIyy, &tempIzz);
	 printf("Mass = %.3f       Inertia = %.3f %.3f %.3f \n",tempMass,tempIxx,tempIyy,tempIzz);
	 NewtonBodyGetMassMatrix (Assy6, &tempMass, &tempIxx, &tempIyy, &tempIzz);
	 printf("Mass = %.3f       Inertia = %.3f %.3f %.3f \n",tempMass,tempIxx,tempIyy,tempIzz);
	 NewtonBodyGetMassMatrix (Assy7, &tempMass, &tempIxx, &tempIyy, &tempIzz);
	 printf("Mass = %.3f       Inertia = %.3f %.3f %.3f \n",tempMass,tempIxx,tempIyy,tempIzz);
	 NewtonBodyGetMassMatrix (Assy8, &tempMass, &tempIxx, &tempIyy, &tempIzz);
	 printf("Mass = %.3f       Inertia = %.3f %.3f %.3f \n",tempMass,tempIxx,tempIyy,tempIzz);
*/
 }
