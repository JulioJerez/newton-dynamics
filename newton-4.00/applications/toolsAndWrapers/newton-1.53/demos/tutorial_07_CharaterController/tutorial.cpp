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
#include "LevelPrimitive.h"
#include "RenderPrimitive.h"
#include "CharaterControl.h"

CHiResTimer timer;


#define DEBUG_LEVEL_COLLISION

static NewtonWorld* nWorld;
static dSceneNode* nGrapphicWorld;
static CharacterController* player;


bool hideObjects = false;
bool debugLinesMode = false;
bool animateUpVector = false;



int debugCount = 0;
dVector debugFace[1024][2];


dVector cameraDir (0.0f, 0.0f, -1.0f);
dVector cameraEyepoint (0.0f, 5.0f, 0.0f);
//dVector playerForceVector (0.0f, 0.0f, 0.0f); 

static void CleanUp ();
static void  Keyboard();
static void*  PhysicsAlloc (int sizeInBytes);
static void  PhysicsFree (void *ptr, int sizeInBytes);
static void  PhysicsApplyForceAndTorque (const NewtonBody* body);


static void  DebugShowCollision ();
static void  DebugShowBodyCollision (const NewtonBody* body);
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
	InitOpenGl (argc, argv, "Newton Tutotial 7: Character controller", DrawScene);

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
	dVector force (0.0f, -mass * 9.8f, 0.0f);
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




void  LevelCollisionCallback (
	const NewtonBody* bodyWithTreeCollision, 
	const NewtonBody* body,
	const dFloat* vertex, 
	int vertexstrideInBytes, 
	int indexCount, 
	const int* indexArray)
{
/*
	int i;
	int j;
	int stride = vertexstrideInBytes / sizeof (dFloat);

	if (debugLinesMode) {
		if (debugCount < 1000) {
			j = indexArray[indexCount - 1];
			dVector p0(vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2]);
			for (i = 0; i < indexCount; i ++) {
				j = indexArray[i];
				dVector p1(vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2]);
				debugFace[debugCount][0] = p0;
				debugFace[debugCount][1] = p1;
				debugCount ++;
				p0 = p1;
			}
		}
	}
*/
}

// show collision geometry in debug mode
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

// show rigid body collision geometry
void  DebugShowBodyCollision (const NewtonBody* body)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	// only show collision for dynamics objects
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	if (mass > 0.0f) {
		NewtonBodyForEachPolygonDo (body, DebugShowGeometryCollision);
	}
}


// show all collision geometry is debug mode
void  DebugShowCollision ()
{
	int i;

	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 0.0f);
	NewtonWorldForEachBodyDo (nWorld, DebugShowBodyCollision);

	glColor3f(0.0f, 0.0f, 1.0f);
	for (i = 0; i <debugCount; i ++) {
		glVertex3f (debugFace[i][0].m_x, debugFace[i][0].m_y, debugFace[i][0].m_z);
		glVertex3f (debugFace[i][1].m_x, debugFace[i][1].m_y, debugFace[i][1].m_z);
	}
	glEnd();
}


// create physics scene
void InitScene()
{
	BoxPrimitive* box;
	NewtonBody* boxBody;
	LevelPrimitive *level;
	NewtonCollision* collision;


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


	// /////////////////////////////////////////////////////////////////////
	//
	// set up all material and  material interactions
	SetupMaterials (nWorld);


	// /////////////////////////////////////////////////////////////////////
	//
	// Load level geometry and add it to the display list 
	#ifdef DEBUG_LEVEL_COLLISION
	level = new LevelPrimitive (nGrapphicWorld, "level1.dg", nWorld, LevelCollisionCallback);
	#else
	level = new LevelPrimitive (nGrapphicWorld, "level1.dg", nWorld, NULL);
	#endif

	// assign the concrete id to the level geometry
	NewtonBodySetMaterialGroupID (level->m_level, levelID);

	// /////////////////////////////////////////////////////////////////////
	//
	// Create a character controller
	dVector spawnSize (0.75f, 2.0f, 0.75f);
	dVector spawnPosit (0.0f, 10.0f, 0.0f);
	player = new CharacterController (nGrapphicWorld, nWorld, spawnPosit, spawnSize);


	// /////////////////////////////////////////////////////////////////////
	//
	// add some object to the background

	// set the initial size
	dVector size (0.75f, 0.75f, 0.75f);

	// create the collision 
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
		
	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = -10.0f; 

	dFloat damp[3];
	damp[0] = 0.0f;
	damp[1] = 0.0f;
	damp[2] = 0.0f;

	for (int k = 0; k < 10; k ++) { 
		location.m_posit.m_z = 0.0f; 
		for (int j = 0; j < 5; j ++) { 
			location.m_posit.m_y = 10.0f; 
			for (int i = 0; i < 1; i ++) {

				// create a graphic box
				box = new BoxPrimitive (nGrapphicWorld, location, size);

				//create the rigid body
				boxBody = NewtonCreateBody (nWorld, collision);

				// set the viscous damping the the minimum
				NewtonBodySetLinearDamping (boxBody, 0.0f);
				NewtonBodySetAngularDamping (boxBody, damp);


				// Set Material Id for this object
				NewtonBodySetMaterialGroupID (boxBody, woodID);

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (boxBody, box);

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (boxBody, box);

				// set a destrutor for this rigid body
				NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

				// set the tranform call back function
				NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyForceAndTorque);

				// set the mass matrix
				NewtonBodySetMassMatrix (boxBody, 1.0f, 1.0f, 1.0f, 1.0f);

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

	// read the keyboard
	Keyboard ();

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

	if (debugLinesMode) {
		DebugShowCollision ();
	}

	debugCount = 0;

	
	dVector color (1.0, 1.0f, 1.0f);
	Print (color, 4, 4, "fps %f  physics(ms) %d", 1.0f / (timeStep + 1.0e-6f), physicsTime);
	Print (color, 4, 34, "f1 - show collition mesh");
	Print (color, 4, 64, "f2 - hide model");
	Print (color, 4, 94, "f3 - toogle animation mode for upvector");
} 


//	Keyboard handler. 
void  Keyboard()
{
	// check for termination
	if (dGetKeyState (VK_ESCAPE) & 0x8000) {
		exit(0);
	}


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

	static unsigned prevF3Key = ~(dGetKeyState (VK_F3) & 0x8000);
	unsigned f3Key = dGetKeyState (VK_F3) & 0x8000;
	if (prevF3Key & f3Key) {
		// toggle debug line mode 
		animateUpVector = ! animateUpVector;
	}
	prevF3Key = ~f3Key;


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


	if (player) {
		dVector force (0.0f, 0.0f, 0.0f); 

		// camera control
		if (dGetKeyState ('W') & 0x8000) {
			force.m_x = 1.0f;
		} else if (dGetKeyState ('S') & 0x8000) {
			force.m_x = -1.0f;
		}

		if (dGetKeyState ('D') & 0x8000) {
			force.m_z = 1.0f;
		} else if (dGetKeyState ('A') & 0x8000) {
			force.m_z = -1.0f;
		}

		// only one jump per scape key hit
		static unsigned prevSpace = ~(dGetKeyState (' ') & 0x8000);
		unsigned space = dGetKeyState (' ') & 0x8000;
		if (prevSpace & space) {
			force.m_y = 1.0f;
		}
		prevSpace = ~space;
		

		player->SetForce (force);

		cameraEyepoint = player->GetMatrix().m_posit;
		cameraEyepoint.m_y += 1.0f;

		cameraEyepoint -= cameraDir.Scale (4.0f);
	}
} 


