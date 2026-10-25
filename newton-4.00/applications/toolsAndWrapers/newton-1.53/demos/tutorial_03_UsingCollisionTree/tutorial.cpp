//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

#include <stdafx.h>
#include "tutorial.h"
#include "HiResTimer.h"
#include "LevelPrimitive.h"
#include "RenderPrimitive.h"



CHiResTimer timer;

static NewtonWorld* nWorld;
static dSceneNode* nGrapphicWorld;
static dVector cameraDir (0.0f, 0.0f, -1.0f);
static dVector cameraEyepoint (0.0f, 5.0f, 0.0f);


static void CleanUp ();
static void  Keyboard();
static void*  PhysicsAlloc (int sizeInBytes);
static void  PhysicsFree (void *ptr, int sizeInBytes);
static void  PhysicsBodyDestructor (const NewtonBody* body);
static void  PhysicsApplyForceAndTorque (const NewtonBody* body);
static void  PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix);

static void  LevelCollisionCallback (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body,
										   const dFloat* vertex, int vertexstrideInBytes, int indexCount, const int* indexArray); 


//********************************************************************
// 
//	Newton Tutorial 2 Using Callbacks
//
//********************************************************************
int main(int argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutotial 3: Using Collition TreeA", DrawScene);

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

// set the tranformation of a rigid body
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


void LevelCollisionCallback (
	const NewtonBody* bodyWithTreeCollision, 
	const NewtonBody* body,
	const dFloat* vertex, 
	int vertexstrideInBytes, 
	int indexCount, 
	const int* indexArray)
{
	// the application can use this function for debugging purpose by writing the 
	// face to a global variable then display which face of the mesh are been used for collision.
    // ,,,,,,,,,,,
	// ..........

	// the application can use this function also to modify the collision geometry by changing the face ID
	// this could be uses full to make surface change for example from dry to wet, or breaking glass
	// for this the application should used the functions:
	// int id = NewtonTreeCollisionGetFaceAtribute (treeCollision, indexArray); 
	// NewtonTreeCollisionSetFaceAtribute (treeCollision, indexArray, new id);

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

	// create the sky box,
	nGrapphicWorld = new SkyBoxPrimitive (NULL);

	// /////////////////////////////////////////////////////////////////////
	//
	// Load level geometry and add it to the display list 
	level = new LevelPrimitive (nGrapphicWorld, "level1.dg", nWorld, LevelCollisionCallback);

	// /////////////////////////////////////////////////////////////////////
	//
	// add some object to the background

	// set the initial size
	dVector size (1.0f, 1.0f, 1.0f);

	// create the collision 
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
		
	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = -10.0f; 

	for (int k = 0; k < 10; k ++) { 
		location.m_posit.m_z = 0.0f; 
		for (int j = 0; j < 10; j ++) { 
			location.m_posit.m_y = 10.0f; 
			for (int i = 0; i < 1; i ++) {

//static xxx;
//xxx ++;
//if (xxx == 16){


				// create a graphic box
				box = new BoxPrimitive (nGrapphicWorld, location, size);

				//create the rigid body
				boxBody = NewtonCreateBody (nWorld, collision);

				//dVector omega(5, 0, 0);
				//NewtonBodySetOmega (boxBody, &omega[0]);

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (boxBody, box);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyForceAndTorque);

				// set the mass matrix
				NewtonBodySetMassMatrix (boxBody, 1.0f, 1.0f, 1.0f, 1.0f);

				// set the matrix for both the rigid body and the graphic body
				NewtonBodySetMatrix (boxBody, &location[0][0]);
				PhysicsSetTransform (boxBody, &location[0][0]);

//}

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


	// camera control
	if (dGetKeyState ('W') & 0x8000) {
		cameraEyepoint += cameraDir.Scale (20.0f / 60.0f);
	} else if (dGetKeyState ('S') & 0x8000) {
		cameraEyepoint -= cameraDir.Scale (20.0f / 60.0f);
	}

	if (dGetKeyState ('D') & 0x8000) {
		dVector up (0.0f, 1.0f, 0.0f);
		dVector right (cameraDir * up);
		cameraEyepoint += right.Scale (10.0f / 60.0f);
	} else if (dGetKeyState ('A') & 0x8000) {
		dVector up (0.0f, 1.0f, 0.0f);
		dVector right (cameraDir * up);
		cameraEyepoint -= right.Scale (10.0f / 60.0f);
	}
} 



