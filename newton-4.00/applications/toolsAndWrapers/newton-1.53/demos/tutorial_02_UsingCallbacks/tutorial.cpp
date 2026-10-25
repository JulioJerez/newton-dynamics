#include <stdafx.h>
#include "tutorial.h"
#include "HiResTimer.h"
#include "RenderPrimitive.h"


CHiResTimer timer;

#define CAMERA_SPEED 4.0f

static dVector cameraDir (0.0f, 0.0f, -1.0f);
static dVector cameraEyepoint (-10.0f, 0.0f, 5.0f);


static NewtonWorld* nWorld;
static dSceneNode* nGrapphicWorld;

static void CleanUp ();
static void  Keyboard();
static void*  PhysicsAlloc (int sizeInBytes);
static void  PhysicsFree (void *ptr, int sizeInBytes);
static void  PhysicsBodyDestructor (const NewtonBody* body);
static void  PhysicsApplyForceAndTorque (const NewtonBody* body);
static void  PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix);



//********************************************************************
// 
//	Newton Tutorial 2 Using Callbacks
//
//********************************************************************
int main(int argc, char **argv)
{
	// initialize opengl	
	InitOpenGl (argc, argv, "Newton Tutorial 2: Using Callbacks", DrawScene);

	// create the physics scene
	InitScene();
	
	// Execute opengl main loop for ever
	glutMainLoop();   

	return 0;
} 

// memory allocation for Newton
void*  PhysicsAlloc (int sizeInBytes)
{
	return malloc (sizeInBytes);
}

// memory de-allocation for Newton
void  PhysicsFree (void *ptr, int sizeInBytes)
{
	free (ptr);
}

// add force and torque to rigid body
void  PhysicsApplyForceAndTorque (const NewtonBody* body)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, -mass * 9.8f, 0.0f);
	NewtonBodySetForce (body, &force.m_x);
}

// set the transformation of a rigid body
void  PhysicsSetTransform (const NewtonBody* body, const dFloat* matrix)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& mat = *((dMatrix*)matrix);
	primitive->SetMatrix (mat);
}


// rigid body destructor
void  PhysicsBodyDestructor (const NewtonBody* body)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete primitive;
}

// destroy the world and every thing in it
void CleanUp ()
{
	// destroy the Newton world
	NewtonDestroy (nWorld);
}


/*
static const float dt = 0.02; 
static const float mass = 2.0; 
static const float radius = 0.5; 
static const float height = 2.0; 
static const float torque[] = {-50.0, 0.0, 0.0}; 


struct xxxx
{
	NewtonWorld* m_world;
	NewtonBody* m_body;
	BoxPrimitive* m_box;
	NewtonJoint* m_joint;
	float m_torqueTime;

	void init (NewtonWorld* nWorld)
	{

		float inertiaZZ = 0.5 * ::mass * ::radius * ::radius; 
		float inertiaYY = 1.0 / 12.0 * ::mass * (3.0 * ::radius * ::radius + ::height * ::height); 
		float inertiaXX = 1.0 / 12.0 * ::mass * (3.0 * ::radius * ::radius + ::height * ::height); 

		m_torqueTime = -1.0; 

		m_world = nWorld; 

		NewtonCollision* collision; 
		collision = NewtonCreateCapsule(m_world, ::radius, ::height + 2 * ::radius, NULL); 
		m_body = NewtonCreateBody(m_world, collision); 
		NewtonBodySetAutoFreeze(m_body, 0); 
		NewtonWorldUnfreezeBody(m_world, m_body); 
		NewtonBodySetForceAndTorqueCallback(m_body, ApplyForceAndTorque); 
		NewtonBodySetUserData(m_body, this); 
		NewtonBodySetMassMatrix(m_body, ::mass, inertiaXX, inertiaYY, inertiaZZ); 
		NewtonReleaseCollision(m_world, collision); 


		// Graphical stuffs 
//		Graphic::Capsule* bodyEntity = new Graphic::Capsule(::height, ::radius); 

		
		dVector size (::radius, ::height + 2 * ::radius, ::radius, ::radius);
		dMatrix location (GetIdentityMatrix());
//		location.m_posit.m_y = -5.0f; 

		m_box = new BoxPrimitive (nGrapphicWorld, location, size);
//		bodyEntity->SetColor(1.0, 0.0, 0.0, 1.0); 
//		bodyEntity->SetScale(scaleFactor); 
//		m_viewer3D->AddEntity(EntitySolidPair(m_body, bodyEntity)); 



		dFloat pivotPoint[3] = {0.0, 0.0, -::height / 2.0}; 
		dFloat pinDir[3] = {1.0, 0.0, 0.0}; 
		m_joint = NewtonConstraintCreateHinge(m_world, pivotPoint, pinDir, m_body, NULL); 
		NewtonHingeSetUserCallback(m_joint, JointCallback); 
		NewtonJointSetUserData(m_joint, this); 
	}
	
	
	
	static void ApplyForceAndTorque(const NewtonBody* body) 
	{ 
		float force[3]; 

		force[0] = 0.0; 
		force[1] = -9.8 * ::mass; 
		force[2] = 0.0; 

		NewtonBodySetForce(body, force); 

		xxxx* xxx = (xxxx*)NewtonBodyGetUserData(body);

		if (xxx->m_torqueTime > 0.0) { 
			xxx->m_torqueTime -= dt; 

			// Here I've also tried AddTorque with the same results 
			NewtonBodySetTorque(body, ::torque); 
		} 
	} 

	static unsigned int JointCallback(NewtonHingeSliderUpdateDesc* desc) 
	{ 
		unsigned retVal = 0; 

		dFloat angle = NewtonHingeGetJointAngle(m_joint); 
		dFloat low = (dFloat) -1.57; 
		dFloat high = (dFloat) 0.0; 

		if (angle >= high) { 
			desc[0].m_accel = NewtonHingeCalculateStopAlpha(m_joint, desc, high); 
			retVal = 1; 
		} else if (angle <= low) { 
			desc[0].m_accel = NewtonHingeCalculateStopAlpha(m_joint, desc, low); 
			retVal = 1; 
		} 

		return retVal; 
	} 

};

xxxx xxx;
*/

// create physics scene
void InitScene()
{
	BoxPrimitive* box;
	BoxPrimitive* floor;
	NewtonBody* boxBody;
	NewtonBody* floorBody; 
	NewtonCollision* collision;

	// create the newton world
	nWorld = NewtonCreate (PhysicsAlloc, PhysicsFree);

	// set the linear solver model for faster speed 
//	NewtonSetSolverModel (nWorld, 10);

	// set the adaptive friction model for faster speed 
//	NewtonSetFrictionModel (nWorld, 1);

	// Set the termination function
	atexit(CleanUp); 

	// create the sky box,
	nGrapphicWorld = new SkyBoxPrimitive (NULL);






	// create the the floor graphic objects
	dVector size (100.0f, 2.0f, 100.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = -5.0f; 
	
	// create a box for floor 
	floor = new BoxPrimitive (nGrapphicWorld, location, size, g_floorTexture);


	// create the the floor collision, and body with default values
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 
	floorBody = NewtonCreateBody (nWorld, collision);
	NewtonReleaseCollision (nWorld, collision);


	// set the transformation for this rigid body
	NewtonBodySetMatrix (floorBody, &location[0][0]);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (floorBody, floor);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);

//xxx.init (nWorld);

	// set the initial size
	size = dVector(0.5f, 0.5f, 0.5f);

	// create the collision 
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, NULL); 

	dMatrix rotate (dgRollMatrix (3.1416 * 0.5));

	// create 100 stacks of 10 boxes each
//	location.m_posit.m_x = -16.0f; 
	location.m_posit.m_x = -8.0f; 
//	for (int k = 0; k < 20; k ++) { 
	for (int k = 0; k < 1; k ++) { 
		location.m_posit.m_z =  0.0f; 
//		for (int j = 0; j < 20; j ++) { 
		for (int j = 0; j < 1; j ++) { 
			location.m_posit.m_y = -3.75f - 0.01f; 

location.m_posit.m_y += 2.0f;

			for (int i = 0; i < 1; i ++) {
//			for (int i = 0; i < 10; i ++) {
				// create a graphic box
				box = new BoxPrimitive (nGrapphicWorld, location, size);

				//create the rigid body
				boxBody = NewtonCreateBody (nWorld, collision);

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (boxBody, box);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyForceAndTorque);

				// set the mass matrix
				//NewtonBodySetMassMatrix (boxBody, 1.0f, 1.0f / 6.0f, 1.0f / 6.0f, 1.0f  / 6.0f);
				NewtonBodySetMassMatrix (boxBody, 1.0f, 1.0f, 1.0f, 1.0f);

				// set the matrix for both the rigid body and the graphic body
				NewtonBodySetMatrix (boxBody, &location[0][0]);
				PhysicsSetTransform (boxBody, &location[0][0]);

				location.m_posit.m_y += size.m_y * 2.0f;
//location.m_posit.m_x += 0.01f;
//				location.m_posit.m_y += size.m_y;
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
//	NewtonUpdate (nWorld, timeStep);
	NewtonUpdate (nWorld, 1.0f/60.0f);

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

} 



//	Keyboard handler. 
void Keyboard()
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



