// NewtonProp.cpp: implementation of the NewtonProp class.
//
//////////////////////////////////////////////////////////////////////

#include "NewtonProp.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

extern int woodID; 
#define GRAVITY	   -10.0f
#define BUFFER_SIZE	20000

NewtonProp::NewtonProp(dSceneNode* parent, const char* fileName, NewtonWorld* nWorld, const dMatrix& matrix)
	:dSceneNode(parent)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* body;

	dInt32 vertexCount; 
	dSceneNode* bodyPart;	
	NewtonCollision* vehicleCollision;
	char fullPathName[256];
	dLoaderContext context;

	// allocate a memory chunk of the stack
	dVector* vertex = new dVector[BUFFER_SIZE]; 	

	// open up and load the model geometry
	GetWorkingFileName (fileName, fullPathName);

	LoadFromFile (fullPathName, context);

	SetMatrix (matrix);

	// find the main body part
	bodyPart = (dSceneNode*) GetChild();
	_ASSERTE (bodyPart);


	// iterate again collecting the vertex array
	vertexCount = bodyPart->GetGeometry()->GetFlatVertexArray(vertex, BUFFER_SIZE);


	// create the collision geometry
	dMatrix localMatrix (bodyPart->CalcGlobalMatrix (this));
	vehicleCollision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), &localMatrix[0][0]);
	

	//create the rigid body
	body = NewtonCreateBody (nWorld, vehicleCollision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (body, this);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (body, woodID);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (body, Destroy);

	// set the transform call back function
	NewtonBodySetTransformCallback (body, SetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (body, ApplyGravityForce);

	
	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (vehicleCollision, &inertia[0], &origin[0]);	

	mass = 20.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

	// Set the vehicle Center of mass
	// the rear spoilers race the center of mass by a lot for a race car
	// we need to lower some more for the geometrical value of the y axis
	NewtonBodySetCentreOfMass (body, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (body, &matrix[0][0]);

	// release the collision 
	NewtonReleaseCollision (nWorld, vehicleCollision);	
	delete[] vertex; 
}

NewtonProp::~NewtonProp()
{

}


void NewtonProp::Destroy (const NewtonBody* body)
{
	NewtonProp* prop;

	// get the graphic object form the rigid body
	prop = (NewtonProp*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete prop;
}


// add force and torque to rigid body
void NewtonProp::SetTransform (const NewtonBody* body, const dFloat* matrix)
{
	NewtonProp* prop;

	// get the graphic object form the rigid body
	prop = (NewtonProp*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& mat = *((dMatrix*)matrix);
	prop->SetMatrix (mat);
}


// set the transformation of a rigid body
void NewtonProp::ApplyGravityForce (const NewtonBody* body)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);

//	dVector xxx (0.0f, 0.0f, 0.0f);
//	NewtonBodySetVelocity(body, &xxx.m_x);
//	NewtonBodySetOmega(body, &xxx.m_x);
	
}
