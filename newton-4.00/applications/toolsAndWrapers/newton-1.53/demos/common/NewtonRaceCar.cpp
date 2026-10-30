// NewtonRaceCar.cpp: implementation of the RaceCar class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "NewtonRaceCar.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// vehicle hierachy:
// MainBody
//   FR_tire
//   FR_axel
//   FR_brake
//   FR_topSusp    
//   FR_bottomSusp    
//       
//   LR_tire
//   LR_axel
//   LR_brake
//   LR_topSusp    
//   LR_bottomSusp    
//
//   RR_tire
//   RR_axel
//   RR_brake
//   RR_topSusp    
//   RR_bottomSusp    
//      
//   RL_tire
//   RL_axel
//   RL_brake
//   RL_topSusp    
//   RL_bottomSusp    


extern int vehicleID; 
extern bool hideVehicle;

#define MAX_TORQUE (2400.0f)
#define VEHICLE_MASS (1000.0f)
#define BUFFER_SIZE	20000

// set frequency to 2 hertz
#define SUSPENSION_FREQUENCE (2.0f)
//#define SUSPENSION_LENGTH	(0.15f)
#define SUSPENSION_LENGTH	(0.3f)

#define MAX_STEER_ANGLE (30.0f * 3.1416f / 180.0f)


NewtonRaceCar::NewtonRaceCar(
	dSceneNode* parent, 
	const char* fileName, 
	NewtonWorld* nWorld, 
	const dMatrix& matrix)
	:NewtonVehicleNode (parent, matrix)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	dInt32 vertexCount; 
	dSceneNode* bodyPart;	
	NewtonCollision* vehicleCollision;
	char fullPathName[256];
	dLoaderContext context;


	// open up and load the model geometry
	GetWorkingFileName (fileName, fullPathName);
	LoadFromFile (fullPathName, context);

	SetMatrix (matrix);

	// create the main vehicle rigid body

	// find the main body part
	bodyPart = (dSceneNode*) Find ("MainBody");
	_ASSERTE (bodyPart);

	// allocate temporary memory buffer
	dVector* vertex = new dVector[BUFFER_SIZE]; 	

	// iterate again collecting the vertex array
	vertexCount = bodyPart->GetGeometry()->GetFlatVertexArray (vertex, BUFFER_SIZE);


	// create the collision geometry
	dMatrix localMatrix (bodyPart->CalcGlobalMatrix (this));
	vehicleCollision = NewtonCreateConvexHull (nWorld, vertexCount, &vertex[0].m_x, sizeof (dVector), &localMatrix[0][0]);



	//create the rigid body
	m_vehicleBody = NewtonCreateBody (nWorld, vehicleCollision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (m_vehicleBody, this);

	// set the material group id for vehicle
	NewtonBodySetMaterialGroupID (m_vehicleBody, vehicleID);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (m_vehicleBody, DestroyVehicle);

	// set the transform call back function
	NewtonBodySetTransformCallback (m_vehicleBody, SetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (m_vehicleBody, ApplyGravityForce);

	
	dVector origin;
	dVector inertia;

	// calculate the moment of inertia and the relative center of mass of the solid
	NewtonConvexCollisionCalculateInertialMatrix (vehicleCollision, &inertia[0], &origin[0]);	

	mass = VEHICLE_MASS;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// set the mass matrix
	NewtonBodySetMassMatrix (m_vehicleBody, mass, Ixx, Iyy, Izz);

	// Set the vehicle Center of mass
	// the rear spoilers race the center of mass by a lot for a race car
	// we need to lower some more for the geometrical value of the y axis
	origin.m_y *= 0.5f;
	NewtonBodySetCentreOfMass (m_vehicleBody, &origin[0]);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (m_vehicleBody, &matrix[0][0]);

	// release the collision 
	NewtonReleaseCollision (nWorld, vehicleCollision);	


	// ////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//  second we need to add a vehicle joint to the body
	//
	// ////////////////////////////////////////////////////////////////////////////////////////////////
	dVector updir (matrix.m_up);
	m_vehicleJoint = NewtonConstraintCreateVehicle (nWorld, &updir[0], m_vehicleBody); 

#if 0
	// Set the vehicle control functions
	NewtonVehicleSetTireCallback (m_vehicleJoint, TireUpdate);

 	// set the linear drag to the minimum
	NewtonBodySetLinearDamping (m_vehicleBody, 0.0f);

	// Add tires;
	m_rearRighTire.Setup (this, "RR");
	m_rearLeftTire.Setup (this, "RL");
	m_frontRighTire.Setup (this, "FR");
	m_frontLeftTire.Setup (this, "FL");
#endif

	delete[] vertex;
}

NewtonRaceCar::~NewtonRaceCar()
{
}


void NewtonRaceCar::DestroyVehicle (const NewtonBody* body)
{
	NewtonRaceCar* vehicle;

	// get the graphic object form the rigid body
	vehicle = (NewtonRaceCar*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete vehicle;
}





//#define USE_BOUYANCY_FORCE

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
void NewtonRaceCar::ApplyGravityForce (const NewtonBody* body)
{

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dFloat speed;
	NewtonVehicleNode* car;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	
	car = (NewtonVehicleNode*) NewtonBodyGetUserData(body);
	
	speed = dAbs (car->GetSpeed());
	mass *= (1.0f + speed / 20.0f);


	dVector force (0.0f, mass * GRAVITY, 0.0f);
	NewtonBodySetForce (body, &force.m_x);


	#ifdef USE_BOUYANCY_FORCE
	static bool waitOneFrame = true;
	if (!waitOneFrame) {
		dVector gravity (0.0f, GRAVITY, 0.0f);
//		NewtonBodyAddBuoyancyForce (body, 1.0f, 0.8f, 0.8f, &gravity[0], PhysicsBouyancyPlane, (void*) body);
		NewtonBodyAddBuoyancyForce (body, 100.0f, 0.8f, 0.8f, &gravity[0], PhysicsBouyancyPlane, (void*) body);
	}
	waitOneFrame = false;

	#endif


/*
	static int xxx;
	xxx ++;
	if (xxx == 2000){
		NewtonDestroyBody (xxxx, body);
	}
*/

}



// Set the vehicle matrix and all tire matrices
void NewtonRaceCar::SetTransform (const NewtonBody* body, const dFloat* matrixPtr)
{
	void* tyreId;
	dFloat sign;
	dFloat angle;
	dFloat brakePosition;
	
	RaceCarTire* tireRecord;
	NewtonRaceCar* vehicle;

	// get the graphic object form the rigid body
	vehicle = (NewtonRaceCar*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& matrix = *((dMatrix*)matrixPtr);
	vehicle->SetMatrix (matrix);

    //Set the global matrix for each tire
	dMatrix invMatrix (matrix.Inverse());
	dMatrix tireMatrix;
	for (tyreId = NewtonVehicleGetFirstTireID (vehicle->m_vehicleJoint); tyreId; tyreId = NewtonVehicleGetNextTireID (vehicle->m_vehicleJoint, tyreId)) {

		// get the graphic object and set the transformation matrix 
		tireRecord = (RaceCarTire*) NewtonVehicleGetTireUserData (vehicle->m_vehicleJoint, tyreId);

		NewtonVehicleGetTireMatrix (vehicle->m_vehicleJoint, tyreId, &tireMatrix[0][0]);

		// calculate the local matrix 
		tireMatrix = tireMatrix * invMatrix * tireRecord->m_tireOffsetMatrix;
		tireRecord->m_tireNode->SetMatrix (tireMatrix);


		// calculate the parametric brake position
		brakePosition = tireMatrix.m_posit.m_y - tireRecord->m_tireRefHeight;
		tireRecord->m_brakeNode->GetMatrix().m_posit.m_y = tireRecord->m_brakeRefPosition.m_y + brakePosition; 


		// set suspensionMatrix;
		sign = (tireRecord->m_brakeRefPosition.m_z > 0.0f) ? 1.0f : -1.0f;
		angle = dAtan2 (sign * brakePosition, dAbs (tireRecord->m_brakeRefPosition.m_z));
		dMatrix rotationMatrix (dgPitchMatrix(angle));
		tireRecord->m_axelNode->SetMatrix (rotationMatrix * tireRecord->m_axelMatrix);
		tireRecord->m_topSuspNode->SetMatrix (rotationMatrix * tireRecord->m_suspentionTopMatrix);
		tireRecord->m_bottomSuspNode->SetMatrix (rotationMatrix * tireRecord->m_suspentionBottomMatrix);
	}
}


void NewtonRaceCar::Render() const
{
	if (!hideVehicle) {
		NewtonVehicleNode::Render();  
	}
}

void NewtonRaceCar::SetSteering(dFloat value)
{
	dFloat speed;
	dFloat scale;
	speed = dAbs (GetSpeed());
	scale = (1.0f - speed / 60.0f) > 0.1f ? (1.0f - speed / 60.0f) : 0.1f;
	if (value > 0.0f) {
		value = MAX_STEER_ANGLE * scale;
	} else if (value < 0.0f) {
		value = -MAX_STEER_ANGLE * scale;
	} else {
		value = 0.0f;
	}

	m_frontRighTire.SetSteer (value);
	m_frontLeftTire.SetSteer (value);
}


void NewtonRaceCar::SetTireTorque(dFloat value)
{
	dFloat speed;

	speed = dAbs (GetSpeed());
	if (value > 0.0f) {
		value = MAX_TORQUE * (1.0f - speed / 200.0f);
	} else if (value < 0.0f) {
		value = -MAX_TORQUE * 0.5f * (1.0f - speed / 200.0f);
	} else {
		value = 0.0f;
	}

	m_rearRighTire.SetTorque (value);
	m_rearLeftTire.SetTorque (value);
}


void NewtonRaceCar::SetApplyHandBrakes (dFloat value)
{
	m_rearLeftTire.SetBrakes (value);
	m_rearRighTire.SetBrakes (value);
}



void NewtonRaceCar::TireUpdate (const NewtonJoint* vehicle)
{
	_ASSERTE(0);
	//void* tyreId;
	//RaceCarTire* tire;
	//
	//// iterate trough each tire applying the tire dynamics
	//for (tyreId = NewtonVehicleGetFirstTireID (vehicle); tyreId; tyreId = NewtonVehicleGetNextTireID (vehicle, tyreId)) {
	//	// get the graphic object and set the transformation matrix 
	//	tire = (RaceCarTire*) NewtonVehicleGetTireUserData (vehicle, tyreId);
	//	tire->SetTirePhysics (vehicle, tyreId);
	//}


	// uncomment this code to simulate losing a tire 
/*
	static xxx;
	xxx	++;
	if ((xxx == 200) || (xxx == 400)){
//		tyreId = NewtonVehicleGetFirstTireID (vehicle);
//		NewtonVehicleRemoveTire (vehicle, tyreId);
	}
	if (xxx == 201){
		NewtonDestroyJoint (xxxx, vehicle);
	}
*/
}


void NewtonRaceCar::RaceCarTire::Setup (NewtonRaceCar *root, const char* label)
{
	int i;
	dFloat tireMass; 
	dFloat tireSuspesionShock;
	dFloat tireSuspesionSpring; 


	// add vehicle tires
	tireMass = 5.0f; 
//	tireSuspesionShock = 1.0f;
//	tireSuspesionSpring = 1.0f; 

 	char name[256];
	sprintf (name, "%s_tire", label);

	// find the geometry mode;
	m_tireNode = root->Find (name);
	_ASSERTE (m_tireNode);

	// find the radius and with of the tire
	m_width = 0.0f;
	m_radius = 0.0f;

	// use the main body part to create the the main collision geometry
	dGeometrySegment& segment = m_tireNode->GetGeometry()->GetFirst()->GetInfo();
	GLfloat *vertex = segment.m_vertex;
	for (i = 0; i < segment.m_vertexCount; i ++) {
		m_width = (vertex[i * 3 + 0] > m_width) ? vertex[i * 3 + 0] : m_width;   
		m_radius = (vertex[i * 3 + 1] > m_radius) ? vertex[i * 3 + 1] : m_radius;   
	}
//m_radius *= 1.5f; 
	
	// set the tire visual offset matrix
	m_tireOffsetMatrix = m_tireNode->GetParent()->CalcGlobalMatrix (root).Inverse();

	// set the tire in local space
	dMatrix tirePosition (m_tireNode->CalcGlobalMatrix (root));
	tirePosition.m_posit.m_y -= SUSPENSION_LENGTH * 0.25f;
//	tirePosition.m_posit.m_y -= SUSPENSION_LENGTH * 1.25f;
 
	// the tire will spin around the lateral axis of the same tire space
	dVector tirePin (0.0, 0.0, 1.0f);

	// calculate the spring and damper contact for the subquestion  
	//
	// from the equation of a simple spring the force is given by
	// a = k * x
	// where k is a spring constant and x is the displacement and a is the spring acceleration.
	// we desire that a rest length the acceleration generated by the spring equal that of the gravity
	// several gravity forces
	// m * gravity = k * SUSPENSION_LENGTH * 0.5f, 
	dFloat x = SUSPENSION_LENGTH;
	tireSuspesionSpring = (200.0f * dAbs (GRAVITY)) / x;
//	tireSuspesionSpring = (100.0f * dAbs (GRAVITY)) / x;

	// from the equation of a simple spring / damper system
	// the system resonate at a frequency 
	// w^2 = ks
	//
	// and the damping attenuation is given by
	// d = ks / 2.0f
	// where:
	// if   d = w = 2 * pi * f -> the system is critically damped
	// if   d > w < 2 * pi * f -> the system is super critically damped
	// if   d < w < 2 * pi * f -> the system is under damped damped 
	// for a vehicle we usually want a suspension that is about 10% super critically damped  
	float w = dSqrt (tireSuspesionSpring);

//  a critically damped suspension act too jumpy for a race car, 	
	tireSuspesionShock = 1.0f * w;

//	make it a little super critical y damped
//	tireSuspesionShock = 2.0f * w;

	_ASSERTE(0);
#if 0
	// add the tire and set this as the user data
	NewtonVehicleAddTire (root->GetJoint(), &tirePosition[0][0], &tirePin[0], tireMass, m_width * 1.25f, m_radius, 
						  tireSuspesionShock, tireSuspesionSpring, SUSPENSION_LENGTH, this, TIRE_COLLITION_ID);


	// save the original tire set h
	m_tireRefHeight = m_tireNode->GetMatrix().m_posit.m_y;

	// calculate brake visual parametric position
	sprintf (name, "%s_brake", label);

	// find the geometry mode;
	m_brakeNode = root->Find (name);
	_ASSERTE (m_brakeNode);

	m_brakeRefPosition = m_brakeNode->GetMatrix().m_posit;

	// save the suspension parametric information
	sprintf (name, "%s_topSusp", label);
 	m_topSuspNode = root->Find (name);
	_ASSERTE (m_topSuspNode);
 	m_suspentionTopMatrix = m_topSuspNode->GetMatrix();

	sprintf (name, "%s_bottomSusp", label);
 	m_bottomSuspNode = root->Find (name);
	_ASSERTE (m_bottomSuspNode);
 	m_suspentionBottomMatrix = m_bottomSuspNode->GetMatrix();
 
	sprintf (name, "%s_axel", label);
 	m_axelNode = root->Find (name);
	_ASSERTE (m_axelNode);
 	m_axelMatrix = m_axelNode->GetMatrix();
#endif
}



void NewtonRaceCar::RaceCarTire::SetTirePhysics(const NewtonJoint* vehicle, void* tireId)
{
/*
	dFloat slipAngle; 
	dFloat tireSpeed; 
	dFloat lateralSpeed; 
	dFloat longitudinalSpeed; 

	// apply a tire model using the tire load, 
	// a simple tire forces circle and a circle of friction
	//
	// first step is calculate the tire slip angle
	if (!NewtonVehicleTireIsAirBorne (vehicle, tireId)) {
		longitudinalSpeed = NewtonVehicleGetTireLongitudinalSpeed (vehicle, tireId);
		lateralSpeed = NewtonVehicleGetTireLateralSpeed (vehicle, tireId);
		tireSpeed = lateralSpeed * lateralSpeed + longitudinalSpeed * longitudinalSpeed;
		// if the tire velocity is too slow this is a static analysis,
		// not tire model is applied
		if (tireSpeed > 1.0f) {
			slipAngle = dAtan2 (lateralSpeed, longitudinalSpeed);
			slipAngle += NewtonVehicleGetTireSteerAngle (vehicle, tireId);
		}
	}
*/
}

void NewtonRaceCar::RearTire::SetTirePhysics(const NewtonJoint* vehicle, void* tireId)
{
	_ASSERTE(0);
	//NewtonVehicleSetTireTorque (vehicle, tireId, m_torque);
	//
	//RaceCarTire::SetTirePhysics (vehicle, tireId);
	//
	//
	//if (m_brakes > 0.0f) {
	//	dFloat speed;
	//	dFloat brakeAcceleration;
	//
	//	// ask Newton for the precise acceleration needed to stop the tire
    // 	brakeAcceleration = NewtonVehicleTireCalculateMaxBrakeAcceleration (vehicle, tireId);
	//
	//	// tell Newton you want this tire stooped but only if the torque need it is less than 
	//	// the brakes pad can withstand (assume max brake pad torque is 500 newton * meter)
	//	NewtonVehicleTireSetBrakeAcceleration (vehicle, tireId, brakeAcceleration, 10000.0f);
	//
	//	// set some side slip as function of the linear speed 
	//	speed = NewtonVehicleGetTireLongitudinalSpeed (vehicle, tireId);
	//	NewtonVehicleSetTireMaxSideSleepSpeed (vehicle, tireId, speed * 0.1f);
	//} else {
	//	m_brakes = 0.0f;
	//}

}


void NewtonRaceCar::FrontTire::SetTirePhysics(const NewtonJoint* vehicle, void* tireId)
{
	_ASSERTE(0);
	//dFloat currSteerAngle;
	//
	//currSteerAngle = NewtonVehicleGetTireSteerAngle (vehicle, tireId);
	//NewtonVehicleSetTireSteerAngle (vehicle, tireId, currSteerAngle +  (m_steer - currSteerAngle) * 0.035f);
	//
	//RaceCarTire::SetTirePhysics (vehicle, tireId);
}

	
