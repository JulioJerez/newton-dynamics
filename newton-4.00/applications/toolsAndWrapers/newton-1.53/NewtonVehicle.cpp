/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonBody.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

class NewtonConvexCastVehicle : public ndConvexCastVehicle
{
	public:
	D_CLASS_REFLECTION(NewtonConvexCastVehicle, ndConvexCastVehicle)

	NewtonConvexCastVehicle(const ndVector& gravityDir)
		:ndConvexCastVehicle()
		,m_gravityDir(gravityDir)
	{
	}

	ndVector m_gravityDir;
};

#if 0

// Name: NewtonVehicleSetTireCallback
// Store a function callback pointer for vehicle update.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *NewtonVehicleTireUpdate* update - pointer to the callback function.
//
// Return: nothing
//
// Remarks: The only way to control a vehicle is by implementing the vehicle update callback. 
// The application should iterate through each tire applying tire dynamics to each one. 
//
// See also: NewtonVehicleGetFirstTireID, NewtonVehicleGetNextTireID
void NewtonVehicleSetTireCallback(const NewtonVehicle* const vehicle, NewtonVehicleTireUpdate update)
{
	dgVehicleConstraint* joint;

	joint = (dgVehicleConstraint*)vehicle;
	joint->SetTireCallback((OnVehicleUpdate)update);
}


// Name: NewtonVehicleAddTire
// Add a new tire to the vehicle container.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *const dFloat* *localMatrix - pointer to an array of 16 floats containing the offset of the tire relative to the vehicle body.
// *const dFloat* pin - pointer to an array of 3 floats containing the rotation axis of the tire, in the space of the tire.
// *dFloat* mass - tire mass, must be much smaller than the vehicle body. ratio of 50:1 to 100:1 are the recommended values. 
// *dFloat* width - width of the tire, must be smaller than the tire radius.
// *dFloat* radius - tire radius.
// *dFloat* suspesionShock - parametrized damping constant for a spring, mass, damper system. A value of one corresponds to a critically damped system.
// *dFloat* suspesionSpring - parametrized spring constant for a spring, mass, damper system. A value of one corresponds to a critically damped system.
// *dFloat* suspesionLength - distance from the tire set position to the upper stop on the vehicle body frame. The total suspension length is twice that. 
// *void* *userData - pointer to a user define data value. Usually used to store the pointer to the graphical representation of the tire.
// *int* collisionID - the collision ID use by the application to identify the tire contacts in a contact callback function.
//
// Return: the tire ID.
//
// Remarks: After the application creates the vehicle joint, it must add the tires. 
// Tires are added one at a time at the graphics set position and with the appropriate.
// the application should calculate the correct tire parameters, like tire mass, position, width height,
// spring and damper constants.
//
// See also: NewtonVehicleRemoveTire
void* NewtonVehicleAddTire(const NewtonVehicle* const vehicle, const dFloat* localMatrix, const dFloat* pin, dFloat mass, dFloat width, dFloat radius,
	dFloat suspesionShock, dFloat suspesionSpring, dFloat suspesionLength, void* userData, int collisionID)
{
	dgVehicleConstraint* joint;

	joint = (dgVehicleConstraint*)vehicle;

	dgMatrix& matrix(*((dgMatrix*)localMatrix));
	dgVector tirePin(pin[0], pin[1], pin[2]);
	//return joint->AddTire (matrix, pin, mass, width, radius, suspesionShock, suspesionSpring, suspesionLength, (void*)(dgUnsigned64(collisionID)), userData);
	return joint->AddTire(matrix, pin, mass, width, radius, suspesionShock, suspesionSpring, suspesionLength, IntToPointer(collisionID), userData);
}

// Name: NewtonVehicleReset
// Reset all tires velocities to zero. 
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
//
// Return: nothing.
//
// This function is useful for reposition the vehicle.
void NewtonVehicleReset(const NewtonVehicle* const vehicle)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->Reset();
}


// Name: NewtonVehicleRemoveTire
// Detach and destroy a tire from the vehicle.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - tire index to be destroyed.
//
// Return: nothing.
void NewtonVehicleRemoveTire(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;

	joint = (dgVehicleConstraint*)vehicle;
	joint->RemoveTire(tireId);
}


// Name: NewtonVehicleGetTireSteerAngle
// Get the tire steering angle.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: steering angle.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireSteerAngle(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetSteerAngle(tireId);
}


// Name: NewtonVehicleSetTireSteerAngle
// Set the tire steering angle.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* angle - new steering angle.
//
// Return: nothing.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
void NewtonVehicleSetTireSteerAngle(const NewtonVehicle* const vehicle, void* tireId, dFloat angle)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetSteerAngle(tireId, angle);
}


// Name: NewtonVehicleSetTireTorque
// Set the tire torque.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* torque - new torque value.
//
// Return: nothing.
//
// Remarks: This function is useful to simulate normal vehicles with wheels that propel by applying torque to a the tire axis
// in order to move.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
void NewtonVehicleSetTireTorque(const NewtonVehicle* const vehicle, void* tireId, dFloat torque)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireTorque(tireId, torque);
}

// Name: NewtonVehicleGetTireOmega
// Retrieve the tire angular velocity.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: angular velocity.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireOmega(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireOmega(tireId);
}

// Name: NewtonVehicleGetTireNormalLoad
// Return the part of the vehicle weight supported by this tire.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: magnitude of the vehicle weight supported by this tire.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireNormalLoad(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireNormalLoad(tireId);
}

// Name: NewtonVehicleTireSetBrakeAcceleration
// Apply the acceleration and max friction torque to a tire axis.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* acceleration - desire tire acceleration.
// *dFloat* maxFrictionTorque - maximum friction torque the tire brake, or tire motor can withstand.
//
// Return: noting.
//
// Remarks: This is a multipurpose function. The more common use is to apply hand or soft brakes to a vehicle. 
// To apply brakes the application may use the function *NewtonVehicleTireSetBrakeAcceleration* to determine the exact acceleration
// needed to stop the tire from continue to spin in one frame. To simulated the variable brakes strength the application can use 
// a nominal maximum friction torque (just like in real life any device will withstand a max value) and modulate this value with an analog 
// control. For hand brakes the application set the control to the maximum and for soft brakes it can just modulate the variable friction.
// Another use for this function is to simulate rolling friction, For this effect the application apply the acceleration to stop
// but with a friction value set to a minimum non zero fixed value. Note that brakes and tire torque are not mutually exclusive, 
// the application can apply then simultaneously. As a matter of fact doing so is quite a satisfying test showing how the vehicles rocks 
// forth and back due to the engine torque, while the tire prevent it from moving. Another use for this function is the simulation of 
// track based vehicles. For this the application apply an arbitrary fix acceleration to 
// the tires on each side of the vehicle. A function as simple as *A = Ad minus Ks x Omega* can do the trick, where Ad is the desire acceleration 
// controlled by the application joystick, Ks is some viscous velocity damping, and omega is the current tire angular velocity reported by the 
// function *NewtonVehicleGetTireOmega*. 
// To make the vehicle take turns the application can elaborate the equation like *A = Ad + At minus Ks x Omega* where At is the differential 
// acceleration supplied by the steering control, for the tires on the right side At is positive while for tires of the left side At is negative.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application wants to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleTireCalculateMaxBrakeAcceleration, NewtonVehicleGetTireOmega
void NewtonVehicleTireSetBrakeAcceleration(const NewtonVehicle* const vehicle, void* tireId, dFloat acceleration, dFloat maxFrictionTorque)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->TireSetBrakeAcceleration(tireId, acceleration, maxFrictionTorque);
}

// Name: NewtonVehicleTireCalculateMaxBrakeAcceleration
// Calculate the exact acceleration needed to be applied to a tire axis in order to bring it to full stop in one time step.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: exact acceleration for full stop of the tire.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleTireSetBrakeAcceleration
dFloat NewtonVehicleTireCalculateMaxBrakeAcceleration(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->TireCalculateMaxBrakeAcceleration(tireId);
}


// Name: NewtonVehicleGetTireLateralSpeed
// Return the tire speed along the tire pin axis.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: tire lateral speed.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireLateralSpeed(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireLateralSpeed(tireId);
}

// Name: NewtonVehicleGetTireLongitudinalSpeed
// Return the tire speed along tire center line.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: tire longitudinal speed.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
dFloat NewtonVehicleGetTireLongitudinalSpeed(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->GetTireLongitudinalSpeed(tireId);
}


// Name: NewtonVehicleSetTireMaxSideSleepSpeed
// Set the maximum side slip velocity for the tire to be considered to lose grip.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* speed - maximum side speed before the vehicle is considered to loose side grip.
//
// Return: nothing.
//
// Remarks: Tire operation involve a mix of elastic distortion and sliding friction. To have and idea how to code a convincing 
// approximation of a real tire we must run some experiment and make some reflection upon the result. First we will run a static 
// test: keeping a tire at equilibrium applying a constant load and without rolling, we will apply a lateral force perpendicular 
// to the tire plane and applied at the tire center. If we run this experiment we will find that the tire will deflect in response 
// to the lateral force. If we increase the magnitude of the lateral force, the magnitude of the deflection is proportional to the 
// magnitude of the lateral force, until the tire begins to slide. This show that when a tire is not moving it behaves like a spring 
// (elastic distortion). If we repeat this experiment but this time increasing the tire load, but still not moving the tire, we will 
// see that the max deflection is proportional to the magnitude of the tire load. This indicates the tire behavior is proportional 
// to two variables, the lateral force and the tire load. (Fortunately the side force in practice is a linear function of the tire 
// load so this keeps the model simple) Now we will run the first experiment but this time we will rotate the tire with a constant 
// angular velocity (think of those tune up machines at check up stations.) With the tire rolling at constant angular velocity if we 
// apply a lateral force we will see that as the tire deflect, the part of the tire in contact with the floor keeps rolling and another
// part take its place, but this part also start to deflect, allowing the tire to move sideways with a velocity proportional to the 
// tire rolling angular velocity. Notice that the tire does this without sliding as the part of it in contact with the floor never 
// loses grip. Now if we increase the lateral force we will find that the lateral speed of the tire will also increase. This suggests 
// that the side speed of the tire is proportional to the lateral force and also proportional to the rolling angular velocity. This 
// is the tire elastic properties give then some kind of damping property when they are rolling. There is not known macroscopic 
// mathematical model that can explain this behavior. The best we can do is to write the values of the experiment and use then to 
// interpolate and extrapolate intermediate values. One thing we know is that the tires operates within some limits, and we can use 
// those parameters to treat then as a constraint optimization problem, which is the Newton approach. When the tire is rolling and 
// side slipping is not that the tire lost grip, nor that the tire is generating some force. It is rather that the tire have the 
// capacity to absorb some of the lateral force by sliding and convert it to side velocity, this means that for the tire to 
// loose grip a stronger force is necessary. In another word at rest a tire will lose grip under a much lower lateral force than 
// if the tire was rolling. In Newton this behavior is treaded as a constrain optimization problem by asking the application how 
// much side slip velocity is the tire allow to have before it is considered to lose grip, and how much of the lateral forces 
// generated by the rigid body dynamics will be adsorbed by the tire at a given speed. It is the application responsibility to 
// set these parameters as close to the real tire as it chooses. This approach allows for a very wide range of behaviors form arcade, 
// to toy cars to very realistic.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireSideSleepCoeficient
void NewtonVehicleSetTireMaxSideSleepSpeed(const NewtonVehicle* const vehicle, void* tireId, dFloat speed)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireMaxSideSleepSpeed(tireId, speed);
}

// Name: NewtonVehicleSetTireSideSleepCoeficient
// Set the coefficient that tell the engine how much of the lateral force can be absorbed by the tire.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* coeficient - side slip coeficient.
//
// Return: nothing.
//
// Remarks: See description of side slip on function *NewtonVehicleSetTireMaxSideSleepSpeed*
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireMaxSideSleepSpeed
void NewtonVehicleSetTireSideSleepCoeficient(const NewtonVehicle* const vehicle, void* tireId, dFloat coeficient)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireSideSleepCoeficient(tireId, coeficient);
}


// Name: NewtonVehicleSetTireMaxLongitudinalSlideSpeed
// Set the maximum side slide velocity for the tire to be considered to lose traction.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* speed - maximum side speed before the vehicle is considered to loose side traction.
//
// Return: nothing.
//
// Remarks: The explanation of longitudinal slide is similar to the side slip, however it is not so critical to achieve realistic behavior.
// See description of side slip on function *NewtonVehicleSetTireMaxSideSleepSpeed*
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireLongitudinalSlideCoeficient
void NewtonVehicleSetTireMaxLongitudinalSlideSpeed(const NewtonVehicle* const vehicle, void* tireId, dFloat speed)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireMaxLongitudinalSlideSpeed(tireId, speed);
}

// Name: NewtonVehicleSetTireLongitudinalSlideCoeficient
// Set the coefficient that tell the engine how much of the longitudinal force can be absorbed by the tire.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* coefficient - longitudinal slide coefficient.
//
// Return: nothing.
//
// Remarks: The explanation of longitudinal slide is similar to the side slip, however it is not so critical to achieve realistic behavior.
// See description of side slip on function *NewtonVehicleSetTireMaxSideSleepSpeed*
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
//
// See also: NewtonVehicleSetTireMaxLongitudinalSlideSpeed
void NewtonVehicleSetTireLongitudinalSlideCoeficient(const NewtonVehicle* const vehicle, void* tireId, dFloat coeficient)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->SetTireLongitudinalSlideCoeficient(tireId, coeficient);
}


// Name: NewtonVehicleTireIsAirBorne
// Return a boolean value that tells the application if this tire is touching the ground.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: airborne state 1 on the air, 0 on the ground.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
int NewtonVehicleTireIsAirBorne(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;

	return joint->IsTireAirBorned(tireId) ? 1 : 0;
}

// Name: NewtonVehicleTireLostSideGrip
// Return a boolean value that tell the application if this tire lost side grip..
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: Grip state.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
int NewtonVehicleTireLostSideGrip(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;
	return joint->TireLostSideGrip(tireId) ? 1 : 0;
}


// Name: NewtonVehicleTireLostTraction
// Return a boolean value that tell the application if this tire lost longitudinal traction.
// 
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: traction state.
//
// Remarks: The vehicle joint provides a rich set of interface functions to the application. Which function to use 
// is only determined by the level of fidelity the application want to achieve. In not case the use of one method is better than 
// other, and it may be that some tweaking and trial is necessary before the desired vehicle behavior is achieved.
//
// Remarks: The parameters applied to a tire are reset to default values each time the update function is called.
// So the application should set the desired value in each simulation frame.
//
// Remarks: This function can only be called from the vehicle update call back. It can be used by the application to generate the custom vehicle dynamics.
int NewtonVehicleTireLostTraction(const NewtonVehicle* const vehicle, void* tireId)
{
	dgVehicleConstraint* joint;
	joint = (dgVehicleConstraint*)vehicle;
	return joint->TireLostTraction(tireId) ? 1 : 0;
}

#endif


// ***************************************************************************************************************
//
// Name: Vehicle constraint interface
//
// ***************************************************************************************************************

// Name: NewtonConstraintCreateVehicle
// Create and empty vehicle joint.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *const dFloat* *upDir - is the unit vector defined by the tires pivot points, usually the opposite direction to the gravity vector.
// *const NewtonBody* *body - is the pointer to the attached rigid body, this body can not be NULL or it can not have an infinity (zero) mass.
//
// Return: Pointer to the vehicle joint.
//
// Remarks: This joint provides basics functionality for simulating real-time simplistic vehicle dynamics. The joint is not meant to be and 
// accurate and realistic representation of a real vehicle, that is out of the scope of a real-time physics engine. The vehicle is made out 
// of a main rigid body visible to the application and attached to it a set of tires not visible to the application directly as rigid bodies.
// The tires are connected to the body via rigid wheel joints providing the ability for the tire to spin, have suspension, and turn. The 
// internal vehicle mechanics like transmission, power transfer shaft, suspension mechanism, doors etc. It only models the vehicle body 
// mounted on a set of wheels, with suspension and the ability to roll. 
NewtonVehicle* NewtonConstraintCreateVehicle(const NewtonWorld* const newtonWorld, const dFloat* upDir, const NewtonBody* chassisBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndVector pin(upDir[0], upDir[1], upDir[2], ndFloat32 (0.0f));
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	NewtonConvexCastVehicle* const vehicle = new NewtonConvexCastVehicle(pin);

	ndBodyKinematic* const body = PointerCast<ndBodyKinematic, NewtonBody>(chassisBody);
	ndBodyListView::ndNode* const sceneNode = body->GetSceneNode();
	ndAssert(sceneNode);
	vehicle->AddChassis(sceneNode->GetInfo());

	world->AddModel(ndSharedPtr<ndModel>(vehicle));
	return PointerCast<NewtonVehicle, NewtonConvexCastVehicle>(vehicle);
}

// Name: NewtonVehicleGetFirstTireID
// Get the index of the first tire of the vehicle tire set.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
//
// Return: tire index.
//
// Remarks: This function is usually used from inside the vehicle update callback. It is used to iterate through the tire set applying the tire dynamics.
//
// See also: NewtonVehicleGetNextTireID
void* NewtonVehicleGetFirstTireID(const NewtonVehicle* const vehicle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgVehicleConstraint* joint;
	//joint = (dgVehicleConstraint*)vehicle;
	//
	//return joint->GetFirstTireIndex();
	return nullptr;
}

// Name: NewtonVehicleGetNextTireID
// Get the index of the next tire on the tire set.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: Next tire index, or zero if index *tireId* was pointing to the last tire in the set.
//
// Remarks: This function is usually used from inside a tire update callback. It is used to iterate through the tire set applying the tire dynamics.
//
// See also: NewtonVehicleGetFirstTireID
void* NewtonVehicleGetNextTireID(const NewtonVehicle* const vehicle, void* tireId)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);

	//dgVehicleConstraint* joint;
	//joint = (dgVehicleConstraint*)vehicle;
	//return joint->GetNextTireIndex(tireId);
	return nullptr;
}

// Name: NewtonVehicleGetTireUserData
// Retrieve the pointer to the tire user data.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
//
// Return: tire user data.
//
// Remarks: This function is usually used from the vehicle update callback or from transformation callback of the vehicle body, 
// It can used do set the transformation matrix of the tire graphical representation.
void* NewtonVehicleGetTireUserData(const NewtonVehicle* const vehicle, void* tireId)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);

	//dgVehicleConstraint* joint;
	//joint = (dgVehicleConstraint*)vehicle;
	//return joint->GetTireUserData(tireId);
	return nullptr;
}
	
// Name: NewtonVehicleGetTireMatrix
// Retrieve the transformation matrix of the tire in global space.
//
// Parameters:
// *const NewtonJoint* *vehicle - pointer to the vehicle joint.
// *void* *tireId - index to current tire.
// *dFloat* matrix - pointer to an array of 16 floats containing the global matrix of the tire.
//
// Return: nothing
//
// Remarks: This function is usually used from the tire update callback or from transformation callback of the vehicle body, 
// It can be used to set the transformation of the tire graphical representation.
void NewtonVehicleGetTireMatrix(const NewtonVehicle* const vehicle, void* tireId, dFloat* matrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndAssert(0);

	//dgVehicleConstraint* joint;
	//joint = (dgVehicleConstraint*)vehicle;
	//
	//dgMatrix& retMatrix = *((dgMatrix*)matrix);
	//joint->GetTireMatrix(tireId, retMatrix);
}
