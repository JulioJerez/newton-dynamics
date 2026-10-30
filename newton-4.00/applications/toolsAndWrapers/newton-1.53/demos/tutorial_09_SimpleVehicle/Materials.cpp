//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// Materials.cpp: implementation of the Materials class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "Materials.h"
#include "NewtonVehicle.h"

int woodID; 
int metalID; 
int levelID; 
int vehicleID;
int characterID; 

extern bool debugLinesMode;
extern int hightMap_debugCount;
extern dVector hightMap_debugLines[1024][2];


// structure use to hold game play especial effects
struct SpecialEffectStruct
{
	void *soundHandle;
	void *particleHandle;

	// create the resources for this materials effect here
	void Init (char* effectName)
	{
	}

	// destroy all resources for this material interaction here
	void Detroy ()
	{
	}

	// play the impact sound
	void PlayImpactSound (dFloat volume)
	{
	}

	// play scratch sound
	void PlayScratchSound (dFloat volume)
	{
	}

	NewtonBody* m_body0;
	NewtonBody* m_body1;
	dVector m_position;
	dFloat m_contactMaxNormalSpeed;
	dFloat m_contactMaxTangentSpeed;
};

// structures use to hold different materials interactions 
SpecialEffectStruct wood_wood;
SpecialEffectStruct wood_metal;
SpecialEffectStruct wood_level;
SpecialEffectStruct metal_metal;
SpecialEffectStruct metal_level;

SpecialEffectStruct vehicle_level;
//SpecialEffectStruct vehicle_wood;
//SpecialEffectStruct vehicle_metal;


static SpecialEffectStruct* g_currectEffect;

// this callback is called when the two aabb boxes of the collision object overlap
int  GenericContactBegin (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1)
{

	// get the pointer to the special effect structure
	g_currectEffect = (SpecialEffectStruct *)NewtonMaterialGetMaterialPairUserData (material);

	// save the collision bodies
	g_currectEffect->m_body0 = (NewtonBody*) body0;
	g_currectEffect->m_body1 = (NewtonBody*) body1;

	// clear the contact normal speed 
	g_currectEffect->m_contactMaxNormalSpeed = 0.0f;

	// clear the contact sliding speed 
	g_currectEffect->m_contactMaxTangentSpeed = 0.0f;

	// return one the tell Newton the application wants to process this contact
	return 1;
}


// this callback is called for every contact between the two bodies
int  GenericContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
	dFloat speed0;
	dFloat speed1;
	dVector normal;

	// Get the maximum normal speed of this impact. this can be used for particles of playing collision sound
	speed0 = NewtonMaterialGetContactNormalSpeed (material, contact);
	if (speed0 > g_currectEffect->m_contactMaxNormalSpeed) {
		// save the position of the contact (for 3d sound of particles effects)
		g_currectEffect->m_contactMaxNormalSpeed = speed0;
		NewtonMaterialGetContactPositionAndNormal (material, &g_currectEffect->m_position.m_x, &normal.m_x);
	}

	// get the maximum of the two sliding contact speed
	speed0 = NewtonMaterialGetContactTangentSpeed (material, contact, 0);
	speed1 = NewtonMaterialGetContactTangentSpeed (material, contact, 1);
	if (speed1 > speed0) {
		speed0 = speed1;
	}

	// Get the maximum tangent speed of this contact. this can be used for particles(sparks) of playing scratch sounds 
	if (speed0 > g_currectEffect->m_contactMaxTangentSpeed) {
		// save the position of the contact (for 3d sound of particles effects)
		g_currectEffect->m_contactMaxTangentSpeed = speed0;
		NewtonMaterialGetContactPositionAndNormal (material, &g_currectEffect->m_position.m_x, &normal.m_x);
	}

	
	#ifdef DEBUG_NEWTON
	// the application can implement some kind of contact debug here
	if (debugLinesMode) {
		if ((unsigned)hightMap_debugCount < (sizeof (hightMap_debugLines) / sizeof (hightMap_debugLines[0]))) {
			dVector point;
			dVector normal;	
			NewtonMaterialGetContactPositionAndNormal (material, &point.m_x, &normal.m_x);

//_ASSERTE (normal.m_y > 0.0f);
			
			hightMap_debugLines[hightMap_debugCount][0] = point;
			hightMap_debugLines[hightMap_debugCount][1] = point + normal;
			hightMap_debugCount ++;
		}
	}
	#endif

	// return one to tell Newton we want to accept this contact
	return 1;
}

// this function is call after all contacts for this pairs is processed
void  GenericContactEnd (const NewtonMaterial* material)
{
	#define MIN_CONTACT_SPEED 15
	#define MIN_SCRATCH_SPEED 5

	// if the max contact speed is larger than some minimum value. play a sound
	if (g_currectEffect->m_contactMaxNormalSpeed > MIN_CONTACT_SPEED) {
		g_currectEffect->PlayImpactSound (g_currectEffect->m_contactMaxNormalSpeed - MIN_CONTACT_SPEED);
	}

	// if the max contact speed is larger than some minimum value. play a sound
	if (g_currectEffect->m_contactMaxNormalSpeed > MIN_SCRATCH_SPEED) {
		g_currectEffect->PlayScratchSound (g_currectEffect->m_contactMaxNormalSpeed - MIN_SCRATCH_SPEED);
	}

	// implement here any other effects
}




// this is use to customized the characater control contact vectors
int  CharacterContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* sphere;
	
	// apply the default behavior
	GenericContactProcess (material, contact);

	// get the spherical body, it is the body with non zero mass
	// this way to determine the body is quit and dirty but it no safe, it only work in this
	// case because one of the two bodies is the terrain which we now have infinite mass.
	// a better way is by getting the user data an finding some object identifier stored with the user data.
	sphere = g_currectEffect->m_body0;
	NewtonBodyGetMassMatrix (g_currectEffect->m_body0, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f) {
		sphere = g_currectEffect->m_body1;
		NewtonBodyGetMassMatrix (g_currectEffect->m_body1, &mass, &Ixx, &Iyy, &Izz);
	}

	// align the tangent contact direction with the velocity vector of the ball
	dVector posit;
	dVector normal;
	
	dVector velocity;
	NewtonBodyGetVelocity(sphere, &velocity.m_x);
	NewtonMaterialGetContactPositionAndNormal (material, &posit.m_x, &normal.m_x);

	// calculate ball velocity perpendicular to the contact normal
	dVector tangentVelocity (velocity - normal.Scale (normal % velocity));

	// align the tangent at the contact point with the tangent velocity vector of the ball
	NewtonMaterialContactRotateTangentDirections (material, &tangentVelocity.m_x);
	


	// return one to tell Newton we want to accept this contact
	return 1;
}





// this is use to customized the character control contact vectors
int  VehicleContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
	int collisionID;
	int faceMaterialID;
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* car;
	
	// apply the default behavior
//	GenericContactProcess (material, contact);

	// get the car, it is the body with non zero mass
	// this way to determine the body is quit and dirty but it no safe, it only works in this
	// case because one of the two bodies is the terrain which we now have infinite mass.
	// a better way is by getting the user data an finding some object identifier stored with the user data.

	car = g_currectEffect->m_body0;
	NewtonBodyGetMassMatrix (g_currectEffect->m_body0, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f) {
		car = g_currectEffect->m_body1;
	}


	collisionID = NewtonMaterialGetBodyCollisionID (material, car);

	switch (collisionID) 
	{
		// the car body is colliding
		case CHASIS_COLLITION_ID:
		{
			// get the contact user data assigned to the terrain face generating the contact
			faceMaterialID = NewtonMaterialGetContactFaceAttribute (material);

			// used the face ID to identifies the terrain material and apply per face material properties
			switch(faceMaterialID) {
				case 0:
					//set the friction values depending on the face material
					NewtonMaterialSetContactStaticFrictionCoef (material, 0.7f, 0);
					NewtonMaterialSetContactStaticFrictionCoef (material, 0.7f, 1);
					NewtonMaterialSetContactKineticFrictionCoef (material, 0.3f, 0);
					NewtonMaterialSetContactKineticFrictionCoef (material, 0.3f, 1);
					// maybe save the contact position to generate skid marks, smokes, any special effect
					// ....

					break;
								
				default:
					//set the friction values dep[ending on the face material
					NewtonMaterialSetContactStaticFrictionCoef (material, 0.7f, 0);
					NewtonMaterialSetContactStaticFrictionCoef (material, 0.7f, 1);
					NewtonMaterialSetContactKineticFrictionCoef (material, 0.3f, 0);
					NewtonMaterialSetContactKineticFrictionCoef (material, 0.3f, 1);
					// maybe save the contact position to generate skid marks, smokes, any special effect
					// ....

			}


			// maybe save the speed to generate impact sound
			break;
		}
			

		// a tire body is colliding
		case TIRE_COLLITION_ID:
		{
			// get the contact user data assigned to the terrain face generating the contact
			// get the contact user data assigned to the terrain face generating the contact
			faceMaterialID = NewtonMaterialGetContactFaceAttribute (material);

			// used the face ID to identifies the terrain material and apply per face material properties
			switch(faceMaterialID) 
			{
				case 0:
					// used the max allowable friction coefficient is 2.0f in Newton
					// set the friction values depending on the face material
					// first axis is the longitudinal tire friction (in general lateral and longitudinal friction should be equal)
					// but the application can play with this values to generate specials effects
					// first axis is the longitudinal tire friction
					NewtonMaterialSetContactStaticFrictionCoef (material, 1.5f, 0);
					NewtonMaterialSetContactKineticFrictionCoef (material, 1.0f, 0);

					// second axis is the lateral tire friction (in general lateral and longitudinal friction should be equal)
					// but the application can play with this values to generate specials effects
					NewtonMaterialSetContactStaticFrictionCoef (material, 1.2f, 1);
					NewtonMaterialSetContactKineticFrictionCoef (material, 1.0f, 1);

					// maybe save the speed to generate impact sound
					// .....

					break;

				default:
					//set the friction values depending on the face material
					// first axis is the longitudinal tire friction (in general lateral and longitudinal friction should be equal)
					// but the application can play with this values to generate specials effects
					// first axis is the longitudinal tire friction
					NewtonMaterialSetContactStaticFrictionCoef (material, 1.0f, 0);
					NewtonMaterialSetContactKineticFrictionCoef (material, 0.8f, 0);

					// second axis is the lateral tire friction (in general lateral and longitudinal friction should be equal)
					// but the application can play with this values to generate specials effects
					NewtonMaterialSetContactStaticFrictionCoef (material, 1.0f, 1);
					NewtonMaterialSetContactKineticFrictionCoef (material, 0.8f, 1);

					// maybe save the contact position to generate skid marks, smokes, any special effect
					// ....
			}
			
			break;
		}
	}


	#ifdef DEBUG_NEWTON
	// the application can implement some kind of contact debug here
	if (debugLinesMode) {
		if ((unsigned)hightMap_debugCount < (sizeof (hightMap_debugLines) / sizeof (hightMap_debugLines[0]))) {
			dVector point;
			dVector normal;	
			NewtonMaterialGetContactPositionAndNormal (material, &point.m_x, &normal.m_x);
			hightMap_debugLines[hightMap_debugCount][0] = point;
			hightMap_debugLines[hightMap_debugCount][1] = point + normal.Scale (2.0f);
			hightMap_debugCount ++;
		}
	}
	#endif


	// return one to tell Newton we want to accept this contact
	return 1;
}






// initialize all material and material interactions
void SetupMaterials (NewtonWorld* nWorld)
{
	int defaultID;

	// initialize the material interaction here
	wood_wood.Init ("wood_wood");
	wood_metal.Init ("wood_metal");
	wood_level.Init ("wood_level");
	metal_metal.Init ("metal_metal");
	metal_level.Init ("metal_level");

	vehicle_level.Init ("vehicle_level");
//	vehicle_wood.Init ("vehicle_wood");
//	vehicle_metal.Init ("vehicle_metal");
	
	
	// get the default material ID
	defaultID = NewtonMaterialGetDefaultGroupID (nWorld);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (nWorld, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (nWorld, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (nWorld, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (nWorld, defaultID, defaultID, 1.0f, 0.5f);
//	NewtonMaterialSetCollisionCallback (nWorld, woodID, woodID, &wood_wood, GenericContactBegin, GenericContactProcess, GenericContactEnd); 
	NewtonMaterialSetCollisionCallback (nWorld, defaultID, defaultID, &wood_wood, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	
	// create all materials ID
	woodID = NewtonMaterialCreateGroupID(nWorld);
	metalID = NewtonMaterialCreateGroupID(nWorld);
	levelID = NewtonMaterialCreateGroupID(nWorld);
	vehicleID = NewtonMaterialCreateGroupID(nWorld);
	characterID = NewtonMaterialCreateGroupID(nWorld);

	// set the material properties for wood on wood
	NewtonMaterialSetDefaultElasticity (nWorld, woodID, woodID, 0.3f);
	NewtonMaterialSetDefaultFriction (nWorld, woodID, woodID, 1.1f, 0.7f);
	NewtonMaterialSetCollisionCallback (nWorld, woodID, woodID, &wood_wood, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// set the material properties for wood on metal
	NewtonMaterialSetDefaultElasticity (nWorld, woodID, metalID, 0.5f);
	NewtonMaterialSetDefaultFriction (nWorld, woodID, metalID, 0.8f, 0.6f);
	NewtonMaterialSetCollisionCallback (nWorld, woodID, metalID, &wood_metal, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// set the material properties for wood on level
	NewtonMaterialSetDefaultElasticity (nWorld, woodID, levelID, 0.3f);
	NewtonMaterialSetDefaultFriction (nWorld, woodID, levelID, 0.9f, 0.3f);
	NewtonMaterialSetCollisionCallback (nWorld, woodID, levelID, &wood_level, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// set the material properties for metal on metal
	NewtonMaterialSetDefaultElasticity (nWorld, metalID, metalID, 0.7f);
	NewtonMaterialSetDefaultFriction (nWorld, metalID, metalID, 0.5f, 0.2f);
	NewtonMaterialSetCollisionCallback (nWorld, metalID, metalID, &metal_metal, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// set the material properties for metal on level
	NewtonMaterialSetDefaultElasticity (nWorld, metalID, levelID, 0.4f);
	NewtonMaterialSetDefaultFriction (nWorld, metalID, levelID, 0.6f, 0.4f);
	NewtonMaterialSetCollisionCallback (nWorld, metalID, levelID, &metal_level, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// level on level will never happens


	// create the character material interactions
	NewtonMaterialSetDefaultElasticity (nWorld, woodID, characterID, 0.3f);
	NewtonMaterialSetDefaultFriction (nWorld, woodID, characterID, 1.1f, 0.7f);
	NewtonMaterialSetCollisionCallback (nWorld, woodID, characterID, &wood_wood, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// set the material properties for metal on metal
	NewtonMaterialSetDefaultElasticity (nWorld, metalID, characterID, 0.7f);
	NewtonMaterialSetDefaultFriction (nWorld, metalID, characterID, 0.5f, 0.2f);
	NewtonMaterialSetCollisionCallback (nWorld, metalID, characterID, &metal_metal, GenericContactBegin, GenericContactProcess, GenericContactEnd); 

	// set the material properties for character on level 
	NewtonMaterialSetDefaultElasticity (nWorld, levelID, characterID, 0.1f);
	NewtonMaterialSetDefaultFriction (nWorld, levelID, characterID, 0.6f, 0.6f);
	NewtonMaterialSetCollisionCallback (nWorld, levelID, characterID, &metal_level, GenericContactBegin, CharacterContactProcess, GenericContactEnd); 


	// set the material properties for vehicle on level

	NewtonMaterialSetDefaultElasticity (nWorld, levelID, vehicleID, 0.1f);
//	NewtonMaterialSetDefaultFriction (nWorld, levelID, vehicleID, 1.0f, 1.0f);
	NewtonMaterialSetCollisionCallback (nWorld, levelID, vehicleID, &vehicle_level, GenericContactBegin, VehicleContactProcess, GenericContactEnd); 
}


// destroy all material resources
void CleanUpMaterials (NewtonWorld* nWorld)
{
	wood_wood.Detroy();
	wood_metal.Detroy();
	wood_level.Detroy();
	metal_metal.Detroy();
	metal_level.Detroy();

	vehicle_level.Detroy();
//	vehicle_wood.Init ("vehicle_wood");
//	vehicle_metal.Init ("vehicle_metal");

}
