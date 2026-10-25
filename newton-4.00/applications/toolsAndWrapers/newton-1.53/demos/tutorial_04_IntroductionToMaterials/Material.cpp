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
#include "Material.h"


int woodID; 
int metalID; 
int levelID; 
int characterID; 

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

	dVector m_charaterCentre;
};

// structures use to hold different materials interaction 
SpecialEffectStruct wood_wood;
SpecialEffectStruct wood_metal;
SpecialEffectStruct wood_level;
SpecialEffectStruct metal_metal;
SpecialEffectStruct metal_level;


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


// this callback is called when the two aabb boxes of the collision object overlap
int  CharacterContactBegin (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* sphere;
	GenericContactBegin (material, body0, body1);

	// kame sure body zero is the sphere 
	NewtonBodyGetMassMatrix (g_currectEffect->m_body0, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f) {
		sphere = g_currectEffect->m_body1;
		g_currectEffect->m_body1 = g_currectEffect->m_body0;
		g_currectEffect->m_body0 = sphere;
	}

	// catch the origin of the sphere
	dMatrix matrix;
	NewtonBodyGetMatrix(g_currectEffect->m_body0, &matrix[0][0]);	
	g_currectEffect->m_charaterCentre = matrix.m_posit;


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
//	if (m_showDegug) {
//		if (m_contactCount < NEWTON_MAX_CONTACTS) {
//			dVector point;
//			dVector normal;	
//			dVector normalForce;
//			dVector frictionForce;
//			dVector restingForce;

			// Get the contact point and the normal
//			NewtonMaterialGetContactPositionAndNormal (material, &point.X, &normal.X);
//			m_contactNormals[m_contactCount * 2] = point * NewtonToIrr;
//			m_contactNormals[m_contactCount * 2 + 1] = m_contactNormals[m_contactCount * 2] + normal * (NewtonToIrr * 0.5f);

			// get the contact force at this point if any
//  			NewtonMaterialGetContactForce(material, &restingForce[0]);
			// decompose the resting force into the normal and friction forces
//			normalForce = normal.Scale (restingForce);
//			frictionForce = restingForce - normalForce ;
			
//			m_contactCount ++;
//		}
//	}
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





// this is use to customized the character control contact vectors
int  CharacterContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
//	dFloat mass;
//	dFloat Ixx;
//	dFloat Iyy;
//	dFloat Izz;
	NewtonBody* sphere;
	
	// apply the default behavior
	GenericContactProcess (material, contact);

	// get the spherical body, it is the body with non zero mass
	// this way to determine the body is quit and dirty but it no safe, it only work in this
	// case because one of the two bodies is the terrain which we now have infinite mass.
	// a better way is by getting the user data an finding some object identifier stored with the user data.
	sphere = g_currectEffect->m_body0;
//	NewtonBodyGetMassMatrix (g_currectEffect->m_body0, &mass, &Ixx, &Iyy, &Izz);
//	if (mass == 0.0f) {
//		sphere = g_currectEffect->m_body1;
//		NewtonBodyGetMassMatrix (g_currectEffect->m_body1, &mass, &Ixx, &Iyy, &Izz);
//	}

	// align the tangent contact direction with the velocity vector of the ball
	dVector posit;
	dVector normal;
	dVector oldNormal;
	
	dVector velocity;
	NewtonBodyGetVelocity(sphere, &velocity.m_x);
	
	// Get the contact normal 
	NewtonMaterialGetContactPositionAndNormal (material, &posit.m_x, &oldNormal.m_x);

	// ingnore the engine normal and calculate the desired ball normal
	normal = g_currectEffect->m_charaterCentre - posit;
	dFloat invMag = 1.0f / dSqrt (normal % normal + 1.0e-8f);
	normal = normal.Scale (invMag);

	// make sure the normal direction pass by the origin of the ball;
	NewtonMaterialSetContactNormalDirection (material, &normal[0]);

	// calculate ball velocity perpendicular to the contact normal
	dVector tangentVelocity (velocity - normal.Scale (velocity % normal));

	// align the tangent at the contact point with the tangent velocity vector of the ball
	NewtonMaterialContactRotateTangentDirections (material, &tangentVelocity.m_x);


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

	// level on level is will never happens


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
	NewtonMaterialSetCollisionCallback (nWorld, levelID, characterID, &metal_level, CharacterContactBegin, CharacterContactProcess, GenericContactEnd); 
}


// destroy all material resources
void CleanUpMaterials (NewtonWorld* nWorld)
{
	wood_wood.Detroy();
	wood_metal.Detroy();
	wood_level.Detroy();
	metal_metal.Detroy();
	metal_level.Detroy();
}
