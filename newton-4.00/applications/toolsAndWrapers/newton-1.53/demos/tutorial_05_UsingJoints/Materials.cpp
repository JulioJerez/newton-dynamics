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

	// destroy all resouces for this material interaction here
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

// strutures use to hold diffrents materials interation 
SpecialEffectStruct wood_wood;
SpecialEffectStruct wood_metal;
SpecialEffectStruct wood_level;
SpecialEffectStruct metal_metal;
SpecialEffectStruct metal_level;


static SpecialEffectStruct* g_currectEffect;

// this callback is called when the two aabb boxes of the collisiong object overlap
int  GenericContactBegin (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1)
{

	// get the pointer to the special effect struture
	g_currectEffect = (SpecialEffectStruct *)NewtonMaterialGetMaterialPairUserData (material);

	// save the collisiong bodies
	g_currectEffect->m_body0 = (NewtonBody*) body0;
	g_currectEffect->m_body1 = (NewtonBody*) body1;

	// clear the contact normal speed 
	g_currectEffect->m_contactMaxNormalSpeed = 0.0f;

	// clear the contact sliding speed 
	g_currectEffect->m_contactMaxTangentSpeed = 0.0f;

	// return one the tell Newton the application wants to proccess this contact
	return 1;
}


// this callback is called for every contact between the two bodies
int  GenericContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
	dFloat speed0;
	dFloat speed1;
	dVector normal;

	// Get the maximun normal speed of this impact. this can be used for particels of playing collision sound
	speed0 = NewtonMaterialGetContactNormalSpeed (material, contact);
	if (speed0 > g_currectEffect->m_contactMaxNormalSpeed) {
		// save the position of the contact (for 3d sound of particles effects)
		g_currectEffect->m_contactMaxNormalSpeed = speed0;
		NewtonMaterialGetContactPositionAndNormal (material, &g_currectEffect->m_position.m_x, &normal.m_x);
	}

	// get the maximun of the two sliding contact speed
	speed0 = NewtonMaterialGetContactTangentSpeed (material, contact, 0);
	speed1 = NewtonMaterialGetContactTangentSpeed (material, contact, 1);
	if (speed1 > speed0) {
		speed0 = speed1;
	}

	// Get the maximun tangent speed of this contact. this can be used for particles(sparks) of playing scratch sounds 
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
//			NewtonMaterialGetContactPositionAndNormal (material, &point.X, &normal.X);
//			m_contactNormals[m_contactCount * 2] = point * NewtonToIrr;
//			m_contactNormals[m_contactCount * 2 + 1] = m_contactNormals[m_contactCount * 2] + normal * (NewtonToIrr * 0.5f);
//			m_contactCount ++;
//		}
//	}
	#endif

	// return one to tell Newton we want to accept this contact
	return 1;
}

// this function is call affter all contacts for this pairs is proccesed
void  GenericContactEnd (const NewtonMaterial* material)
{
	#define MIN_CONTACT_SPEED 15
	#define MIN_SCRATCH_SPEED 5

	// if the max contact speed is larger than some minumum value. play a sound
	if (g_currectEffect->m_contactMaxNormalSpeed > MIN_CONTACT_SPEED) {
		g_currectEffect->PlayImpactSound (g_currectEffect->m_contactMaxNormalSpeed - MIN_CONTACT_SPEED);
	}

	// if the max contact speed is larger than some minumum value. play a sound
	if (g_currectEffect->m_contactMaxNormalSpeed > MIN_SCRATCH_SPEED) {
		g_currectEffect->PlayScratchSound (g_currectEffect->m_contactMaxNormalSpeed - MIN_SCRATCH_SPEED);
	}

	// implement here any other effects
}





// this is use to constomized the chatarer contril contact vectors
int  CharacterContactProcess (const NewtonMaterial* material, const NewtonContact* contact)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* sphere;
	
	// apply the default behaviuor
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









// initialize all material and material interations
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

	// level on level is will never hhappens


	// create the charater material interactions
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
