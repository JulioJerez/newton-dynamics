//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// JointsTutorial.h: interface for the JointsTutorial class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CHARACTER_CONTROLL_H__7A1C323F_E267_4C98_BD55_3355B21F049E__INCLUDED_)
#define AFX_CHARACTER_CONTROLL_H__7A1C323F_E267_4C98_BD55_3355B21F049E__INCLUDED_

#include "Newton.h"
#include "dMatrix.h"
#include "RenderPrimitive.h"

//#define BOX_COLLISION    0x0f
//#define SPHERE_COLLISION 0x0e
//extern RenderPrimitive* player;
//void AddCharacter (NewtonWorld* nWorld);

class CharacterController: public SpherePrimitive
{
	public: 
	CharacterController (dSceneNode* parent, NewtonWorld* nWorld, const dVector& position, const dVector& size);
	void SetForce (const dVector& force);

	void OnApplyForceAndTorque ();
	void OnCollisionWithLevel(const NewtonMaterial* material, const NewtonContact* contact);


	private:
	// utility functions
	void Render() const;
	dFloat FindFloor (NewtonWorld* world, const dVector& centre, dFloat maxDist);

	// Newton interface callbacks
	static void  OnApplyForceAndTorque (const NewtonBody* body);
	static dFloat OnRayCastPlacement (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam);
	
	NewtonBody* m_myBody;
	NewtonWorld* m_myWorld;
	NewtonCollision* m_myCollision;
	NewtonJoint* m_upVector;
	dVector m_size;
	dVector m_stepContact;
	dVector m_forceVector;
	
	bool m_isStopped;
	bool m_isAirBorne;
	unsigned m_jumpTimer;
	dFloat m_maxStepHigh;
	dFloat m_yawAngle;
	dFloat m_maxTranslation;
};

#endif 
