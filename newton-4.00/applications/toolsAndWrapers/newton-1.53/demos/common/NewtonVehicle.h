//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// NewtonVehicle.h: interface for the NewtonVehicle class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NEWTONVEHICLE_H__DD5FB692_E86A_43F6_ABCA_6F889285D7FC__INCLUDED_)
#define AFX_NEWTONVEHICLE_H__DD5FB692_E86A_43F6_ABCA_6F889285D7FC__INCLUDED_

#include <stdafx.h>
#include "dSceneNode.h"
#include "OpenGlUtil.h"


#define GRAVITY	   -10.0f
#define TIRE_COLLITION_ID	0x100
#define CHASIS_COLLITION_ID	0x200


class NewtonVehicle: public dSceneNode  
{
	public:
	NewtonVehicle(dSceneNode* parent, const dMatrix& matrix);
	virtual ~NewtonVehicle();

	virtual void SetSteering(dFloat value) = 0;
	virtual void SetTireTorque(dFloat torque) = 0;
	virtual void SetApplyHandBrakes (dFloat value) = 0;
	virtual void SetControl(int index, dFloat value) = 0;

	NewtonJoint* GetJoint() const;
	NewtonBody* GetRigidBody() const;

	dFloat GetSpeed() const;
	
	protected:
	NewtonBody* m_vehicleBody;
	NewtonJoint* m_vehicleJoint;
};


#endif 
