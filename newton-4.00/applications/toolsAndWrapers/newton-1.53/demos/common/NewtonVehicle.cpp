//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// NewtonVehicle.cpp: implementation of the NewtonVehicle class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "NewtonVehicle.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NewtonVehicle::NewtonVehicle(dSceneNode* parent, const dMatrix& matrix)
  :dSceneNode(parent)
{
	SetMatrix (matrix);

	m_vehicleBody = NULL;
	m_vehicleJoint = NULL;
}

NewtonVehicle::~NewtonVehicle()
{
}


NewtonJoint* NewtonVehicle::GetJoint() const
{
	return m_vehicleJoint;
}

NewtonBody* NewtonVehicle::GetRigidBody() const
{
	return m_vehicleBody;
}

dFloat NewtonVehicle::GetSpeed() const
{
	dVector veloc;
	NewtonBodyGetVelocity(m_vehicleBody, &veloc[0]);

//veloc[2] = 0.1f;
//NewtonBodySetVelocity(m_vehicleBody, &veloc[0]);
 	return m_matrix.m_front % veloc;
}

