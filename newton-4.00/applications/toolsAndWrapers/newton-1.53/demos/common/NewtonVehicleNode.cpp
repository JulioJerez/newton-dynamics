//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// NewtonVehicleNode.cpp: implementation of the NewtonVehicleNode class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "NewtonVehicleNode.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NewtonVehicleNode::NewtonVehicleNode(dSceneNode* parent, const dMatrix& matrix)
  :dSceneNode(parent)
{
	SetMatrix (matrix);

	m_vehicleBody = NULL;
	m_vehicleJoint = NULL;
}

NewtonVehicleNode::~NewtonVehicleNode()
{
}


NewtonVehicle* NewtonVehicleNode::GetJoint() const
{
	return m_vehicleJoint;
}

NewtonBody* NewtonVehicleNode::GetRigidBody() const
{
	return m_vehicleBody;
}

dFloat NewtonVehicleNode::GetSpeed() const
{
	dVector veloc;
	NewtonBodyGetVelocity(m_vehicleBody, &veloc[0]);

//veloc[2] = 0.1f;
//NewtonBodySetVelocity(m_vehicleBody, &veloc[0]);
 	return m_matrix.m_front % veloc;
}

