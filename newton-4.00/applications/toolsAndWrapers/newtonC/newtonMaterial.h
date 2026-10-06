/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef D_NEWTON_CONTACT_CALLBACK_H_
#define D_NEWTON_CONTACT_CALLBACK_H_

#include "newtonStdafx.h"
#include "Newton.h"

class ndNewtonMaterial : public ndApplicationMaterial
{
	public:
	ndNewtonMaterial();
	ndNewtonMaterial(const ndNewtonMaterial& src);
	virtual ~ndNewtonMaterial();

	virtual ndApplicationMaterial* Clone() const
	{
		return new ndNewtonMaterial(*this);
	}

	virtual void OnContactCallback(const ndContact* const joint, ndFloat32 timestep) const;
	virtual bool OnAabbOverlap(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const;
	virtual bool OnAabbOverlap(const ndContact* const joint, ndFloat32 timestep, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const;

	NewtonOnAABBOverlap m_onAABBOverlap;
	NewtonContactsProcess m_onContactsProcess;
};

#endif