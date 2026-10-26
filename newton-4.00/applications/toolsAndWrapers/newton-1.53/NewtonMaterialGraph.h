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

#ifndef D_NEWTON_MATERIAL_GRAPH_H_
#define D_NEWTON_MATERIAL_GRAPH_H_

#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

struct NewtonBody;
struct NewtonJoint;
struct NewtonWorld;

class ndNewtonMaterial : public ndApplicationMaterial
{
	public:
	typedef int (*NewtonContactBegin) (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1);
	typedef int (*NewtonContactProcess) (const NewtonMaterial* material, const NewtonContact* contact);
	typedef void (*NewtonContactEnd) (const NewtonMaterial* material);

	ndNewtonMaterial();
	ndNewtonMaterial(const ndNewtonMaterial& copy);
	virtual ~ndNewtonMaterial() override;
	virtual ndApplicationMaterial* Clone() const override;
	virtual void OnContactCallback(const ndContact* const contact, ndFloat32 timestep) const override;
	virtual bool OnAabbOverlap(const ndBodyKinematic* const bodyPtr0, const ndBodyKinematic* const bodyPtr1) const override;
	virtual bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const override;

	ndWeakPtr<void> m_usedData;
	NewtonContactBegin m_onAABBOverlap;
	NewtonContactProcess m_onContactsProcess;
	NewtonContactEnd m_onContactEnd;
	//NewtonOnCompoundSubCollisionAABBOverlap m_onSubShapeAABBOverlap;
};

#endif