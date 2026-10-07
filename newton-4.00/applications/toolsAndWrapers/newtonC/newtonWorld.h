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

#ifndef D_NEWTON_WORLD_H_
#define D_NEWTON_WORLD_H_

#include "newtonStdafx.h"

class NewtonWorld;

class ndNewtonWorld: public ndWorld
{
	public:
	typedef void (*NewtonPostUpdateCallback) (const NewtonWorld* const world, ndFloat32 timestep);
	ndNewtonWorld();
	virtual ~ndNewtonWorld() override;

	void ClearMaterials();
	ndMaterial* GetMaterial(int id0, int id1) const;
	
	virtual void Update(ndFloat32 timestep) override;
	virtual void PostUpdate(ndFloat32 timestep) override;

	ndWeakPtr<void> m_userData;
	ndInt32 m_bodyMaterialGroup;


	NewtonPostUpdateCallback m_onPostUpdate;
};

#endif