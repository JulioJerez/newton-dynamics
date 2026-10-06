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

class ndNewtonWorld: public ndWorld
{
	public:
	ndNewtonWorld();
	~ndNewtonWorld();


	ndMaterial* GetMaterial(int id0, int id1) const;
	
	 
	//void Update(ndFloat32 timestep);
	//void SetSubSteps(ndInt32 substeps);
	//void SetIterations(ndInt32 iterations);

	ndWeakPtr<void> m_userData;
	ndInt32 m_bodyMaterialGroup;
};

#endif