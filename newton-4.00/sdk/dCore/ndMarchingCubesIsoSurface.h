/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_MARCHING_CUBES_ISO_SURFACE_H__
#define __ND_MARCHING_CUBES_ISO_SURFACE_H__

#include "ndCoreStdafx.h"
#include "ndMarchingCubes.h"

class ndMarchingCubeIsoSurface : public ndMarchingCubes
{
	class ndIsoCell;

	public:
	D_CORE_API ndMarchingCubeIsoSurface(const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize);
	D_CORE_API virtual ~ndMarchingCubeIsoSurface();

	D_CORE_API virtual void GenerateMesh() override;
	virtual ndReal GetIsoValue(const ndVector& posit) const = 0;
	D_CORE_API ndVector PositionToGrid(const ndVector& posit) const;

	protected:
	void GenerateIndexList();
	ndVector m_boxP0;
	ndVector m_boxP1;
	ndArray<ndReal> m_densityWindow0;
	ndArray<ndReal> m_densityWindow1;
};

#endif