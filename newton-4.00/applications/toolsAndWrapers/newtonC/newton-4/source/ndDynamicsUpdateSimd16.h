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

#ifndef ND_DYNAMICS_UPDATE_SIMD16_H_
#define ND_DYNAMICS_UPDATE_SIMD16_H_

#include <ndNewton.h>

class ndMatrixSimd16Array;
class ndJointMaskSimd16Array;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndDynamicsUpdateSimd16: public ndDynamicsUpdate
{
	public:
	ndDynamicsUpdateSimd16(ndWorld* const world);
	virtual ~ndDynamicsUpdateSimd16() override;

	virtual const char* GetStringId() const override;

	protected:
	virtual void Update() override;

	private:
	void SortJoints();
	void SortIslands();
	void BuildIsland();
	void CalculateForces();
	void InitJacobianMatrix();
	void CalculateJointsForce();
	void CalculateJointsAcceleration();

	ndArray<ndInt8> m_groupType;
	ndArray<ndInt32> m_simdJointRows;

	ndJointMaskSimd16Array* m_jointMask;
	ndMatrixSimd16Array* m_simdMassMatrixArray;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif
