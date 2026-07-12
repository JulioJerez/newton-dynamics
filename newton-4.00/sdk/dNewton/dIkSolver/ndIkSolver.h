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

#ifndef __ND_SKELETON_IMMEDIATE_SOLVER_H__
#define __ND_SKELETON_IMMEDIATE_SOLVER_H__

#include "ndNewtonStdafx.h"
#include "ndSkeletonContainer.h"

class ndWorld;
class ndConstraint;

class ndIkSolver: public ndClassAlloc
{
	public:	
	D_NEWTON_API ndIkSolver();
	D_NEWTON_API ~ndIkSolver();

	D_NEWTON_API void SetMaxAccel(ndFloat32 maxAccel, ndFloat32 maxAlpha);
	D_NEWTON_API bool IsSleeping(ndSkeletonContainer* const skeleton) const;

	D_NEWTON_API void SolverBegin(ndSkeletonContainer* const skeleton, ndConstraint* const* loopConstraint, ndInt32 loopsCount, ndWorld* const world, ndFloat32 timestep, ndInt32 threadId);
	D_NEWTON_API void Solve();
	D_NEWTON_API void UpdateJointAcceleration(ndConstraint* const joint);
	D_NEWTON_API void SolverEnd();

	private:
	void BuildMassMatrix(ndInt32 threadId);
	void GetJacobianDerivatives(ndConstraint* const joint);
	void BuildJacobianMatrix(ndConstraint* const joint); 

	class ndSurrogates
	{
		public:
		ndContact* m_contact;
		ndBodyDynamic* m_body;
		ndContact* m_surrogateContact;
		ndBodyDynamic* m_surrogateBody;
	};

	template<class T>
	class ndSurrogateList : public ndList<ndSharedPtr<T>, ndContainersFreeListAlloc<ndSharedPtr<T>>>
	{
		public:
		ndSurrogateList()
			:ndList<ndSharedPtr<T>, ndContainersFreeListAlloc<ndSharedPtr<T>>>()
			,m_array(0)
		{
		}

		void Add(ndSharedPtr<T>& object)
		{
			this->Append(object);
			m_array.PushBack(*object);
		}

		T* operator[] (ndInt32 i)
		{
			return m_array[i];
		}

		const T* operator[] (ndInt32 i) const
		{
			return m_array[i];
		}

		ndFixSizeArray<T*, D_INV_IK_MAX_LINKS > m_array;
	};

	ndFixSizeArray<ndContact*, D_INV_IK_MAX_LINKS> m_contacts;
	ndFixSizeArray<ndBodyKinematic*, D_INV_IK_MAX_LINKS> m_bodies;
	ndFixSizeArray<ndInt32, D_INV_IK_MAX_LINKS> m_savedBodiesIndex;
	ndFixSizeArray<ndLeftHandSide, D_INV_IK_MAX_LINKS> m_leftHandSide;
	ndFixSizeArray<ndRightHandSide, D_INV_IK_MAX_LINKS> m_rightHandSide;
	
	//ndFixSizeArray<ndSharedPtr<ndContact>, D_INV_IK_MAX_LINKS> m_surrogateContact;
	//ndFixSizeArray<ndSharedPtr<ndBodyDynamic>, D_INV_IK_MAX_LINKS> m_surrogateBodies;
	ndSurrogateList<ndContact> m_surrogateContact;
	ndSurrogateList<ndBodyDynamic> m_surrogateBodies;
	
	ndWeakPtr<ndWorld> m_world;
	ndWeakPtr<ndSkeletonContainer> m_skeleton;
	ndFloat32 m_timestep;
	ndFloat32 m_invTimestep;
	ndFloat32 m_maxAccel;
	ndFloat32 m_maxAlpha;

	friend class ndSkeletonContainer;
};


#endif


