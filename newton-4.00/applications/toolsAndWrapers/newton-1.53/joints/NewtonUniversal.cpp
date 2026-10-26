/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

class NewtonUniversal: public ndJointDoubleHinge
{
	public:
	D_CLASS_REFLECTION(NewtonUniversal, ndJointDoubleHinge)

	typedef unsigned (*NewtonUniversalCallBack) (const NewtonJoint* universal, NewtonHingeSliderUpdateDesc* desc);

	NewtonUniversal(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
		:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
		,m_callback(nullptr)
	{
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		if (m_callback && (**GetUserData()))
		{
			ndAssert(0);
			//NewtonHingeSliderUpdateDesc userDesc;
			//unsigned state = 0;
			//userDesc.m_accel = 0.0f;
			//userDesc.m_timestep = desc.m_timestep;
			//userDesc.m_minFriction = -1.0e-20f;
			//userDesc.m_maxFriction = 1.0e-20f;
			//NewtonJoint* const joint = PointerCast<NewtonJoint, NewtonCylinder>(this);
			//SetAsMotor(ndFloat32 (0.0f), ndFloat32(0.0f));
			//if (m_callback(joint, &userDesc))
			//{
			//	SetAsMotor(GetSpeed() + userDesc.m_accel * desc.m_invTimestep, ndMax (-userDesc.m_minFriction, userDesc.m_maxFriction));
			//}
		}

		ndJointDoubleHinge::JacobianDerivative(desc);
	}

	NewtonUniversalCallBack m_callback;
};

//
// Create a double hinge joint.
//
NewtonJoint* NewtonConstraintCreateUniversal(const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndBodyKinematic* const body0 = PointerCast<ndBody, NewtonBody>(childBody)->GetAsBodyKinematic();
	ndBodyKinematic* const body1 = parentBody ? PointerCast<ndBody, NewtonBody>(parentBody)->GetAsBodyKinematic() : world->GetSentinelBody();
	ndAssert(body0);
	ndAssert(body1);
	ndAssert(body0 != body1);

	const ndVector pivot(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(0.0f));
	const ndMatrix& bodyMatrix = body0->GetMatrix();

	ndVector pin(pinDir[0], pinDir[1], pinDir[2], ndFloat32(0.0f));
	ndMatrix matrix(ndGramSchmidtMatrix(pin));
	matrix.m_posit = ndVector(pivotPoint[0], pivotPoint[1], pivotPoint[2], ndFloat32(1.0f));
	ndSharedPtr<ndJointBilateralConstraint> joint(new NewtonUniversal(matrix, body0, body1));
	world->AddJoint(joint);
	return PointerCast<NewtonJoint, ndJointBilateralConstraint>(*joint);
}
