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
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndConvexCastVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTireJoint.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"


ndConvexCastVehicle::ndConvexCastVehicle(ndFloat32 gravityMagnitud)
	:ndMultiBodyVehicle(gravityMagnitud)
	,m_solver()
{
}

void ndConvexCastVehicle::ConvertToMotorVehicle()
{
	ndMultiBodyVehicle::ConvertToMotorVehicle();

	m_skeleton = ndSharedPtr<ndSkeletonContainer>(new ndSkeletonContainer);
	//m_skeleton->Init(owner, rootBody, id);
	//return container;


	// remove the drive train from simulation
	ndList<ndNode, ndContainersFreeListAlloc<ndNode>>& loops = GetCloseLoops();
	ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* nextNode;
	for (ndList<ndNode, ndContainersFreeListAlloc<ndNode>>::ndNode* node = loops.GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		ndSharedPtr<ndJointBilateralConstraint> joint(node->GetInfo().m_joint);
		if (joint->IsType(ndMultiBodyVehicleGearBox::StaticClassName()))
		{
			m_castGearBox = joint;
			loops.Remove(node);
		}
		else if (joint->IsType(ndMultiBodyVehicleDifferentialAxle::StaticClassName()))
		{
			m_castDifferentialAxelList.Append(joint);
			loops.Remove(node);
		}
	}

	//auto RemoveStructuralJoint
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		joint->SetActive(false);
	}
}
