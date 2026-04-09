/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndNewAssetStdafx.h"
#include "ndUndoRedo.h"
#include "ndAssetEditor.h"
#include "ndNomalizeMassDistribution.h"

ndNomalizeMassDistribution::ndNomalizeMassDistribution(ndAssetEditor* const owner)
	:ndAssetTool(owner)
	,m_totalMass(100.0f)
{
}

void ndNomalizeMassDistribution::Execute()
{
	ImGui::Begin("normalize mass distribution", &m_owner->m_toolActive);
	
	ImGui::InputFloat("total mass", &m_totalMass, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue);

	if (ImGui::Button("execute"))
	{
		ndFloat32 volume = ndFloat32(0.0f);
		auto TotalVolume = [this, &volume](ndMesh* const node)
		{
			ndSharedPtr<ndMeshBody> body (node->GetRigidBody());
			if (body)
			{
				ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
				ndSharedPtr<ndShapeInstance> instance (kinBody->m_shapeInstance.CreateObject());
				ndFloat32 v = instance->GetVolume() * kinBody->m_massVolumeWeigh;
				volume += v;
			}
		};
		m_owner->m_mesh->NodeIterator(TotalVolume);

		ndFloat32 density = m_totalMass / volume;
		auto SetBodyMass = [this, density](ndMesh* const node)
		{
			ndSharedPtr<ndMeshBody> body(node->GetRigidBody());
			if (body)
			{
				ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
				ndVector inertia(kinBody->m_invMass.Reciproc());
				inertia = inertia.Scale(ndFloat32(1.0f) / inertia.m_w);

				ndSharedPtr<ndShapeInstance> instance(kinBody->m_shapeInstance.CreateObject());
				ndFloat32 v = instance->GetVolume() * kinBody->m_massVolumeWeigh;
				ndFloat32 bodyMass = density * v;
				
				inertia = inertia.Scale(bodyMass);
				inertia = inertia.Reciproc();
				kinBody->m_invMass = inertia;
			}
		};
		m_owner->m_mesh->NodeIterator(SetBodyMass);
	}

	ImGui::End();
}