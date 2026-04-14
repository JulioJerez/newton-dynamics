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
#include "ndAssetEditor.h"
#include "ndNomalizeMassDistribution.h"

class ndUndoRedoNormalizeMass : public ndUndoRedoCommand
{
	public:
	class MassDistibution
	{
		public:
		ndVector m_invMass;
		ndMeshBodyDynamic* m_body;
	};

	ndUndoRedoNormalizeMass(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		auto CollectInertia = [this](ndMesh* const node)
		{
			ndSharedPtr<ndMeshBody> body(node->GetRigidBody());
			if (body)
			{
				MassDistibution distribution;
				distribution.m_body = (ndMeshBodyDynamic*)*body;
				distribution.m_invMass = distribution.m_body->m_invMass;
				m_massDitribution.PushBack(distribution);
			}
		};
		mesh->NodeIterator(CollectInertia);
	}

	virtual ndUndoRedoNormalizeMass* GetAsUndoRedoResizeNormalizeMass() const override
	{
		return (ndUndoRedoNormalizeMass*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoNormalizeMass* const other = command.GetAsUndoRedoResizeNormalizeMass();
			if (other)
			{
				bool test = true;
				for (ndInt32 i = 0; test && (i < m_massDitribution.GetCount()); ++i)
				{
					ndVector diff(m_massDitribution[i].m_invMass - other->m_massDitribution[i].m_invMass);
					test = test && (diff.DotProduct(diff).GetScalar() < ndFloat32(1.0e-6f));
				}
				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		for (ndInt32 i = 0; i < m_massDitribution.GetCount() ; ++i)
		{
			m_massDitribution[i].m_body->m_invMass = m_massDitribution[i].m_invMass;
		}
	}

	ndArray<MassDistibution> m_massDitribution;
};


ndNomalizeMassDistribution::ndNomalizeMassDistribution(ndAssetEditor* const owner)
	:ndAssetTool(owner)
	,m_totalMass(100.0f)
	,m_inertialRatio(0.25f)
{
}

void ndNomalizeMassDistribution::Execute()
{
	ImGuiWindowFlags flags = ImGuiWindowFlags_None;
	flags |= ImGuiWindowFlags_NoDocking;
	flags |= ImGuiWindowFlags_AlwaysAutoResize;

	bool toolActive = m_owner->GetActiveTool();
	ImGui::Begin("normalize mass distribution", &toolActive, flags);
	m_owner->SetActiveTool(toolActive);
	
	if (ImGui::InputFloat("total mass", &m_totalMass, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_totalMass = ndMax(m_totalMass, ndReal(1.0f));
	}
	if (ImGui::InputFloat("principal Inertia ratio", &m_inertialRatio, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_inertialRatio = ndClamp(m_inertialRatio, ndReal(0.1f), ndReal(1.0f));
	}

	if (ImGui::Button("execute"))
	{
		m_owner->m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoNormalizeMass(*m_owner, m_owner->GetMesh())));

		ndFloat32 volume = ndFloat32(0.0f);
		auto TotalVolume = [this, &volume](ndMesh* const node)
		{
			ndSharedPtr<ndMeshBody> body (node->GetRigidBody());
			if (body)
			{
				ndMeshBodyDynamic* const dynBody = (ndMeshBodyDynamic*)*body;
				ndSharedPtr<ndShapeInstance> instance (dynBody->m_shapeInstance.CreateObject());
				ndFloat32 v = instance->GetVolume() * dynBody->m_massVolumeWeigh;
				volume += v;
			}
		};
		m_owner->GetMesh()->NodeIterator(TotalVolume);

		ndFloat32 density = m_totalMass / volume;
		auto SetBodyMass = [this, density](ndMesh* const node)
		{
			ndSharedPtr<ndMeshBody> body(node->GetRigidBody());
			if (body)
			{
				ndMeshBodyKinematic* const kinBody = (ndMeshBodyKinematic*)*body;
				ndVector inertia(kinBody->m_invMass.Reciproc());
				inertia = inertia.Scale(ndFloat32(1.0f) / inertia.m_w);
				ndFloat32 maxInertia = ndMax(inertia.m_x, ndMax(inertia.m_x, inertia.m_z));
				ndFloat32 minInertia = ndMin(inertia.m_x, ndMin(inertia.m_x, inertia.m_z));
				if (maxInertia * m_inertialRatio > minInertia)
				{
					minInertia = maxInertia * m_inertialRatio;
					for (ndInt32 i = 0; i < 3; ++i)
					{
						inertia[i] = ndMax(inertia[i], minInertia);
					}
				}

				ndSharedPtr<ndShapeInstance> instance(kinBody->m_shapeInstance.CreateObject());
				ndFloat32 v = instance->GetVolume() * kinBody->m_massVolumeWeigh;
				ndFloat32 bodyMass = density * v;
				
				inertia = inertia.Scale(bodyMass);
				inertia = inertia.Reciproc();
				kinBody->m_invMass = inertia;
			}
		};
		m_owner->GetMesh()->NodeIterator(SetBodyMass);
		m_owner->m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoNormalizeMass(*m_owner, m_owner->GetMesh())));
	}

	ImGui::End();
}