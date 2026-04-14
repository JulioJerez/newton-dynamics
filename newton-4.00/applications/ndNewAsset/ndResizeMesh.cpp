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
#include "ndResizeMesh.h"
#include "ndDebugDisplayRenderPass.h"

class ndUndoRedoResizeMesh : public ndUndoRedoCommand
{
	public:
	ndUndoRedoResizeMesh(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh, ndFloat32 scale)
		:ndUndoRedoCommand(editor, mesh)
		,m_scale(scale)
	{
	}

	virtual ndUndoRedoResizeMesh* GetAsUndoRedoResizeMesh() const override
	{
		return (ndUndoRedoResizeMesh*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoResizeMesh* const other = command.GetAsUndoRedoResizeMesh();
			if (other)
			{
				bool test = m_scale == other->m_scale;
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
		ndResizeMesh resizeMesh(*m_editor);
		resizeMesh.m_scale = m_scale;
		resizeMesh.ApplyScale();
	}

	ndFloat32 m_scale;
};

ndResizeMesh::ndResizeMesh(ndAssetEditor* const owner)
	:ndAssetTool(owner)
	,m_scale(1.0f)
{
}

void ndResizeMesh::ApplyScale() const
{
	ndMatrix scaleMatrix(ndGetIdentityMatrix());
	scaleMatrix[0][0] = m_scale;
	scaleMatrix[1][1] = m_scale;
	scaleMatrix[2][2] = m_scale;

	ndMatrix invScaleMatrix(ndGetIdentityMatrix());
	invScaleMatrix[0][0] = ndReal(1.0f) / m_scale;
	invScaleMatrix[1][1] = ndReal(1.0f) / m_scale;
	invScaleMatrix[2][2] = ndReal(1.0f) / m_scale;

	auto ScaleMesh = [this, &scaleMatrix, &invScaleMatrix](ndMesh* const node)
	{
		ndSharedPtr<ndMeshEffect>& mesh = node->GetMesh();
		if (mesh)
		{
			mesh->ApplyTransform(scaleMatrix);
			node->SetMatrix(invScaleMatrix * node->GetMatrix() * scaleMatrix);
			node->SetGeometryMatrix(invScaleMatrix * node->GetGeometryMatrix() * scaleMatrix);
		}
		ndSharedPtr<ndMeshBody>& body(node->GetRigidBody());
		if (body)
		{
			// scale center of mass
			ndMeshBodyDynamic* const dynBody = (ndMeshBodyDynamic*)*body;
			dynBody->m_localCentreOfMass = scaleMatrix.RotateVector(dynBody->m_localCentreOfMass);

			// scale the diagonal inertia matrix (assume of box pinciapl axis)
			ndVector invInertia(dynBody->m_invMass);
			ndVector inertia(invInertia.Reciproc());
			ndVector unitInertia2(inertia.Scale(ndFloat32(1.0f) / inertia.m_w));
			ndVector unitInertia(unitInertia2.Sqrt());
			ndVector scaledInertia(scaleMatrix.RotateVector(unitInertia));
			scaledInertia = scaledInertia * scaledInertia;
			scaledInertia = scaledInertia.Scale(inertia.m_w);
			scaledInertia.m_w = inertia.m_w;
			ndVector scaleInvInertia(scaledInertia.Reciproc());
			dynBody->m_invMass = scaleInvInertia;

			// scale collsion shape;
			dynBody->m_shapeInstance.ApplyScale(scaleMatrix);
		}
		ndSharedPtr<ndMeshJoint>& joint(node->GetJoint());
		if (joint)
		{
			joint->ApplyTransform(scaleMatrix);
		}
	};
	m_owner->GetMesh()->NodeIterator(ScaleMesh);

	ndRenderMeshLoader loader(*m_owner->GetRenderer());
	loader.m_mesh = m_owner->GetMesh();
	loader.m_renderMesh = ndRenderMeshLoader::CreateRenderSceneMesh(*m_owner->GetRenderer(), *loader.m_mesh, loader.GetPath(m_owner->GetPath()));
	m_owner->SetVisualScene(loader);
}

void ndResizeMesh::Execute()
{
	ImGuiWindowFlags flags = ImGuiWindowFlags_None;
	flags |= ImGuiWindowFlags_NoDocking;
	flags |= ImGuiWindowFlags_AlwaysAutoResize;

	bool toolActive = m_owner->GetActiveTool();
	ImGui::Begin("resize mesh", &toolActive, flags);
	m_owner->SetActiveTool(toolActive);

	ndVector minBox(ndFloat32(1.0e10f));
	ndVector maxBox(ndFloat32(-1.0e10f));
	auto CalculateAABB = [this, &minBox, &maxBox](ndMesh* const node)
	{
		ndSharedPtr<ndMeshEffect>& mesh = node->GetMesh();
		if (mesh)
		{
			const ndInt32 vertexcount = mesh->GetVertexCount();
			const ndFloat64* const vertexPool = mesh->GetVertexPool();
			const ndInt32 stride = ndInt32(mesh->GetVertexStrideInByte() / sizeof(ndFloat64));

			const ndMatrix matrix(node->GetGeometryMatrix() * node->CalculateGlobalMatrix());
			for (ndInt32 i = 0; i < vertexcount; ++i)
			{
				ndVector p(matrix.TransformVector(ndVector(vertexPool[i * stride + 0], vertexPool[i * stride + 1], vertexPool[i * stride + 2], ndFloat64(1.0f))));
				minBox = minBox.GetMin(p);
				maxBox = maxBox.GetMax(p);
			}
		}
	};
	m_owner->GetMesh()->NodeIterator(CalculateAABB);

	ndVector boxSize(maxBox - minBox);

	ndReal size[3];
	size[0] = boxSize.m_x;
	size[1] = boxSize.m_y;
	size[2] = boxSize.m_z;
	ImGui::InputFloat3("size##4", size, "%.3f", ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_EnterReturnsTrue);

	if (ImGui::InputFloat("apply scale", &m_scale, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
	{
		m_scale = ndClamp(m_scale, ndReal(0.01f), ndReal(100.0f));
	}

	if (ImGui::Button("execute"))
	{
		if (m_scale != ndReal(1.0f))
		{
			m_owner->m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoResizeMesh(*m_owner, m_owner->GetMesh(), ndFloat32 (1.0f) / m_scale)));
			ApplyScale();
			m_owner->m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoResizeMesh(*m_owner, m_owner->GetMesh(), m_scale)));

			m_scale = ndReal(1.0f);
		}
	}
	
	ImGui::End();
}