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
#include "ndRotateMesh.h"
#include "ndDebugDisplayRenderPass.h"

ndRotateMesh::ndRotateMesh(ndAssetEditor* const owner)
	:ndAssetTool(owner)
{
	m_angles[0] = ndReal(0.0f);
	m_angles[1] = ndReal(0.0f);
	m_angles[2] = ndReal(0.0f);
}

void ndRotateMesh::Execute()
{
	ImGuiWindowFlags flags = ImGuiWindowFlags_None;
	flags |= ImGuiWindowFlags_NoDocking;
	flags |= ImGuiWindowFlags_AlwaysAutoResize;

	bool toolActive = m_owner->GetActiveTool();
	ImGui::Begin("rotate mesh", &toolActive, flags);
	m_owner->SetActiveTool(toolActive);

	ImGui::InputFloat3("angles##4", m_angles, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue);

	if (ImGui::Button("execute"))
	{
		bool test = m_angles[0] != ndReal(0.0f);
		test = test || (m_angles[1] != ndReal(0.0f));
		test = test || (m_angles[2] != ndReal(0.0f));
		if (test)
		{
			ndMatrix rotation(ndPitchMatrix(m_angles[0] * ndDegreeToRad) * ndYawMatrix(m_angles[1] * ndDegreeToRad) * ndRollMatrix(m_angles[0] * ndDegreeToRad));
			ndMatrix invRotation(rotation.OrthoInverse());

			auto RotateMesh = [this, &rotation, &invRotation](ndMesh* const node)
			{
				ndSharedPtr<ndMeshEffect>& mesh = node->GetMesh();
				if (mesh)
				{
					mesh->ApplyTransform(rotation);
					node->SetMatrix(invRotation * node->GetMatrix() * rotation);
					node->SetGeometryMatrix(invRotation * node->GetGeometryMatrix() * rotation);
				}
				ndSharedPtr<ndMeshBody>& body(node->GetRigidBody());
				if (body)
				{
					// scale center of mass
					ndMeshBodyDynamic* const dynBody = (ndMeshBodyDynamic*)*body;
					dynBody->m_localCentreOfMass = rotation.RotateVector(dynBody->m_localCentreOfMass);

					// scale the diagonal inertia matrix (assume of box pinciapl axis)
					ndVector invInertia(dynBody->m_invMass);
					ndVector inertia(invInertia.Reciproc());
					ndMatrix diagonalInertia(ndGetIdentityMatrix());
					diagonalInertia[0][0] = inertia[0];
					diagonalInertia[1][1] = inertia[1];
					diagonalInertia[2][2] = inertia[2];

					ndVector inertiaAxis(dynBody->m_inertiaPrincipalAxis.Scale (ndDegreeToRad));
					ndMatrix axisAngles(ndPitchMatrix(inertiaAxis[0]) * ndYawMatrix(inertiaAxis[1]) * ndRollMatrix(inertiaAxis[2]));
					ndMatrix newRotation(axisAngles * rotation);
					ndMatrix newIntertia(newRotation.OrthoInverse() * diagonalInertia * newRotation);

					ndVector eigenValues (newIntertia.EigenVectors());
					eigenValues.m_w = inertia.m_w;
					ndVector newEigenValues(eigenValues.Reciproc());
					dynBody->m_invMass = newEigenValues;

					ndVector eulers1;
					ndVector eulers0 (newIntertia.CalcPitchYawRoll(eulers1));
					dynBody->m_inertiaPrincipalAxis = eulers0.Scale(ndRadToDegree);

					ndMeshShapeInstance& shapeInstance = dynBody->m_shapeInstance;
					shapeInstance.m_localMatrix = shapeInstance.m_localMatrix * rotation;
				}

				ndSharedPtr<ndMeshJoint>& joint(node->GetJoint());
				if (joint)
				{
					joint->ApplyTransform(rotation);
				}
			};
			m_owner->GetMesh()->NodeIterator(RotateMesh);

			ndRenderMeshLoader loader(*m_owner->GetRenderer());
			loader.m_mesh = m_owner->GetMesh();
			loader.m_renderMesh = ndRenderMeshLoader::CreateRenderSceneMesh(*m_owner->GetRenderer(), *loader.m_mesh, loader.GetPath(m_owner->GetPath()));
			m_owner->SetVisualScene(loader);

			m_angles[0] = ndReal(0.0f);
			m_angles[1] = ndReal(0.0f);
			m_angles[2] = ndReal(0.0f);
		}
	}
	
	ImGui::End();
}