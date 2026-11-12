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
#include "ndVehicleUnderSteerGradient.h"

ndVehicleUnderSteerGradient::ndVehicleUnderSteerGradient(ndAssetEditor* const owner)
	:ndAssetTool(owner)
	,m_mesh(owner->GetMesh()->CreateClone())
{
	m_mesh->CreateCloneFixDependencies();
	CalculateUndesteerGradient();
}

bool ndVehicleUnderSteerGradient::MeshChanged() const
{
	ndFixSizeArray<const ndMesh*, 256> self;
	ndFixSizeArray<const ndMesh*, 256> copy;

	self.PushBack(*m_mesh);
	copy.PushBack(*m_owner->GetMesh());

	while (self.GetCount() && copy.GetCount())
	{
		const ndMesh* const node0 = self.Pop();
		const ndMesh* const node1 = copy.Pop();
		if (node0->GetName() != node1->GetName())
		{
			return true;
		}
		if (node0->GetRigidBody())
		{
			ndAssert(node1->GetRigidBody());
			const ndMeshBody* const body0 = *node0->GetRigidBody();
			const ndMeshBody* const body1 = *node1->GetRigidBody();
			bool test = (*body0 == *body1);
			if (!test)
			{
				return true;
			}

			if (node0->GetJoint())
			{
				ndAssert(node1->GetJoint());
				const ndMeshJoint* const joint0 = *node0->GetJoint();
				const ndMeshJoint* const joint1 = *node1->GetJoint();
				bool test1 = (*joint0 == *joint1);
				if (!test1)
				{
					return true;
				}
			}
		}

		const ndList<ndSharedPtr<ndMesh>>& children0 = node0->GetChildren();
		const ndList<ndSharedPtr<ndMesh>>& children1 = node1->GetChildren();
		ndList<ndSharedPtr<ndMesh>>::ndNode* childNode1 = children1.GetFirst();
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* childNode0 = children0.GetFirst(); childNode0; childNode0 = childNode0->GetNext())
		{
			const ndMesh* const child0 = *childNode0->GetInfo();
			const ndMesh* const child1 = *childNode1->GetInfo();
			childNode1 = childNode1->GetNext();

			self.PushBack(child0);
			copy.PushBack(child1);
		}
	}
	return false;
}

void ndVehicleUnderSteerGradient::CalculateUndesteerGradient()
{
	ndMultiBodyVehicle m_vehicleModel;
	m_vehicleModel.Deserialize(*m_mesh);

	m_vehicleModel.ConvertToMotorVehicle();

	m_vehicleModel.CalculateRestSprungWeight();
	m_dradientK = m_vehicleModel.CalculateUndersteerGradient();

	m_topSpeed = m_vehicleModel.GetMotor()->GetTopSpeed();
}

void ndVehicleUnderSteerGradient::Execute()
{
	ImGuiWindowFlags flags = ImGuiWindowFlags_None;
	flags |= ImGuiWindowFlags_NoDocking;
	//flags |= ImGuiWindowFlags_AlwaysAutoResize;

	bool toolActive = m_owner->GetActiveTool();
	ImGui::Begin("vehicle teering gradient, (k1 stability)", &toolActive, flags);
	m_owner->SetActiveTool(toolActive);

	if (MeshChanged())
	{
		m_mesh = ndSharedPtr<ndMesh>(m_owner->GetMesh()->CreateClone());
		m_mesh->CreateCloneFixDependencies();
		CalculateUndesteerGradient();
	}

	auto PlotTorqueCurve = [this]()
	{
		ImVec2 canvas_pos = ImGui::GetCursorScreenPos();

		ImDrawList* const drawList = ImGui::GetWindowDrawList();

		ndReal y0 = 30.0f;
		ndReal x0 = 30.0f;
		ndReal high = 200.0f;
		ndReal width = 400.0f;
		ImU32 grayColor = IM_COL32(180, 180, 180, 255);

		// vertical axis
		{
			ImGui::Text("under steer gradient");
			const ImVec2 p0(canvas_pos.x + x0, canvas_pos.y + y0 + 5.0f);
			const ImVec2 p1(canvas_pos.x + x0, canvas_pos.y + high + y0 + 5.0f);
			drawList->AddLine(p0, p1, IM_COL32_WHITE, 1.0f);
			
			ndReal y = y0;
			ndInt32 divisions = 10;
			ndReal divisionStep = high / ndReal(divisions);
			for (ndInt32 i = 0; i < divisions; ++i)
			{
				y += divisionStep;
				const ImVec2 q0(canvas_pos.x + x0 - 5.0f, canvas_pos.y + y);
				const ImVec2 q1(canvas_pos.x + x0 + 5.0f, canvas_pos.y + y);
				drawList->AddLine(q0, q1, grayColor);
			}
		}

		// horizontal axis
		{
			for (ndInt32 i = 0; i < 10; ++i)
			{
				ImGui::NewLine();
			}
			ImGui::Text("                                           kph");
			const ImVec2 p0(canvas_pos.x + x0 - 10.0f, canvas_pos.y + high + y0);
			const ImVec2 p1(canvas_pos.x + width + x0 + 10.0f, canvas_pos.y + high + y0);
			drawList->AddLine(p0, p1, IM_COL32_WHITE, 1.0f);

			ndInt32 divisions = 10;
			ndReal x = x0;
			ndReal divisionStep = width / ndReal(divisions);
			for (ndInt32 i = 0; i < divisions; ++i)
			{
				x += divisionStep;
				const ImVec2 q0(canvas_pos.x + x, canvas_pos.y + high + y0 - 5.0f);
				const ImVec2 q1(canvas_pos.x + x, canvas_pos.y + high + y0 + 5.0f);
				drawList->AddLine(q0, q1, grayColor);
			}
		}

		// draw steer gradine couves
		{
			unsigned red(IM_COL32(255, 0, 0, 255));
			const ImVec2 p0(canvas_pos.x + x0 - 10.0f, canvas_pos.y + high / 2 + y0);
			const ImVec2 p1(canvas_pos.x + width + x0 + 10.0f, canvas_pos.y + high / 2 + y0);
			drawList->AddLine(p0, p1, red, 1.0f);

			ndInt32 steps = 16;
			ndFloat32 x = canvas_pos.x + x0 - 10.0f;
			ndFloat32 y = canvas_pos.y + high / 2 + y0;
			ndFloat32 deltaSpeed = m_topSpeed / ndFloat32(steps);
			ndFloat32 dx = (canvas_pos.x + width + x0 + 10.0f - x) / ndFloat32(steps);

			unsigned yellow(IM_COL32(255, 255, 0, 255));
			if (m_dradientK < 0.0f)
			{
				ndReal underSteerLimit = ndReal(ndSqrt(ndFloat32(-1.0f) / m_dradientK));
				ndFloat32 h = 0.5f * high / (m_dradientK * underSteerLimit * underSteerLimit);
				for (ndInt32 i = 1; i < steps; ++i)
				{
					ndReal x00 = dx * ndReal(i - 1);
					ndReal x01 = dx * ndReal(i - 0);

					ndReal v0 = ndReal(i - 1) * deltaSpeed;
					ndReal v1 = ndReal(i - 0) * deltaSpeed;
					ndReal y00 = y + h * m_dradientK * v0 * v0;
					ndReal y01 = y + h * m_dradientK * v1 * v1;

					const ImVec2 q0(x + x00, y00);
					const ImVec2 q1(x + x01, y01);
					drawList->AddLine(q0, q1, yellow, 1.0f);
				}
			}
			else
			{
				ndReal underSteerLimit = ndReal(ndSqrt(ndFloat32(1.0f) / m_dradientK));
				ndFloat32 h = -0.5f * high / (m_dradientK * underSteerLimit * underSteerLimit);
				for (ndInt32 i = 1; i < steps; ++i)
				{
					ndReal x00 = dx * ndReal(i - 1);
					ndReal x01 = dx * ndReal(i - 0);

					ndReal v0 = ndReal(i - 1) * deltaSpeed;
					ndReal v1 = ndReal(i - 0) * deltaSpeed;
					ndReal y00 = y + h * m_dradientK * v0 * v0;
					ndReal y01 = y + h * m_dradientK * v1 * v1;

					const ImVec2 q0(x + x00, y00);
					const ImVec2 q1(x + x01, y01);
					drawList->AddLine(q0, q1, yellow, 1.0f);
				}
			}
		}

		ImGui::Text("top speed %f", m_topSpeed * ndReal(3.6f));
		if (m_dradientK < 0.0f)
		{
			ndReal underSteerLimit = ndReal(ndSqrt(ndFloat32(-1.0f) / m_dradientK));
			ImGui::Text("under steer limit %f", underSteerLimit * ndReal(3.6f));
		}

	};
	PlotTorqueCurve();

	ImGui::End();
}
