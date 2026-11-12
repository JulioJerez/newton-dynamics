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
#include "ndMotorTorqueRpmCurve.h"

ndMotorTorqueRpmCurve::ndMotorTorqueRpmCurve(ndAssetEditor* const owner, ndMeshJointVehicleMotor* const joint)
	:ndAssetTool(owner)
	,m_motor(joint)
{
}

void ndMotorTorqueRpmCurve::Execute()
{
	ImGuiWindowFlags flags = ImGuiWindowFlags_None;
	flags |= ImGuiWindowFlags_NoDocking;
	//flags |= ImGuiWindowFlags_AlwaysAutoResize;

	bool toolActive = m_owner->GetActiveTool();
	ImGui::Begin("torque rpm curve", &toolActive, flags);
	m_owner->SetActiveTool(toolActive);

	//for (ndInt32 i = 0; i < 6; ++i)
	//{
	//	ImGui::NewLine();
	//	ImGui::NewLine();
	//	ImGui::NewLine();
	//	ImGui::NewLine();
	//	ImGui::NewLine();
	//}

	auto PlotTorqueCurve = []()
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
			ImGui::Text("torque (nm)");
			const ImVec2 p0(canvas_pos.x + x0, canvas_pos.y + y0 + 5.0f);
			const ImVec2 p1(canvas_pos.x + x0, canvas_pos.y + high + y0 + 5.0f);
			drawList->AddLine(p0, p1, IM_COL32_WHITE, 1.0f);

			ndInt32 divisions = 10;
			ndReal y = y0;
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
			ImGui::Text("                                           rpm");
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

		//ndMultiBodyVehicleMotor::ndEngineTorqueCurve& engineCurve = m_motor->m_engineCurve;
	};
	PlotTorqueCurve();

	ImGui::End();
}
