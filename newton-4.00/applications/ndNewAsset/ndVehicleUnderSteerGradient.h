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

#ifndef D_VEHICLE_UNDER_STEER_GRADIENT_TOOL_H_
#define D_VEHICLE_UNDER_STEER_GRADIENT_TOOL_H_

class ndVehicleUnderSteerGradient : public ndAssetEditor::ndAssetTool
{
	public:
	ndVehicleUnderSteerGradient(ndAssetEditor* const owner);
	virtual void Execute() override;

	bool MeshChanged() const;
	void CalculateUndesteerGradient();
	

	ndSharedPtr<ndMesh> m_mesh;
	ndFloat32 m_dradientK;
	ndFloat32 m_topSpeed;
};

#endif
