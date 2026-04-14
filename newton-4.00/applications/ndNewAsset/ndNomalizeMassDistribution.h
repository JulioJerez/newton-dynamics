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

#ifndef __D_NORMALIZE_MASS_TOOL_H__
#define __D_NORMALIZE_MASS_TOOL_H__

class ndNomalizeMassDistribution : public ndAssetEditor::ndAssetTool
{
	public:
	ndNomalizeMassDistribution(ndAssetEditor* const owner);

	virtual void Execute() override;

	ndReal m_totalMass;
	ndReal m_inertialRatio;
};

#endif