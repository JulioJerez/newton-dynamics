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


#ifndef __EDITOR_CAMERA_NODE_FLYBY_H__
#define __EDITOR_CAMERA_NODE_FLYBY_H__

#include "ndNewAssetStdafx.h"
#include "ndEditorCameraNode.h"

class ndAssetEditor;

class ndEditorCameraFlyby: public ndEditorCameraNode
{
	public:
	ndEditorCameraFlyby(ndAssetEditor* const editor);

	void TickUpdate(ndFloat32 timestep);
	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position) override;

	void MouseSelection();

	void CalculateCameraMatrix();
	void SetView(ndAssetEditor::ndCameraMode mode);

	ndVector m_posit;
	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndFloat32 m_yawRate;
	ndFloat32 m_pitchRate;
	ndFloat32 m_mousePosX;
	ndFloat32 m_mousePosY;
	ndFloat32 m_frontSpeed;
	bool m_mouseClick;
	ndWeakPtr<ndAssetEditor> m_editor;
};

#endif 
