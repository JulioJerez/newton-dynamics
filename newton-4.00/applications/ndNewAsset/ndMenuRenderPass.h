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
#ifndef ND_MENU_RENDER_PASS_H_
#define ND_MENU_RENDER_PASS_H_

class ndAssetEditor;

class ndMenuRenderPass: public ndRenderPassGui
{
	public:
	ndMenuRenderPass(ndAssetEditor* const owner);
	virtual ~ndMenuRenderPass() override;

	void RenderScene() override;

	ndAssetEditor* m_owner;
};

#endif

