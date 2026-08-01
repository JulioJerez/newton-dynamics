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
#ifndef __RENDER_SHADER_BLOCK_H__
#define __RENDER_SHADER_BLOCK_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"

class ndRenderPrimitiveImplement;

class ndRenderShaderBlock
{
	public:
	ndRenderShaderBlock();
	virtual ~ndRenderShaderBlock();

	void SetWidingMode(bool clockwise) const;
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) = 0;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const = 0;

	protected:
	void EndParameters();
	virtual void SetParameters(ndUnsigned32 shader);

	ndUnsigned32 m_shader;
};

#if 0
// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderSetZbufferCleanBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;
	ndInt32 m_viewModelProjectionMatrix;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateShadowMapBlock : public ndRenderShaderSetZbufferCleanBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateAlphaTestShadowMapBlock : public ndRenderShaderGenerateShadowMapBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;
	//protected:
	//virtual void SetParameters(ndUnsigned32 shader) override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateInstanceShadowMapBlock : public ndRenderShaderGenerateShadowMapBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;

	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDebugFlatShadedDiffusedBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_diffuseColor;
	ndInt32 m_directionalLightAmbient;
	ndInt32 m_directionalLightIntesity;
	ndInt32 m_directionalLightDirection;
	ndInt32 m_projectMatrixLocation;
	ndInt32 m_viewModelMatrixLocation;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDebugWireframeDiffuseBlock : public ndRenderShaderDebugFlatShadedDiffusedBlock
{
	public:
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedColorBlock : public ndRenderShaderDebugFlatShadedDiffusedBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_texture;
	ndInt32 m_cameraToWorld;
	ndInt32 m_specularColor;
	ndInt32 m_specularAlpha;
	ndInt32 m_environmentMap;
	ndInt32 m_reflectionColor;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedShadowColorBlock : public ndRenderShaderOpaqueDiffusedColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_worldMatrix;
	ndInt32 m_shadowSlices;
	ndInt32 m_depthMapTexture;
	ndInt32 m_directionLightViewProjectionMatrixShadow;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderTransparentDiffusedShadowColorBlock : public ndRenderShaderOpaqueDiffusedColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_opacity;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderInstancedOpaqueDiffusedShadowBlock : public ndRenderShaderOpaqueDiffusedShadowColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;
};


// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateSkinShadowMapBlock : public ndRenderShaderGenerateShadowMapBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_matrixPalette;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedShadowSkinColorBlock : public ndRenderShaderOpaqueDiffusedShadowColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_matrixPalette;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderStaticLinesArrayBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_viewModelProjectionMatrix;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDynamicLinesArrayBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_viewModelProjectionMatrix;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDynamicPointsArrayBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_viewModelProjectionMatrix;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDynamicTrianglesArrayBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(ndUnsigned32 shader) override;

	ndInt32 m_directionalLightDirection;
	ndInt32 m_projectMatrixLocation;
	ndInt32 m_viewModelMatrixLocation;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedShadowColorAlphaTestBlock : public ndRenderShaderOpaqueDiffusedShadowColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
};

#endif
#endif
