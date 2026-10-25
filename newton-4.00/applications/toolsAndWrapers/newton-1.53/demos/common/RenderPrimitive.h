//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// RenderPrimitive.h: interface for the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RENDERPRIMITIVE_H__0C9BDF8F_FF8D_4F5A_9692_4E76E3C8878A__INCLUDED_)
#define AFX_RENDERPRIMITIVE_H__0C9BDF8F_FF8D_4F5A_9692_4E76E3C8878A__INCLUDED_

#include <stdafx.h>
#include "dSceneNode.h"

struct NewtonWorld;



class RenderPrimitive: public dSceneNode
{
	public:
	RenderPrimitive(dSceneNode* parent, const dMatrix& matrix, int texture = -1);

	virtual ~RenderPrimitive();
	void SetTexture (int ID);
	public:
	unsigned m_texture;

	dFloat m_modifierScaleAngleX;
	dFloat m_modifierScaleAngleY;
	dFloat m_modifierScaleAngleZ;

	dFloat m_modifierSkewAngleY;
};


class BoxPrimitive: public RenderPrimitive
{
	public:
	BoxPrimitive(dSceneNode* parent, const dMatrix& matrix, const dVector& size, int texture = -1);
	void Render() const;
	
	dVector m_size;
};




class SpherePrimitive: public RenderPrimitive
{
	public:
	SpherePrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radiusX, dFloat radiusY, dFloat radiusZ, int texture = -1);
	void Render() const;
	
	dFloat m_radiusX;
	dFloat m_radiusY;
	dFloat m_radiusZ;
};


class CylinderPrimitive: public RenderPrimitive
{
	public:
	CylinderPrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture = -1);
	void Render() const;
	
	dFloat m_radius;
	dFloat m_height;
};

class ConePrimitive: public RenderPrimitive
{
	public:
	ConePrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture = -1);
	void Render() const;
	
	dFloat m_radius;
	dFloat m_height;
};


class CapsulePrimitive: public RenderPrimitive
{
	public:
	CapsulePrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture = -1);
	void Render() const;
	
	dFloat m_radius;
	dFloat m_height;
};


class ChamferCylinderPrimitive: public RenderPrimitive
{
	public:
	ChamferCylinderPrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture = -1);
	void Render() const;
	GLuint m_list;
};


class ConvexHullPrimitive: public RenderPrimitive
{
	public:
	ConvexHullPrimitive(dSceneNode* parent, const dMatrix& matrix, NewtonWorld* nWorld, NewtonCollision* collision, int texture = -1);
	void Render() const;

	static void GetShapeFromCollision (const NewtonBody* body, int vertexCount, const dFloat* faceVertex, int id);
	
	GLuint m_list;
};

class TorusPrimitive: public RenderPrimitive
{
	public:
	TorusPrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radiusA, dFloat radiusC, int texture = -1);
	void Render() const;
	
	GLuint m_list;
};



class SkyBoxPrimitive: public RenderPrimitive
{
	public:
	SkyBoxPrimitive(dSceneNode* parent);
	~SkyBoxPrimitive();

	void Render() const;

	private:
	GLuint m_textures[6];

	dVector m_size;
//	GLuint m_list;
};

#endif 
