//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// basic Hierarchical Scene Node Class  
//********************************************************************

#if !defined(AFX_DGEOMETRY_H__1CDA3BB6_2A4A_4639_83C1_F3B0F8C2A276__INCLUDED_)
#define AFX_DGEOMETRY_H__1CDA3BB6_2A4A_4639_83C1_F3B0F8C2A276__INCLUDED_

#include <stdafx.h>

class dGeometrySegment
{
	public:

	dGeometrySegment ();
	~dGeometrySegment ();
	void AllocVertexData (dInt32 vertexCount);
	void AllocIndexData (dInt32 indexCount);
	
	GLuint m_texture;
	dInt32 m_indexCount;
	dInt32 m_vertexCount;
	GLfloat *m_uv;
	GLfloat *m_vertex;
	GLfloat *m_normal;
	GLushort *m_indexes;
};


class dGeometry: public dList<dGeometrySegment>  
{
	public:
	dGeometry(dInt32 m_crc);
	
	dInt32 GetTotalVertexCount() const;
	dInt32 GetFlatVertexArray (dVector* const vertex, dInt32 maxSizeInVertex) const;
	virtual ~dGeometry();

	public:
//	private:
	dInt32 m_crc;
	dInt32 m_refCount;
};

#endif 
