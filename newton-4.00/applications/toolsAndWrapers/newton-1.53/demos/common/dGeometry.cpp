// dGeometry.cpp: implementation of the dGeometry class.
//
//////////////////////////////////////////////////////////////////////

#include "dGeometry.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
dGeometrySegment::dGeometrySegment ()
{
	m_uv = NULL;
	m_vertex = NULL;
	m_normal = NULL;
	m_indexes = NULL;
	m_indexCount = 0;
	m_vertexCount = 0;
	m_texture = 0;
};


dGeometrySegment::~dGeometrySegment ()
{
	if (m_vertex) {
		free (m_vertex);
		free (m_indexes);
		free (m_normal);
		free (m_uv);
	}
}


void dGeometrySegment::AllocVertexData (dInt32 vertexCount)
{
	m_vertexCount = vertexCount;

	m_vertex = (GLfloat*) malloc (3 * m_vertexCount * sizeof (GLfloat)); 
	m_normal = (GLfloat*) malloc (3 * m_vertexCount * sizeof (GLfloat)); 
	m_uv = (GLfloat*) malloc (2 * m_vertexCount * sizeof (GLfloat)); 
	memset (m_uv, 0, 2 * m_vertexCount * sizeof (GLfloat));
}


void dGeometrySegment::AllocIndexData (dInt32 indexCount)
{
	m_indexCount = indexCount;
	m_indexes = (GLushort *) malloc (m_indexCount * sizeof (GLushort )); 
}


dGeometry::dGeometry(dInt32 crc)
	:dList<dGeometrySegment>()
{
	m_crc = crc;
	m_refCount = 1;
}

dGeometry::~dGeometry()
{
}


dInt32 dGeometry::GetTotalVertexCount() const
{
	dInt32 count;
	dListNode* node;

	count = 0;
	for (node = GetFirst(); node; node = node->GetNext()) {
		dGeometrySegment& segment = node->GetInfo();
		count += segment.m_vertexCount;
	}
	return count;
}


dInt32 dGeometry::GetFlatVertexArray (dVector* const vertex, dInt32 maxSizeInVertex) const
{
	dInt32 i;
	dInt32 count;
	dListNode* node;

	count = 0;
	for (node = GetFirst(); node && (count + node->GetInfo().m_vertexCount < maxSizeInVertex); node = node->GetNext()) {
		dGeometrySegment& segment = node->GetInfo();
		for (i = 0; i < segment.m_vertexCount; i ++) {
			vertex[count + i].m_x = segment.m_vertex[i * 3 + 0];
			vertex[count + i].m_y = segment.m_vertex[i * 3 + 1];
			vertex[count + i].m_z = segment.m_vertex[i * 3 + 2];
			vertex[count + i].m_w = 1.0f;
		}
		count += segment.m_vertexCount;
	}
	return count;
}
