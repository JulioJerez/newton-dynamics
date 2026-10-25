//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// basic Hierarchical Scene Node Class  
//********************************************************************


#if !defined(AFX_SIMPLESCENENODE_H__B1AC6903_8356_4935_882F_000492C28410__INCLUDED_)
#define AFX_SIMPLESCENENODE_H__B1AC6903_8356_4935_882F_000492C28410__INCLUDED_

#include <stdafx.h>
#include "dGeometry.h"
#include "dBaseHierarchy.h"

class dChunkHeader;
class dMaterialList;
class dSceneNode;

typedef void (dSceneNode::*NodeCallback) (const dMatrix& globalMatrix, void* userData) const;

class dLoaderContext
{
	public:
	dLoaderContext ();
	virtual ~dLoaderContext ();
	virtual dSceneNode* CreateNode();
};



class dSceneNode: public dHierarchy<dSceneNode>
{
/*
	public:
	struct GeometrySegment
	{
		GeometrySegment ();
		~GeometrySegment ();
		void AllocVertexData (int vertexCount);
		void AllocIndexData (int indexCount);
		
		GLuint m_texture;
		int m_indexCount;
		int m_vertexCount;
		GLfloat *m_uv;
		GLfloat *m_vertex;
		GLfloat *m_normal;
		GLushort *m_indexes;
	};
	
	class RenderList: public dList<GeometrySegment>
	{
		public:
		RenderList();
		int GetTotalVertexCount() const;
		int GetFlatVectexArray (dVector* const vertex, int maxSizeInVertex) const;
	};
*/	
	public:
	dSceneNode(dSceneNode* parent);
	virtual ~dSceneNode();
	virtual void Render() const;
	
	dMatrix& GetMatrix();
	void SetMatrix (const dMatrix& matrix);

	const dGeometry* GetGeometry() const;
	dMatrix CalcGlobalMatrix (dSceneNode* root = NULL) const;
	void LoadFromFile(const char *fileName, dLoaderContext& context);
	void ForEachNode (const dMatrix& matrix, NodeCallback callback, void* userData) const;
			
	private:
		
	protected:
	dMatrix m_matrix;
	dGeometry* m_geometry;
//	RenderList m_renderList;
			
	friend class dTGFileParcel;
	friend class GraphicManager;
};


inline dMatrix& dSceneNode::GetMatrix ()
{
	return m_matrix;
}

inline void dSceneNode::SetMatrix (const dMatrix& matrix)
{
	m_matrix = matrix;	
}


#endif 
