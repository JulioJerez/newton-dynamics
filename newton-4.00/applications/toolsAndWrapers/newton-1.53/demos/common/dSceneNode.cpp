//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// basic Hierarchical Scene Node Class
//********************************************************************

#include <stdafx.h>
#include "OpenGlUtil.h"
#include "dSceneNode.h"
#include "dChunkParsel.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

// file format chunk ids

/*	dgMainBlock													= 'MAIN',
		dgCollisionBlock										= 'COLL',
		dgModelBlock											= 'MODL',
			dgGlobalBBox  										= 'GBOX',
			dgTextureListBlock	 								= 'TEXT',
				dgTextureName									= 'TNAM',
			dgdMaterialListBlock								= 'MTRL',
				dgMaterialBlock	 								= 'MATE',
			dgMeshBlock											= 'MESH',
				dgNameBlock										= 'NAME',
				dgParentNameBlock								= 'PRNT',
				dgMeshType										= 'MTYP',
				dgMeshHiddenFlag								= 'HIDN',
				dgMeshMatrix									= 'TRNS',
				dgGeometryCRC									= '_CRC'
				dgMeshGeometryBlock								= 'GEOM',
					dgMeshdGeometrySegmentBlock					= 'SEGM',
						dgMeshSegmentInfoBlock					= 'INFO',	
							dgMeshSegmentMaterialIndexBlock		= 'MIND',
							dgMeshSegmentTriangleCountBlock 	= 'TRGI',
							dgMeshSegmentStripIndexCountBlock 	= 'STRI',
							dgMeshSegmentVertexCountBlock		= 'VCNT',
							dgMeshSegmentVertexFlagsBlock		= 'VFLG',
						dgMeshSegmentPolygonsBlock				= 'GEOM',	
							dgMeshSegmentIndexArrayBlock 		= 'INDX',
							dgMeshSegmentVertexArrayBlock		= 'VERT',
							dgMeshSegmentNormalArrayBlock		= 'NORM',
							dgMeshSegmentColorArrayBlock		= 'COLR',
							dgMeshSegmentUV0ArrayBlock			= 'UV_0',
							dgMeshSegmentUV1ArrayBlock			= 'UV_1',
							dgMeshSegmentWeightsArrayBlock		= 'WGHT',
*/	



class dGeometryCache: public dTree<dGeometry*, dUnsigned32>
{
	public:
	dGeometryCache ()
		:dTree<dGeometry*, dUnsigned32>()
	{
	}
};

static dGeometryCache geometryCache;


//class dTGFileParcel: public dChunkParsel
class dTGFileParcel: public dChunkParsel
{
	class TextureList: public dTree<GLuint, dInt32>
	{

		public: 
		TextureList()
			:dTree<GLuint, dInt32>()
		{
			index = 0;
		}

		void AddTexture (const char* name)
		{
			GLuint texture;
			LoadTexture(name, texture);
			Insert (texture, index);
			index ++;
		}

		

		GLuint GetTexture  (int i)  const
		{
			dTreeNode* node;

			node = Find (i);
			_ASSERTE (node);
			return node->GetInfo();
		}

		dInt32 index;
	};


	struct Material
	{
		GLuint m_texture;
	};

	class dMaterialList: public dTree<Material, dInt32>
	{

		public: 
		dMaterialList()
			:dTree<Material, dInt32>()
		{
			m_index = 0;
		}

		void AddMaterial (const Material& material)
		{
			Insert (material, m_index);
			m_index ++;
		}

		Material* GetMaterial (int index) const
		{
			dTreeNode* node;
			
			node = Find (index);
			_ASSERTE (node);
			return &node->GetInfo();
		}

		dInt32 m_index;
	};


	public:
	dTGFileParcel(FILE *file, dSceneNode* rootNode, dLoaderContext* context)
		:dChunkParsel (file)
	{
		m_geometryCRC = 0;
		m_rootNode = rootNode;
		m_loaderContext = context;

		m_rootNode->SetNameID ("dommyroot");
		BeginParse ('MAIN', (ChunkProcess) &dTGFileParcel::ParceMainChunk, "dgMainBlock");
	}

	// this is a legacy chunk not used
	dInt32 ParceCollisionChunk(void* handle, const char* label)
	{
		_ASSERTE (0);
		return 0;
	}


	dInt32 ReadMeshName(void* handle, const char* label)
	{
		char name[128];

		ReadData (handle, name);
		m_currMesh->SetNameID(name);
		return 0;
	}

	dInt32 ReadParentName(void* handle, const char* label)
	{
		char name[128];
		dSceneNode *parentNode;

		ReadData (handle, name);

		parentNode = m_rootNode->Find (name);
		if (!parentNode) {
			parentNode = m_rootNode;
		}
		m_currMesh->Attach (parentNode);
		return 0;
	}

	dInt32 ReadLocalTransformName(void* handle, const char* label)
	{
		dInt32 i;
		dInt32 j;
		float tmp[4][4];
		dMatrix matrix(GetIdentityMatrix());	

		ReadData (handle, &tmp[0][0]);
		SWAP_FLOAT32_ARRAY (&tmp[0][0], 16);
		for (i = 0; i < 4; i ++) {
			for (j = 0; j < 4; j ++) {
				matrix[i][j] = tmp[i][j];
			}
		}
		m_currMesh->SetMatrix (matrix);		
		return 0;
	}

	dInt32 ReadGeometrySegmentBlockVertexCount (void* handle, const char* label)
	{
		dInt32 vertexCount;

		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();

		ReadData (handle, &vertexCount);
		vertexCount = SWAP_INT32(vertexCount);

		geometry.AllocVertexData (vertexCount);
  		return 0;
	}

	dInt32 ReadGeometrySegmentBlockIndexCount (void* handle, const char* label)
	{
		dInt32 indexCount;
		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();

		ReadData (handle, &indexCount);
		indexCount = SWAP_INT32(indexCount);

		geometry.AllocIndexData (indexCount);
  		return 0;
	}

	
	dInt32 ReadGeometrySegmentBlockMaterialIndex (void* handle, const char* label)
	{
		Material *material;
		dInt32 materialIndex;

		ReadData (handle, &materialIndex);
		materialIndex = SWAP_INT32(materialIndex);

		material = m_materialList.GetMaterial (materialIndex);
		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();
		geometry.m_texture = material->m_texture;
  		return 0;
	}


	dInt32 ReadGeometrySegmentBlockInfo(void* handle, const char* label)
	{
		
		//RegisterChunk ('STRI',	"dgMeshSegmentStripIndexCountBlock");
		//RegisterChunk ('VFLG',	"dgMeshSegmentVertexFlagsBlock");
		//RegisterChunk ('INDX',	"dgMeshSegmentIndexArrayBlock");
		RegisterChunk ('MIND', (ChunkProcess) &dTGFileParcel::ReadGeometrySegmentBlockMaterialIndex, "dgMeshSegmentMaterialIndexBlock");
		RegisterChunk ('VCNT', (ChunkProcess) &dTGFileParcel::ReadGeometrySegmentBlockVertexCount, "dgMeshSegmentVertexCountBlock");
		RegisterChunk ('TRGI', (ChunkProcess) &dTGFileParcel::ReadGeometrySegmentBlockIndexCount, "dgMeshSegmentTriangleCountBlock");
	
		ParceChunk (handle);
		return 0;
	}


	dInt32 ReadGeometrySegmentBlockVertex (void* handle, const char* label)
	{
		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();

		ReadData (handle, geometry.m_vertex);
		SWAP_FLOAT32_ARRAY (geometry.m_vertex, geometry.m_vertexCount * 3);
  		return 0;
	}

	dInt32 ReadGeometrySegmentBlockNormals (void* handle, const char* label)
	{
		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();

		ReadData (handle, geometry.m_normal);
		SWAP_FLOAT32_ARRAY (geometry.m_normal, geometry.m_vertexCount * 3);
  		return 0;
	}

	dInt32 ReadGeometrySegmentBlockUV_0 (void* handle, const char* label)
	{
		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();

		ReadData (handle, geometry.m_uv);
		SWAP_FLOAT32_ARRAY (geometry.m_uv, geometry.m_vertexCount * 2);
  		return 0;
	}

	dInt32 ReadGeometrySegmentBlockIndex (void* handle, const char* label)
	{
		dInt32 i;
		dGeometrySegment& geometry = m_currMesh->m_geometry->GetLast()->GetInfo();

		ReadData (handle, geometry.m_indexes);
//		geometry.m_indexes = SWAP_INT32(geometry.m_indexes);
		for (i = 0; i < geometry.m_indexCount; i ++) {
			geometry.m_indexes[i] = SWAP_INT16(geometry.m_indexes[i]);
		};

  		return 0;
	}

	dInt32 ReadGeometrySegmentBlockData(void* handle, const char* label)
	{
		//RegisterChunk ('COLR',	"dgMeshSegmentColorArrayBlock");
		//RegisterChunk ('UV_1',	"dgMeshSegmentUV1ArrayBlock");
		//RegisterChunk ('WGHT',	"dgMeshSegmentWeightsArrayBlock");
		RegisterChunk ('VERT',	(ChunkProcess)&dTGFileParcel::ReadGeometrySegmentBlockVertex, "dgMeshSegmentVertexArrayBlock");
		RegisterChunk ('NORM',	(ChunkProcess)&dTGFileParcel::ReadGeometrySegmentBlockNormals, "dgMeshSegmentNormalArrayBlock");
		RegisterChunk ('UV_0',	(ChunkProcess)&dTGFileParcel::ReadGeometrySegmentBlockUV_0, "dgMeshSegmentUV0ArrayBlock");
		RegisterChunk ('INDX',	(ChunkProcess)&dTGFileParcel::ReadGeometrySegmentBlockIndex, "dgMeshSegmentIndexArrayBlock");
		
		ParceChunk (handle);
		return 0;
	}




	dInt32 ReadGeometrySegment(void* handle, const char* label)
	{
		m_currMesh->m_geometry->Append ();
		RegisterChunk ('INFO', (ChunkProcess) &dTGFileParcel::ReadGeometrySegmentBlockInfo, "dgMeshSegmentInfoBlock");
		RegisterChunk ('GEOM', (ChunkProcess) &dTGFileParcel::ReadGeometrySegmentBlockData, "dgMeshSegmentPolygonsBlock");
		ParceChunk (handle);		
		return 0;
	}


	dInt32 ReadGeometryCRC(void* handle, const char* label)
	{
		ReadData (handle, &m_geometryCRC);
		m_geometryCRC = SWAP_INT32(m_geometryCRC);
		return 0;
	}

	dInt32 ReadGeometry(void* handle, const char* label)
	{
		dGeometryCache::dTreeNode* node;
		_ASSERTE (!m_currMesh->m_geometry);

		node = geometryCache.Find(m_geometryCRC);
		if (!node) {
			// this is a new mesh data it must be read and cache
			m_currMesh->m_geometry = new dGeometry (m_geometryCRC);
			RegisterChunk ('SEGM', (ChunkProcess) &dTGFileParcel::ReadGeometrySegment, "dgMeshdGeometrySegmentBlock");
			ParceChunk (handle);
			if (m_geometryCRC) {
				geometryCache.Insert(m_currMesh->m_geometry, m_geometryCRC);
				m_currMesh->m_geometry->m_refCount ++;
			}
		} else {
			// reuse mesh data
			m_currMesh->m_geometry = node->GetInfo();
			m_currMesh->m_geometry->m_refCount ++;
			ParceChunk (handle);
		}
		
		m_geometryCRC = 0;
		return 0;
	}


	dInt32 ParceMeshChunk(void* handle, const char* label)
	{
		m_currMesh = m_loaderContext->CreateNode ();		

		RegisterChunk ('NAME', (ChunkProcess) &dTGFileParcel::ReadMeshName, "dgNameBlock");
		RegisterChunk ('PRNT', (ChunkProcess) &dTGFileParcel::ReadParentName, "dgParentNameBlock");
		RegisterChunk ('TRNS', (ChunkProcess) &dTGFileParcel::ReadLocalTransformName, "dgMeshMatrix");
		RegisterChunk ('_CRC', (ChunkProcess) &dTGFileParcel::ReadGeometryCRC, "dgGeometryCRC");
		RegisterChunk ('GEOM', (ChunkProcess) &dTGFileParcel::ReadGeometry, "dgMeshGeometryBlock");
		//RegisterChunk ('MTYP', (ChunkProcess) &dTGFileParcel::ParceMeshChunk, "dgMeshType");
		//RegisterChunk ('HIDN', (ChunkProcess) &dTGFileParcel::ParceMeshChunk, "dgMeshHiddenFlag");

		ParceChunk (handle);
		if (!m_currMesh->GetParent()) {
			m_currMesh->Attach (m_rootNode);
		}
		return 0;
	}

	dInt32 ParceTextureName(void* handle, const char* label)
	{
		char name[256];

		ReadData (handle, name);
		m_textureList.AddTexture (name);
		return 0;
	}

	dInt32 ParceTextureListChunk(void* handle, const char* label)
	{
		RegisterChunk ('TNAM', (ChunkProcess) &dTGFileParcel::ParceTextureName, "dgTextureName");
		ParceChunk (handle);		
		return 0;
	}

	dInt32 ParceMaterial(void* handle, const char* label)
	{
		Material material;
		dInt32 materalIndex;

		ReadData (handle, &materalIndex);
		materalIndex = SWAP_INT32(materalIndex);
		if (materalIndex != -1) { 
			material.m_texture = m_textureList.GetTexture(materalIndex);
		} else {
			material.m_texture = dUnsigned32 (-1);
		}
		m_materialList.AddMaterial (material);

  		return 0;
	}

	dInt32 ParceMaterialListChunk(void* handle, const char* label)
	{
		RegisterChunk ('MATE', (ChunkProcess) &dTGFileParcel::ParceMaterial, "dgMaterialBlock");
		ParceChunk (handle);		
  		return 0;
	}


	// here is where all the model geometry is located
	dInt32 ParceModelGeometryChunk(void* handle, const char* label)
	{
		//RegisterChunk ('GBOX', "dgGlobalBBox");
		RegisterChunk ('TEXT', (ChunkProcess) &dTGFileParcel::ParceTextureListChunk, "dgTextureListBlock");
		RegisterChunk ('MESH', (ChunkProcess) &dTGFileParcel::ParceMeshChunk, "dgMeshBlock");
		RegisterChunk ('MTRL', (ChunkProcess) &dTGFileParcel::ParceMaterialListChunk, "dgdMaterialListBlock");
		ParceChunk (handle);
		return 0;
	}

	dInt32 ParceMainChunk(void* handle, const char* label)
	{
		//RegisterChunk ('COLL', (ChunkProcess) &dTGFileParcel::ParceMainChunk, "dgCollisionBlock");
		RegisterChunk ('MODL', (ChunkProcess) &dTGFileParcel::ParceModelGeometryChunk, "dgModelBlock");
		ParceChunk (handle);
		return 0;
	}

	dUnsigned32 m_geometryCRC;
	dSceneNode *m_currMesh;
	dSceneNode* m_rootNode;
	TextureList m_textureList;
	dMaterialList m_materialList;
	dLoaderContext* m_loaderContext;
};



dLoaderContext::dLoaderContext ()
{
}

dLoaderContext::~dLoaderContext ()
{
}


// used this to create a basic object model
dSceneNode* dLoaderContext::CreateNode() 
{
	return new dSceneNode (NULL);
}


// create and empty model
dSceneNode::dSceneNode(dSceneNode* parent)
	:dHierarchy<dSceneNode>(), m_matrix (GetIdentityMatrix())
{
	if (parent) {
		Attach (parent);
	}
	m_geometry = NULL;
}


dSceneNode::~dSceneNode()
{
	if (m_geometry) {
		m_geometry->m_refCount --;
		if (m_geometry->m_refCount == 1) {
			geometryCache.Remove(m_geometry->m_crc);
			delete m_geometry;
		}
	}
}


// load Model from file
void dSceneNode::LoadFromFile(const char *fileName, dLoaderContext& context)
{
	FILE * file;

	file = fopen (fileName, "rb"); 
	if (file) {
		dTGFileParcel parcel(file, this, &context);

		// make sure the matrices are correct
		SetMatrix (GetIdentityMatrix());
	}

}


void dSceneNode::ForEachNode (const dMatrix& globalMatrix, NodeCallback callback, void* userData) const
{
	dSceneNode *ptr;
	dMatrix matrix (m_matrix * globalMatrix);

	// this function will add function declared on the derive class to the base class 
	(this->*callback)(matrix, userData);

	// recurse down the entire hierarchy
	for (ptr = GetChild(); ptr; ptr = ptr->GetSibling()) {
		ptr->ForEachNode(matrix, callback, userData);
	}
}


dMatrix dSceneNode::CalcGlobalMatrix (dSceneNode* root) const
{
	dSceneNode* ptr;
	dMatrix matrix (m_matrix);

	for (ptr = GetParent(); ptr != root; ptr = ptr->GetParent()) {
		matrix = matrix * ptr->m_matrix;
	}
	return matrix;
}


const dGeometry* dSceneNode::GetGeometry() const
{
	return m_geometry;
}


void dSceneNode::Render()const
{
	dSceneNode* ptr;
	dList<dGeometrySegment>::dListNode *nodes;

	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);


	glPushMatrix();
	glMultMatrix(&m_matrix[0][0]);
	if (m_geometry) {
		for (nodes = m_geometry->GetFirst(); nodes; nodes = nodes->GetNext()) {
			dGeometrySegment& segment = nodes->GetInfo();

			if (segment.m_texture != 0xffffffff) {
				glEnable(GL_TEXTURE_2D);		
				glBindTexture(GL_TEXTURE_2D, segment.m_texture);
			} else {
				glDisable(GL_TEXTURE_2D);
			}

			glEnableClientState (GL_VERTEX_ARRAY);
			glEnableClientState (GL_NORMAL_ARRAY);
			glEnableClientState (GL_TEXTURE_COORD_ARRAY);
			glVertexPointer (3, GL_FLOAT, 0, segment.m_vertex);
			glNormalPointer (GL_FLOAT, 0, segment.m_normal);
			glTexCoordPointer (2, GL_FLOAT, 0, segment.m_uv);
			glDrawElements (GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_SHORT, segment.m_indexes);
		}
	}
	
	for (ptr = GetChild(); ptr; ptr = ptr->GetSibling()) {
		ptr->Render ();
	}
	glPopMatrix();
}
