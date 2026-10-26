//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// LevelPrimitive.cpp: implementation of the LevelPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "OpenGlUtil.h"
#include "LevelPrimitive.h"

/*
class PlaneCollision
{
	public:
	PlaneCollision(NewtonWorld* nWorld, const dVector& plane)
	{
		int j;
		dVector m_minBox (-1000.0f, -1000.0f, -1000.0f);
		dVector m_maxBox ( 1000.0f,  1000.0f,  1000.0f);

		// get the transformation matrix that takes the plane to the world local space
		m_rotation = dgGrammSchmidt(plane);

		// build a unit grid in local space (this will be the shadow at projection of the collision aabb)
		m_sphape[0] = dVector (0.0f,  1.0f,  1.0f);
		m_sphape[1] = dVector (0.0f, -1.0f,  1.0f);
		m_sphape[2] = dVector (0.0f, -1.0f, -1.0f);
		m_sphape[3] = dVector (0.0f,  1.0f, -1.0f);

		// save the plane
		m_plane = plane;

		
		m_attribute = 0;
		m_faceIndices = 4;
		for (j = 0; j < 4; j ++) {
			m_indexArray[j] = j;
		}
	

		// create a Newton user collision 
		m_collision = NewtonCreateUserMeshCollision (nWorld, &m_minBox[0], &m_maxBox[0], this, 
			PlaneCollisionCollideCallback, PlaneMeshCollisionRayHitCallback, 
			PlaneCollisionDestroyCallback);
	}
	~PlaneCollision(void)
	{
	}

	static void PlaneCollisionDestroyCallback(void* userData)
	{
		PlaneCollision* me;
		me = (PlaneCollision*) userData;
		delete me;
	}

	static void PlaneCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* collideDesc)
	{
		float t;
		float s;
		dInt32 i;
		dFloat dist;
		PlaneCollision* me;
		me = (PlaneCollision*) collideDesc->m_userData;

		// get bounding bos of teh collidong objects
		dVector p0 (collideDesc->m_boxP0[0], collideDesc->m_boxP0[1], collideDesc->m_boxP0[2]);
		dVector p1 (collideDesc->m_boxP1[0], collideDesc->m_boxP1[1], collideDesc->m_boxP1[2]);

		// if the support vetexs of the aabb is not intesection the plane teh there is not collision
		dVector suportVertex ((me->m_plane.m_x > 0.0f) ? p0.m_x : p1.m_x, 
			(me->m_plane.m_y > 0.0f) ? p0.m_y : p1.m_y,
			(me->m_plane.m_z > 0.0f) ? p0.m_z : p1.m_z);


		dist = me->m_plane % suportVertex + me->m_plane.m_w;

		// if support the lower support vertex is close enought to the plane the report 
		// a single grid on eteh plane below the AABB and porpostional of the size of the AABB  
		if (dist < 0.25f) {
			// calculate the aabb center
			dVector centre ((p1 + p0).Scale (0.5f));

			//find the projection of center point over the plane
			t = - (me->m_plane % centre + me->m_plane.m_w);
			centre += me->m_plane.Scale (t);

			//know calculate the scale factor
			dVector size (p1 - p0);
			s = max (size.m_x, max (size.m_y, size.m_z));


			// initialize the callback data structure
			collideDesc->m_faceCount = 1;
			collideDesc->m_vertexStrideInBytes = sizeof (dVector);
			collideDesc->m_userAttribute = &me->m_attribute;
			collideDesc->m_faceIndexCount = &me->m_faceIndices;
			collideDesc->m_faceVertexIndex = &me->m_indexArray[0];
			collideDesc->m_vertex = &me->m_collisionVertex[0][0];
			for (i = 0; i < 4; i ++) {
				me->m_collisionVertex[i] = centre + me->m_rotation.RotateVector(me->m_sphape[i].Scale (s));
			}


			if (debugLinesMode) {
				dInt32 i;
				if (hightMap_debugCount < (MAX_DEBUG_CONTACTS - 256)) {
					dVector p0 (me->m_collisionVertex[threadNumber][3]);
					for (i = 0; i < 4; i ++) {
						hightMap_debugLines[hightMap_debugCount].p0 = p0;
						hightMap_debugLines[hightMap_debugCount].p1 = dVector (me->m_collisionVertex[threadNumber][i]);
						hightMap_debugLines[hightMap_debugCount].isContact = 0;
						p0 = hightMap_debugLines[hightMap_debugCount].p1;
						hightMap_debugCount ++;
					}
				}
			}

		}
	}

	static dFloat PlaneMeshCollisionRayHitCallback (NewtonUserMeshCollisionRayHitDesc* rayDesc)
	{
		dFloat t;
		PlaneCollision* me;

		dVector q0 (rayDesc->m_p0[0], rayDesc->m_p0[1], rayDesc->m_p0[2]);
		dVector q1 (rayDesc->m_p1[0], rayDesc->m_p1[1], rayDesc->m_p1[2]);
		dVector dq (q1 - q0);

		// calculate intersection between point lien a plane and return intesetion parameter
		me = (PlaneCollision*) rayDesc->m_userData;
		t = -(me->m_plane % q0 + me->m_plane.m_w) / (me->m_plane % dq);
		if ((t > 0.0f) && (t < 1.0f)) {
			rayDesc->m_normalOut[0] = me->m_plane[0];
			rayDesc->m_normalOut[1] = me->m_plane[1];
			rayDesc->m_normalOut[2] = me->m_plane[2];
		} else {
			t = -1.0f;
		}

		return t;
	}


	static void PlaneCollisionGetFacesInAABB (
		void* me, 
		const dFloat* p0, 
		const dFloat* p1,
		const dFloat** vertexArray, 
		int* vertexCount, 
		int* vertexStrideInBytes, 
		const int* indexList, 
		int maxIndexCount)
	{
		_ASSERTE (0);
	}


	dVector m_sphape[4];
	dVector m_plane;

	dMatrix m_rotation;
	NewtonCollision* m_collision;

	dInt32 m_attribute;
	dInt32 m_faceIndices;
	dInt32 m_indexArray[4];
	dVector m_collisionVertex[4];
};

NewtonCollision* CreatePlaneCollidion (NewtonWorld* nWorld, const dVector& Plane)
{
	PlaneCollision* plane;

	plane = new PlaneCollision (nWorld, Plane);
	return plane->m_collision;
}
*/



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define SERIALIZE_MESH

#ifdef SERIALIZE_MESH
void SerializeFile (void* serializeHandle, const void* buffer, size_t size)
{
	fwrite (buffer, size, 1, (FILE*) serializeHandle);
}

void DeSerializeFile (void* serializeHandle, void* buffer, size_t size)
{
	fread (buffer, size, 1, (FILE*) serializeHandle);
}

#endif


LevelPrimitive::LevelPrimitive(
	dSceneNode* parent,
    const char* name, 
    NewtonWorld* nWorld, 
	NewtonTreeCollisionCallback levelCallback)
	:dSceneNode(parent)
{
	NewtonCollision* collision;
	
	// open the level data
	char fullPathName[256];
	GetWorkingFileName (name, fullPathName);

	dLoaderContext context;
	LoadFromFile(fullPathName, context);
  
	// create the collision tree geometry
	collision = NewtonCreateTreeCollision(nWorld, levelCallback);
  
	// prepare to create collision geometry
	NewtonTreeCollisionBeginBuild(collision);

	// iterate the entire geometry an build the collision
	dMatrix mat(GetIdentityMatrix());
	ForEachNode (mat, (NodeCallback)&LevelPrimitive::BuildCollision, collision);

	// finalize the collision tree build
	NewtonTreeCollisionEndBuild(collision, 1);
//	NewtonTreeCollisionEndBuild(collision, 0);


	int xxx;
	dVector p0(-100, -100, -100);
	dVector p1(100, 100, 100);
	const dFloat* vertexArray;
	int vertexStrideInBytes;
	int vertexCount;
	int indexList[256];
	xxx = NewtonTreeCollisionGetVertexListIndexListInAABB (collision, &p0[0], &p1[0], 
		&vertexArray, &vertexCount, &vertexStrideInBytes, 
		indexList, sizeof (indexList)/sizeof (indexList[0])); 

//	NewtonCollisionApplyScale(collision, 2.0f);

#ifdef SERIALIZE_MESH
	FILE* file;
	strtok (fullPathName, ".");
	strcat (fullPathName, ".bin");

	// save the collision file
	file = fopen (fullPathName, "wb");
	NewtonTreeCollisionSerialize (collision, SerializeFile, file);
	fclose (file);

	// load the collision file
	NewtonReleaseCollision (nWorld, collision);

	file = fopen (fullPathName, "rb");
	collision = NewtonCreateTreeCollisionFromSerialization (nWorld, levelCallback, DeSerializeFile, file);
	fclose (file);

#endif

//dVector plane (0, 1, 0, 0);
//collision = CreatePlaneCollidion (nWorld, plane);

	// create the level rigid body
	m_level = NewtonCreateBody(nWorld, collision);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonReleaseCollision (nWorld, collision);

	// set the global position of this body
	NewtonBodySetMatrix (m_level, &m_matrix[0][0]); 


	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (m_level, this);


	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (m_level, Destructor);

	
	dVector boxP0; 
	dVector boxP1; 
	// get the position of the aabb of this geometry
	NewtonCollisionCalculateAABB (collision, &m_matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// add some extra padding the world size
	boxP0.m_x -= 50.0f;
	boxP0.m_y -= 50.0f;
	boxP0.m_z -= 50.0f;
	boxP1.m_x += 50.0f;
	boxP1.m_y += 50.0f;
	boxP1.m_z += 50.0f;

	// set the world size
	NewtonSetWorldSize (nWorld, &boxP0.m_x, &boxP1.m_x); 



#if 0
	int count;
	NewtonCollision* collisionB;
	collisionB = NewtonCreateBox (nWorld, 3.5f, 3.5f, 3.5f, NULL);

	dMatrix matrixA (GetIdentityMatrix());
	dMatrix matrixB (GetIdentityMatrix());
	matrixA.m_posit.m_y += 10.0f;
	matrixB.m_posit.m_z += 0.0f;

	dVector contacts[20];
	dVector normals[20];
	dFloat depths[20];
	count = NewtonCollisionCollide (nWorld, 10, 
									collision, &matrixA[0][0], 
									collisionB, &matrixB[0][0],
									&contacts[0][0], &normals[0][0], &depths[0]);
#endif
}

LevelPrimitive::~LevelPrimitive()
{
}

// rigid body destructor
void LevelPrimitive::Destructor (const NewtonBody* body)
{
	LevelPrimitive* level;

	// get the graphic object form the rigid body
	level = (LevelPrimitive*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete level;
}


void LevelPrimitive::BuildCollision (const dMatrix& globalMatrix,  NewtonCollision* collision) const
{
	dInt32 i;
	dInt32 index;
	dList<dGeometrySegment>::dListNode *nodes;

	if (m_geometry) {
		for (nodes = m_geometry->GetFirst(); nodes; nodes = nodes->GetNext()) {
			dGeometrySegment& segment = nodes->GetInfo();
			for (i = 0; i < segment.m_indexCount; i += 3) {
				dVector face[3];

				index = segment.m_indexes[i + 0] * 3;
				face[0] = dVector (segment.m_vertex[index + 0], segment.m_vertex[index + 1], segment.m_vertex[index + 2]);

				index = segment.m_indexes[i + 1] * 3;
				face[1] = dVector (segment.m_vertex[index + 0], segment.m_vertex[index + 1], segment.m_vertex[index + 2]);

				index = segment.m_indexes[i + 2] * 3;
				face[2] = dVector (segment.m_vertex[index + 0], segment.m_vertex[index + 1], segment.m_vertex[index + 2]);

				globalMatrix.TransformTriplex (face, sizeof (dVector), face, sizeof (dVector), 3);
				NewtonTreeCollisionAddFace(collision, 3, &face[0].m_x, sizeof (dVector), 1);
			}
		}
	}
}

