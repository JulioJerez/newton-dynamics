/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "newtonStdafx.h"
#include "Newton.h"
#include "newtonWorld.h"
#include "newtonMaterial.h"
#include "newtonBodyNotify.h"


NewtonMesh* NewtonMeshCreateFromMesh(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const srcMesh = (dgMeshEffect*)mesh;
	//
	//dgMeshEffect* const clone = new (srcMesh->GetAllocator()) dgMeshEffect(*srcMesh);
	//return (NewtonMesh*)clone;
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshCreateConvexHull(const NewtonWorld* const newtonWorld, int count, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgStack<dgBigVector> pool(count);
	//
	//dgInt32 stride = strideInBytes / sizeof(dgFloat32);
	//for (dgInt32 i = 0; i < count; i++) {
	//	pool[i].m_x = vertexCloud[i * stride + 0];
	//	pool[i].m_y = vertexCloud[i * stride + 1];
	//	pool[i].m_z = vertexCloud[i * stride + 2];
	//	pool[i].m_w = dgFloat64(0.0);
	//}
	//dgMeshEffect* const mesh = new (world->dgWorld::GetAllocator()) dgMeshEffect(world->dgWorld::GetAllocator(), &pool[0].m_x, count, sizeof(dgBigVector), tolerance);
	//return (NewtonMesh*)mesh;
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshCreateTetrahedraIsoSurface(const NewtonMesh* const closeManifoldMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)closeManifoldMesh;
	//return (NewtonMesh*)meshEffect->CreateTetrahedraIsoSurface();
	ndAssert(0);
	return 0;
}

void NewtonCreateTetrahedraLinearBlendSkinWeightsChannel(const NewtonMesh* const tetrahedraMesh, NewtonMesh* const skinMesh)
{
	//dgAssert(0);
	//TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)skinMesh;
	//meshEffect->CreateTetrahedraLinearBlendSkinWeightsChannel((const dgMeshEffect*)tetrahedraMesh);
	ndAssert(0);
}

NewtonMesh* NewtonMeshCreateVoronoiConvexDecomposition(const NewtonWorld* const newtonWorld, int pointCount, const dFloat* const vertexCloud, int strideInBytes, int materialID, const dFloat* const textureMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonMesh*)dgMeshEffect::CreateVoronoiConvexDecomposition(world->dgWorld::GetAllocator(), pointCount, strideInBytes, vertexCloud, materialID, dgMatrix(textureMatrix));
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshCreateFromSerialization(const NewtonWorld* const newtonWorld, NewtonDeserializeCallback deserializeFunction, void* const serializeHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonMesh*)dgMeshEffect::CreateFromSerialization(world->dgWorld::GetAllocator(), (dgDeserialize)deserializeFunction, serializeHandle);
	ndAssert(0);
	return 0;
}

void NewtonMeshSerialize(const NewtonMesh* const mesh, NewtonSerializeCallback serializeFunction, void* const serializeHandle)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->Serialize((dgSerialize)serializeFunction, serializeHandle);
	ndAssert(0);
}

void NewtonMeshSaveOFF(const NewtonMesh* const mesh, const char* const filename)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgAssert(0);
	////dgMeshEffect* const meshEffect = (dgMeshEffect*) mesh;
	////meshEffect->SaveOFF(filename);
	ndAssert(0);
}

NewtonMesh* NewtonMeshLoadOFF(const NewtonWorld* const newtonWorld, const char* const filename)
{
	TRACE_FUNCTION(__FUNCTION__);
	////Newton* const world = (Newton *) newtonWorld;
	////dgMemoryAllocator* const allocator = world->dgWorld::GetAllocator();
	////dgMeshEffect* const mesh = new (allocator) dgMeshEffect (allocator);
	////mesh->LoadOffMesh(filename);
	////return (NewtonMesh*) mesh;
	//dgAssert(0);
	//return NULL;
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshLoadTetrahedraMesh(const NewtonWorld* const newtonWorld, const char* const filename)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//dgMemoryAllocator* const allocator = world->dgWorld::GetAllocator();
	//dgMeshEffect* const mesh = new (allocator) dgMeshEffect(allocator);
	//mesh->LoadTetraMesh(filename);
	//return (NewtonMesh*)mesh;
	ndAssert(0);
	return 0;
}

void NewtonMeshApplyTransform(const NewtonMesh* const mesh, const dFloat* const matrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//
	//meshEffect->ApplyTransform(dgMatrix(matrix));
	ndAssert(0);
}

void NewtonMeshFlipWinding(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->FlipWinding();
	ndAssert(0);
}

void NewtonMeshCalculateOOBB(const NewtonMesh* const mesh, dFloat* const matrix, dFloat* const x, dFloat* const y, dFloat* const z)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//
	//dgBigVector size;
	//dgMatrix alignMatrix(meshEffect->CalculateOOBB(size));
	//
	////	*((dgMatrix *)matrix) = alignMatrix; 
	//memcpy(matrix, &alignMatrix[0][0], sizeof(dgMatrix));
	//*x = dgFloat32(size.m_x);
	//*y = dgFloat32(size.m_y);
	//*z = dgFloat32(size.m_z);
	ndAssert(0);
}

void NewtonMeshCalculateVertexNormals(const NewtonMesh* const mesh, dFloat angleInRadians)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->CalculateNormals(angleInRadians);
	ndAssert(0);
}

void NewtonMeshApplyAngleBasedMapping(const NewtonMesh* const mesh, int material, NewtonReportProgress reportPrograssCallback, void* const reportPrgressUserData, dFloat* const aligmentMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMatrix matrix(aligmentMatrix);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->AngleBaseFlatteningMapping(material, (dgReportProgress)reportPrograssCallback, reportPrgressUserData);
	ndAssert(0);
}

void NewtonMeshApplySphericalMapping(const NewtonMesh* const mesh, int material, const dFloat* const aligmentMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMatrix matrix(aligmentMatrix);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->SphericalMapping(material, matrix);
	ndAssert(0);
}

void NewtonMeshApplyCylindricalMapping(const NewtonMesh* const mesh, int cylinderMaterial, int capMaterial, const dFloat* const aligmentMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMatrix matrix(aligmentMatrix);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->CylindricalMapping(cylinderMaterial, capMaterial, matrix);
	ndAssert(0);
}

void NewtonMeshTriangulate(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//((dgMeshEffect*)mesh)->Triangulate();
	ndAssert(0);
}

void NewtonMeshPolygonize(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//((dgMeshEffect*)mesh)->ConvertToPolygons();
	ndAssert(0);
}

int NewtonMeshIsOpenMesh(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);

	//return ((dgMeshEffect*)mesh)->HasOpenEdges() ? 1 : 0;
	ndAssert(0);
	return 0;
}

void NewtonMeshFixTJoints(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);

	//return ((dgMeshEffect*)mesh)->RepairTJoints();
	ndAssert(0);
}


void NewtonMeshClip(const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix, NewtonMesh** const topMesh, NewtonMesh** const bottomMesh)
{
	TRACE_FUNCTION(__FUNCTION__);

	//*topMesh = NULL;
	//*bottomMesh = NULL;
	//((dgMeshEffect*)mesh)->ClipMesh(dgMatrix(clipperMatrix), (dgMeshEffect*)clipper, (dgMeshEffect**)topMesh, (dgMeshEffect**)bottomMesh);
	ndAssert(0);
}


NewtonMesh* NewtonMeshSimplify(const NewtonMesh* const mesh, int maxVertexCount, NewtonReportProgress progressReportCallback, void* const reportPrgressUserData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return (NewtonMesh*)((dgMeshEffect*)mesh)->CreateSimplification(maxVertexCount, (dgReportProgress)progressReportCallback, reportPrgressUserData);
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshApproximateConvexDecomposition(const NewtonMesh* const mesh, dFloat maxConcavity, dFloat backFaceDistanceFactor, int maxCount, int maxVertexPerHull, NewtonReportProgress progressReportCallback, void* const reportProgressUserData)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return (NewtonMesh*)((dgMeshEffect*)mesh)->CreateConvexApproximation(maxConcavity, backFaceDistanceFactor, maxCount, maxVertexPerHull, (dgReportProgress)progressReportCallback, reportProgressUserData);
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshUnion(const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return (NewtonMesh*)((dgMeshEffect*)mesh)->Union(dgMatrix(clipperMatrix), (dgMeshEffect*)clipper);
	ndAssert(0);
	return 0;
}


NewtonMesh* NewtonMeshDifference(const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return (NewtonMesh*)((dgMeshEffect*)mesh)->Difference(dgMatrix(clipperMatrix), (dgMeshEffect*)clipper);
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshIntersection(const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return (NewtonMesh*)((dgMeshEffect*)mesh)->Intersection(dgMatrix(clipperMatrix), (dgMeshEffect*)clipper);
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshConvexMeshIntersection(const NewtonMesh* const mesh, const NewtonMesh* const convexMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return (NewtonMesh*)((dgMeshEffect*)mesh)->ConvexMeshIntersection((dgMeshEffect*)convexMesh);
	ndAssert(0);
	return 0;
}

void NewtonRemoveUnusedVertices(const NewtonMesh* const mesh, int* const vertexRemapTable)
{
	TRACE_FUNCTION(__FUNCTION__);
	//((dgMeshEffect*)mesh)->RemoveUnusedVertices(vertexRemapTable);
	ndAssert(0);
}


void NewtonMeshAddLayer(const NewtonMesh* const mesh, int layer)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->AddLayer(layer);
	ndAssert(0);
}

void NewtonMeshAddBinormal(const NewtonMesh* const mesh, dFloat x, dFloat y, dFloat z)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->AddBinormal(x, y, z);
	ndAssert(0);
}

void NewtonMeshAddUV0(const NewtonMesh* const mesh, dFloat u, dFloat v)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->AddUV0(u, v);
	ndAssert(0);
}

void NewtonMeshAddUV1(const NewtonMesh* const mesh, dFloat u, dFloat v)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->AddUV1(u, v);
	ndAssert(0);
}

void NewtonMeshAddVertexColor(const NewtonMesh* const mesh, dFloat32 r, dFloat32 g, dFloat32 b, dFloat32 a)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->AddVertexColor(r, g, b, a);
	ndAssert(0);
}


void NewtonMeshOptimizePoints(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->OptimizePoints();
	ndAssert(0);
}

void NewtonMeshOptimizeVertex(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->OptimizeAttibutes();
	ndAssert(0);
}

void NewtonMeshOptimize(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//NewtonMeshOptimizePoints(mesh);
	//NewtonMeshOptimizeVertex(mesh);
	ndAssert(0);
}

int NewtonMeshGetVertexCount(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->GetVertexCount();
	ndAssert(0);
	return 0;
}

int NewtonMeshGetVertexBaseCount(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->GetVertexBaseCount();
	ndAssert(0);
	return 0;
}

int NewtonMeshGetVertexStrideInByte(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//
	//return meshEffect->GetVertexStrideInByte();
	ndAssert(0);
	return 0;
}

const dFloat64* NewtonMeshGetVertexArray(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//
	//return meshEffect->GetVertexPool();
	ndAssert(0);
	return 0;
}

const int* NewtonMeshGetIndexToVertexMap(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->GetIndexToVertexMap();
	ndAssert(0);
	return 0;
}

int NewtonMeshHasNormalChannel(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->HasNormalChannel() ? 1 : 0;
	ndAssert(0);
	return 0;
}

int NewtonMeshHasBinormalChannel(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->HasBinormalChannel() ? 1 : 0;
	ndAssert(0);
	return 0;
}

int NewtonMeshHasUV0Channel(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->HasUV0Channel() ? 1 : 0;
	ndAssert(0);
	return 0;
}

int NewtonMeshHasUV1Channel(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->HasUV1Channel() ? 1 : 0;
	ndAssert(0);
	return 0;
}

int NewtonMeshHasVertexColorChannel(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//return meshEffect->HasVertexColorChannel() ? 1 : 0;
	ndAssert(0);
	return 0;
}

void NewtonMeshGetVertexDoubleChannel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat64* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->GetVertexChannel64(vertexStrideInByte, (dgFloat64*)outBuffer);
	ndAssert(0);
}

void NewtonMeshGetBinormalChannel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//meshEffect->GetBinormalChannel(vertexStrideInByte, (dgFloat32*)outBuffer);
	ndAssert(0);
}

void NewtonMeshMaterialGetIndexStreamShort(const NewtonMesh* const mesh, void* const handle, int materialId, short int* const index)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMeshEffect* const meshEffect = (dgMeshEffect*)mesh;
	//
	//meshEffect->GetMaterialGetIndexStreamShort((dgMeshEffect::dgIndexArray*)handle, materialId, index);
	ndAssert(0);
}


NewtonMesh* NewtonMeshCreateFirstSingleSegment(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgMeshEffect* const effectMesh = (dgMeshEffect*)mesh;
	//dgPolyhedra segment(effectMesh->GetAllocator());
	//
	//effectMesh->BeginConectedSurface();
	//if (effectMesh->GetConectedSurface(segment)) {
	//	dgMeshEffect* const solid = new (effectMesh->GetAllocator()) dgMeshEffect(segment, *((dgMeshEffect*)mesh));
	//	return (NewtonMesh*)solid;
	//}
	//else {
	//	return NULL;
	//}
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshCreateNextSingleSegment(const NewtonMesh* const mesh, const NewtonMesh* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgMeshEffect* const effectMesh = (dgMeshEffect*)mesh;
	//dgPolyhedra nextSegment(effectMesh->GetAllocator());
	//
	//dgAssert(segment);
	//dgInt32 moreSegments = effectMesh->GetConectedSurface(nextSegment);
	//
	//dgMeshEffect* solid;
	//if (moreSegments) {
	//	solid = new (effectMesh->GetAllocator()) dgMeshEffect(nextSegment, *effectMesh);
	//}
	//else {
	//	solid = NULL;
	//	effectMesh->EndConectedSurface();
	//}
	//
	//return (NewtonMesh*)solid;
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshCreateFirstLayer(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgMeshEffect* const effectMesh = (dgMeshEffect*)mesh;
	//return (NewtonMesh*)effectMesh->GetFirstLayer();
	ndAssert(0);
	return 0;
}

NewtonMesh* NewtonMeshCreateNextLayer(const NewtonMesh* const mesh, const NewtonMesh* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgMeshEffect* const effectMesh = (dgMeshEffect*)mesh;
	//return (NewtonMesh*)effectMesh->GetNextLayer((dgMeshEffect*)segment);
	ndAssert(0);
	return 0;
}



int NewtonMeshGetTotalFaceCount(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetTotalFaceCount();
	ndAssert(0);
	return 0;
}

int NewtonMeshGetTotalIndexCount(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetTotalIndexCount();
	ndAssert(0);
	return 0;
}

void NewtonMeshGetFaces(const NewtonMesh* const mesh, int* const faceIndexCount, int* const faceMaterial, void** const faceIndices)
{
	TRACE_FUNCTION(__FUNCTION__);
	//((dgMeshEffect*)mesh)->GetFaces(faceIndexCount, faceMaterial, faceIndices);
	ndAssert(0);
}


void* NewtonMeshGetFirstVertex(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetFirstVertex();
	ndAssert(0);
	return 0;
}

void* NewtonMeshGetNextVertex(const NewtonMesh* const mesh, const void* const vertex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetNextVertex(vertex);
	ndAssert(0);
	return 0;
}

int NewtonMeshGetVertexIndex(const NewtonMesh* const mesh, const void* const vertex)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetVertexIndex(vertex);
	ndAssert(0);
	return 0;
}

void* NewtonMeshGetFirstPoint(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetFirstPoint();
	ndAssert(0);
	return 0;
}

void* NewtonMeshGetNextPoint(const NewtonMesh* const mesh, const void* const point)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetNextPoint(point);
	ndAssert(0);
	return 0;
}

int NewtonMeshGetPointIndex(const NewtonMesh* const mesh, const void* const point)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetPointIndex(point);
	ndAssert(0);
	return 0;
}

int NewtonMeshGetVertexIndexFromPoint(const NewtonMesh* const mesh, const void* const point)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetVertexIndexFromPoint(point);
	ndAssert(0);
	return 0;
}

void* NewtonMeshGetFirstEdge(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetFirstEdge();
	ndAssert(0);
	return 0;
}

void* NewtonMeshGetNextEdge(const NewtonMesh* const mesh, const void* const edge)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetNextEdge(edge);
	ndAssert(0);
	return 0;
}

void NewtonMeshGetEdgeIndices(const NewtonMesh* const mesh, const void* const edge, int* const v0, int* const v1)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetEdgeIndex(edge, *v0, *v1);
	ndAssert(0);
}


//void NewtonMeshGetEdgePointIndices (const NewtonMesh* const mesh, const void* const edge, int* const v0, int* const v1)
//{
//	return ((dgMeshEffect*)mesh)->GetEdgeAttributeIndex (edge, *v0, *v1);
//}

void* NewtonMeshGetFirstFace(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetFirstFace();
	ndAssert(0);
	return 0;
}

void* NewtonMeshGetNextFace(const NewtonMesh* const mesh, const void* const face)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetNextFace(face);
	ndAssert(0);
	return 0;
}

int NewtonMeshIsFaceOpen(const NewtonMesh* const mesh, const void* const face)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->IsFaceOpen(face);
	ndAssert(0);
	return 0;
}

int NewtonMeshGetFaceIndexCount(const NewtonMesh* const mesh, const void* const face)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetFaceIndexCount(face);
	ndAssert(0);
	return 0;
}

int NewtonMeshGetFaceMaterial(const NewtonMesh* const mesh, const void* const face)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->GetFaceMaterial(face);
	ndAssert(0);
	return 0;
}

void NewtonMeshSetFaceMaterial(const NewtonMesh* const mesh, const void* const face, int matId)
{
	TRACE_FUNCTION(__FUNCTION__);
	//return ((dgMeshEffect*)mesh)->SetFaceMaterial(face, matId);
	ndAssert(0);
}

void NewtonMeshGetFaceIndices(const NewtonMesh* const mesh, const void* const face, int* const indices)
{
	TRACE_FUNCTION(__FUNCTION__);
	//((dgMeshEffect*)mesh)->GetFaceIndex(face, indices);
	ndAssert(0);
}

void NewtonMeshGetFacePointIndices(const NewtonMesh* const mesh, const void* const face, int* const indices)
{
	TRACE_FUNCTION(__FUNCTION__);
	//((dgMeshEffect*)mesh)->GetFaceAttributeIndex(face, indices);
	ndAssert(0);
}

void NewtonMeshCalculateFaceNormal(const NewtonMesh* const mesh, const void* const face, dFloat64* const normal)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgBigVector n(((dgMeshEffect*)mesh)->CalculateFaceNormal(face));
	//normal[0] = n.m_x;
	//normal[1] = n.m_y;
	//normal[2] = n.m_z;
	ndAssert(0);
}

NewtonCollision* NewtonCreateDeformableSolid(const NewtonWorld* const newtonWorld, const NewtonMesh* const mesh, int shapeID)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonCollision*)world->CreateDeformableSolid((dgMeshEffect*)mesh, shapeID);
	ndAssert(0);
	return 0;
}


int NewtonDeformableMeshGetParticleCount(const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)deformableMesh;
	//if (collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
	//	dgCollisionLumpedMassParticles* const deformableShape = (dgCollisionLumpedMassParticles*)collision->GetChildShape();
	//	return deformableShape->GetCount();
	//}
	//return 0;
	ndAssert(0);
	return 0;
}


const dFloat* NewtonDeformableMeshGetParticleArray(const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)deformableMesh;
	//if (collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
	//	dgCollisionLumpedMassParticles* const deformableShape = (dgCollisionLumpedMassParticles*)collision->GetChildShape();
	//	const dgVector* const posit = deformableShape->GetPositions();
	//	return &posit[0].m_x;
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}


int NewtonDeformableMeshGetParticleStrideInBytes(const NewtonCollision* const deformableMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)deformableMesh;
	//if (collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
	//	dgCollisionLumpedMassParticles* const deformableShape = (dgCollisionLumpedMassParticles*)collision->GetChildShape();
	//	return deformableShape->GetStrideInByte();
	//}
	//return 0;
	ndAssert(0);
	return 0;
}

NewtonCollision* NewtonCreateFracturedCompoundCollision(const NewtonWorld* const newtonWorld, const NewtonMesh* const solidMesh, int shapeID, int fracturePhysicsMaterialID, int pointcloudCount, const dFloat* const vertexCloud, int strideInBytes, int materialID, const dFloat* const textureMatrix,
	NewtonFractureCompoundCollisionReconstructMainMeshCallBack regenerateMainMeshCallback,
	NewtonFractureCompoundCollisionOnEmitCompoundFractured emitFracturedCompound, NewtonFractureCompoundCollisionOnEmitChunk emitFracfuredChunk)
{
	TRACE_FUNCTION(__FUNCTION__);

	//Newton* const world = (Newton*)newtonWorld;
	//dgMeshEffect* const mesh = (dgMeshEffect*)solidMesh;
	//
	//dgMatrix textMatrix(textureMatrix);
	//dgCollisionInstance* const collision = world->CreateFracturedCompound(mesh, shapeID, fracturePhysicsMaterialID, pointcloudCount, vertexCloud, strideInBytes, materialID, textMatrix,
	//	(dgCollisionCompoundFractured::OnEmitFractureChunkCallBack)emitFracfuredChunk,
	//	(dgCollisionCompoundFractured::OnEmitNewCompundFractureCallBack)emitFracturedCompound,
	//	(dgCollisionCompoundFractured::OnReconstructFractureMainMeshCallBack)regenerateMainMeshCallback);
	//return (NewtonCollision*)collision;
	ndAssert(0);
	return 0;
}

NewtonCollision* NewtonFracturedCompoundPlaneClip(const NewtonCollision* const fracturedCompound, const dFloat* const plane)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	dgWorld* const world = (dgWorld*)collision->GetWorld();
	//	dgCollisionCompoundFractured* const newCompound = compound->PlaneClip(dgVector(plane[0], plane[1], plane[2], plane[3]));
	//	if (newCompound) {
	//		dgCollisionInstance* const newCollision = world->CreateInstance(newCompound, int(collision->GetUserDataID()), dgGetIdentityMatrix());
	//		newCompound->Release();
	//		return (NewtonCollision*)newCollision;
	//	}
	//}
	//return NULL;
	ndAssert(0);
	return 0;
}

void NewtonFracturedCompoundSetCallbacks(const NewtonCollision* const fracturedCompound,
	NewtonFractureCompoundCollisionReconstructMainMeshCallBack regenerateMainMeshCallback,
	NewtonFractureCompoundCollisionOnEmitCompoundFractured emitFracturedCompound, NewtonFractureCompoundCollisionOnEmitChunk emitFracfuredChunk)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	compound->SetCallbacks((dgCollisionCompoundFractured::OnEmitFractureChunkCallBack)emitFracfuredChunk, (dgCollisionCompoundFractured::OnEmitNewCompundFractureCallBack)emitFracturedCompound, (dgCollisionCompoundFractured::OnReconstructFractureMainMeshCallBack)regenerateMainMeshCallback);
	//}
	ndAssert(0);
}


int NewtonFracturedCompoundNeighborNodeList(const NewtonCollision* const fracturedCompound, void* const collisionNode, void** const nodesArray, int maxCount)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	return  compound->GetFirstNiegborghArray((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode, (dgCollisionCompound::dgTreeArray::dgTreeNode**)nodesArray, maxCount);
	//}
	//return 0;
	ndAssert(0);
	return 0;
}



int NewtonFracturedCompoundIsNodeFreeToDetach(const NewtonCollision* const fracturedCompound, void* const collisionNode)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	return compound->IsNodeSaseToDetach((dgCollisionCompound::dgTreeArray::dgTreeNode*)collisionNode) ? 1 : 0;
	//}
	//return 0;
	ndAssert(0);
	return 0;
}

NewtonFracturedCompoundMeshPart* NewtonFracturedCompoundGetFirstSubMesh(const NewtonCollision* const fracturedCompound)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//NewtonFracturedCompoundMeshPart* mesh = NULL;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	mesh = (NewtonFracturedCompoundMeshPart*)compound->GetFirstMesh();
	//}
	//return mesh;
	ndAssert(0);
	return 0;
}

NewtonFracturedCompoundMeshPart* NewtonFracturedCompoundGetNextSubMesh(const NewtonCollision* const fracturedCompound, NewtonFracturedCompoundMeshPart* const subMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//NewtonFracturedCompoundMeshPart* mesh = NULL;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	mesh = (NewtonFracturedCompoundMeshPart*)compound->GetNextMesh((dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)subMesh);
	//}
	//return mesh;
	ndAssert(0);
	return 0;
}

NewtonFracturedCompoundMeshPart* NewtonFracturedCompoundGetMainMesh(const NewtonCollision* const fracturedCompound)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//NewtonFracturedCompoundMeshPart* mesh = NULL;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	mesh = (NewtonFracturedCompoundMeshPart*)compound->GetMainMesh();
	//}
	//return mesh;
	ndAssert(0);
	return 0;
}


int NewtonFracturedCompoundCollisionGetVertexCount(const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//dgInt32 count = 0;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	count = compound->GetVertecCount((dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)meshOwner);
	//}
	//return count;
	ndAssert(0);
	return 0;
}


const dFloat* NewtonFracturedCompoundCollisionGetVertexPositions(const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//const dgFloat32* points = NULL;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	points = compound->GetVertexPositions((dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)meshOwner);
	//}
	//return points;
	ndAssert(0);
	return 0;
}


const dFloat* NewtonFracturedCompoundCollisionGetVertexNormals(const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//const dgFloat32* points = NULL;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	points = compound->GetVertexNormal((dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)meshOwner);
	//}
	//return points;
	ndAssert(0);
	return 0;
}

const dFloat* NewtonFracturedCompoundCollisionGetVertexUVs(const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//
	//const dgFloat32* points = NULL;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision->GetChildShape();
	//	points = compound->GetVertexUVs((dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)meshOwner);
	//}
	//return points;
	ndAssert(0);
	return 0;
}

int NewtonFracturedCompoundMeshPartGetIndexStream(const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner, const void* const segment, int* const index)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgInt32 count = 0;
	//dgCollisionInstance* const collision = (dgCollisionInstance*)fracturedCompound;
	//if (collision->IsType(dgCollision::dgCollisionCompoundBreakable_RTTI)) {
	//	dgCollisionCompoundFractured* const compound = (dgCollisionCompoundFractured*)collision;
	//	count = compound->GetSegmentIndexStream((dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)meshOwner, (dgCollisionCompoundFractured::dgMesh::dgListNode*)segment, index);
	//}
	//return count;
	ndAssert(0);
	return 0;
}


void* NewtonFracturedCompoundMeshPartGetFirstSegment(const NewtonFracturedCompoundMeshPart* const breakableComponentMesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionCompoundFractured::dgConectivityGraph::dgListNode* const node = (dgCollisionCompoundFractured::dgConectivityGraph::dgListNode*)breakableComponentMesh;
	//return node->GetInfo().m_nodeData.m_mesh->GetFirst();
	ndAssert(0);
	return 0;
}

void* NewtonFracturedCompoundMeshPartGetNextSegment(const void* const breakableComponentSegment)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgCollisionCompoundFractured::dgMesh::dgListNode* const node = (dgCollisionCompoundFractured::dgMesh::dgListNode*)breakableComponentSegment;
	//return node->GetNext();
	ndAssert(0);
	return 0;
}

int NewtonFracturedCompoundMeshPartGetMaterial(const void* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgCollisionCompoundFractured::dgMesh::dgListNode* const node = (dgCollisionCompoundFractured::dgMesh::dgListNode*)segment;
	//return node->GetInfo().m_material;
	ndAssert(0);
	return 0;
}


int NewtonFracturedCompoundMeshPartGetIndexCount(const void* const segment)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgCollisionCompoundFractured::dgMesh::dgListNode* const node = (dgCollisionCompoundFractured::dgMesh::dgListNode*)segment;
	//return node->GetInfo().m_faceCount * 3;
	ndAssert(0);
	return 0;
}


static ndMeshEffect::ndMeshVertexFormat ConvertFormat(const NewtonMeshVertexFormat* const data)
{
	ndMeshEffect::ndMeshVertexFormat format;

	format.m_faceCount = data->m_faceCount;
	format.m_faceMaterial = data->m_faceMaterial;
	format.m_faceIndexCount = data->m_faceIndexCount;

	format.m_vertex.m_data = data->m_vertex.m_data;
	format.m_vertex.m_indexList = data->m_vertex.m_indexList;
	format.m_vertex.m_strideInBytes = data->m_vertex.m_strideInBytes;

	if (data->m_normal.m_data)
	{
		format.m_normal.m_data = data->m_normal.m_data;
		format.m_normal.m_indexList = data->m_normal.m_indexList;
		format.m_normal.m_strideInBytes = data->m_normal.m_strideInBytes;
	}

	if (data->m_binormal.m_data)
	{
		format.m_binormal.m_data = data->m_binormal.m_data;
		format.m_binormal.m_indexList = data->m_binormal.m_indexList;
		format.m_binormal.m_strideInBytes = data->m_binormal.m_strideInBytes;
	}

	if (data->m_uv0.m_data)
	{
		format.m_uv0.m_data = data->m_uv0.m_data;
		format.m_uv0.m_indexList = data->m_uv0.m_indexList;
		format.m_uv0.m_strideInBytes = data->m_uv0.m_strideInBytes;
	}

	if (data->m_uv1.m_data)
	{
		format.m_uv1.m_data = data->m_uv1.m_data;
		format.m_uv1.m_indexList = data->m_uv1.m_indexList;
		format.m_uv1.m_strideInBytes = data->m_uv1.m_strideInBytes;
	}

	if (data->m_vertexColor.m_data)
	{
		format.m_vertexColor.m_data = data->m_vertexColor.m_data;
		format.m_vertexColor.m_indexList = data->m_vertexColor.m_indexList;
		format.m_vertexColor.m_strideInBytes = data->m_vertexColor.m_strideInBytes;
	}

	return format;
}

NewtonMesh* NewtonMeshCreate(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndSharedPtr<ndMeshEffect>* const mesh = new ndSharedPtr<ndMeshEffect>(new ndMeshEffect());
	return reinterpret_cast<NewtonMesh*>(mesh);
}

NewtonMesh* NewtonMeshCreateFromCollision(const NewtonCollision* const collision)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndShapeInstance* const instance = const_cast<ndShapeInstance*>(reinterpret_cast<const ndShapeInstance*>(collision));
	ndSharedPtr<ndMeshEffect>* const mesh = new ndSharedPtr<ndMeshEffect>(new ndMeshEffect(*instance));
	return reinterpret_cast<NewtonMesh*>(mesh);
}

void NewtonMeshDestroy(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndSharedPtr<ndMeshEffect>* const instance(SharedObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh));
	delete instance;
}

void NewtonMeshBeginBuild(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->BeginBuild();
}

void NewtonMeshBeginFace(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->BeginBuildFace();
}

void NewtonMeshAddPoint(const NewtonMesh* const mesh, dFloat64 x, dFloat64 y, dFloat64 z)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->AddPoint(x, y, z);
}

void NewtonMeshAddNormal(const NewtonMesh* const mesh, dFloat x, dFloat y, dFloat z)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->AddNormal(x, y, z);
}

void NewtonMeshAddMaterial(const NewtonMesh* const mesh, int materialIndex)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->AddMaterial(materialIndex);
}

void NewtonMeshEndFace(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->EndBuildFace();
}

void NewtonMeshEndBuild(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->EndBuild(false);
}

void NewtonMeshClearVertexFormat(NewtonMeshVertexFormat* const format)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMemSet(reinterpret_cast<ndInt8*>(format), ndInt8(0), sizeof(NewtonMeshVertexFormat));
}

void NewtonMeshBuildFromVertexListIndexList(const NewtonMesh* const mesh, const NewtonMeshVertexFormat* const format)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const instance = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	ndMeshEffect::ndMeshVertexFormat meshFormat(ConvertFormat(format));
	instance->BuildFromIndexList(&meshFormat);
}

void NewtonMeshSetVertexBaseCount(const NewtonMesh* const mesh, int baseCount)
{
	TRACE_FUNCTION(__FUNCTION__);
	// do nothing
}

int NewtonMeshGetPointCount(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	return meshEffect->GetPropertiesCount();
}

void NewtonMeshGetVertexChannel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->GetVertexChannel(vertexStrideInByte, (ndFloat32*)outBuffer);
}

void NewtonMeshGetNormalChannel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->GetNormalChannel(vertexStrideInByte, (ndFloat32*)outBuffer);
}

void NewtonMeshGetUV0Channel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->GetUV0Channel(vertexStrideInByte, (ndFloat32*)outBuffer);
}

void NewtonMeshGetUV1Channel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->GetUV1Channel(vertexStrideInByte, (ndFloat32*)outBuffer);
}

void NewtonMeshGetVertexColorChannel(const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->GetVertexColorChannel(vertexStrideInByte, (ndFloat32*)outBuffer);
}

void* NewtonMeshBeginHandle(const NewtonMesh* const mesh)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	return meshEffect->MaterialGeometryBegin();
}

void NewtonMeshEndHandle(const NewtonMesh* const mesh, void* const handle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	ndIndexArray* const indexArray = reinterpret_cast<ndIndexArray*> (handle);
	meshEffect->MaterialGeometryEnd(indexArray);
}

int NewtonMeshNextMaterial(const NewtonMesh* const mesh, void* const handle, int materialId)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	
	ndIndexArray* const indexArray = reinterpret_cast<ndIndexArray*> (handle);
	return meshEffect->GetNextMaterial(indexArray, materialId);
}

int NewtonMeshFirstMaterial(const NewtonMesh* const mesh, void* const handle)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);

	ndIndexArray* const indexArray = reinterpret_cast<ndIndexArray*> (handle);
	return meshEffect->GetFirstMaterial(indexArray);
}

int NewtonMeshMaterialGetMaterial(const NewtonMesh* const mesh, void* const handle, int materialId)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	
	ndIndexArray* const indexArray = reinterpret_cast<ndIndexArray*> (handle);
	return  meshEffect->GetMaterialID(indexArray, materialId);
}

int NewtonMeshMaterialGetIndexCount(const NewtonMesh* const mesh, void* const handle, int materialId)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	
	ndIndexArray* const indexArray = reinterpret_cast<ndIndexArray*> (handle);
	return meshEffect->GetMaterialIndexCount(indexArray, materialId);
}

void NewtonMeshMaterialGetIndexStream(const NewtonMesh* const mesh, void* const handle, int materialId, int* const index)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	
	ndIndexArray* const indexArray = reinterpret_cast<ndIndexArray*> (handle);
	meshEffect->GetMaterialGetIndexStream(indexArray, materialId, index);
}


void NewtonMeshApplyBoxMapping(const NewtonMesh* const mesh, int front, int side, int top, const dFloat* const aligmentMatrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndMatrix matrix(aligmentMatrix);
	if (!CheckFloat(&matrix[0][0], 16))
	{
		ndExpandTraceMessage(("uninitialized matrix, setting to identity\n"));
		matrix = ndGetIdentityMatrix();
	}

	ndMeshEffect* const meshEffect = ObjectFromHandle<ndMeshEffect, NewtonMesh>(mesh);
	meshEffect->BoxMapping(front, side, top, matrix);
}
