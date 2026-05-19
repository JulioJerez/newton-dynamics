
/* Copyright (c) <2003-2019> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// Regression test: a dynamic body inside a concave static mesh (inverted box
// with inward-facing normals) triggers a debug assertion in
// ndShapeConvexPolygon::GenerateConvexCap() at the dot-product threshold check.

#include <ndNewtonInc.h>
#include <gtest/gtest.h>

// Build a static body whose collision is an inverted box (normals point inward).
// The box spans [-5, +5] on each axis, made of 12 triangles (2 per face).
static ndBodyKinematic* BuildInvertedBoxMesh(const ndVector& pos)
{
	ndBodyKinematic* const body = new ndBodyKinematic();
	body->SetNotifyCallback(new ndBodyNotify(ndVector::m_zero));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = pos;
	body->SetMatrix(matrix);

	// 8 vertices of a 10x10x10 box centered at origin
	const ndFloat32 s = ndFloat32(5.0f);
	ndVector verts[8] =
	{
		ndVector(-s, -s, -s, ndFloat32(0.0f)), // 0: left-bottom-front
		ndVector( s, -s, -s, ndFloat32(0.0f)), // 1: right-bottom-front
		ndVector( s,  s, -s, ndFloat32(0.0f)), // 2: right-top-front
		ndVector(-s,  s, -s, ndFloat32(0.0f)), // 3: left-top-front
		ndVector(-s, -s,  s, ndFloat32(0.0f)), // 4: left-bottom-back
		ndVector( s, -s,  s, ndFloat32(0.0f)), // 5: right-bottom-back
		ndVector( s,  s,  s, ndFloat32(0.0f)), // 6: right-top-back
		ndVector(-s,  s,  s, ndFloat32(0.0f)), // 7: left-top-back
	};

	// 12 triangles, wound so that normals point INWARD.
	// Each face has 2 triangles. Winding verified by cross-product.
	const int indices[12][3] =
	{
		// Bottom face (y = -5), normal = (0, +1, 0)
		{0, 4, 1},
		{4, 5, 1},
		// Top face (y = +5), normal = (0, -1, 0)
		{3, 2, 7},
		{2, 6, 7},
		// Front face (z = -5), normal = (0, 0, +1)
		{0, 1, 3},
		{1, 2, 3},
		// Back face (z = +5), normal = (0, 0, -1)
		{4, 7, 5},
		{7, 6, 5},
		// Left face (x = -5), normal = (+1, 0, 0)
		{0, 3, 4},
		{3, 7, 4},
		// Right face (x = +5), normal = (-1, 0, 0)
		{1, 5, 2},
		{5, 6, 2},
	};

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	for (int i = 0; i < 12; ++i)
	{
		ndVector triangle[3];
		triangle[0] = verts[indices[i][0]];
		triangle[1] = verts[indices[i][1]];
		triangle[2] = verts[indices[i][2]];

		ndInt32 materialId = 0;
		meshBuilder.AddFace(triangle, 3, materialId);
	}

	// End with optimize=false to preserve exact triangulation.
	// The optimizer may merge coplanar triangles, reducing concave edges.
	meshBuilder.End(false);

	ndShapeInstance meshShape(new ndShapeStatic_bvh(meshBuilder));
	body->SetCollisionShape(meshShape);

	return body;
}

// Build a small dynamic sphere with downward gravity.
static ndBodyDynamic* BuildFallingSphere(const ndVector& pos)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndVector gravity(ndFloat32(0.0f), ndFloat32(-9.8f), ndFloat32(0.0f), ndFloat32(0.0f));
	body->SetNotifyCallback(new ndBodyNotify(gravity));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = pos;
	body->SetMatrix(matrix);

	ndShapeInstance sphere(new ndShapeSphere(ndFloat32(0.5f)));
	body->SetCollisionShape(sphere);
	body->SetMassMatrix(ndFloat32(1.0f), sphere);

	return body;
}

// A dynamic sphere inside an inverted-box static mesh should trigger
// the assertion in ndShapeConvexPolygon::GenerateConvexCap() in Debug builds
// when colliding with concave interior edges.
// In Release builds this test verifies the sphere stays within the box bounds.
TEST(ConcaveMesh, DynamicBodyInsideInvertedBoxCrash)
{
	ndWorld world;
	world.SetSubSteps(2);

	// Static inverted box at origin
	ndVector meshPos(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f));
	ndSharedPtr<ndBody> meshBody(BuildInvertedBoxMesh(meshPos));
	world.AddBody(meshBody);

	// Dynamic sphere starts above center, falls under gravity toward bottom face
	ndVector spherePos(ndFloat32(0.0f), ndFloat32(2.0f), ndFloat32(0.0f), ndFloat32(1.0f));
	ndSharedPtr<ndBody> sphere(BuildFallingSphere(spherePos));
	world.AddBody(sphere);

	// Simulate 2 seconds (120 frames at 60Hz).
	// The sphere should hit the bottom face and bounce into corner edges.
	const ndFloat32 timestep = ndFloat32(1.0f) / ndFloat32(60.0f);
	for (int i = 0; i < 120; i++)
	{
		world.Update(timestep);
		world.Sync();
	}

	// Release-build fallback: verify the sphere is still contained within
	// the box bounds (with a small margin for the sphere radius).
	const ndFloat32 bound = ndFloat32(5.5f);
	ndVector p = sphere->GetMatrix().m_posit;
	EXPECT_GT(p.m_x, -bound);
	EXPECT_LT(p.m_x,  bound);
	EXPECT_GT(p.m_y, -bound);
	EXPECT_LT(p.m_y,  bound);
	EXPECT_GT(p.m_z, -bound);
	EXPECT_LT(p.m_z,  bound);
}
