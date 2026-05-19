
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

// Regression test: dynamic boxes stacked above a flat static mesh ground
// should not penetrate through the mesh after settling under gravity.

#include <ndNewtonInc.h>
#include <gtest/gtest.h>

// Build a flat static mesh ground at Y=0 using two triangles (a quad).
// The mesh spans [-50, +50] on X and Z axes.
static ndBodyKinematic* BuildFlatMeshGround(const ndVector& pos)
{
	ndBodyKinematic* const body = new ndBodyKinematic();
	body->SetNotifyCallback(new ndBodyNotify(ndVector::m_zero));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = pos;
	body->SetMatrix(matrix);

	const ndFloat32 s = ndFloat32(50.0f);
	ndVector verts[4] =
	{
		ndVector(-s, ndFloat32(0.0f), -s, ndFloat32(0.0f)),
		ndVector( s, ndFloat32(0.0f), -s, ndFloat32(0.0f)),
		ndVector( s, ndFloat32(0.0f),  s, ndFloat32(0.0f)),
		ndVector(-s, ndFloat32(0.0f),  s, ndFloat32(0.0f)),
	};

	// Two triangles wound so normals point up (+Y).
	// cross(v1-v0, v2-v0) must yield +Y, so wind counter-clockwise
	// when viewed from above: 0-2-1 and 0-3-2.
	const int indices[2][3] =
	{
		{0, 2, 1},
		{0, 3, 2},
	};

	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();

	for (int i = 0; i < 2; ++i)
	{
		ndVector triangle[3];
		triangle[0] = verts[indices[i][0]];
		triangle[1] = verts[indices[i][1]];
		triangle[2] = verts[indices[i][2]];

		ndInt32 materialId = 0;
		meshBuilder.AddFace(triangle, 3, materialId);
	}

	meshBuilder.End(false);
	//meshBuilder.End(true);

	ndShapeInstance meshShape(new ndShapeStatic_bvh(meshBuilder));
	body->SetCollisionShape(meshShape);

	return body;
}

// Build a dynamic box with gravity.
static ndBodyDynamic* BuildDynamicBox(const ndVector& pos)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndVector gravity(ndFloat32(0.0f), ndFloat32(-9.8f), ndFloat32(0.0f), ndFloat32(0.0f));
	body->SetNotifyCallback(new ndBodyNotify(gravity));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = pos;
	body->SetMatrix(matrix);

	ndShapeInstance box(new ndShapeBox(ndFloat32(1.0f), ndFloat32(1.0f), ndFloat32(1.0f)));
	body->SetCollisionShape(box);
	body->SetMassMatrix(ndFloat32(1.0f), box);

	return body;
}

// Drop 100 stacked boxes onto a flat static mesh ground.
// After settling, no box should have penetrated through the ground.
TEST(MeshPenetration, StackedBoxesOnFlatMesh)
{
	ndWorld world;
	world.SetSubSteps(2);

	// Static mesh ground at origin
	ndVector groundPos(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f));
	ndSharedPtr<ndBody> ground(BuildFlatMeshGround(groundPos));
	world.AddBody(ground);

	// 100 dynamic boxes stacked in a column above the ground
	const int numBoxes = 100;
	ndBodyDynamic* boxes[numBoxes];

	for (int i = 0; i < numBoxes; ++i)
	{
		ndFloat32 y = ndFloat32(1.0f) + ndFloat32(i) * ndFloat32(1.2f);
		ndVector pos(ndFloat32(0.0f), y, ndFloat32(0.0f), ndFloat32(1.0f));
		boxes[i] = BuildDynamicBox(pos);
		ndSharedPtr<ndBody> boxBody(boxes[i]);
		world.AddBody(boxBody);
	}

	// Simulate 10 seconds at 60Hz (600 steps)
	const ndFloat32 timestep = ndFloat32(1.0f) / ndFloat32(60.0f);
	for (int i = 0; i < 600; i++)
	{
		world.Update(timestep);
		world.Sync();

		// Verify no box penetrated the ground.
		// Ground is at Y=0, box half-height is 0.5, so center should be >= -0.5
		// (using small tolerance for floating point).
		const ndFloat32 minY = ndFloat32(-0.5f);
		for (int j = 0; j < numBoxes; ++j)
		{
			ndVector p = boxes[j]->GetMatrix().m_posit;
			EXPECT_GE(p.m_y, minY) << "Box " << j << " penetrated the mesh ground (Y=" << p.m_y << ")";
		}
	}

	// Verify no box penetrated the ground.
	// Ground is at Y=0, box half-height is 0.5, so center should be >= -0.5
	// (using small tolerance for floating point).
	const ndFloat32 minY = ndFloat32(-0.5f);
	for (int i = 0; i < numBoxes; ++i)
	{
		ndVector p = boxes[i]->GetMatrix().m_posit;
		EXPECT_GE(p.m_y, minY) << "Box " << i << " penetrated the mesh ground (Y=" << p.m_y << ")";
	}
}
