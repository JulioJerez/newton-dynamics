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

#include <ndNewtonInc.h>
#include <gtest/gtest.h>

TEST(Collisions, BasicSceneSetUp)
{
	// instance a world
	ndWorld world;

	ndMatrix matrix(ndGetIdentityMatrix());

	// make a floor box
	ndShapeInstance shapeinst(new ndShapeBox(ndFloat32(100.0f), ndFloat32(0.5f), ndFloat32(100.0f)));
	ndBodyDynamic* staticbody = new ndBodyDynamic();
	staticbody->SetCollisionShape(shapeinst);
	staticbody->SetMatrix(matrix);
	ndSharedPtr<ndBody> staticPtr(staticbody);
	world.AddBody(staticPtr);

	// make a dynamics sphere
	matrix.m_posit.m_y += 10.0f;
	ndShapeInstance sphereinst(new ndShapeSphere(ndFloat32(0.5f)));
	ndBodyDynamic* const movingbody = new ndBodyDynamic();
	movingbody->SetNotifyCallback(new ndBodyNotify(ndBigVector(ndFloat32(0), ndFloat32(-9.81f), ndFloat32(0), ndFloat32(0))));
	movingbody->SetCollisionShape(sphereinst);
	movingbody->SetMatrix(matrix);
	movingbody->SetMassMatrix(ndFloat32(10), sphereinst);
	ndSharedPtr<ndBody> movingPtr(movingbody);
	world.AddBody(movingPtr);

	// iterate the world, unti the end
	for (int i = 0; i < 480; i++) 
	{
		world.Update(1.0f / 60.0f);
		world.Sync();
	}
}