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

#include "ndNewton.h"
#include <gtest/gtest.h>

class ndApplyImpulseNotify : public ndBodyNotify
{
	public:
	ndApplyImpulseNotify()
		:ndBodyNotify(ndVector(0.0f, -9.8f, 0.0f, 0.0f))
		,m_addImpulse(false)
	{
	}

	void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		ndBodyNotify::OnApplyExternalForce(threadIndex, timestep);

		ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
		ndVector pos = body->GetPosition();
		bool sleeping = body->GetSleepState();
		printf("Body sleep state: %s, position.y: %f\n", sleeping ? "sleeping" : "awake", pos.m_y);

		if (m_addImpulse)
		{

			if (body->GetInvMass() > 0.0f)
			{
				ndVector upImpulse(ndVector::m_zero);
				upImpulse.m_y = ndFloat32(5.0f) / body->GetInvMass();
				body->ApplyImpulsePair(upImpulse, ndVector::m_zero, timestep);
			}
			m_addImpulse = false;
		}
	}

	bool m_addImpulse;
};

static ndBodyDynamic* BuildDynamicBox(const ndVector& pos)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndApplyImpulseNotify());

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = pos;
	body->SetMatrix(matrix);

	ndShapeInstance box(new ndShapeBox(1.0f, 1.0f, 1.0f));
	body->SetCollisionShape(box);
	body->SetMassMatrix(1.0f, box);

	body->SetAngularDamping(ndVector(0.f));
	body->SetLinearDamping(0.f);

	return body;
}

TEST(Impulse, ApplyImpulseViaCallback)
{
	ndWorld world;
	world.SetSubSteps(2);

	// Static ground plane: a thin wide box at Y = -0.5 (top surface at Y = 0).
	ndBodyKinematic* const groundBody = new ndBodyKinematic();
	ndShapeInstance groundShape(new ndShapeBox(100.0f, 1.0f, 100.0f));
	groundBody->SetCollisionShape(groundShape);
	ndMatrix groundMatrix(ndGetIdentityMatrix());
	groundMatrix.m_posit = ndVector(0.0f, -0.5f, 0.0f, 1.0f);
	groundBody->SetMatrix(groundMatrix);
	ndSharedPtr<ndBody> ground(groundBody);
	world.AddBody(ground);

	// Dynamic box sitting on the ground (center at Y = 0.5 so bottom touches Y = 0).
	ndVector startPos(0.0f, 0.5f, 0.0f, 1.0f);
	ndBodyDynamic* const boxRaw = BuildDynamicBox(startPos);
	ndSharedPtr<ndBody> box(boxRaw);
	world.AddBody(box);

	// Trigger the impulse via the callback flag.
	ndApplyImpulseNotify* const notify =
		static_cast<ndApplyImpulseNotify*>(*boxRaw->GetNotifyCallback());
	notify->m_addImpulse = true;

	// Let the body move upward 
	for (int i = 0; i < 120; i++)
	{
		world.Update(1.0f / 60.0f);
		world.Sync();
	}

	// Body must have moved upward from the origin.
	ndVector finalPos = boxRaw->GetPosition();
	EXPECT_GT(finalPos.m_y, 0.0f);
}
