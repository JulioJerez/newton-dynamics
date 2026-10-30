// NewtonTractor.h: interface for the RaceCar class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NEWTONTRACTOR_H__96CF72A9_A2BD_4A11_9F6D_40DD9A3CCC12__INCLUDED_)
#define AFX_NEWTONTRACTOR_H__96CF72A9_A2BD_4A11_9F6D_40DD9A3CCC12__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <stdafx.h>
#include "OpenGlUtil.h"
#include "NewtonVehicle.h"


class TractorControlJoint;

class NewtonTractor: public NewtonVehicle  
{
	public:
	NewtonTractor(dSceneNode* parent, const char* fileName, NewtonWorld* nWorld, const dMatrix& matrix);
	virtual ~NewtonTractor();

	virtual void SetSteering(dFloat value);
	virtual void SetTireTorque(dFloat torque);
	virtual void SetApplyHandBrakes (dFloat value);
	virtual void SetControl(int index, dFloat value);


	private:
	void Render() const;

	TractorControlJoint* CreateAxel (NewtonWorld* nWorld, const char *name);

	void AddRearAxel (NewtonWorld* nWorld, dVector* vertex);
	void AddFrontAxel (NewtonWorld* nWorld, dVector* vertex);
	void AddFrontBucket (NewtonWorld* nWorld, dVector* vertex);
	void AddBackBucket (NewtonWorld* nWorld, dVector* vertex);

	static void DestroyVehicle (const NewtonBody* body);

	static void ApplyGravityForce (const NewtonBody* body);
	static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr);

	TractorControlJoint* m_rearAxel;
	TractorControlJoint* m_frontAxel;

	TractorControlJoint* m_backBucket;
	TractorControlJoint* m_backBucketArm1;
	TractorControlJoint* m_backBucketArm2;

	TractorControlJoint* m_frontBucket;
	TractorControlJoint* m_frontBucketArm;

	friend class RealisticHeavyAxel;

};

#endif // !defined(AFX_NEWTONTRACTOR_H__96CF72A9_A2BD_4A11_9F6D_40DD9A3CCC12__INCLUDED_)
