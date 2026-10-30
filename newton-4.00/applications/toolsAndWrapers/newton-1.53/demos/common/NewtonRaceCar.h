// NewtonRaceCar.h: interface for the RaceCar class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NEWTONRACECAR_H__96CF72A9_A2BD_4A11_9F6D_40DD9A3CCC12__INCLUDED_)
#define AFX_NEWTONRACECAR_H__96CF72A9_A2BD_4A11_9F6D_40DD9A3CCC12__INCLUDED_


#include <stdafx.h>
#include "OpenGlUtil.h"
#include "NewtonVehicle.h"

class NewtonRaceCar: public NewtonVehicle  
{
	struct RaceCarTire
	{
		RaceCarTire() {};
		virtual ~RaceCarTire() {};
		void Setup (NewtonRaceCar *root, const char* label);
		virtual void SetTirePhysics(const NewtonJoint* vehicle, void* tireId);
	

		dFloat m_width;
		dFloat m_radius;
		dFloat m_tireRefHeight;

		dVector m_brakeRefPosition;
		dMatrix m_tireOffsetMatrix;
		dMatrix m_axelMatrix;
		dMatrix m_suspentionTopMatrix;
		dMatrix m_suspentionBottomMatrix;
		dSceneNode *m_tireNode;
		dSceneNode *m_axelNode;
		dSceneNode *m_brakeNode;
		dSceneNode *m_topSuspNode;
		dSceneNode *m_bottomSuspNode;
	};


	struct FrontTire: public RaceCarTire
	{
		FrontTire() 
		{
			m_steer = 0.0f;
		}

		void SetSteer (dFloat value)
		{
			m_steer = value;
		}
		virtual void SetTirePhysics(const NewtonJoint* vehicle, void* tireId);

		dFloat m_steer;
	};
	
	struct RearTire: public RaceCarTire
	{

		RearTire ()
		{
			m_torque = 0.0f;
		}

		void SetTorque (dFloat value)
		{
			m_torque = value;
		}


		void SetBrakes (dFloat brake)
		{
			m_brakes = brake;
		}

		virtual void SetTirePhysics(const NewtonJoint* vehicle, void* tireId);

		dFloat m_torque;
		dFloat m_brakes;
	};

	public:
	NewtonRaceCar(dSceneNode* parent, const char* fileName, NewtonWorld* nWorld, const dMatrix& matrix);
	virtual ~NewtonRaceCar();

	virtual void SetSteering(dFloat value);
	virtual void SetTireTorque(dFloat torque);
	virtual void SetApplyHandBrakes (dFloat value);
	virtual void SetControl(int index, dFloat value) {};


	private:
	void Render() const;
	static void DestroyVehicle (const NewtonBody* body);
	static void ApplyGravityForce (const NewtonBody* body);
	static void SetTransform (const NewtonBody* body, const dFloat* matrixPtr);
	static void TireUpdate (const NewtonJoint* vehicle);

	RearTire m_rearRighTire;
	RearTire m_rearLeftTire;
	FrontTire m_frontRighTire;
	FrontTire m_frontLeftTire;
};

#endif // !defined(AFX_NEWTONRACECAR_H__96CF72A9_A2BD_4A11_9F6D_40DD9A3CCC12__INCLUDED_)
