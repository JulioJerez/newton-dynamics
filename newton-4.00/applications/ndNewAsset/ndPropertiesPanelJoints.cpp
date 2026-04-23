/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndNewAssetStdafx.h"
#include "ndUndoRedo.h"
#include "ndAssetEditor.h"

class ndUndoRedoJoint : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJoint(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_localFrame(m_mesh->GetJoint()->m_localFrame0)
	{
	}

	virtual ndUndoRedoJoint* GetAsUndoRedoJoint() const override
	{
		return (ndUndoRedoJoint*)this;
	}
	
	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJoint* const other = command.GetAsUndoRedoJoint();
			if (other)
			{
				//ndMatrix matrix(m_localFrame * other->m_localFrame.OrthoInverse());
				bool test = (m_localFrame * other->m_localFrame.OrthoInverse()).TestIdentity();
				if (test)
				{
					return false;
				}
			}
		}
	
		return true;
	}
	
	virtual void Undo() override
	{
		ndMeshJoint* const joint = *m_mesh->GetJoint();
		joint->m_localFrame0 = m_localFrame;

		ndMatrix globalMatrix(m_localFrame * m_mesh->CalculateGlobalMatrix());
		joint->m_localFrame1 = globalMatrix * m_mesh->GetParent()->CalculateGlobalMatrix().OrthoInverse();
	}

	ndMatrix m_localFrame;
};

class ndUndoRedoJointChange : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointChange(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
		,m_joint(m_mesh->GetJoint())
	{
	}

	virtual ndUndoRedoJointChange* GetAsUndoRedoJointChange() const override
	{
		return (ndUndoRedoJointChange*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointChange* const other = command.GetAsUndoRedoJointChange();
			if (other)
			{
				if (m_joint->m_constructor == other->m_joint->m_constructor)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		m_mesh->SetJoint(m_joint);
	}

	ndSharedPtr<ndMeshJoint> m_joint;
};

class ndUndoRedoJointFix6dof : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointFix6dof(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointFix6dof* const joint = (ndMeshJointFix6dof*)*m_mesh->GetJoint();
		m_softness = joint->m_softness;
		m_maxForce = joint->m_maxForce;
		m_maxTorque = joint->m_maxTorque;
	}

	virtual ndUndoRedoJointFix6dof* GetAsUndoRedoJointFix6dof() const override
	{
		return (ndUndoRedoJointFix6dof*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointFix6dof* const other = command.GetAsUndoRedoJointFix6dof();
			if (other)
			{
				bool test = other->m_softness == m_softness;
				test = test && other->m_maxForce == m_maxForce;
				test = test && other->m_maxTorque == m_maxTorque;
				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndUndoRedoJointFix6dof* const joint = (ndUndoRedoJointFix6dof*)*m_mesh->GetJoint();

		joint->m_softness = m_softness;
		joint->m_maxForce = m_maxForce;
		joint->m_maxTorque = m_maxTorque;
	}

	ndFloat32 m_softness;
	ndFloat32 m_maxForce;
	ndFloat32 m_maxTorque;
};

class ndUndoRedoJointHinge : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointHinge(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointHinge* const joint = (ndMeshJointHinge*)*m_mesh->GetJoint();

		m_axis.m_springK = joint->m_axis.m_springK;
		m_axis.m_damperC = joint->m_axis.m_damperC;
		m_axis.m_minLimit = joint->m_axis.m_minLimit;
		m_axis.m_maxLimit = joint->m_axis.m_maxLimit;
		m_axis.m_limitState = joint->m_axis.m_limitState;
		m_axis.m_springDamperRegularizer = joint->m_axis.m_springDamperRegularizer;
	}

	virtual ndUndoRedoJointHinge* GetAsUndoRedoJointHinge() const override
	{
		return (ndUndoRedoJointHinge*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointHinge* const other = command.GetAsUndoRedoJointHinge();
			if (other)
			{
				bool test = other->m_axis.m_springK == m_axis.m_springK;
				test = test && other->m_axis.m_damperC == m_axis.m_damperC;
				test = test && other->m_axis.m_minLimit == m_axis.m_minLimit;
				test = test && other->m_axis.m_maxLimit == m_axis.m_maxLimit;
				test = test && other->m_axis.m_limitState == m_axis.m_limitState;
				test = test && other->m_axis.m_springDamperRegularizer == m_axis.m_springDamperRegularizer;
				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointHinge* const joint = (ndMeshJointHinge*)*m_mesh->GetJoint();

		joint->m_axis.m_springK = m_axis.m_springK;
		joint->m_axis.m_damperC = m_axis.m_damperC;
		joint->m_axis.m_minLimit = m_axis.m_minLimit;
		joint->m_axis.m_maxLimit = m_axis.m_maxLimit;
		joint->m_axis.m_limitState = m_axis.m_limitState;
		joint->m_axis.m_springDamperRegularizer = m_axis.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_axis;
};

class ndUndoRedoJointSlider : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointSlider(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointSlider* const joint = (ndMeshJointSlider*)*m_mesh->GetJoint();

		m_axis.m_springK = joint->m_axis.m_springK;
		m_axis.m_damperC = joint->m_axis.m_damperC;
		m_axis.m_minLimit = joint->m_axis.m_minLimit;
		m_axis.m_maxLimit = joint->m_axis.m_maxLimit;
		m_axis.m_limitState = joint->m_axis.m_limitState;
		m_axis.m_springDamperRegularizer = joint->m_axis.m_springDamperRegularizer;
	}

	virtual ndUndoRedoJointSlider* GetAsUndoRedoJointSlider() const override
	{
		return (ndUndoRedoJointSlider*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointSlider* const other = command.GetAsUndoRedoJointSlider();
			if (other)
			{
				bool test = other->m_axis.m_springK == m_axis.m_springK;
				test = test && other->m_axis.m_damperC == m_axis.m_damperC;
				test = test && other->m_axis.m_minLimit == m_axis.m_minLimit;
				test = test && other->m_axis.m_maxLimit == m_axis.m_maxLimit;
				test = test && other->m_axis.m_limitState == m_axis.m_limitState;
				test = test && other->m_axis.m_springDamperRegularizer == m_axis.m_springDamperRegularizer;
				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointSlider* const joint = (ndMeshJointSlider*)*m_mesh->GetJoint();

		joint->m_axis.m_springK = m_axis.m_springK;
		joint->m_axis.m_damperC = m_axis.m_damperC;
		joint->m_axis.m_minLimit = m_axis.m_minLimit;
		joint->m_axis.m_maxLimit = m_axis.m_maxLimit;
		joint->m_axis.m_limitState = m_axis.m_limitState;
		joint->m_axis.m_springDamperRegularizer = m_axis.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_axis;
};

class ndUndoRedoJointDoubleHinge : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointDoubleHinge(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointDoubleHinge* const joint = (ndMeshJointDoubleHinge*)*m_mesh->GetJoint();

		m_axis0.m_springK = joint->m_axis0.m_springK;
		m_axis0.m_damperC = joint->m_axis0.m_damperC;
		m_axis0.m_minLimit = joint->m_axis0.m_minLimit;
		m_axis0.m_maxLimit = joint->m_axis0.m_maxLimit;
		m_axis0.m_limitState = joint->m_axis0.m_limitState;
		m_axis0.m_springDamperRegularizer = joint->m_axis0.m_springDamperRegularizer;

		m_axis1.m_springK = joint->m_axis1.m_springK;
		m_axis1.m_damperC = joint->m_axis1.m_damperC;
		m_axis1.m_minLimit = joint->m_axis1.m_minLimit;
		m_axis1.m_maxLimit = joint->m_axis1.m_maxLimit;
		m_axis1.m_limitState = joint->m_axis1.m_limitState;
		m_axis1.m_springDamperRegularizer = joint->m_axis1.m_springDamperRegularizer;
	}

	virtual ndUndoRedoJointDoubleHinge* GetAsUndoRedoJointDoubleHinge() const override
	{
		return (ndUndoRedoJointDoubleHinge*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointDoubleHinge* const other = command.GetAsUndoRedoJointDoubleHinge();
			if (other)
			{
				bool test = other->m_axis0.m_springK == m_axis0.m_springK;
				test = test && other->m_axis0.m_damperC == m_axis0.m_damperC;
				test = test && other->m_axis0.m_minLimit == m_axis0.m_minLimit;
				test = test && other->m_axis0.m_maxLimit == m_axis0.m_maxLimit;
				test = test && other->m_axis0.m_limitState == m_axis0.m_limitState;
				test = test && other->m_axis0.m_springDamperRegularizer == m_axis0.m_springDamperRegularizer;

				test = test && other->m_axis1.m_springK == m_axis1.m_springK;
				test = test && other->m_axis1.m_damperC == m_axis1.m_damperC;
				test = test && other->m_axis1.m_minLimit == m_axis1.m_minLimit;
				test = test && other->m_axis1.m_maxLimit == m_axis1.m_maxLimit;
				test = test && other->m_axis1.m_limitState == m_axis1.m_limitState;
				test = test && other->m_axis1.m_springDamperRegularizer == m_axis1.m_springDamperRegularizer;

				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointDoubleHinge* const joint = (ndMeshJointDoubleHinge*)*m_mesh->GetJoint();

		joint->m_axis0.m_springK = m_axis0.m_springK;
		joint->m_axis0.m_damperC = m_axis0.m_damperC;
		joint->m_axis0.m_minLimit = m_axis0.m_minLimit;
		joint->m_axis0.m_maxLimit = m_axis0.m_maxLimit;
		joint->m_axis0.m_limitState = m_axis0.m_limitState;
		joint->m_axis0.m_springDamperRegularizer = m_axis0.m_springDamperRegularizer;

		joint->m_axis1.m_springK = m_axis1.m_springK;
		joint->m_axis1.m_damperC = m_axis1.m_damperC;
		joint->m_axis1.m_minLimit = m_axis1.m_minLimit;
		joint->m_axis1.m_maxLimit = m_axis1.m_maxLimit;
		joint->m_axis1.m_limitState = m_axis1.m_limitState;
		joint->m_axis1.m_springDamperRegularizer = m_axis1.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_axis0;
	ndMeshJoint::ndAxis m_axis1;
};

class ndUndoRedoJointRoller : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointRoller(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointRoller* const joint = (ndMeshJointRoller*)*m_mesh->GetJoint();

		m_positAxis.m_springK = joint->m_positAxis.m_springK;
		m_positAxis.m_damperC = joint->m_positAxis.m_damperC;
		m_positAxis.m_minLimit = joint->m_positAxis.m_minLimit;
		m_positAxis.m_maxLimit = joint->m_positAxis.m_maxLimit;
		m_positAxis.m_limitState = joint->m_positAxis.m_limitState;
		m_positAxis.m_springDamperRegularizer = joint->m_positAxis.m_springDamperRegularizer;

		m_angleAxis.m_springK = joint->m_angleAxis.m_springK;
		m_angleAxis.m_damperC = joint->m_angleAxis.m_damperC;
		m_angleAxis.m_minLimit = joint->m_angleAxis.m_minLimit;
		m_angleAxis.m_maxLimit = joint->m_angleAxis.m_maxLimit;
		m_angleAxis.m_limitState = joint->m_angleAxis.m_limitState;
		m_angleAxis.m_springDamperRegularizer = joint->m_angleAxis.m_springDamperRegularizer;
	}

	virtual ndUndoRedoJointRoller* GetAsUndoRedoJointRoller() const override
	{
		return (ndUndoRedoJointRoller*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointRoller* const other = command.GetAsUndoRedoJointRoller();
			if (other)
			{
				bool test = other->m_positAxis.m_springK == m_positAxis.m_springK;
				test = test && other->m_positAxis.m_damperC == m_positAxis.m_damperC;
				test = test && other->m_positAxis.m_minLimit == m_positAxis.m_minLimit;
				test = test && other->m_positAxis.m_maxLimit == m_positAxis.m_maxLimit;
				test = test && other->m_positAxis.m_limitState == m_positAxis.m_limitState;
				test = test && other->m_positAxis.m_springDamperRegularizer == m_positAxis.m_springDamperRegularizer;

				test = test && other->m_angleAxis.m_springK == m_angleAxis.m_springK;
				test = test && other->m_angleAxis.m_damperC == m_angleAxis.m_damperC;
				test = test && other->m_angleAxis.m_minLimit == m_angleAxis.m_minLimit;
				test = test && other->m_angleAxis.m_maxLimit == m_angleAxis.m_maxLimit;
				test = test && other->m_angleAxis.m_limitState == m_angleAxis.m_limitState;
				test = test && other->m_angleAxis.m_springDamperRegularizer == m_angleAxis.m_springDamperRegularizer;

				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointRoller* const joint = (ndMeshJointRoller*)*m_mesh->GetJoint();

		joint->m_positAxis.m_springK = m_positAxis.m_springK;
		joint->m_positAxis.m_damperC = m_positAxis.m_damperC;
		joint->m_positAxis.m_minLimit = m_positAxis.m_minLimit;
		joint->m_positAxis.m_maxLimit = m_positAxis.m_maxLimit;
		joint->m_positAxis.m_limitState = m_positAxis.m_limitState;
		joint->m_positAxis.m_springDamperRegularizer = m_positAxis.m_springDamperRegularizer;

		joint->m_angleAxis.m_springK = m_angleAxis.m_springK;
		joint->m_angleAxis.m_damperC = m_angleAxis.m_damperC;
		joint->m_angleAxis.m_minLimit = m_angleAxis.m_minLimit;
		joint->m_angleAxis.m_maxLimit = m_angleAxis.m_maxLimit;
		joint->m_angleAxis.m_limitState = m_angleAxis.m_limitState;
		joint->m_angleAxis.m_springDamperRegularizer = m_angleAxis.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_positAxis;
	ndMeshJoint::ndAxis m_angleAxis;
};

class ndUndoRedoJointCylinder : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointCylinder(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointCylinder* const joint = (ndMeshJointCylinder*)*m_mesh->GetJoint();

		m_axis0.m_springK = joint->m_axis0.m_springK;
		m_axis0.m_damperC = joint->m_axis0.m_damperC;
		m_axis0.m_minLimit = joint->m_axis0.m_minLimit;
		m_axis0.m_maxLimit = joint->m_axis0.m_maxLimit;
		m_axis0.m_limitState = joint->m_axis0.m_limitState;
		m_axis0.m_springDamperRegularizer = joint->m_axis0.m_springDamperRegularizer;

		m_axis1.m_springK = joint->m_axis1.m_springK;
		m_axis1.m_damperC = joint->m_axis1.m_damperC;
		m_axis1.m_minLimit = joint->m_axis1.m_minLimit;
		m_axis1.m_maxLimit = joint->m_axis1.m_maxLimit;
		m_axis1.m_limitState = joint->m_axis1.m_limitState;
		m_axis1.m_springDamperRegularizer = joint->m_axis1.m_springDamperRegularizer;
	}

	virtual ndUndoRedoJointCylinder* GetAsUndoRedoJointCylinder() const override
	{
		return (ndUndoRedoJointCylinder*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointCylinder* const other = command.GetAsUndoRedoJointCylinder();
			if (other)
			{
				bool test = other->m_axis0.m_springK == m_axis0.m_springK;
				test = test && other->m_axis0.m_damperC == m_axis0.m_damperC;
				test = test && other->m_axis0.m_minLimit == m_axis0.m_minLimit;
				test = test && other->m_axis0.m_maxLimit == m_axis0.m_maxLimit;
				test = test && other->m_axis0.m_limitState == m_axis0.m_limitState;
				test = test && other->m_axis0.m_springDamperRegularizer == m_axis0.m_springDamperRegularizer;

				test = test && other->m_axis1.m_springK == m_axis1.m_springK;
				test = test && other->m_axis1.m_damperC == m_axis1.m_damperC;
				test = test && other->m_axis1.m_minLimit == m_axis1.m_minLimit;
				test = test && other->m_axis1.m_maxLimit == m_axis1.m_maxLimit;
				test = test && other->m_axis1.m_limitState == m_axis1.m_limitState;
				test = test && other->m_axis1.m_springDamperRegularizer == m_axis1.m_springDamperRegularizer;

				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointCylinder* const joint = (ndMeshJointCylinder*)*m_mesh->GetJoint();

		joint->m_axis0.m_springK = m_axis0.m_springK;
		joint->m_axis0.m_damperC = m_axis0.m_damperC;
		joint->m_axis0.m_minLimit = m_axis0.m_minLimit;
		joint->m_axis0.m_maxLimit = m_axis0.m_maxLimit;
		joint->m_axis0.m_limitState = m_axis0.m_limitState;
		joint->m_axis0.m_springDamperRegularizer = m_axis0.m_springDamperRegularizer;

		joint->m_axis1.m_springK = m_axis1.m_springK;
		joint->m_axis1.m_damperC = m_axis1.m_damperC;
		joint->m_axis1.m_minLimit = m_axis1.m_minLimit;
		joint->m_axis1.m_maxLimit = m_axis1.m_maxLimit;
		joint->m_axis1.m_limitState = m_axis1.m_limitState;
		joint->m_axis1.m_springDamperRegularizer = m_axis1.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_axis0;
	ndMeshJoint::ndAxis m_axis1;
};

class ndUndoRedoJointWheel : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointWheel(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointWheel* const joint = (ndMeshJointWheel*)*m_mesh->GetJoint();

		m_brakeTorque = joint->m_brakeTorque;
		m_steeringAngle = joint->m_steeringAngle;
		m_handBrakeTorque = joint->m_handBrakeTorque;
		m_axis.m_springK = joint->m_axis.m_springK;
		m_axis.m_damperC = joint->m_axis.m_damperC;
		m_axis.m_minLimit = joint->m_axis.m_minLimit;
		m_axis.m_maxLimit = joint->m_axis.m_maxLimit;
		m_axis.m_limitState = 1;
		m_axis.m_springDamperRegularizer = joint->m_axis.m_springDamperRegularizer;

	}

	virtual ndUndoRedoJointWheel* GetAsUndoRedoJointWheel() const override
	{
		return (ndUndoRedoJointWheel*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointWheel* const other = command.GetAsUndoRedoJointWheel();
			if (other)
			{
				bool test = other->m_axis.m_springK == m_axis.m_springK;
				test = test && other->m_axis.m_damperC == m_axis.m_damperC;
				test = test && other->m_axis.m_minLimit == m_axis.m_minLimit;
				test = test && other->m_axis.m_maxLimit == m_axis.m_maxLimit;
				test = test && other->m_axis.m_springDamperRegularizer == m_axis.m_springDamperRegularizer;
				test = test && other->m_brakeTorque == m_brakeTorque;
				test = test && other->m_steeringAngle == m_steeringAngle;
				test = test && other->m_handBrakeTorque == m_handBrakeTorque;

				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointWheel* const joint = (ndMeshJointWheel*)*m_mesh->GetJoint();

		joint->m_brakeTorque = m_brakeTorque;
		joint->m_steeringAngle = m_steeringAngle;
		joint->m_handBrakeTorque = m_handBrakeTorque;
		joint->m_axis.m_springK = m_axis.m_springK;
		joint->m_axis.m_damperC = m_axis.m_damperC;
		joint->m_axis.m_minLimit = m_axis.m_minLimit;
		joint->m_axis.m_maxLimit = m_axis.m_maxLimit;
		joint->m_axis.m_limitState = m_axis.m_limitState;
		joint->m_axis.m_springDamperRegularizer = m_axis.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_axis;
	ndFloat32 m_brakeTorque;
	ndFloat32 m_steeringAngle;
	ndFloat32 m_handBrakeTorque;
};

class ndUndoRedoJointSpherical : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointSpherical(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointSpherical* const joint = (ndMeshJointSpherical*)*m_mesh->GetJoint();

		m_maxConeAngle = joint->m_maxConeAngle;
		m_coneAngleState = joint->m_coneAngleState ? true : false;
		m_axis.m_springK = joint->m_axis.m_springK;
		m_axis.m_damperC = joint->m_axis.m_damperC;
		m_axis.m_minLimit = joint->m_axis.m_minLimit;
		m_axis.m_maxLimit = joint->m_axis.m_maxLimit;
		m_axis.m_limitState = joint->m_axis.m_limitState;
		m_axis.m_springDamperRegularizer = joint->m_axis.m_springDamperRegularizer;
	}

	virtual ndUndoRedoJointSpherical* GetAsUndoRedoJointSpherical() const override
	{
		return (ndUndoRedoJointSpherical*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointSpherical* const other = command.GetAsUndoRedoJointSpherical();
			if (other)
			{
				bool test = other->m_axis.m_springK == m_axis.m_springK;
				test = test && other->m_axis.m_damperC == m_axis.m_damperC;
				test = test && other->m_axis.m_minLimit == m_axis.m_minLimit;
				test = test && other->m_axis.m_maxLimit == m_axis.m_maxLimit;
				test = test && other->m_axis.m_limitState == m_axis.m_limitState;
				test = test && other->m_axis.m_springDamperRegularizer == m_axis.m_springDamperRegularizer;
				test = test && other->m_coneAngleState == m_coneAngleState;
				test = test && other->m_maxConeAngle == m_maxConeAngle;

				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointHinge* const joint = (ndMeshJointHinge*)*m_mesh->GetJoint();

		joint->m_axis.m_springK = m_axis.m_springK;
		joint->m_axis.m_damperC = m_axis.m_damperC;
		joint->m_axis.m_minLimit = m_axis.m_minLimit;
		joint->m_axis.m_maxLimit = m_axis.m_maxLimit;
		joint->m_axis.m_limitState = m_axis.m_limitState;
		joint->m_axis.m_springDamperRegularizer = m_axis.m_springDamperRegularizer;
	}

	ndMeshJoint::ndAxis m_axis;
	ndFloat32 m_maxConeAngle;
	bool m_coneAngleState;
};

class ndUndoRedoJointPlane : public ndUndoRedoCommand
{
	public:
	ndUndoRedoJointPlane(ndAssetEditor* const editor, const ndSharedPtr<ndMesh>& mesh)
		:ndUndoRedoCommand(editor, mesh)
	{
		ndMeshJointPlane* const joint = (ndMeshJointPlane*)*m_mesh->GetJoint();
		m_controlRotation = joint->m_controlRotation ? true : false;
	}

	virtual ndUndoRedoJointPlane* GetAsUndoRedoJointPlane() const override
	{
		return (ndUndoRedoJointPlane*)this;
	}

	virtual bool operator!=(const ndUndoRedoCommand& command) const override
	{
		if (*m_mesh == *command.m_mesh)
		{
			const ndUndoRedoJointPlane* const other = command.GetAsUndoRedoJointPlane();
			if (other)
			{
				bool test = other->m_controlRotation == m_controlRotation;
				if (test)
				{
					return false;
				}
			}
		}

		return true;
	}

	virtual void Undo() override
	{
		ndMeshJointPlane* const joint = (ndMeshJointPlane*)*m_mesh->GetJoint();
		joint->m_controlRotation = m_controlRotation;
	}

	bool m_controlRotation;
};

void ndAssetEditor::ShowPropertiesJointInfo()
{
	if (ImGui::CollapsingHeader("Constraint joint"))
	{
		ndSharedPtr<ndMeshJoint> joint (m_currentSelection->GetJoint());
		if (ImGui::BeginCombo("joints", joint->m_constructor.GetStr()))
		{
			auto SetDropdownList = [this, &joint](const char* const name)
			{
				bool selected = strcmp(name, joint->m_constructor.GetStr()) ? false : true;
				if (ImGui::Selectable(name, selected))
				{
					auto InitNewJoint = [this, &joint](ndSharedPtr<ndJointBilateralConstraint>& newJoint)
					{
						newJoint->SetLocalMatrix0(joint->m_localFrame0);
						newJoint->SetLocalMatrix1(joint->m_localFrame1);
						m_currentSelection->SetJoint(newJoint->GetMeshJoint(*m_currentSelection));
						joint = m_currentSelection->GetJoint();
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
					};
					if (strcmp(name, ndJointHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointHinge());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointSlider::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSlider());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointPlane::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointPlane());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointDoubleHinge::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointDoubleHinge());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointSpherical::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointSpherical());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointFix6dof::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointFix6dof());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointRoller::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointRoller());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointCylinder::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointCylinder());
						InitNewJoint(newJoint);
					}
					else if (strcmp(name, ndJointWheel::StaticClassName()) == 0)
					{
						m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointChange(this, m_currentSelection)));
						ndSharedPtr<ndJointBilateralConstraint> newJoint(new ndJointWheel());
						InitNewJoint(newJoint);
					}
					else
					{
						ndAssert(0);
					}
				}
			};
			SetDropdownList(ndJointHinge::StaticClassName());
			SetDropdownList(ndJointSlider::StaticClassName());
			SetDropdownList(ndJointPlane::StaticClassName());
			SetDropdownList(ndJointRoller::StaticClassName());
			SetDropdownList(ndJointCylinder::StaticClassName());
			SetDropdownList(ndJointDoubleHinge::StaticClassName());
			SetDropdownList(ndJointWheel::StaticClassName());
			SetDropdownList(ndJointSpherical::StaticClassName());
			SetDropdownList(ndJointFix6dof::StaticClassName());

			ImGui::EndCombo();
		}

		if (strcmp(joint->m_constructor.GetStr(), ndJointFix6dof::StaticClassName()) == 0)
		{
			ndMeshJointFix6dof* const subJoint = (ndMeshJointFix6dof*)*joint;
			ndReal value = subJoint->m_softness;
			if (ImGui::InputFloat("softness", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
				subJoint->m_softness = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
			}
			value = subJoint->m_maxForce;
			if (ImGui::InputFloat("max Force", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
				subJoint->m_maxForce = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
			}
			value = subJoint->m_maxTorque;
			if (ImGui::InputFloat("max_torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
				subJoint->m_maxTorque = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointFix6dof(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointHinge::StaticClassName()) == 0)
		{
			ndMeshJointHinge* const subJoint = (ndMeshJointHinge*)*joint;
			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("spring const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("damper const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("min limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("max limit", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			bool limitState = subJoint->m_axis.m_limitState ? true : false;
			if (ImGui::Checkbox("limit State", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_limitState = m_showSelectedNode ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSlider::StaticClassName()) == 0)
		{
			ndMeshJointSlider* const subJoint = (ndMeshJointSlider*)*joint;
			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("spring const##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("damper const##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("regularizer##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("min limit##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("max limit##1", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
			bool limitState = subJoint->m_axis.m_limitState ? true : false;
			if (ImGui::Checkbox("limit State##1", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
				subJoint->m_axis.m_limitState = m_showSelectedNode ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointHinge(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointDoubleHinge::StaticClassName()) == 0)
		{
			ndMeshJointDoubleHinge* const subJoint = (ndMeshJointDoubleHinge*)*joint;
			ImGui::SeparatorText("child pin");
			{
				ndReal value = subJoint->m_axis0.m_springK;
				if (ImGui::InputFloat("spring const##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_springK = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_damperC;
				if (ImGui::InputFloat("damper const##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_minLimit;
				if (ImGui::InputFloat("min limit##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_maxLimit;
				if (ImGui::InputFloat("max limit##2", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_axis0.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##2", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis0.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
			}
			ImGui::SeparatorText("parent pin");
			{
				ndReal value = subJoint->m_axis1.m_springK;
				if (ImGui::InputFloat("spring const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_springK = ndMax (value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_damperC;
				if (ImGui::InputFloat("damper const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_springDamperRegularizer = ndMax (value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_minLimit;
				if (ImGui::InputFloat("min limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_maxLimit;
				if (ImGui::InputFloat("max limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_axis1.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##3", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
					subJoint->m_axis1.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointDoubleHinge(this, m_currentSelection)));
				}
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointRoller::StaticClassName()) == 0)
		{
			ndMeshJointRoller* const subJoint = (ndMeshJointRoller*)*joint;
			ImGui::SeparatorText("child pin");
			{
				ndReal value = subJoint->m_positAxis.m_springK;
				if (ImGui::InputFloat("spring const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_positAxis.m_springK = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_positAxis.m_damperC;
				if (ImGui::InputFloat("damper const##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_positAxis.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_positAxis.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_positAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_positAxis.m_minLimit;
				if (ImGui::InputFloat("min limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_positAxis.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_positAxis.m_maxLimit;
				if (ImGui::InputFloat("max limit##3", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_positAxis.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_positAxis.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##3", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_positAxis.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
			}
			ImGui::SeparatorText("parent pin");
			{
				ndReal value = subJoint->m_angleAxis.m_springK;
				if (ImGui::InputFloat("spring const##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_angleAxis.m_springK = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_angleAxis.m_damperC;
				if (ImGui::InputFloat("damper const##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_angleAxis.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_angleAxis.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_angleAxis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_angleAxis.m_minLimit;
				if (ImGui::InputFloat("min limit##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_angleAxis.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				value = subJoint->m_angleAxis.m_maxLimit;
				if (ImGui::InputFloat("max limit##4", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_angleAxis.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_angleAxis.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##4", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
					subJoint->m_angleAxis.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointRoller(this, m_currentSelection)));
				}
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointCylinder::StaticClassName()) == 0)
		{
			ndMeshJointCylinder* const subJoint = (ndMeshJointCylinder*)*joint;
			ImGui::SeparatorText("child pin");
			{
				ndReal value = subJoint->m_axis0.m_springK;
				if (ImGui::InputFloat("spring const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis0.m_springK = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_damperC;
				if (ImGui::InputFloat("damper const##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis0.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis0.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_minLimit;
				if (ImGui::InputFloat("min limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis0.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis0.m_maxLimit;
				if (ImGui::InputFloat("max limit##5", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis0.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_axis0.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##5", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis0.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
			}
			ImGui::SeparatorText("parent pin");
			{
				ndReal value = subJoint->m_axis1.m_springK;
				if (ImGui::InputFloat("spring const##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis1.m_springK = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_damperC;
				if (ImGui::InputFloat("damper const##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis1.m_damperC = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_springDamperRegularizer;
				if (ImGui::InputFloat("regularizer##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis1.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_minLimit;
				if (ImGui::InputFloat("min limit##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis1.m_minLimit = ndMin(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				value = subJoint->m_axis1.m_maxLimit;
				if (ImGui::InputFloat("max limit##6", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis1.m_maxLimit = ndMax(value, ndReal(0.0f));
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
				bool limitState = subJoint->m_axis1.m_limitState ? true : false;
				if (ImGui::Checkbox("limit State##6", &limitState))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
					subJoint->m_axis1.m_limitState = m_showSelectedNode ? 1 : 0;
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointCylinder(this, m_currentSelection)));
				}
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointWheel::StaticClassName()) == 0)
		{
			ndMeshJointWheel* const subJoint = (ndMeshJointWheel*)*joint;
			ImGui::SeparatorText("baseFrame");

			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("suspension spring", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("suspension const", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("suspension regularizer", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("lower stop", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("upper stop", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
			value = subJoint->m_steeringAngle;
			if (ImGui::InputFloat("steering angle", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_steeringAngle = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}

			value = subJoint->m_brakeTorque;
			if (ImGui::InputFloat("brake torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_brakeTorque = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
			value = subJoint->m_handBrakeTorque;
			if (ImGui::InputFloat("hand brake torque", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
				subJoint->m_handBrakeTorque = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointWheel(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointSpherical::StaticClassName()) == 0)
		{
			ndMeshJointSpherical* const subJoint = (ndMeshJointSpherical*)*joint;
			ndReal value = subJoint->m_axis.m_springK;
			if (ImGui::InputFloat("spring const##8", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_springK = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_damperC;
			if (ImGui::InputFloat("damper const##8", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_damperC = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_springDamperRegularizer;
			if (ImGui::InputFloat("regularizer##8", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_springDamperRegularizer = ndMax(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_maxConeAngle;
			if (ImGui::InputFloat("max cone angle", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_maxConeAngle = ndClamp (value, ndReal(0.0), ndReal(180.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			bool limitState = subJoint->m_coneAngleState ? true : false;;
			if (ImGui::Checkbox("cone limit state", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_coneAngleState = limitState;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}

			value = subJoint->m_axis.m_minLimit;
			if (ImGui::InputFloat("min twist", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_minLimit = ndMin(value, ndReal(0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			value = subJoint->m_axis.m_maxLimit;
			if (ImGui::InputFloat("max twist", &value, 0.0, 0.0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_maxLimit = ndMax (value, ndReal (0.0f));
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
			limitState = subJoint->m_axis.m_limitState ? true : false;
			if (ImGui::Checkbox("twist limit state", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
				subJoint->m_axis.m_limitState = m_showSelectedNode ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointSpherical(this, m_currentSelection)));
			}
		}
		else if (strcmp(joint->m_constructor.GetStr(), ndJointPlane::StaticClassName()) == 0)
		{
			ndMeshJointPlane* const subJoint = (ndMeshJointPlane*)*joint;
			bool limitState = subJoint->m_controlRotation ? true : false;
			if (ImGui::Checkbox("control rotation", &limitState))
			{
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointPlane(this, m_currentSelection)));
				subJoint->m_controlRotation = limitState ? 1 : 0;
				m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJointPlane(this, m_currentSelection)));
			}
		}
		else
		{
			ndAssert(0);
		}

		// child local frame
		{
			ImGui::SeparatorText("child local Frame");

			if (m_showPreTransform)
			{
				const ndMatrix matrix(joint->m_localFrame0);
				ndReal position[3];
				position[0] = ndReal(matrix.m_posit.m_x);
				position[1] = ndReal(matrix.m_posit.m_y);
				position[2] = ndReal(matrix.m_posit.m_z);
				if (ImGui::InputFloat3("position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localFrame0(joint->m_localFrame0);
					localFrame0.m_posit.m_x = position[0];
					localFrame0.m_posit.m_y = position[1];
					localFrame0.m_posit.m_z = position[2];

					ndMatrix globalMatrix(localFrame0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localFrame1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

					joint->m_localFrame0 = localFrame0;
					joint->m_localFrame1 = localFrame1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}

				ndReal euler[3];
				ndVector tmp;
				ndVector radians(matrix.CalcPitchYawRoll(tmp).Scale(ndRadToDegree));
				euler[0] = ndReal(radians[0]);
				euler[1] = ndReal(radians[1]);
				euler[2] = ndReal(radians[2]);

				if (ImGui::InputFloat3("rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad));
					localMatrix0.m_posit = joint->m_localFrame0.m_posit;
					ndMatrix globalMatrix(localMatrix0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localMatrix1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());
					localMatrix0.m_posit = joint->m_localFrame0.m_posit;

					joint->m_localFrame0 = localMatrix0;
					joint->m_localFrame1 = localMatrix1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}
			}
			else
			{
				ndReal position[3];
				position[0] = ndReal(0.0f);
				position[1] = ndReal(0.0f);
				position[2] = ndReal(0.0f);
				if (ImGui::InputFloat3("rel position##2", position, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localFrame0(joint->m_localFrame0);
					const ndVector delta(position[0], position[1], position[2], ndFloat32(0.0f));
					localFrame0.m_posit += localFrame0.RotateVector(delta);

					ndMatrix globalMatrix(localFrame0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localFrame1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());

					joint->m_localFrame0 = localFrame0;
					joint->m_localFrame1 = localFrame1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}

				ndReal euler[3];
				euler[0] = ndReal(0.0f);
				euler[1] = ndReal(0.0f);
				euler[2] = ndReal(0.0f);

				if (ImGui::InputFloat3("rel rotation##2", euler, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue))
				{
					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
					ndMatrix localMatrix0(ndPitchMatrix(euler[0] * ndDegreeToRad) * ndYawMatrix(euler[1] * ndDegreeToRad) * ndRollMatrix(euler[2] * ndDegreeToRad) * joint->m_localFrame0);
					ndMatrix globalMatrix(localMatrix0 * m_currentSelection->CalculateGlobalMatrix());
					ndMatrix localMatrix1(globalMatrix * m_currentSelection->GetParent()->CalculateGlobalMatrix().OrthoInverse());
					localMatrix0.m_posit = joint->m_localFrame0.m_posit;

					joint->m_localFrame0 = localMatrix0;
					joint->m_localFrame1 = localMatrix1;

					m_undoRedo.Push(ndSharedPtr<ndUndoRedoCommand>(new ndUndoRedoJoint(this, m_currentSelection)));
				}
			}
		}
	}
}

