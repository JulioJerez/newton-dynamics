/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_MESH_COMPONENTS_H__
#define __ND_MESH_COMPONENTS_H__

#include "ndNewtonStdafx.h"

class ndBody;
class ndMeshEffect;
class ndShapeInstance;
class ndJointBilateralConstraint;

class ndMeshBodyDynamic : public ndMeshBodyKinematic
{
	public:
	D_NEWTON_API ndMeshBodyDynamic();

	D_NEWTON_API virtual ndBody* CreateObject() const override;
	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const override;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent) override;
	ndVector m_intrinsicDamping;
};

class ndMeshJointFix6dof : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointFix6dof();
	D_NEWTON_API ndMeshJointFix6dof(const ndJointBilateralConstraint* const joint);

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndFloat32 m_softness;
	ndFloat32 m_maxForce;
	ndFloat32 m_maxTorque;
};

class ndMeshJointHinge : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointHinge();
	D_NEWTON_API ndMeshJointHinge(const ndJointBilateralConstraint* const joint);

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_springDamperRegularizer;
	ndInt8 m_limitState;
};

class ndMeshJointDoubleHinge : public ndMeshJoint
{
	public:
	class ndAxis
	{
		public:
		ndFloat32 m_springK;
		ndFloat32 m_damperC;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
		ndFloat32 m_springDamperRegularizer;
		ndInt8 m_limitState;
	};

	D_NEWTON_API ndMeshJointDoubleHinge();
	D_NEWTON_API ndMeshJointDoubleHinge(const ndJointBilateralConstraint* const joint);

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndAxis m_axis0;
	ndAxis m_axis1;
};

class ndMeshJointSlider : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointSlider();
	D_NEWTON_API ndMeshJointSlider(const ndJointBilateralConstraint* const joint);

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);

	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_springDamperRegularizer;
	ndInt8 m_limitState;
};

class ndMeshJointSpherical : public ndMeshJoint
{
	public:
	D_NEWTON_API ndMeshJointSpherical();
	D_NEWTON_API ndMeshJointSpherical(const ndJointBilateralConstraint* const joint);

	D_NEWTON_API virtual void SerializeToXml(nd::TiXmlElement* const parent) const;
	D_NEWTON_API virtual void DeserializeFromXml(const nd::TiXmlElement* const parent);
	D_NEWTON_API virtual ndJointBilateralConstraint* CreateObject(ndBodyKinematic* const child, ndBodyKinematic* const parent) const override;

	ndMatrix m_rotation;
	ndFloat32 m_springK;
	ndFloat32 m_damperC;
	ndFloat32 m_maxConeAngle;
	ndFloat32 m_minTwistAngle;
	ndFloat32 m_maxTwistAngle;
	ndFloat32 m_springDamperRegularizer;
};

#endif

