/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef D_NEWTON_WORLD_H_
#define D_NEWTON_WORLD_H_

#include "newtonStdafx.h"

class NewtonBody;
class NewtonWorld;
class NewtonJoint;

class ndNewtonWorld: public ndWorld
{
	public:
	typedef void (*NewtonPostUpdateCallback) (const NewtonWorld* const world, ndFloat32 timestep);
	typedef void (*NewtonSerializeCallback) (void* const serializeHandle, const void* const buffer, int size);
	typedef void (*NewtonDeserializeCallback) (void* const serializeHandle, void* const buffer, int size);

	typedef void (*NewtonOnJointSerializationCallback) (const NewtonJoint* const joint, NewtonSerializeCallback function, void* const serializeHandle);
	typedef void (*NewtonOnJointDeserializationCallback) (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback function, void* const serializeHandle);
	typedef void (*NewtonOnBodySerializationCallback) (NewtonBody* const body, void* const userData, NewtonSerializeCallback function, void* const serializeHandle);
	typedef void (*NewtonOnBodyDeserializationCallback) (NewtonBody* const body, void* const userData, NewtonDeserializeCallback function, void* const serializeHandle);

	typedef void(*NewtonCreateContactCallback) (const NewtonWorld* const newtonWorld, NewtonJoint* const contact);
	typedef void(*NewtonDestroyContactCallback) (const NewtonWorld* const newtonWorld, NewtonJoint* const contact);


	ndNewtonWorld();
	virtual ~ndNewtonWorld() override;

	void ClearMaterials();
	ndMaterial* GetMaterial(int id0, int id1) const;
	
	virtual void Update(ndFloat32 timestep) override;
	virtual void PostUpdate(ndFloat32 timestep) override;

	virtual void OnAddBody(ndBody* const body) const override;
	virtual void OnRemoveBody(ndBody* const body) const override;

	ndWeakPtr<void> m_userData;
	ndInt32 m_bodyMaterialGroup;

	NewtonPostUpdateCallback m_onPostUpdate;
	NewtonOnJointSerializationCallback m_onJointSerialize;
	NewtonOnJointDeserializationCallback m_onJointDeserialize;
	NewtonOnJointSerializationCallback m_onBodySerialize;
	NewtonOnJointDeserializationCallback m_onBodyDeserialize;

	NewtonCreateContactCallback m_onCreateContact;
	NewtonDestroyContactCallback m_onDestroyContact;

	static ndSpinLock m_globalCriticalSection;
};

#endif