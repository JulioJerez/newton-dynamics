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

NewtonMesh* NewtonMeshCreate(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndSharedPtr<ndMeshEffect>* const mesh = new ndSharedPtr<ndMeshEffect>(new ndMeshEffect());
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
