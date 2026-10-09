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

static void* NewtonDefaultAlloc(int sizeInBytes)
{
	return malloc(sizeInBytes);
}

static void NewtonDefaultFree(void* const ptr, int sizeInBytes)
{
	free(ptr);
}

NewtonFreeMemory newtonFree = NewtonDefaultFree;
NewtonAllocMemory newtonAlloc = NewtonDefaultAlloc;

static void* ndNewtonAllocator(size_t size)
{
	return newtonAlloc(int(size));
}

static void ndNewtonFree(void* const ptr)
{
	newtonFree(ptr, int(ndMemory::GetSize(ptr)));
}

void* NewtonAlloc(int sizeInBytes)
{
	return ndMemory::Malloc(sizeInBytes);
}

void NewtonFree(void* const ptr)
{
	ndMemory::Free(ptr);
}

#if !defined (_NEWTON_STATIC_LIB) 
void* operator new(std::size_t count)
{
	return ndMemory::Malloc(count);
}

void operator delete(void* ptr) noexcept
{
	ndMemory::Free(ptr);
}
#endif

bool CheckFloat(ndFloat32* ptr, ndInt32 size)
{
	for (ndInt32 i = 0; i < size; ++i)
	{
		if (!_finite(ptr[i]) || _isnan(ptr[i]))
		{
			return false;
		}
	}
	return true;
}

// fixme: needs docu
// @param mallocFnt is a pointer to the memory allocator callback function. If this parameter is NULL the standard *malloc* function is used.
// @param mfreeFnt is a pointer to the memory release callback function. If this parameter is NULL the standard *free* function is used.
//
void NewtonSetMemorySystem(NewtonAllocMemory mallocFnt, NewtonFreeMemory mfreeFnt)
{
	newtonFree = mfreeFnt;
	newtonAlloc = mallocFnt;
	ndMemory::SetMemoryAllocators(ndNewtonAllocator, ndNewtonFree);
}

/*!
  Return the exact amount of memory (in Bytes) use by the engine at any given time.

  @return total memory use by the engine.

  Applications can use this function to ascertain that the memory use by the
  engine is balanced at all times.

  See also: ::NewtonCreate
*/
int NewtonGetMemoryUsed()
{
	TRACE_FUNCTION(__FUNCTION__);
	return int(ndMemory::GetMemoryUsed());
}

/*!
  Return the current library version number.

  @return version number as an integer, eg 314.

  The version number is a three-digit integer.

  First digit:  major version (interface changes among other things)
  Second digit: major patch number (new features, and bug fixes)
  Third Digit:  minor bug fixed patch.
*/
int NewtonWorldGetVersion()
{
	TRACE_FUNCTION(__FUNCTION__);
	return NEWTON_MAJOR_VERSION * 100 + NEWTON_MINOR_VERSION;
}

/*!
  Return the size of a Newton dFloat in bytes.

  @return sizeof(dFloat)
*/
int NewtonWorldFloatSize()
{
	TRACE_FUNCTION(__FUNCTION__);
	return sizeof(ndFloat32);
}


/*!
  Get the three Euler angles from a 4x4 rotation matrix arranged in row-major order.

  @param matrix pointer to the 4x4 rotation matrix.
  @param  angles0 - fixme
  @param  angles1 - pointer to an array of at least three floats to hold the Euler angles.

  @return Nothing.

  The motivation for this function is that many graphics engines still use Euler angles to represent the orientation
  of graphics entities.
  The angles are expressed in radians and represent:
  *angle[0]* - rotation about first matrix row
  *angle[1]* - rotation about second matrix row
  *angle[2]* - rotation about third matrix row

  See also: ::NewtonSetEulerAngle
*/
void NewtonGetEulerAngle(const dFloat* const matrix, dFloat* const angles0, dFloat* const angles1)
{
	TRACE_FUNCTION(__FUNCTION__);

	//dgMatrix mat(matrix);
	//
	//dgVector euler0;
	//dgVector euler1;
	//mat.CalcPitchYawRoll(euler0, euler1);
	//
	//angles0[0] = euler0.m_x;
	//angles0[1] = euler0.m_y;
	//angles0[2] = euler0.m_z;
	//
	//angles1[0] = euler1.m_x;
	//angles1[1] = euler1.m_y;
	//angles1[2] = euler1.m_z;
	ndAssert(0);
}


/*!
  Build a rotation matrix from the Euler angles in radians.

  @param matrix pointer to the 4x4 rotation matrix.
  @param angles pointer to an array of at least three floats to hold the Euler angles.

  @return Nothing.

  The motivation for this function is that many graphics engines still use Euler angles to represent the orientation
  of graphics entities.
  The angles are expressed in radians and represent:
  *angle[0]* - rotation about first matrix row
  *angle[1]* - rotation about second matrix row
  *angle[2]* - rotation about third matrix row

  See also: ::NewtonGetEulerAngle
*/
void NewtonSetEulerAngle(const dFloat* const angles, dFloat* const matrix)
{
	TRACE_FUNCTION(__FUNCTION__);
	//dgMatrix mat(dgPitchMatrix(angles[0]) * dgYawMatrix(angles[1]) * dgRollMatrix(angles[2]));
	////dgMatrix retMatrix (matrix);
	//dgMatrix& retMatrix = *((dgMatrix*)matrix);
	//
	//for (dgInt32 i = 0; i < 3; i++) {
	//	retMatrix[3][i] = 0.0f;
	//	for (dgInt32 j = 0; j < 4; j++) {
	//		retMatrix[i][j] = mat[i][j];
	//	}
	//}
	//retMatrix[3][3] = dgFloat32(1.0f);
	ndAssert(0);
}


/*!
  Calculates the acceleration to satisfy the specified the spring damper system.

  @param dt integration time step.
  @param ks spring stiffness, it must be a positive value.
  @param x spring position.
  @param kd desired spring damper, it must be a positive value.
  @param s spring velocity.

  return: the spring acceleration.

  the acceleration calculated by this function represent the mass, spring system of the form
  a = -ks * x - kd * v.
*/
dFloat NewtonCalculateSpringDamperAcceleration(dFloat dt, dFloat ks, dFloat x, dFloat kd, dFloat v)
{
	TRACE_FUNCTION(__FUNCTION__);
	////at = - (ks * x + kd * v);
	////at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	//dgFloat32 ksd = dt * ks;
	//dgFloat32 num = ks * x + kd * v + ksd * v;
	//dgFloat32 den = dgFloat32(1.0f) + dt * kd + dt * ksd;
	//dgAssert(den > 0.0f);
	//dFloat accel = -num / den;
	//return accel;
	ndAssert(0);
	return 0;
}


NewtonCollision* NewtonCreateMassSpringDamperSystem(const NewtonWorld* const newtonWorld, int shapeID,
	const dFloat* const points, int pointCount, int strideInBytes, const dFloat* const pointMass,
	const int* const links, int linksCount, const dFloat* const linksSpring, const dFloat* const linksDamper)
{
	TRACE_FUNCTION(__FUNCTION__);
	//Newton* const world = (Newton*)newtonWorld;
	//return (NewtonCollision*)world->CreateMassSpringDamperSystem(shapeID, pointCount, points, strideInBytes, pointMass, linksCount, links, linksSpring, linksDamper);
	ndAssert(0);
	return 0;
}

int NewtonAtomicSwap(int* const ptr, int value)
{
	TRACE_FUNCTION(__FUNCTION__);

	ndScopeSpinLock lock(ndNewtonWorld::m_globalCriticalSection);
	ndInt32 ret = *ptr;
	*ptr = value;
	return ret;
}

void NewtonYield()
{
	ndInt32 loop = 0;
	ndThreadYield(loop);
}

