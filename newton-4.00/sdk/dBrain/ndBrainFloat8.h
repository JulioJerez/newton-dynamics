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

#ifndef _ND_BRAIN_FLOAT8_H__
#define _ND_BRAIN_FLOAT8_H__

#include "ndBrainStdafx.h"

// Minimal simd support.
// this class is only defined for avx instructions 
// this is because the neural net code make use of misaligned read and writes, 
// which cause segment fault when using SS, but not when using AVX.
// the second reason it is the only way I found to make use of some 
// simd intrinsics in visual studio, like tanh, which is expensive.
 
// the good part is that visual studio 2022 is capable of generate the simd version of the class without having to 
// use explicit simd code. 
// This is much better since is let the compiler do other optimizations.

D_MSV_NEWTON_CLASS_ALIGN_32
class ndBrainFloat8 
{
	public: 
	ndBrainFloat8();
	ndBrainFloat8(const ndBrainFloat a);
	ndBrainFloat8(const ndBrainFloat* const ptr);
	ndBrainFloat8(const ndInt32* const indexArray);
	ndBrainFloat8(const ndBrainFloat* const ptr, const ndBrainFloat8& index);

	ndBrainFloat HorizontalAdd() const;
	void Store(ndBrainFloat* const ptr) const;
	void Store(ndBrainFloat* const ptr, const ndBrainFloat8& index) const;

	ndBrainFloat8 Tanh() const;
	ndBrainFloat8 Min(const ndBrainFloat8& src) const;
	ndBrainFloat8 Max(const ndBrainFloat8& src) const;
	ndBrainFloat8 Clamp(const ndBrainFloat8& min, const ndBrainFloat8& max) const;
	
	ndBrainFloat8 operator+ (const ndBrainFloat8& A) const;
	ndBrainFloat8 operator- (const ndBrainFloat8& A) const;
	ndBrainFloat8 operator* (const ndBrainFloat8& A) const;
	
	// logical operations;
	ndBrainFloat8 operator~ () const;
	ndBrainFloat8 operator& (const ndBrainFloat8& data) const;
	ndBrainFloat8 operator| (const ndBrainFloat8& data) const;
	ndBrainFloat8 operator> (const ndBrainFloat8& data) const;
	ndBrainFloat8 operator< (const ndBrainFloat8& data) const;
	ndBrainFloat8 operator>= (const ndBrainFloat8& data) const;
	ndBrainFloat8 operator<= (const ndBrainFloat8& data) const;

	//static void Transpose(
	//	ndBrainFloat8& dst0, ndBrainFloat8& dst1, ndBrainFloat8& dst2, ndBrainFloat8& dst3,
	//	ndBrainFloat8& dst4, ndBrainFloat8& dst5, ndBrainFloat8& dst6, ndBrainFloat8& dst7,
	//	const ndBrainFloat8& src0, const ndBrainFloat8& src1, const ndBrainFloat8& src2, const ndBrainFloat8& src3,
	//	const ndBrainFloat8& src4, const ndBrainFloat8& src5, const ndBrainFloat8& src6, const ndBrainFloat8& src7)
	//{
	//	ndBrainFloat8 dst[8];
	//	ndBrainFloat8 src[8];
	//
	//	src[0] = src0;
	//	src[1] = src1;
	//	src[2] = src2;
	//	src[3] = src3;
	//	src[4] = src4;
	//	src[5] = src5;
	//	src[6] = src6;
	//	src[7] = src7;
	//
	//	for (ndInt32 j = 0; j < 8; ++j)
	//	{
	//		for (ndInt32 i = 0; i < 8; ++i)
	//		{
	//			dst[i].m_f[j] = src[j].m_f[i];
	//		}
	//	}
	//
	//	dst0 = dst[0];
	//	dst1 = dst[1];
	//	dst2 = dst[2];
	//	dst3 = dst[3];
	//	dst4 = dst[4];
	//	dst5 = dst[5];
	//	dst6 = dst[6];
	//	dst7 = dst[7];
	//}

	union
	{
		ndBrainFloat m_f[8];
		ndInt32 m_i[8];
	};

} D_GCC_NEWTON_CLASS_ALIGN_32;

inline ndBrainFloat8::ndBrainFloat8()
{
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat a)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = a;
	}
}

inline ndBrainFloat8::ndBrainFloat8(const ndInt32* const indexArray)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_i[i] = indexArray[i];
	}
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat* const ptr)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = ptr[i];
	}
}

inline ndBrainFloat8::ndBrainFloat8(const ndBrainFloat* const baseAddr, const ndBrainFloat8& index)
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		m_f[i] = baseAddr[index.m_i[i]];
	}
}

inline ndBrainFloat ndBrainFloat8::HorizontalAdd() const
{
	ndBrainFloat acc = ndBrainFloat(0.0f);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		acc += m_f[i];
	}
	return acc;
}

inline void ndBrainFloat8::Store(ndBrainFloat* const ptr) const
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		ptr[i] = m_f[i];
	}
}

inline void ndBrainFloat8::Store(ndBrainFloat* const dstAddress, const ndBrainFloat8& index) const
{
	for (ndInt32 i = 0; i < 8; ++i)
	{
		dstAddress[index.m_i[i]] = m_f[i];
	}
}

inline ndBrainFloat8 ndBrainFloat8::Clamp(const ndBrainFloat8& min, const ndBrainFloat8& max) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndClamp(m_f[i], min.m_f[i], max.m_f[i]);
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::Min(const ndBrainFloat8& min) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndMin(m_f[i], min.m_f[i]);
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::Max(const ndBrainFloat8& max) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndMax(m_f[i], max.m_f[i]);
		ndAssert(tmp.m_f[i] <= ndFloat32(1000.0f));
		ndAssert(tmp.m_f[i] >= ndFloat32(-1000.0f));
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::Tanh() const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = ndBrainFloat(ndTanh(m_f[i]));
		ndAssert(tmp.m_f[i] <= ndFloat32(1000.0f));
		ndAssert(tmp.m_f[i] >= ndFloat32(-1000.0f));
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator+ (const ndBrainFloat8& A) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = m_f[i] + A.m_f[i];
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator- (const ndBrainFloat8& A) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = m_f[i] - A.m_f[i];
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator* (const ndBrainFloat8& A) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_f[i] = m_f[i] * A.m_f[i];
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator> (const ndBrainFloat8& data) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] > data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator>= (const ndBrainFloat8& data) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] >= data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator< (const ndBrainFloat8& data) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] < data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator<= (const ndBrainFloat8& data) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_f[i] <= data.m_f[i] ? -1 : 0;
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator~ () const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = ~m_i[i];
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator| (const ndBrainFloat8& data) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_i[i] | data.m_i[i];
	}
	return tmp;
}

inline ndBrainFloat8 ndBrainFloat8::operator& (const ndBrainFloat8& data) const
{
	ndBrainFloat8 tmp;
	for (ndInt32 i = 0; i < 8; ++i)
	{
		tmp.m_i[i] = m_i[i] & data.m_i[i];
	}
	return tmp;
}

#endif 
