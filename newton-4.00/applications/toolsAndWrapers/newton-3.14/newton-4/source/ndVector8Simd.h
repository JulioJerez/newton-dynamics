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

#ifndef ND_VECTOR8_SIMD_H_
#define ND_VECTOR8_SIMD_H_

#include "ndVector.h"

#ifdef D_NEWTON_USE_DOUBLE
	#if defined(D_NEWTON_USE_AVX2_OPTION)

		D_MSV_NEWTON_CLASS_ALIGN_32
		class ndVector8
		{
			public:
			inline ndVector8()
			{
			}

			inline ndVector8(const ndFloat64 val)
				:m_simdFloat(val)
			{
			}

			inline ndVector8(const ndInt32 val)
				:m_simdFloat(val)
			{
			}

			inline ndVector8(const __m256d low, const __m256d high)
				:m_simdFloat(low, high)
			{
			}

			inline ndVector8(const ndVector8& copy)
				:m_simdFloat(copy.m_simdFloat)
			{
			}

			inline ndVector8(const ndVector& low, const ndVector& high)
				:m_simdFloat(low, high)
			{
			}

			inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
				:m_simdFloat(baseAddr, index)
			{
			}

			inline ndFloat32& operator[] (ndInt32 i)
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline const ndFloat32& operator[] (ndInt32 i) const
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline ndVector8& operator= (const ndVector8& A)
			{
				m_simdFloat.m_low = A.m_simdFloat.m_low;
				m_simdFloat.m_high = A.m_simdFloat.m_high;
				return *this;
			}

			inline ndVector8 operator+ (const ndVector8& A) const
			{
				return ndVector8(_mm256_add_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_add_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 operator- (const ndVector8& A) const
			{
				return ndVector8(_mm256_sub_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_sub_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 operator* (const ndVector8& A) const
			{
				return ndVector8(_mm256_mul_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_mul_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
			{
				return ndVector8(_mm256_fmadd_pd(A.m_simdFloat.m_low, B.m_simdFloat.m_low, m_simdFloat.m_low), _mm256_fmadd_pd(A.m_simdFloat.m_high, B.m_simdFloat.m_high, m_simdFloat.m_high));
			}

			inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
			{
				return ndVector8(_mm256_fnmadd_pd(A.m_simdFloat.m_low, B.m_simdFloat.m_low, m_simdFloat.m_low), _mm256_fnmadd_pd(A.m_simdFloat.m_high, B.m_simdFloat.m_high, m_simdFloat.m_high));
			}

			inline ndVector8 operator> (const ndVector8& A) const
			{
				return ndVector8(_mm256_cmp_pd(m_simdFloat.m_low, A.m_simdFloat.m_low, _CMP_GT_OQ), _mm256_cmp_pd(m_simdFloat.m_high, A.m_simdFloat.m_high, _CMP_GT_OQ));
			}

			inline ndVector8 operator< (const ndVector8& A) const
			{
				return ndVector8(_mm256_cmp_pd(m_simdFloat.m_low, A.m_simdFloat.m_low, _CMP_LT_OQ), _mm256_cmp_pd(m_simdFloat.m_high, A.m_simdFloat.m_high, _CMP_LT_OQ));
			}

			inline ndVector8 operator| (const ndVector8& A) const
			{
				return ndVector8(_mm256_or_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_or_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 operator& (const ndVector8& A) const
			{
				return ndVector8(_mm256_and_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_and_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 GetMin(const ndVector8& A) const
			{
				return ndVector8(_mm256_min_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_min_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 GetMax(const ndVector8& A) const
			{
				return ndVector8(_mm256_max_pd(m_simdFloat.m_low, A.m_simdFloat.m_low), _mm256_max_pd(m_simdFloat.m_high, A.m_simdFloat.m_high));
			}

			inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
			{
				// (((b ^ a) & mask)^a)
				//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
				//return  _mm256_xor_ps(m_type, _mm256_and_ps(mask.m_type, _mm256_xor_ps(m_type, data.m_type)));
				__m256d low(_mm256_xor_pd(m_simdFloat.m_low, _mm256_and_pd(mask.m_simdFloat.m_low, _mm256_xor_pd(m_simdFloat.m_low, data.m_simdFloat.m_low))));
				__m256d high(_mm256_xor_pd(m_simdFloat.m_high, _mm256_and_pd(mask.m_simdFloat.m_high, _mm256_xor_pd(m_simdFloat.m_high, data.m_simdFloat.m_high))));
				return ndVector8(low, high);
			}

			inline ndVector GetLow() const
			{
				return m_vector.m_linear;
			}

			inline ndVector GetHigh() const
			{
				return m_vector.m_angular;
			}

			inline ndFloat32 GetMax() const
			{
				__m256d tmp0(_mm256_max_pd(m_simdFloat.m_low, m_simdFloat.m_high));
				__m256d tmp1(_mm256_max_pd(tmp0, _mm256_permute2f128_pd(tmp0, tmp0, 1)));
				__m256d tmp2(_mm256_max_pd(tmp1, _mm256_unpackhi_pd(tmp1, tmp1)));
				return _mm256_cvtsd_f64(tmp2);
			}

			inline ndFloat32 AddHorizontal() const
			{
				__m256d tmp0(_mm256_add_pd(m_simdFloat.m_low, m_simdFloat.m_high));
				__m256d tmp1(_mm256_add_pd(tmp0, _mm256_permute2f128_pd(tmp0, tmp0, 1)));
				__m256d tmp2(_mm256_add_pd(tmp1, _mm256_unpackhi_pd(tmp1, tmp1)));
				return _mm256_cvtsd_f64(tmp2);
			}

			static inline void FlushRegisters()
			{
				_mm256_zeroall();
			}

			static inline void Transpose(
				__m256d& dst0, __m256d& dst1, __m256d& dst2, __m256d& dst3,
				const __m256d& src0, const __m256d& src1, const __m256d& src2, const __m256d& src3)
			{
				__m256d r0(_mm256_permute2f128_pd(src0, src2, 0x20));
				__m256d r1(_mm256_permute2f128_pd(src1, src3, 0x20));
				__m256d r2(_mm256_permute2f128_pd(src0, src2, 0x31));
				__m256d r3(_mm256_permute2f128_pd(src1, src3, 0x31));

				dst0 = _mm256_unpacklo_pd(r0, r1);
				dst1 = _mm256_unpackhi_pd(r0, r1);
				dst2 = _mm256_unpacklo_pd(r2, r3);
				dst3 = _mm256_unpackhi_pd(r2, r3);
			}

			static inline void Transpose(
				ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
				ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
				const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
				const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
			{
				Transpose(
					dst0.m_simdFloat.m_low, dst1.m_simdFloat.m_low, dst2.m_simdFloat.m_low, dst3.m_simdFloat.m_low,
					src0.m_simdFloat.m_low, src1.m_simdFloat.m_low, src2.m_simdFloat.m_low, src3.m_simdFloat.m_low);

				Transpose(
					dst0.m_simdFloat.m_high, dst1.m_simdFloat.m_high, dst2.m_simdFloat.m_high, dst3.m_simdFloat.m_high,
					src4.m_simdFloat.m_low, src5.m_simdFloat.m_low, src6.m_simdFloat.m_low, src7.m_simdFloat.m_low);

				Transpose(
					dst4.m_simdFloat.m_low, dst5.m_simdFloat.m_low, dst6.m_simdFloat.m_low, dst7.m_simdFloat.m_low,
					src0.m_simdFloat.m_high, src1.m_simdFloat.m_high, src2.m_simdFloat.m_high, src3.m_simdFloat.m_high);

				Transpose(
					dst4.m_simdFloat.m_high, dst5.m_simdFloat.m_high, dst6.m_simdFloat.m_high, dst7.m_simdFloat.m_high,
					src4.m_simdFloat.m_high, src5.m_simdFloat.m_high, src6.m_simdFloat.m_high, src7.m_simdFloat.m_high);
			}

			struct vector8
			{
				ndBigVector m_linear;
				ndBigVector m_angular;
			};

			struct simdFloat
			{
				inline simdFloat(const ndFloat64 val)
					:m_low(_mm256_set1_pd(val))
					,m_high(_mm256_set1_pd(val))
				{
				}

				inline simdFloat(const ndInt32 val)
					:m_low(_mm256_castsi256_pd(_mm256_set1_epi64x(ndInt64(val))))
					,m_high(_mm256_castsi256_pd(_mm256_set1_epi64x(ndInt64(val))))
				{
				}

				inline simdFloat(const __m256d low, const __m256d high)
					:m_low(low)
					,m_high(high)
				{
				}

				inline simdFloat(const simdFloat& copy)
					:m_low(copy.m_low)
					,m_high(copy.m_high)
				{
				}

				inline simdFloat(const ndVector& low, const ndVector& high)
					:m_low(_mm256_set_m128d(low.m_typeHigh, low.m_typeLow))
					,m_high(_mm256_set_m128d(high.m_typeHigh, high.m_typeLow))
				{
				}

				inline simdFloat(const ndVector8* const baseAddr, const ndVector8& index)
					:m_low(_mm256_i64gather_pd(&(*baseAddr)[0], index.m_simdInt.m_lowInt, 8))
					,m_high(_mm256_i64gather_pd(&(*baseAddr)[0], index.m_simdInt.m_highInt, 8))
				{
				}

				__m256d m_low;
				__m256d m_high;
			};
			
			struct simdInt
			{
				__m256i m_lowInt;
				__m256i m_highInt;
			};

			union
			{
				ndFloat64 m_float[ND_SIMD8_WORK_GROUP_SIZE];
				ndInt64 m_int[ND_SIMD8_WORK_GROUP_SIZE];
				vector8 m_vector;
				simdInt m_simdInt;
				simdFloat m_simdFloat;
			};

			D_CORE_API static ndVector8 m_one;
			D_CORE_API static ndVector8 m_zero;
			D_CORE_API static ndVector8 m_mask;
			D_CORE_API static ndVector8 m_ordinals;
		} D_GCC_NEWTON_CLASS_ALIGN_32;

	#else		
		// the none avx just falls back to two simd4 vectors
		D_MSV_NEWTON_CLASS_ALIGN_32
		class ndVector8
		{
			public:
			inline ndVector8()
			{
			}

			inline ndVector8(const ndFloat64 val)
				:m_pair(val)
			{
			}

			inline ndVector8(const ndInt32 val)
				:m_pair(val)
			{
			}

			inline ndVector8(const ndVector8& copy)
				:m_pair(copy.m_pair)
			{
			}

			inline ndVector8(const ndVector& low, const ndVector& high)
				:m_pair(low, high)
			{
			}

			inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
				:m_pair(baseAddr, index)
			{
			}

			inline ndFloat64& operator[] (ndInt32 i)
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline const ndFloat64& operator[] (ndInt32 i) const
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline ndVector8& operator= (const ndVector8& A)
			{
				m_pair.m_linear = A.m_pair.m_linear;
				m_pair.m_angular = A.m_pair.m_angular;
				return *this;
			}

			inline ndVector8 operator+ (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear + A.m_pair.m_linear, m_pair.m_angular + A.m_pair.m_angular);
			}

			inline ndVector8 operator- (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear - A.m_pair.m_linear, m_pair.m_angular - A.m_pair.m_angular);
			}

			inline ndVector8 operator* (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear * A.m_pair.m_linear, m_pair.m_angular * A.m_pair.m_angular);
			}

			inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
			{
				return ndVector8(m_pair.m_linear.MulAdd(A.m_pair.m_linear, B.m_pair.m_linear), m_pair.m_angular.MulAdd(A.m_pair.m_angular, B.m_pair.m_angular));
			}

			inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
			{
				return ndVector8(m_pair.m_linear.MulSub(A.m_pair.m_linear, B.m_pair.m_linear), m_pair.m_angular.MulSub(A.m_pair.m_angular, B.m_pair.m_angular));
			}

			inline ndVector8 operator> (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear > A.m_pair.m_linear, m_pair.m_angular > A.m_pair.m_angular);
			}

			inline ndVector8 operator< (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear < A.m_pair.m_linear, m_pair.m_angular < A.m_pair.m_angular);
			}

			inline ndVector8 operator| (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear | A.m_pair.m_linear, m_pair.m_angular | A.m_pair.m_angular);
			}

			inline ndVector8 operator& (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear & A.m_pair.m_linear, m_pair.m_angular & A.m_pair.m_angular);
			}

			inline ndVector8 GetMin(const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear.GetMin(A.m_pair.m_linear), m_pair.m_angular.GetMin(A.m_pair.m_angular));
			}

			inline ndVector8 GetMax(const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear.GetMax(A.m_pair.m_linear), m_pair.m_angular.GetMax(A.m_pair.m_angular));
			}

			inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
			{
				return ndVector8(m_pair.m_linear.Select(data.m_pair.m_linear, mask.m_pair.m_linear), m_pair.m_angular.Select(data.m_pair.m_angular, mask.m_pair.m_angular));
			}

			inline ndVector GetLow() const
			{
				return m_pair.m_linear;
			}

			inline ndVector GetHigh() const
			{
				return m_pair.m_angular;
			}

			inline ndFloat64 GetMax() const
			{
				return m_pair.m_linear.GetMax(m_pair.m_angular).GetScalar();
			}

			inline ndFloat64 AddHorizontal() const
			{
				return (m_pair.m_linear + m_pair.m_angular).AddHorizontal().GetScalar();
			}

			static inline void FlushRegisters()
			{
			}

			static inline void Transpose(
				ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
				ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
				const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
				const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
			{
				const ndMatrix off01(src0.m_pair.m_angular, src1.m_pair.m_angular, src2.m_pair.m_angular, src3.m_pair.m_angular);
				const ndMatrix off10(src4.m_pair.m_linear, src5.m_pair.m_linear, src6.m_pair.m_linear, src7.m_pair.m_linear);

				ndVector::Transpose4x4(
					dst0.m_pair.m_linear, dst1.m_pair.m_linear, dst2.m_pair.m_linear, dst3.m_pair.m_linear,
					src0.m_pair.m_linear, src1.m_pair.m_linear, src2.m_pair.m_linear, src3.m_pair.m_linear);

				ndVector::Transpose4x4(
					dst0.m_pair.m_angular, dst1.m_pair.m_angular, dst2.m_pair.m_angular, dst3.m_pair.m_angular,
					off10[0], off10[1], off10[2], off10[3]);

				ndVector::Transpose4x4(
					dst4.m_pair.m_linear, dst5.m_pair.m_linear, dst6.m_pair.m_linear, dst7.m_pair.m_linear,
					off01[0], off01[1], off01[2], off01[3]);

				ndVector::Transpose4x4(
					dst4.m_pair.m_angular, dst5.m_pair.m_angular, dst6.m_pair.m_angular, dst7.m_pair.m_angular,
					src4.m_pair.m_angular, src5.m_pair.m_angular, src6.m_pair.m_angular, src7.m_pair.m_angular);
			}

			struct pair
			{
				inline  pair()
				{
				}

				inline pair(const ndFloat64 val)
					:m_linear(val)
					,m_angular(val)
				{
				}

				inline pair(const ndInt32 val)
					:m_linear(val, val, val, val)
					,m_angular(val, val, val, val)
				{
				}

				inline pair(const pair& copy)
					: m_linear(copy.m_linear)
					, m_angular(copy.m_angular)
				{
				}

				inline pair(const ndVector& low, const ndVector& high)
					:m_linear(low)
					,m_angular(high)
				{
				}

				inline pair(const ndVector8* const baseAddr, const ndVector8& index)
					:m_linear(&baseAddr->m_pair.m_linear[0], index.m_pair.m_linear.m_i)
					,m_angular(&baseAddr->m_pair.m_linear[0], index.m_pair.m_angular.m_i)
				{
				}

				ndBigVector m_linear;
				ndBigVector m_angular;
			};

			struct simdPair
			{
				__m256 m_typeLow;
				__m256 m_typeHigh;
			};

			union
			{
				ndFloat64 m_float[ND_SIMD8_WORK_GROUP_SIZE];
				ndInt64 m_int[ND_SIMD8_WORK_GROUP_SIZE];
				pair m_pair;
				simdPair m_simd4;
			};

			D_CORE_API static ndVector8 m_one;
			D_CORE_API static ndVector8 m_zero;
			D_CORE_API static ndVector8 m_mask;
			D_CORE_API static ndVector8 m_ordinals;
		} D_GCC_NEWTON_CLASS_ALIGN_32;

	#endif
#else

	#if defined(D_NEWTON_USE_AVX2_OPTION)
		// the avx use explict avx2 intrisics
		D_MSV_NEWTON_CLASS_ALIGN_32
		class ndVector8
		{
			public:
			#ifndef PERMUTE_MASK
				#define PERMUTE_MASK(w, z, y, x) _MM_SHUFFLE (w, z, y, x)
			#endif

			inline ndVector8()
			{
			}

			inline ndVector8(const ndFloat32 val)
				:m_type(_mm256_set1_ps(val))
			{
			}

			inline ndVector8(const ndInt32 val)
				:m_type(_mm256_castsi256_ps(_mm256_set1_epi32(val)))
			{
			}

			inline ndVector8(const __m256 type)
				:m_type(type)
			{
			}

			inline ndVector8(const ndVector8& copy)
				: m_type(copy.m_type)
			{
			}

			inline ndVector8(const ndVector& low, const ndVector& high)
				:m_type(_mm256_set_m128(high.m_type, low.m_type))
			{
			}

			inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
				:m_type(_mm256_i32gather_ps(&(*baseAddr)[0], index.m_typeInt, 4))
			{
			}

			inline ndFloat32& operator[] (ndInt32 i)
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline const ndFloat32& operator[] (ndInt32 i) const
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline ndVector8& operator= (const ndVector8& A)
			{
				m_type = A.m_type;
				return *this;
			}

			inline ndVector8 operator+ (const ndVector8& A) const
			{
				return _mm256_add_ps(m_type, A.m_type);
			}

			inline ndVector8 operator- (const ndVector8& A) const
			{
				return _mm256_sub_ps(m_type, A.m_type);
			}

			inline ndVector8 operator* (const ndVector8& A) const
			{
				return _mm256_mul_ps(m_type, A.m_type);
			}

			inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
			{
				return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
			}

			inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
			{
				return _mm256_fnmadd_ps(A.m_type, B.m_type, m_type);
			}

			inline ndVector8 operator> (const ndVector8& A) const
			{
				return _mm256_cmp_ps(m_type, A.m_type, _CMP_GT_OQ);
			}

			inline ndVector8 operator< (const ndVector8& A) const
			{
				return _mm256_cmp_ps(m_type, A.m_type, _CMP_LT_OQ);
			}

			inline ndVector8 operator| (const ndVector8& A) const
			{
				return _mm256_or_ps(m_type, A.m_type);
			}

			inline ndVector8 operator& (const ndVector8& A) const
			{
				return _mm256_and_ps(m_type, A.m_type);
			}

			inline ndVector8 GetMin(const ndVector8& A) const
			{
				return _mm256_min_ps(m_type, A.m_type);
			}

			inline ndVector8 GetMax(const ndVector8& A) const
			{
				return _mm256_max_ps(m_type, A.m_type);
			}

			inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
			{
				// (((b ^ a) & mask)^a)
				//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
				return  _mm256_xor_ps(m_type, _mm256_and_ps(mask.m_type, _mm256_xor_ps(m_type, data.m_type)));
			}

			inline ndVector GetLow() const
			{
				return m_simd4.m_typeLow;
			}

			inline ndVector GetHigh() const
			{
				return m_simd4.m_typeHigh;
			}

			inline ndFloat32 GetMax() const
			{
				__m128 tmp0(_mm_max_ps(m_simd4.m_typeLow, m_simd4.m_typeHigh));
				__m128 tmp1(_mm_max_ps(tmp0, _mm_movehl_ps(tmp0, tmp0)));
				__m128 tmp2(_mm_max_ps(tmp1, _mm_permute_ps(tmp1, PERMUTE_MASK(2, 3, 0, 1))));
				return _mm_cvtss_f32(tmp2);
			}

			inline ndFloat32 AddHorizontal() const
			{
				__m128 tmp0(_mm_add_ps(m_simd4.m_typeLow, m_simd4.m_typeHigh));
				__m128 tmp1(_mm_add_ps(tmp0, _mm_movehl_ps(tmp0, tmp0)));
				__m128 tmp2(_mm_add_ps(tmp1, _mm_permute_ps(tmp1, PERMUTE_MASK(2, 3, 0, 1))));
				return _mm_cvtss_f32(tmp2);
			}

			static inline void FlushRegisters()
			{
				_mm256_zeroall();
			}

			static inline void Transpose(
				ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
				ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
				const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
				const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
			{
				__m256 r0(_mm256_permute2f128_ps(src0.m_type, src4.m_type, 0x20));
				__m256 r1(_mm256_permute2f128_ps(src0.m_type, src4.m_type, 0x31));
				__m256 r2(_mm256_permute2f128_ps(src1.m_type, src5.m_type, 0x20));
				__m256 r3(_mm256_permute2f128_ps(src1.m_type, src5.m_type, 0x31));
				__m256 r4(_mm256_permute2f128_ps(src2.m_type, src6.m_type, 0x20));
				__m256 r5(_mm256_permute2f128_ps(src2.m_type, src6.m_type, 0x31));
				__m256 r6(_mm256_permute2f128_ps(src3.m_type, src7.m_type, 0x20));
				__m256 r7(_mm256_permute2f128_ps(src3.m_type, src7.m_type, 0x31));

				__m256 s0(_mm256_unpacklo_ps(r0, r4));
				__m256 s1(_mm256_unpackhi_ps(r0, r4));
				__m256 s2(_mm256_unpacklo_ps(r1, r5));
				__m256 s3(_mm256_unpackhi_ps(r1, r5));
				__m256 s4(_mm256_unpacklo_ps(r2, r6));
				__m256 s5(_mm256_unpackhi_ps(r2, r6));
				__m256 s6(_mm256_unpacklo_ps(r3, r7));
				__m256 s7(_mm256_unpackhi_ps(r3, r7));

				dst0.m_type = _mm256_unpacklo_ps(s0, s4);
				dst1.m_type = _mm256_unpackhi_ps(s0, s4);
				dst2.m_type = _mm256_unpacklo_ps(s1, s5);
				dst3.m_type = _mm256_unpackhi_ps(s1, s5);
				dst4.m_type = _mm256_unpacklo_ps(s2, s6);
				dst5.m_type = _mm256_unpackhi_ps(s2, s6);
				dst6.m_type = _mm256_unpacklo_ps(s3, s7);
				dst7.m_type = _mm256_unpackhi_ps(s3, s7);
			}

			struct pair
			{
				ndVector m_linear;
				ndVector m_angular;
			};

			struct simdPair
			{
				__m128 m_typeLow;
				__m128 m_typeHigh;
			};

			union
			{
				ndFloat32 m_float[ND_SIMD8_WORK_GROUP_SIZE];
				ndInt32 m_int[ND_SIMD8_WORK_GROUP_SIZE];
				__m256 m_type;
				__m256i m_typeInt;
				pair m_pair;
				simdPair m_simd4;
			};

			D_CORE_API static ndVector8 m_one;
			D_CORE_API static ndVector8 m_zero;
			D_CORE_API static ndVector8 m_mask;
			D_CORE_API static ndVector8 m_ordinals;
		} D_GCC_NEWTON_CLASS_ALIGN_32;

	#else
		// the none avx just falls back to two simd4 vectors
		D_MSV_NEWTON_CLASS_ALIGN_32
		class ndVector8
		{
			public:
			inline ndVector8()
			{
			}

			inline ndVector8(const ndFloat32 val)
				:m_pair(val)
			{
			}

			inline ndVector8(const ndInt32 val)
				:m_pair(val)
			{
			}

			inline ndVector8(const ndVector8& copy)
				:m_pair(copy.m_pair)
			{
			}

			inline ndVector8(const ndVector& low, const ndVector& high)
				:m_pair(low, high)
			{
			}

			inline ndVector8(const ndVector8* const baseAddr, const ndVector8& index)
				:m_pair(baseAddr, index)
			{
			}

			inline ndFloat32& operator[] (ndInt32 i)
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline const ndFloat32& operator[] (ndInt32 i) const
			{
				ndAssert(i >= 0);
				ndAssert(i < ND_SIMD8_WORK_GROUP_SIZE);
				return m_float[i];
			}

			inline ndVector8& operator= (const ndVector8& A)
			{
				m_pair.m_linear = A.m_pair.m_linear;
				m_pair.m_angular = A.m_pair.m_angular;
				return *this;
			}

			inline ndVector8 operator+ (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear + A.m_pair.m_linear, m_pair.m_angular + A.m_pair.m_angular);
			}

			inline ndVector8 operator- (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear - A.m_pair.m_linear, m_pair.m_angular - A.m_pair.m_angular);
			}

			inline ndVector8 operator* (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear * A.m_pair.m_linear, m_pair.m_angular * A.m_pair.m_angular);
			}

			inline ndVector8 MulAdd(const ndVector8& A, const ndVector8& B) const
			{
				return ndVector8(m_pair.m_linear.MulAdd(A.m_pair.m_linear, B.m_pair.m_linear), m_pair.m_angular.MulAdd(A.m_pair.m_angular, B.m_pair.m_angular));
			}

			inline ndVector8 MulSub(const ndVector8& A, const ndVector8& B) const
			{
				return ndVector8(m_pair.m_linear.MulSub(A.m_pair.m_linear, B.m_pair.m_linear), m_pair.m_angular.MulSub(A.m_pair.m_angular, B.m_pair.m_angular));
			}

			inline ndVector8 operator> (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear > A.m_pair.m_linear, m_pair.m_angular > A.m_pair.m_angular);
			}

			inline ndVector8 operator< (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear < A.m_pair.m_linear, m_pair.m_angular < A.m_pair.m_angular);
			}

			inline ndVector8 operator| (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear | A.m_pair.m_linear, m_pair.m_angular | A.m_pair.m_angular);
			}

			inline ndVector8 operator& (const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear & A.m_pair.m_linear, m_pair.m_angular & A.m_pair.m_angular);
			}

			inline ndVector8 GetMin(const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear.GetMin(A.m_pair.m_linear), m_pair.m_angular.GetMin(A.m_pair.m_angular));
			}

			inline ndVector8 GetMax(const ndVector8& A) const
			{
				return ndVector8(m_pair.m_linear.GetMax(A.m_pair.m_linear), m_pair.m_angular.GetMax(A.m_pair.m_angular));
			}

			inline ndVector8 Select(const ndVector8& data, const ndVector8& mask) const
			{
				return ndVector8(m_pair.m_linear.Select(data.m_pair.m_linear, mask.m_pair.m_linear), m_pair.m_angular.Select(data.m_pair.m_angular, mask.m_pair.m_angular));
			}

			inline ndVector GetLow() const
			{
				return m_pair.m_linear;
			}

			inline ndVector GetHigh() const
			{
				return m_pair.m_angular;
			}

			inline ndFloat32 GetMax() const
			{
				return m_pair.m_linear.GetMax(m_pair.m_angular).GetScalar();
			}

			inline ndFloat32 AddHorizontal() const
			{
				return (m_pair.m_linear + m_pair.m_angular).AddHorizontal().GetScalar();
			}

			static inline void FlushRegisters()
			{
			}

			static inline void Transpose(
				ndVector8& dst0, ndVector8& dst1, ndVector8& dst2, ndVector8& dst3,
				ndVector8& dst4, ndVector8& dst5, ndVector8& dst6, ndVector8& dst7,
				const ndVector8& src0, const ndVector8& src1, const ndVector8& src2, const ndVector8& src3,
				const ndVector8& src4, const ndVector8& src5, const ndVector8& src6, const ndVector8& src7)
			{
				const ndMatrix off01(src0.m_pair.m_angular, src1.m_pair.m_angular, src2.m_pair.m_angular, src3.m_pair.m_angular);
				const ndMatrix off10(src4.m_pair.m_linear, src5.m_pair.m_linear, src6.m_pair.m_linear, src7.m_pair.m_linear);

				ndVector::Transpose4x4(
					dst0.m_pair.m_linear, dst1.m_pair.m_linear, dst2.m_pair.m_linear, dst3.m_pair.m_linear,
					src0.m_pair.m_linear, src1.m_pair.m_linear, src2.m_pair.m_linear, src3.m_pair.m_linear);

				ndVector::Transpose4x4(
					dst0.m_pair.m_angular, dst1.m_pair.m_angular, dst2.m_pair.m_angular, dst3.m_pair.m_angular,
					off10[0], off10[1], off10[2], off10[3]);

				ndVector::Transpose4x4(
					dst4.m_pair.m_linear, dst5.m_pair.m_linear, dst6.m_pair.m_linear, dst7.m_pair.m_linear,
					off01[0], off01[1], off01[2], off01[3]);

				ndVector::Transpose4x4(
					dst4.m_pair.m_angular, dst5.m_pair.m_angular, dst6.m_pair.m_angular, dst7.m_pair.m_angular,
					src4.m_pair.m_angular, src5.m_pair.m_angular, src6.m_pair.m_angular, src7.m_pair.m_angular);
			}

			struct pair
			{
				inline  pair()
				{
				}

				inline pair(const ndFloat32 val)
					:m_linear(val)
					,m_angular(val)
				{
				}

				inline pair(const ndInt32 val)
					:m_linear(val, val, val, val)
					,m_angular(val, val, val, val)
				{
				}

				inline pair(const pair& copy)
					:m_linear(copy.m_linear)
					,m_angular(copy.m_angular)
				{
				}

				inline pair(const ndVector& low, const ndVector& high)
					:m_linear(low)
					,m_angular(high)
				{
				}

				inline pair(const ndVector8* const baseAddr, const ndVector8& index)
					:m_linear(&baseAddr->m_pair.m_linear[0], (ndInt32*)&index.m_pair.m_linear.m_i)
					,m_angular(&baseAddr->m_pair.m_linear[0], (ndInt32*)&index.m_pair.m_angular.m_i)
				{
				}

				ndVector m_linear;
				ndVector m_angular;
			};

			struct simdPair
			{
				__m128 m_typeLow;
				__m128 m_typeHigh;
			};

			union
			{
				ndFloat32 m_float[ND_SIMD8_WORK_GROUP_SIZE];
				ndInt32 m_int[ND_SIMD8_WORK_GROUP_SIZE];
				__m256 m_type;
				__m256i m_typeInt;
				pair m_pair;
				simdPair m_simd4;
			};

			D_CORE_API static ndVector8 m_one;
			D_CORE_API static ndVector8 m_zero;
			D_CORE_API static ndVector8 m_mask;
			D_CORE_API static ndVector8 m_ordinals;
		} D_GCC_NEWTON_CLASS_ALIGN_32;
	#endif

#endif

#endif
