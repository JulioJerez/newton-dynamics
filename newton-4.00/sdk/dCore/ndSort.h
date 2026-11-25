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

#ifndef ND_SORT_H_
#define ND_SORT_H_

#include "ndCoreStdafx.h"
#include "ndArray.h"
#include "ndUtils.h"
#include "ndProfiler.h"
#include "ndThreadPool.h"
#include "ndFixSizeArray.h"

#define ND_CACHE_LINE_PAD	ndInt32(64/sizeof (ndInt32))

template <class T, typename ndCompareKey>
void ndSort(T* const array, ndInt32 elements, ndCompareKey compare);

template <class T, typename ndEvaluateKey>
void ndCountingSort(const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest);

template <class T, typename ndEvaluateKey>
void ndCountingSort(ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest);

template <class T, typename ndEvaluateKey>
void ndCountingSort(ndThreadPool& threadPool, const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest);

template <class T, typename ndEvaluateKey>
void ndCountingSort(ndThreadPool& threadPool, ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest);

template <class T, typename ndEvaluateKey>
void ndCountingSortInPlace(T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest);

template <class T, typename ndEvaluateKey>
void ndCountingSortInPlace(ndThreadPool& threadPool, const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest);

// *****************************************************
// standard quick sort template implementation
// *****************************************************
template <class T, typename ndCompareKey>
void ndSort(T* const array, ndInt32 elements, ndCompareKey compare)
{
	//ND_PROFILE_ZONE();
	const ndInt32 batchSize = 8;

	class ndIntervale
	{
		public:
		ndIntervale() {}
		ndIntervale(ndInt32 start, ndInt32 end)
			:m_start(start)
			,m_end(end)
		{
		}

		ndInt32 m_start;
		ndInt32 m_end;
	};

	ndFixSizeArray<ndIntervale, 64> stack;

	stack.PushBack(ndIntervale(0, elements - 1));

	while (stack.GetCount())
	{
		ndIntervale span(stack.Pop());
		ndInt32 lo = span.m_start;
		ndInt32 hi = span.m_end;
		if ((hi - lo) > batchSize)
		{
			ndInt32 mid = (lo + hi) >> 1;
			if (compare(array[lo], array[mid]) > 0)
			{
				ndSwap(array[lo], array[mid]);
			}
			if (compare(array[mid], array[hi]) > 0)
			{
				ndSwap(array[mid], array[hi]);
			}
			if (compare(array[lo], array[mid]) > 0)
			{
				ndSwap(array[lo], array[mid]);
			}
			ndInt32 i = lo + 1;
			ndInt32 j = hi - 1;
			const T pivot(array[mid]);
			do
			{
				while (compare(array[i], pivot) < 0)
				{
					i++;
				}
				while (compare(array[j], pivot) > 0)
				{
					j--;
				}
	
				if (i <= j)
				{
					ndSwap(array[i], array[j]);
					i++;
					j--;
				}
			} while (i <= j);
	
			if (i < hi)
			{
				stack.PushBack(ndIntervale(i, hi));
			}
			if (lo < j)
			{
				stack.PushBack(ndIntervale(lo, j));
			}
		}
	}
	
	ndInt32 stride = batchSize + 1;
	if (elements < stride)
	{
		stride = elements;
	}
	for (ndInt32 i = 1; i < stride; ++i)
	{
		if (compare(array[0], array[i]) > 0)
		{
			ndSwap(array[0], array[i]);
		}
	}
	
	for (ndInt32 i = 1; i < elements; ++i)
	{
		ndInt32 j = i;
		const T tmp(array[i]);
		for (; compare(array[j - 1], tmp) > 0; --j)
		{
			ndAssert(j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

#if defined (_DEBUG) && !(defined (D_DISABLE_ASSERT) || defined(__clang__) || defined(__GNUC__))
	for (ndInt32 i = 0; i < (elements - 1); ++i)
	{
		ndAssert(compare(array[i], array[i + 1]) <= 0);
	}
	#endif
}

template <class T, typename ndEvaluateKey>
void ndCountingSortInPlace(T* const array, T* const scratchBuffer, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey testKey)
{
	//ND_PROFILE_ZONE();

	ndAssert(bitSize > 0);
	ndAssert(bitSize <= 10);
	const ndInt32 scanSize = 1 << bitSize;

	ndUnsigned32 scans[1024];
	ndMemSet(scans, ndUnsigned32(0), scanSize);

	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = array[i];
		scratchBuffer[i] = entry;
		const ndInt32 key = testKey(entry);
		ndAssert(key >= 0);
		ndAssert(key < scanSize);
		scans[key] ++;
	}

	ndUnsigned32 sum = 0;
	for (ndInt32 i = 0; i < scanSize; ++i)
	{
		ndUnsigned32 partialSum = scans[i];
		scans[i] = sum;
		sum += partialSum;
	}

	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			prefixScanOut[i] = scans[i];
		}
		prefixScanOut[scanSize] = sum;
	}

	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = scratchBuffer[i];
		const ndInt32 key = testKey(entry);
		ndAssert(key >= 0);
		ndAssert(key < scanSize);
		const ndUnsigned32 index = scans[key];
		array[index] = entry;
		scans[key] = index + 1;
	}

#if defined (_DEBUG) && !(defined (D_DISABLE_ASSERT) || defined(__clang__) || defined(__GNUC__))
	for (ndInt32 i = size - 2; i >= 0; --i)
	{
		ndInt32 key0 = testKey(array[i - 0]);
		ndInt32 key1 = testKey(array[i + 1]);
		ndAssert(key0 <= key1);
	}
#endif
}

template <class T, typename ndEvaluateKey>
void ndCountingSortInPlace(ndThreadPool& threadPool, T* const array, T* const scratchBuffer, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey testKey)
{
	ND_PROFILE_ZONE();
	ndAssert(bitSize > 0);
	ndAssert(bitSize <= 10);
	const ndInt32 minGroupSize = 1024;
	const ndInt32 scanSize = 1 << bitSize;
	const ndInt32 threadCount = threadPool.GetThreadCount();
	const ndInt32 itemsPerThreads = (size + threadCount - 1) / threadCount;
	const ndInt32 groupSize = (itemsPerThreads < minGroupSize) ? minGroupSize : itemsPerThreads;
	const ndInt32 numberOfGroups = (size + groupSize - 1) / groupSize;
	ndAssert(numberOfGroups <= threadCount);
	
	ndUnsigned32* const sum = ndAlloca(ndUnsigned32, (numberOfGroups + 1) * (scanSize + ND_CACHE_LINE_PAD));
	ndUnsigned32* const scans = &sum[scanSize + ND_CACHE_LINE_PAD];
	
	auto ndBuildHistogram = ndMakeObject::ndFunction([&array, &scratchBuffer, size, &testKey, scanSize, &scans, groupSize](ndInt32 groupId, ndInt32, ndInt32)
	{
		ND_PROFILE_ZONE_NAMED("ndBuildHistogram");
		ndUnsigned32* const scan = &scans[groupId * (scanSize + ND_CACHE_LINE_PAD)];
	
		ndMemSet(scan, ndUnsigned32(0), scanSize);
	
		ndInt32 start = groupId * groupSize;
		ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			const T& entry = array[start + i];
			const ndInt32 key = testKey(entry);
			ndAssert(key >= 0);
			ndAssert(key < scanSize);
			scan[key]++;
			scratchBuffer[start + i] = entry;
		}
	});
	threadPool.ParallelExecute(ndBuildHistogram, numberOfGroups, 1);
	
	ndMemSet(sum, ndUnsigned32(0), scanSize);
	for (ndInt32 j = 0; j < numberOfGroups; ++j)
	{
		ndUnsigned32* const scan = &scans[j * (scanSize + ND_CACHE_LINE_PAD)];
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			ndUnsigned32 partialSum = scan[i];
			scan[i] = sum[i];
			sum[i] += partialSum;
		}
	}
	
	ndUnsigned32 accSum = 0;
	for (ndInt32 i = 0; i < scanSize; ++i)
	{
		ndUnsigned32 partialSum = sum[i];
		sum[i] = accSum;
		accSum += partialSum;
	}
	
	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			prefixScanOut[i] = sum[i];
		}
		prefixScanOut[scanSize] = ndUnsigned32(size);
	}
	
	auto ndShuffleArray = ndMakeObject::ndFunction([&array, &scratchBuffer, size, &testKey, scanSize, &scans, &sum, groupSize](ndInt32 groupId, ndInt32, ndInt32)
	{
		ND_PROFILE_ZONE_NAMED("ndShuffleArray");
		ndUnsigned32* const scan = &scans[groupId * (scanSize + ND_CACHE_LINE_PAD)];
	
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			scan[i] += sum[i];
		}
	
		ndInt32 start = groupId * groupSize;
		ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			const T& entry = scratchBuffer[start + i];
			const ndInt32 key = testKey(entry);
			ndAssert(key >= 0);
			ndAssert(key < scanSize);
			const ndUnsigned32 index = scan[key];
			array[index] = entry;
			scan[key] = index + 1;
		}
	});
	threadPool.ParallelExecute(ndShuffleArray, numberOfGroups, 1);

#if defined (_DEBUG) && !(defined (D_DISABLE_ASSERT) || defined(__clang__) || defined(__GNUC__))
	for (ndInt32 i = size - 2; i >= 0; --i)
	{
		ndInt32 key0 = testKey(array[i - 0]);
		ndInt32 key1 = testKey(array[i + 1]);
		ndAssert(key0 <= key1);
	}
#endif
}

template <class T, typename ndEvaluateKey>
void ndCountingSort(const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 keyBitSize, ndEvaluateKey keyTest)
{
	ndAssert(keyBitSize > 0);
	ndAssert(keyBitSize <= 10);
	const ndInt32 scanSize = 1 << keyBitSize;

	ndUnsigned32 scans[1024];
	ndMemSet(scans, ndUnsigned32(0), scanSize);

	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = srcArray[i];
		const ndInt32 key = keyTest(entry);
		ndAssert(key >= 0);
		ndAssert(key < scanSize);
		scans[key]++;
	}

	ndUnsigned32 sum = 0;
	for (ndInt32 i = 0; i < scanSize; ++i)
	{
		ndUnsigned32 partialSum = scans[i];
		scans[i] = sum;
		sum += partialSum;
	}

	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			prefixScanOut[i] = scans[i];
		}
		prefixScanOut[scanSize] = sum;
	}

	for (ndInt32 i = 0; i < size; ++i)
	{
		const T& entry = srcArray[i];
		const ndInt32 key = keyTest(entry);
		ndAssert(key >= 0);
		ndAssert(key < scanSize);
		const ndUnsigned32 index = scans[key];
		dstArray[index] = entry;
		scans[key] = index + 1;
	}

#if defined (_DEBUG) && !(defined (D_DISABLE_ASSERT) || defined(__clang__) || defined(__GNUC__))
	for (ndInt32 i = size - 2; i >= 0; --i)
	{
		ndInt32 key0 = keyTest(dstArray[i - 0]);
		ndInt32 key1 = keyTest(dstArray[i + 1]);
		ndAssert(key0 <= key1);
	}
#endif
}

template <class T, typename ndEvaluateKey>
void ndCountingSort(ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, ndInt32 keyBitSize, ndEvaluateKey keyTest)
{
	scratchBuffer.SetCount(array.GetCount());
	ndCountingSort(&array[0], &scratchBuffer[0], ndInt32(array.GetCount()), prefixScanOut, keyBitSize, keyTest);
	array.Swap(scratchBuffer);
}

template <class T, typename ndEvaluateKey>
void ndCountingSort(ndThreadPool& threadPool, const T* const srcArray, T* const dstArray, ndInt32 size, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest)
{
	ND_PROFILE_ZONE();

	ndAssert(bitSize > 0);
	ndAssert(bitSize <= 10);
	const ndInt32 minGroupSize = 1024;
	const ndInt32 scanSize = 1 << bitSize;
	const ndInt32 threadCount = threadPool.GetThreadCount();
	const ndInt32 itemsPerThreads = (size + threadCount - 1) / threadCount;
	const ndInt32 groupSize = (itemsPerThreads < minGroupSize) ? minGroupSize : itemsPerThreads;
	const ndInt32 numberOfGroups = (size + groupSize - 1) / groupSize;
	ndAssert(numberOfGroups <= threadCount);
	
	ndUnsigned32* const sum = ndAlloca(ndUnsigned32, (numberOfGroups + 1) * (scanSize + ND_CACHE_LINE_PAD));
	ndUnsigned32* const scans = &sum[scanSize + ND_CACHE_LINE_PAD];
	
	auto ndBuildHistogram = ndMakeObject::ndFunction([&srcArray, size, scanSize, &keyTest, &scans, groupSize](ndInt32 groupId, ndInt32, ndInt32)
	{
		ND_PROFILE_ZONE_NAMED("ndBuildHistogram");
		ndUnsigned32* const scan = &scans[groupId * (scanSize + ND_CACHE_LINE_PAD)];
	
		ndMemSet(scan, ndUnsigned32(0), scanSize);

		ndInt32 start = groupId * groupSize;
		ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			const T& entry = srcArray[start + i];
			const ndInt32 key = keyTest(entry);
			ndAssert(key >= 0);
			ndAssert(key < scanSize);
			scan[key]++;
		}
	});
	threadPool.ParallelExecute(ndBuildHistogram, numberOfGroups, 1);
	
	ndMemSet(sum, ndUnsigned32(0), scanSize);
	for (ndInt32 j = 0; j < numberOfGroups; ++j)
	{
		ndUnsigned32* const scan = &scans[j * (scanSize + ND_CACHE_LINE_PAD)];
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			ndUnsigned32 partialSum = scan[i];
			scan[i] = sum[i];
			sum[i] += partialSum;
		}
	}
	
	ndUnsigned32 accSum = 0;
	for (ndInt32 i = 0; i < scanSize; ++i)
	{
		ndUnsigned32 partialSum = sum[i];
		sum[i] = accSum;
		accSum += partialSum;
	}
	
	if (prefixScanOut)
	{
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			prefixScanOut[i] = sum[i];
		}
		prefixScanOut[scanSize] = ndUnsigned32(size);
	}
	
	auto ndShuffleArray = ndMakeObject::ndFunction([&srcArray, &dstArray, size, scanSize, &keyTest, &scans, &sum, groupSize](ndInt32 groupId, ndInt32, ndInt32)
	{
		ND_PROFILE_ZONE_NAMED("ndShuffleArray");
		ndUnsigned32* const scan = &scans[groupId * (scanSize + ND_CACHE_LINE_PAD)];
	
		for (ndInt32 i = 0; i < scanSize; ++i)
		{
			scan[i] += sum[i];
		}
	
		ndInt32 start = groupId * groupSize;
		ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			const T& entry = srcArray[start + i];
			const ndInt32 key = keyTest(entry);
			ndAssert(key >= 0);
			ndAssert(key < scanSize);
			const ndUnsigned32 index = scan[key];
			dstArray[index] = entry;
			scan[key] = index + 1;
		}
	});
	threadPool.ParallelExecute(ndShuffleArray, numberOfGroups, 1);

#if defined (_DEBUG) && !(defined (D_DISABLE_ASSERT) || defined(__clang__) || defined(__GNUC__))
	for (ndInt32 i = size - 2; i >= 0; --i)
	{
		ndInt32 key0 = keyTest(dstArray[i - 0]);
		ndInt32 key1 = keyTest(dstArray[i + 1]);
		ndAssert(key0 <= key1);
	}
#endif
}

template <class T, typename ndEvaluateKey>
void ndCountingSort(ndThreadPool& threadPool, ndArray<T>& array, ndArray<T>& scratchBuffer, ndUnsigned32* const prefixScanOut, ndInt32 bitSize, ndEvaluateKey keyTest)
{
	scratchBuffer.SetCount(array.GetCount());
	ndCountingSort(threadPool, &array[0], &scratchBuffer[0], ndInt32(array.GetCount()), prefixScanOut, bitSize, keyTest);
	array.Swap(scratchBuffer);
}

#endif
