#pragma once
/*
 * glatter_atomic.h
 *
 * Tiny, zero-dependency atomic helpers for both C++ and C toolchains.
 *
 * We expose TWO small atomic "families":
 *   1) POINTER ATOMICS (generic where supported)
 *      - Use for function pointers, module handles, etc.
 *      - Macros: glatter_atomic(T), GLATTER_ATOMIC_LOAD/STORE/CAS, GLATTER_ATOMIC_INIT_PTR(x)
 *      - NOTE (Windows C / Interlocked branch): glatter_atomic(T) is a pointer-sized slot.
 *        Do NOT use it for integers there; use the integer family below.
 *
 *   2) INTEGER ATOMICS (int gates/counters)
 *      - Use for integer state (e.g., WSI decision gate).
 *      - Types/macros: glatter_atomic_int, GLATTER_ATOMIC_INT_LOAD/STORE/CAS, GLATTER_ATOMIC_INT_INIT(v)
 *
 * Memory order:
 *   - LOAD  : acquire
 *   - STORE : release
 *   - CAS   : success=acq_rel, failure=acquire
 *
 * This header intentionally avoids heavier abstractions. It simply unifies:
 *   - C++11 std::atomic
 *   - C11 <stdatomic.h>
 *   - Windows Interlocked APIs (for C toolchains without C11 atomics)
 *   - GCC/Clang __atomic builtins
 */

/* ============================= */
/*  POINTER ATOMICS (generic)    */
/* ============================= */

#if defined(__cplusplus)

/* ----- C++11+: std::atomic<T> ----- */
#  include <atomic>
#  define glatter_atomic(T)             std::atomic<T>
#  define GLATTER_ATOMIC_LOAD(a)        ((a).load(std::memory_order_acquire))
#  define GLATTER_ATOMIC_STORE(a,v)     ((a).store((v), std::memory_order_release))
#  define GLATTER_ATOMIC_CAS(a,exp,des) ((a).compare_exchange_strong((exp),(des), \
                                        std::memory_order_acq_rel, std::memory_order_acquire))
#  define GLATTER_ATOMIC_INIT_PTR(x)    (x)

#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && !defined(__STDC_NO_ATOMICS__)

/* ----- C11: _Atomic(T) ----- */
#  include <stdatomic.h>
#  define glatter_atomic(T)             _Atomic(T)
#  define GLATTER_ATOMIC_LOAD(a)        atomic_load_explicit(&(a), memory_order_acquire)
#  define GLATTER_ATOMIC_STORE(a,v)     atomic_store_explicit(&(a), (v), memory_order_release)
#  define GLATTER_ATOMIC_CAS(a,exp,des) atomic_compare_exchange_strong_explicit(&(a), &(exp), (des), \
                                        memory_order_acq_rel, memory_order_acquire)
#  define GLATTER_ATOMIC_INIT_PTR(x)    (x)

#elif defined(_WIN32)

/* ----- Windows C (no C11): use Interlocked* on PVOID ----- */
#  include <windows.h>
#  include <stdint.h>
   typedef PVOID glatter_atomic_voidp;
/* On this branch, glatter_atomic(T) is a pointer-sized slot. Use glatter_atomic_int for integers. */
/* NOTE: Casting function pointers through uintptr_t/PVOID is non-portable per ISO C.
 * We intentionally rely on the Windows/MSVC guarantee that Interlocked*Pointer accepts
 * any pointer representation (including function pointers) on this branch. */
#  define glatter_atomic(T)             glatter_atomic_voidp
#  define GLATTER_ATOMIC_CAST_PTR(v)    ((PVOID)(uintptr_t)(v))
#  define GLATTER_ATOMIC_LOAD(a)        (InterlockedCompareExchangePointer((volatile PVOID*)&(a), NULL, NULL))
#  define GLATTER_ATOMIC_STORE(a,v)     ((void)InterlockedExchangePointer((volatile PVOID*)&(a), GLATTER_ATOMIC_CAST_PTR(v)))
#  define GLATTER_ATOMIC_CAS(a,exp,des) \
        (InterlockedCompareExchangePointer((volatile PVOID*)&(a), GLATTER_ATOMIC_CAST_PTR(des), \
                                           GLATTER_ATOMIC_CAST_PTR(exp)) == GLATTER_ATOMIC_CAST_PTR(exp))
#  define GLATTER_ATOMIC_INIT_PTR(x)    GLATTER_ATOMIC_CAST_PTR(x)

#else

/* ----- GCC/Clang C builtins ----- */
#  define glatter_atomic(T)             T
#  define GLATTER_ATOMIC_LOAD(a)        __atomic_load_n(&(a), __ATOMIC_ACQUIRE)
#  define GLATTER_ATOMIC_STORE(a,v)     __atomic_store_n(&(a), (v), __ATOMIC_RELEASE)
#  define GLATTER_ATOMIC_CAS(a,exp,des) __atomic_compare_exchange_n(&(a), &(exp), (des), 0, \
                                        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)
#  define GLATTER_ATOMIC_INIT_PTR(x)    (x)

#endif /* pointer atomics selection */


/* ============================= */
/*  INTEGER ATOMICS (int gates)  */
/* ============================= */

#if defined(__cplusplus)

/* ----- C++11: std::atomic<int> ----- */
typedef std::atomic<int> glatter_atomic_int;
#  define GLATTER_ATOMIC_INT_INIT(x)    (x)
#  define GLATTER_ATOMIC_INT_LOAD(a)    ((a).load(std::memory_order_acquire))
#  define GLATTER_ATOMIC_INT_STORE(a,v) ((a).store((v), std::memory_order_release))
#  define GLATTER_ATOMIC_INT_CAS(a,exp,des) \
        ((a).compare_exchange_strong((exp),(des), std::memory_order_acq_rel, std::memory_order_acquire))

#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && !defined(__STDC_NO_ATOMICS__)

/* ----- C11: _Atomic(int) ----- */
typedef _Atomic(int) glatter_atomic_int;
#  define GLATTER_ATOMIC_INT_INIT(x)    (x)
#  define GLATTER_ATOMIC_INT_LOAD(a)    atomic_load_explicit(&(a), memory_order_acquire)
#  define GLATTER_ATOMIC_INT_STORE(a,v) atomic_store_explicit(&(a), (v), memory_order_release)
#  define GLATTER_ATOMIC_INT_CAS(a,exp,des) \
        atomic_compare_exchange_strong_explicit(&(a), &(exp), (des), memory_order_acq_rel, memory_order_acquire)

#elif defined(_WIN32)

/* ----- Windows C: Interlocked* on LONG ----- */
#  include <windows.h>
typedef LONG glatter_atomic_int;
#  define GLATTER_ATOMIC_INT_INIT(x)    ((LONG)(x))
#  define GLATTER_ATOMIC_INT_LOAD(a)    InterlockedCompareExchange((volatile LONG*)&(a), 0, 0)
#  define GLATTER_ATOMIC_INT_STORE(a,v) ((void)InterlockedExchange((volatile LONG*)&(a), (LONG)(v)))
#  define GLATTER_ATOMIC_INT_CAS(a,exp,des) \
        (InterlockedCompareExchange((volatile LONG*)&(a), (LONG)(des), (LONG)(exp)) == (LONG)(exp))

#else

/* ----- GCC/Clang C builtins ----- */
typedef int glatter_atomic_int;
#  define GLATTER_ATOMIC_INT_INIT(x)    (x)
#  define GLATTER_ATOMIC_INT_LOAD(a)    __atomic_load_n(&(a), __ATOMIC_ACQUIRE)
#  define GLATTER_ATOMIC_INT_STORE(a,v) __atomic_store_n(&(a), (v), __ATOMIC_RELEASE)
#  define GLATTER_ATOMIC_INT_CAS(a,exp,des) \
        __atomic_compare_exchange_n(&(a), &(exp), (des), 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)

#endif /* integer atomics selection */


/* ============================= */
/*  Usage notes                  */
/* ============================= */
/*
 * - For pointer-like state (function pointers, module handles), declare:
 *       static glatter_atomic(<ptr_type>) g_ptr = GLATTER_ATOMIC_INIT_PTR(initial_value);
 *   and use GLATTER_ATOMIC_LOAD/STORE/CAS.
 *
 * - For integer gates/flags, declare:
 *       static glatter_atomic_int g_gate = GLATTER_ATOMIC_INT_INIT(0);
 *   and use GLATTER_ATOMIC_INT_LOAD/STORE/CAS.
 *
 * - On Windows C (no C11), pointer atomics are implemented with Interlocked* on PVOID.
 *   Do NOT use glatter_atomic(T) for integers there—use glatter_atomic_int.
 */
