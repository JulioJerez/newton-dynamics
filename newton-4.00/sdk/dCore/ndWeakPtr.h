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

#ifndef _ND_WEAK_PTR_H_
#define _ND_WEAK_PTR_H_

#include "ndCoreStdafx.h"

template <typename T>
class ndWeakPtr
{
	public:
	ndWeakPtr();
	ndWeakPtr(T* const ptr);
	ndWeakPtr(const ndWeakPtr<T>& sp);
	~ndWeakPtr();

	void Swap(ndWeakPtr& src);

	T* operator->();
	T* operator->() const;

	T* operator* ();
	const T* operator* () const;

	operator bool() const;
	ndWeakPtr<T>& operator = (const ndWeakPtr<T>& sp);

	bool operator> (const ndWeakPtr<T>& other) const;
	bool operator< (const ndWeakPtr<T>& other) const;
	bool operator==(const ndWeakPtr<T>& other) const;

	protected:
	T* m_ptr;
};

template <typename T>
ndWeakPtr<T>::ndWeakPtr()
	:m_ptr(nullptr)
{
}

template <typename T>
ndWeakPtr<T>::ndWeakPtr(T* const ptr)
	:m_ptr(ptr)
{
}

template <typename T>
ndWeakPtr<T>::ndWeakPtr(const ndWeakPtr<T>& sp)
	:m_ptr(sp.m_ptr)
{
}

template <typename T>
ndWeakPtr<T>::~ndWeakPtr()
{
	// do nothing
}

template <typename T>
ndWeakPtr<T>& ndWeakPtr<T>::operator = (const ndWeakPtr<T>& src)
{
	m_ptr = src.m_ptr;
	return *this;
}

template <typename T>
void ndWeakPtr<T>::Swap(ndWeakPtr& src)
{
	ndSwap(m_ptr, src.m_ptr);
}

template <typename T>
T* ndWeakPtr<T>::operator* ()
{
	return m_ptr;
}

template <typename T>
const T* ndWeakPtr<T>::operator* () const
{
	return m_ptr;
}

template <typename T>
T* ndWeakPtr<T>::operator-> ()
{
	return m_ptr;
}

template <typename T>
T* ndWeakPtr<T>::operator-> () const
{
	return m_ptr;
}

template <typename T>
ndWeakPtr<T>::operator bool() const
{
	return m_ptr != nullptr;
}

template <typename T>
bool ndWeakPtr<T>::operator ==(const ndWeakPtr<T>& other) const
{
	return m_ptr == other.m_ptr;
}

template <typename T>
bool ndWeakPtr<T>::operator> (const ndWeakPtr<T>& other) const
{
	return m_ptr > other.m_ptr;
}

template <typename T>
bool ndWeakPtr<T>::operator< (const ndWeakPtr<T>& other) const
{
	return m_ptr < other.m_ptr;
}

#endif 

