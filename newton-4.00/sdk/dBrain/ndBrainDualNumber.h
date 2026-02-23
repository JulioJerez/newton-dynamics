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

#ifndef _ND_BRAIN_DUAL_NUMBER_H__
#define _ND_BRAIN_DUAL_NUMBER_H__

#include "ndBrainStdafx.h"

class ndBrainDualNumber
{
	public: 
	ndBrainDualNumber();
	ndBrainDualNumber(ndBrainFloat real);
	ndBrainDualNumber(const ndBrainDualNumber& src);
	ndBrainDualNumber(ndBrainFloat real, ndBrainFloat gradient);
	~ndBrainDualNumber();

	ndBrainDualNumber& operator=(const ndBrainDualNumber& other);

	ndBrainDualNumber operator+(const ndBrainDualNumber& other) const;
	ndBrainDualNumber operator-(const ndBrainDualNumber& other) const;
	ndBrainDualNumber operator*(const ndBrainDualNumber& other) const;
	ndBrainDualNumber operator/(const ndBrainDualNumber& other) const;

	ndBrainFloat m_real;
	ndBrainFloat m_gradient;
};

inline ndBrainDualNumber::ndBrainDualNumber()
	:m_real(ndBrainFloat(0.0f))
	,m_gradient(ndBrainFloat(0.0f))
{
}

inline ndBrainDualNumber::ndBrainDualNumber(ndBrainFloat real)
	:m_real(real)
	,m_gradient(ndBrainFloat(0.0f))
{
}

inline ndBrainDualNumber::ndBrainDualNumber(const ndBrainDualNumber& src)
	:m_real(src.m_real)
	,m_gradient(src.m_gradient)
{
}

inline ndBrainDualNumber::ndBrainDualNumber(ndBrainFloat real, ndBrainFloat gradient)
	:m_real(real)
	,m_gradient(gradient)
{
}

inline ndBrainDualNumber::~ndBrainDualNumber()
{
}

inline ndBrainDualNumber& ndBrainDualNumber::operator=(const ndBrainDualNumber& other)
{
	m_real = other.m_real;
	m_gradient = other.m_gradient;
	return *this;
}

inline ndBrainDualNumber ndBrainDualNumber::operator+(const ndBrainDualNumber& other) const
{
	return ndBrainDualNumber(m_real + other.m_real, m_gradient + other.m_gradient);
}

inline ndBrainDualNumber ndBrainDualNumber::operator-(const ndBrainDualNumber& other) const
{
	return ndBrainDualNumber(m_real - other.m_real, m_gradient - other.m_gradient);
}

inline ndBrainDualNumber ndBrainDualNumber::operator*(const ndBrainDualNumber& other) const
{
	const ndBrainFloat real = m_real * other.m_real;
	const ndBrainFloat grad = m_real * other.m_gradient + m_gradient * other.m_real;
	return ndBrainDualNumber(real, grad);
}

inline ndBrainDualNumber ndBrainDualNumber::operator/(const ndBrainDualNumber& other) const
{
	const ndBrainFloat real = m_real / other.m_real;
	const ndBrainFloat den = other.m_gradient * other.m_gradient;
	const ndBrainFloat num = m_gradient * other.m_real - m_real * other.m_gradient;
	const ndBrainFloat grad = num / den;
	return ndBrainDualNumber(real, grad);
}

#endif 
