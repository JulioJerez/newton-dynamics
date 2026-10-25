//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

#ifndef __TIMER_H_INCLUDED__
#define __TIMER_H_INCLUDED__

#include <stdafx.h>


class CHiResTimer
{
	public:
	CHiResTimer(); 
	~CHiResTimer();
	dFloat GetElapsedSeconds();
	unsigned GetTimeInMiliseconds();
	
	private:
	unsigned m_prevTime;
	unsigned m_totalFrames;
};

#endif 
