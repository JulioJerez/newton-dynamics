//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************


#include <stdafx.h>
#include "HiResTimer.h"
#include "OpenGlUtil.h"

#define LOCK_FRAME_RATE

const dFloat TICKS2SEC = 1.0e-3f;

CHiResTimer::CHiResTimer() 
{
	m_prevTime = 0;
}

CHiResTimer::~CHiResTimer() 
{
}



dFloat CHiResTimer::GetElapsedSeconds()
{
	dFloat timeStep;
	unsigned miliseconds;



	
	miliseconds = glutGet (GLUT_ELAPSED_TIME);

	// optimal keep the fps below 70 fps
	#ifdef LOCK_FRAME_RATE
	while ((miliseconds - m_prevTime) < 14) {
		miliseconds = glutGet (GLUT_ELAPSED_TIME);
	}
	#endif

	timeStep = dFloat (miliseconds - m_prevTime) * TICKS2SEC;
	m_prevTime = miliseconds;
	
	if (timeStep > 0.1f) {
		timeStep = 0.1f;
	}
	if (timeStep < 0.005f) {
		timeStep = 0.005f;
	}


#ifdef RECORD_LOG
//	Sleep (100);
	extern FILE * file;
	fwrite (&timeStep, sizeof (float), 1, file);
	fflush (file);
#endif

#ifdef READ_LOG
	extern FILE * file;
	//Sleep (150);
	fread (&timeStep, sizeof (float), 1, file);
#endif

	
	return timeStep;
} 

unsigned CHiResTimer::GetTimeInMiliseconds()
{
	return glutGet (GLUT_ELAPSED_TIME);
}



