//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// JointsTutorial.h: interface for the JointsTutorial class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_JOINTSTUTORIAL_H__7A1C323F_E267_4C98_BD55_3355B21F049E__INCLUDED_)
#define AFX_JOINTSTUTORIAL_H__7A1C323F_E267_4C98_BD55_3355B21F049E__INCLUDED_

#include "Newton.h"
#include "dMatrix.h"
#include "RenderPrimitive.h"


void AddRope (NewtonWorld* nWorld, dSceneNode* root);
void AddRollingBeats (NewtonWorld* nWorld, dSceneNode* root);
void AddDoubleSwingDoors (NewtonWorld* nWorld, dSceneNode* root);

#endif 
