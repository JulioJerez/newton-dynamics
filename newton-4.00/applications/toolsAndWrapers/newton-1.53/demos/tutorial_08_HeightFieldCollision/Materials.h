//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple 4d vector class
//********************************************************************


// Materials.h: interface for the Materials class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MATERIALS_H__C8152D99_8176_4CE4_BF86_9D241D3B54ED__INCLUDED_)
#define AFX_MATERIALS_H__C8152D99_8176_4CE4_BF86_9D241D3B54ED__INCLUDED_


struct NewtonWorld;

extern int woodID; 
extern int metalID; 
extern int levelID; 

extern int characterID; 


void SetupMaterials (NewtonWorld* nWorld);
void CleanUpMaterials (NewtonWorld* nWorld);

#endif 
