//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// OpenGlUtil.h: interface for the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_OPENGLUTIL_H__E4008E54_A9CD_4C97_86D2_A2CD12AE8548__INCLUDED_)
#define AFX_OPENGLUTIL_H__E4008E54_A9CD_4C97_86D2_A2CD12AE8548__INCLUDED_

#include <stdafx.h>


//#define RECORD_LOG
//#define READ_LOG


extern GLuint g_cubeTexture;
extern GLuint g_ballTexture;
extern GLuint g_floorTexture;
extern GLuint g_buggyTexture;
extern GLuint g_tireTexture;


typedef void (*Cleanup) ();
typedef void (*RenderLoop) ();

class dVector;

#define dRAND_MAX  0x0fffffff

unsigned dRand ();
void Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... );
void SetCamera (const dVector& eye, const dVector& target); 
void InitOpenGl(int argc, char **argv, const char *title, RenderLoop renderFnt);
void LoadTexture(const char *filename, GLuint &texture);

struct MOUSE_POINT
{
	int x;
	int y;
};

void GetCursorPos(MOUSE_POINT& point);
dVector ScreenToWorld (int screenx, int screeny, int screenz);
void ShowMousePicking (const dVector& p0, const dVector& p1, dFloat radius); 


#ifndef _MSC_VER	
	#define VK_ADD		'+'
	#define VK_SUBTRACT '-'
	#define VK_LBUTTON  0x7f
	#define VK_ESCAPE   0x1b
	#define VK_F1       0x7e 
	#define VK_F2       0x7d
	#define VK_F3       0x7c
#endif	

int dGetKeyState(int key);



#endif 
