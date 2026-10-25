//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// OpenGlUtil.cpp: implementation of the OpenGlUtil class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "OpenGlUtil.h"


#ifdef RECORD_LOG
	FILE * file;
#else 
	#ifdef READ_LOG
		FILE * file;
	#endif
#endif



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


// opengl callbacks

static int windowWidth; 
static int windowHeight;
static RenderLoop g_render;


// index for the texture we'll load for the cube
GLuint g_cubeTexture;
GLuint g_ballTexture;
GLuint g_floorTexture;
GLuint g_buggyTexture;
GLuint g_tireTexture;


#ifdef _MSC_VER
	#if (_MSC_VER < 1300) 
		//VC7 or later, building with pre-VC7 runtime libraries
		//defined by VC6 C libs
		extern "C" long _ftol (double); 
		extern "C" long _ftol2( double dblSource ) 
		{ 
			return _ftol( dblSource ); 
		}
	#endif

	#if (_MSC_VER < 1400) 
//		extern "C" long _ftol2_sse() 
//		{ 
//			double val;
//			_asm fstp qword ptr val;
//			return _ftol( val ); 
//		}
	#endif
#endif


  
// Rotate the cube by 4 degrees and force a redisplay.
static void Animate()
{
	glutPostRedisplay();
} 


//	Reset the viewport for window changes
static void Reshape(int width, int height)
{
	if (height == 0)
		return;
	glViewport(0, 0, (GLsizei) width, (GLsizei) height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, width/height, 0.2, 5000.0);
	
	glMatrixMode(GL_MODELVIEW);
} 


//	Clear and redraw the scene.
static void Display()
{
	// clear the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	g_render ();
	
	// draw everything and swap the display buffer
	glFlush();
	glutSwapBuffers();
} 


#ifndef _MSC_VER	
static int lastKey = 0;
				
static void NormalKeboard(unsigned char key, int x, int y) 
{
	lastKey = key; 
}

static void ReleaseNormalKeboard(unsigned char key, int x, int y) 
{
	if (lastKey == key) {
		lastKey = key; 
	}
}


static void SpecialKeboard(int key, int x, int y) 
{
/*
	GLUT_KEY_F1		F1 function key
	GLUT_KEY_F2		F2 function key
	GLUT_KEY_F3		F3 function key
	GLUT_KEY_F4		F4 function key
	GLUT_KEY_F5		F5 function key
	GLUT_KEY_F6		F6 function key
	GLUT_KEY_F7		F7 function key
	GLUT_KEY_F8		F8 function key
	GLUT_KEY_F9		F9 function key
	GLUT_KEY_F10		F10 function key
	GLUT_KEY_F11		F11 function key
	GLUT_KEY_F12		F12 function key
	GLUT_KEY_LEFT		Left function key
	GLUT_KEY_RIGHT		Up function key
	GLUT_KEY_UP		Right function key
	GLUT_KEY_DOWN		Down function key
	GLUT_KEY_PAGE_UP	Page Up function key
	GLUT_KEY_PAGE_DOWN	Page Down function key
	GLUT_KEY_HOME		Home function key
	GLUT_KEY_END		End function key
	GLUT_KEY_INSERT	
*/

	switch (key) {
		case GLUT_KEY_F1:
			lastKey = VK_F1; 
			break;

		case GLUT_KEY_F2:
			lastKey = VK_F2; 
			break;

		case GLUT_KEY_F3:
			lastKey = VK_F3; 
			break;
	}
}

#endif 


static int mouse_x = -1;
static int mouse_y = -1;
static int mouseButton_Down = 0;

static void MouseMotion (int x, int y)
{
	mouse_x = x;
	mouse_y = y;
}

static void CaptureMousePos (int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON) {
		mouseButton_Down = (state == GLUT_DOWN) ? true : false ;
	}
}




//	Loads the texture from the specified file and stores it in iTexture. Note
//	that we're using the GLAUX library here, which is generally discouraged,
//	but in this case spares us having to write a bitmap loading routine.
void LoadTexture(const char *filename, GLuint &texture)
{
	#pragma pack(1)
	struct TGAHEADER
	{
		GLbyte	identsize;              // Size of ID field that follows header (0)
		GLbyte	colorMapType;           // 0 = None, 1 = palette
		GLbyte	imageType;              // 0 = none, 1 = indexed, 2 = rgb, 3 = grey, +8=rle
		unsigned short	colorMapStart;  // First color map entry
		unsigned short	colorMapLength; // Number of colors
		unsigned char 	colorMapBits;   // bits per palette entry
		unsigned short	xstart;         // image x origin
		unsigned short	ystart;         // image y origin
		unsigned short	width;          // width in pixels
		unsigned short	height;         // height in pixels
		GLbyte	bits;                   // bits per pixel (8 16, 24, 32)
		GLbyte	descriptor;             // image descriptor
	};
	#pragma pack(8)

	


	FILE *pFile;				// File pointer
	TGAHEADER tgaHeader;		// TGA file header
	unsigned lImageSize;		// Size in bytes of image
	short sDepth;				// Pixel depth;
	GLbyte	*pBits = NULL;      // Pointer to bits
	//char fullPathName[256];
	
	GLint iWidth;
	GLint iHeight;
	GLint iComponents;
	GLenum eFormat;
    
    	// Default/Failed values
    	eFormat = GL_RGBA;
    	iComponents = GL_RGB8;

	texture = 0;

	//char workingPath[256];
	char fullPathName[256];
	GetWorkingFileName (filename, fullPathName);
	pFile = fopen (fullPathName, "rb");
   	if(pFile == NULL) {
		return;
	}
    
	//_ASSERTE (sizeof (TGAHEADER) == 18);
    // Read in header (binary) sizeof(TGAHEADER) = 18
    fread(&tgaHeader, 18, 1, pFile);

    
    // Do byte swap for big vs little Indian
	tgaHeader.colorMapStart = SWAP_INT16(tgaHeader.colorMapStart);
	tgaHeader.colorMapLength = SWAP_INT16(tgaHeader.colorMapLength);
	tgaHeader.xstart = SWAP_INT16(tgaHeader.xstart);
	tgaHeader.ystart = SWAP_INT16(tgaHeader.ystart);
	tgaHeader.width = SWAP_INT16(tgaHeader.width);
	tgaHeader.height = SWAP_INT16(tgaHeader.height);
	        
    // Get width, height, and depth of texture
    iWidth = tgaHeader.width;
    iHeight = tgaHeader.height;
    sDepth = tgaHeader.bits / 8;
	_ASSERTE (sDepth == 4);

    
    // Put some validity checks here. Very simply, I only understand
    // or care about 8, 24, or 32 bit targa's.
    if(tgaHeader.bits != 8 && tgaHeader.bits != 24 && tgaHeader.bits != 32) {
		fclose(pFile);
		return;
	}


    // Calculate size of image buffer
    lImageSize = tgaHeader.width * tgaHeader.height * sDepth;
    
    // Allocate memory and check for success
    pBits = (GLbyte *)malloc(lImageSize * sizeof(GLbyte));
    if(pBits == NULL) {
		fclose(pFile);
		return;
	}


    
    // Read in the bits
    // Check for read error. This should catch RLE or other 
    // weird formats that I don't want to recognize
    if(fread(pBits, lImageSize, 1, pFile) != 1)  {
		fclose(pFile);
        free(pBits);
        return;
   }

    
    // Set OpenGL format expected
	#ifdef _MIPS_ARCH
		switch(sDepth)
		{
			// mac and mips const
			case 3:     // Most likely case
				eFormat = GL_RGBA;
				iComponents = GL_RGB8;
				break;
			case 4:
				eFormat = GL_BGRA;
				iComponents = GL_RGBA8;
				break;
			case 1:
				eFormat = GL_LUMINANCE;
				iComponents = GL_LUMINANCE8;
				break;
		};	
	#else
		switch(sDepth)
		{
			// intel arch
			case 3:     // Most likely case
				eFormat = GL_RGB;
				iComponents = GL_RGB8;
				break;
			case 4:
				eFormat = GL_BGRA_EXT;
				iComponents = GL_RGBA8;
				break;
			case 1:
				eFormat = GL_LUMINANCE;
				iComponents = GL_LUMINANCE8;
				break;
		};
	#endif
        

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

   // select modulate to mix texture with color for shading
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

	

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexImage2D(GL_TEXTURE_2D, 0, iComponents, iWidth, iHeight, 0, eFormat, GL_UNSIGNED_BYTE, pBits);

    // when texture area is small, bilinear filter the closest mipmap
//  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST );

	// when texture area is small, trilinear filter mipmaped
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    // when texture area is large, bilinear filter the first mipmap
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    // build our texture mipmaps
    gluBuild2DMipmaps (GL_TEXTURE_2D, iComponents, iWidth, iHeight, eFormat, GL_UNSIGNED_BYTE, pBits);


    // Done with File
    fclose(pFile);
	free(pBits);        
} 


void InitOpenGl(
	int argc, 
	char **argv, 
	const char *title,
	RenderLoop renderFnt)
{
	// initialize opengl	
	int argc_ = 0;
	glutInit (&argc_, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	windowWidth = 600; 
	windowHeight = 480;

//windowWidth = 1024; 
//windowHeight = 780;

	
	// Create the window
	glutInitWindowSize(windowWidth, windowHeight);
//	glutInitWindowPosition(400, 350);
	glutCreateWindow(title);

	// uncomment this to run in full screen (very slow)
//	glutFullScreen();

	// set the background color
	glClearColor(0.5, 0.5, 0.5, 0.75);
	
	// set the shading model
	glShadeModel(GL_SMOOTH);
	
	// set up a single white light
	
	GLfloat lightColor[] = { 0.7f, 0.7f, 0.7f, 0.0 };
	GLfloat lightAmbientColor[] = { 0.125f, 0.125f, 0.125f, 0.0 };
	GLfloat lightPoition[] = { 500.0f, 200.0f, 500.0f, 0.0 };
	
	glLightfv(GL_LIGHT0, GL_POSITION, lightPoition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbientColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor);
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);


	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	
	// load the texture for the cube
	LoadTexture("newton.tga", g_cubeTexture);

	// load the texture for the cube
	LoadTexture("earthmap.tga", g_ballTexture);

	// load the texture for the font
	LoadTexture("floor.tga", g_floorTexture);

	// load the texture for the font
	LoadTexture("buggy_sk.tga", g_buggyTexture);


	// load the texture for the font
	LoadTexture("tyre.tga", g_tireTexture);

	
	// make the modelview matrix active, and initialize it
	glMatrixMode(GL_MODELVIEW);

	
	// Register the event callback functions
	glutDisplayFunc(Display); 
	glutReshapeFunc(Reshape);

	glutMouseFunc(CaptureMousePos);
	glutMotionFunc(MouseMotion); 
	glutPassiveMotionFunc(MouseMotion); 

//#endif
	#ifndef _MSC_VER	
		glutKeyboardFunc(NormalKeboard);
		glutSpecialFunc(SpecialKeboard);	
		glutKeyboardUpFunc(ReleaseNormalKeboard);
	#endif

	// set the call to restore the keyboard before exit
	glutIdleFunc(Animate);

	g_render = renderFnt;

#ifdef RECORD_LOG
	file = fopen ("log.log", "wb");
#endif

#ifdef READ_LOG
	file = fopen ("log.log", "rb");
#endif

}


void SetCamera (const dVector& eye, const dVector& target)
{

	glMatrixMode(GL_MODELVIEW);
	// set up the view orientation looking at the origin from slightly above
	// and to the left
	glLoadIdentity();
	//gluLookAt(0.0, 1.0, 6.0, 0.0, 0.0, 0.0,	0.0, 1.0, 0.0);
	gluLookAt(eye.m_x, eye.m_y, eye.m_z, target.m_x, target.m_y, target.m_z, 0.0, 1.0, 0.0);
}


void Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... )
{	
	int i;
	int len;
	char string[1024];
	
	va_list argptr;
	
	va_start (argptr, fmt);
	vsprintf (string, fmt, argptr);
	va_end( argptr );

	glColor3f(color.m_x, color.m_y, color.m_z);
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, windowWidth, 0, windowHeight );

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	glRasterPos3f(x, windowHeight - 15 - y, 0);
	len = (int) strlen (string );
	for (i = 0; i < len; i++) {
		glutBitmapCharacter (GLUT_BITMAP_TIMES_ROMAN_24, string[i] );
	}
	
	glMatrixMode(GL_TEXTURE);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glEnable( GL_DEPTH_TEST );
}



dVector ScreenToWorld (int screenx, int screeny, int screenz)
{
	//Where the values will be stored
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	glGetIntegerv(GL_VIEWPORT, viewport); 

	//Where the 16 doubles of the matrix are to be stored
	GLdouble modelview[16]; 

	//Retrieve the matrix
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 

	GLdouble projection[16]; 
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLdouble winX = 0.0;
	GLdouble winY = 0.0;
	GLdouble winZ = 0.0; //The coordinates to pass in

	winX = dFloat(screenx); //Store the x cord
	winY = dFloat(screeny); //Store the y cord
	winZ = dFloat(screenz); //Store the Z cord

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	winY = (dFloat)viewport[3] - winY; 

	GLdouble objx;
	GLdouble objy;
	GLdouble objz;
	gluUnProject (winX, winY, winZ, modelview, projection, viewport, &objx, &objy, &objz);

	//gluProject(objx, objy, objz, modelview, projection, viewport, &winX, &winY, &winZ);
	return dVector (dFloat(objx), dFloat(objy), dFloat(objz));
}



unsigned dRand ()
{
	#define RAND_MUL 31415821u
	static unsigned randSeed = 0;
 	randSeed = RAND_MUL * randSeed + 1; 
	return randSeed & dRAND_MAX;
}


void ShowMousePicking (const dVector& p0, const dVector& p1, dFloat radius)
{
//	dFloat radius = 0.125f;
	// set the color of the cube's surface
//	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
//	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
//	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
//	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	// set up the cube's texture
	glColor3f(1.0f, 1.0f, 1.0f);
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
/*
	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricTexture(pObj, false);
	dMatrix matrix (GetIdentityMatrix());

	glPushMatrix();
	matrix.m_posit = p0;
	matrix.m_posit.m_w = 1.0f;
	glMultMatrix(&matrix[0][0]);
	gluSphere(pObj, radius, 10, 10);					
	glPopMatrix();

	glPushMatrix();
	matrix.m_posit = p1;
	matrix.m_posit.m_w = 1.0f;
	glMultMatrix(&matrix[0][0]);
	gluSphere(pObj, radius, 10, 10);					
	glPopMatrix();

	gluDeleteQuadric(pObj);	

	dVector dir (p1 - p0);
	dFloat lenght (dir % dir);
	if (lenght > 1.0e-2f) {
	
		glPushMatrix();
		
		lenght = dSqrt (lenght);
		dMatrix align (dgYawMatrix(0.5f * 3.1426f));
		align.m_posit.m_x = -lenght * 0.5f;
		matrix = align * dgGrammSchmidt(dir);
		matrix.m_posit += (p1 + p0).Scale (0.5f);
		glMultMatrix(&matrix[0][0]);
		
		// Get a new Quadric off the stack
		pObj = gluNewQuadric();				
		gluCylinder(pObj, radius * 0.5f, radius * 0.5f, lenght, 10, 2);
		gluDeleteQuadric(pObj);	
		glPopMatrix();
	}

//	glEnable (GL_LIGHTING);
*/


	glLineWidth(3.0f);
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(p0.m_x, p0.m_y, p0.m_z);
	glVertex3f(p1.m_x, p1.m_y, p1.m_z);

	glEnd();
	glLineWidth(1.0f);

	// draw the bone points
	glPointSize(10.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 0.0f);
	glVertex3f(p0.m_x, p0.m_y, p0.m_z);
	glVertex3f(p1.m_x, p1.m_y, p1.m_z);
	glEnd();
	glPointSize(1.0f);
}



void GetCursorPos(MOUSE_POINT& point)
{
	point.x = mouse_x;
	point.y = mouse_y;

#ifdef RECORD_LOG
	fwrite (& point, sizeof (MOUSE_POINT), 1, file);
	fflush (file);
#endif

#ifdef READ_LOG
	fread (& point, sizeof (MOUSE_POINT), 1, file);
#endif

}

int dGetKeyState(int key)
{
	#ifdef _MSC_VER	
		int code;
  		code = GetAsyncKeyState (key);

#ifdef RECORD_LOG
		fwrite (&code, sizeof (int), 1, file);
		fflush (file);
#endif

#ifdef READ_LOG
		fread (&code, sizeof (int), 1, file);
#endif


		return code;
	#else
		if (key == VK_LBUTTON) {
			return mouseButton_Down ? 0x8000 : 0;
		} else {
			key = tolower (key); 
			if (key == lastKey) {
				lastKey = 0;
				return 0x8000;
			} else {
				return 0;
			}
		}
	#endif
}
