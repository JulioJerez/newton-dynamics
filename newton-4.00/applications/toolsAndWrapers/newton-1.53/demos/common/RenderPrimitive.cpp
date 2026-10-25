//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// 
//********************************************************************

// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "OpenGlUtil.h"
#include "RenderPrimitive.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



RenderPrimitive::RenderPrimitive(dSceneNode* parent, const dMatrix& matrix, int texture)
	:dSceneNode(parent)
{
	SetMatrix (matrix);
	m_texture = (texture == -1) ? g_cubeTexture: texture;

	m_modifierSkewAngleY = dFloat (dRand () % 360) / 6.2832f;
	m_modifierScaleAngleX = dFloat (dRand () % 360) / 6.2832f;
	m_modifierScaleAngleY = dFloat (dRand () % 360) / 6.2832f;
	m_modifierScaleAngleZ = dFloat (dRand () % 360) / 6.2832f;
}

RenderPrimitive::~RenderPrimitive()
{
}

void RenderPrimitive::SetTexture (int ID)
{
	 m_texture = (ID == -1) ? g_cubeTexture: ID;
}



BoxPrimitive::BoxPrimitive(dSceneNode* parent, const dMatrix& matrix, const dVector& size, int texture)
	:RenderPrimitive (parent, matrix, texture), m_size (size.Scale(0.5f))
{
}



//  Draws the rotating cube
void BoxPrimitive::Render()  const
{
	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glPushMatrix();	
	glMultMatrix(&m_matrix[0][0]);
	
	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	// the cube will just be drawn as six quads for the sake of simplicity
	// for each face, we specify the quad's normal (for lighting), then
	// specify the quad's 4 vertices's and associated texture coordinates
	glBegin(GL_QUADS);
	// front
	glNormal3f(0.0, 0.0, 1.0);
	glTexCoord2f(0.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y, m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f( m_size.m_x, -m_size.m_y, m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f( m_size.m_x,  m_size.m_y, m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y, m_size.m_z);
	
	// back
	glNormal3f(0.0, 0.0, -1.0);
	glTexCoord2f(0.0, 0.0); glVertex3f( m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f( m_size.m_x,  m_size.m_y, -m_size.m_z);
	
	// top
	glNormal3f(0.0, 1.0, 0.0);
	glTexCoord2f(0.0, 0.0); glVertex3f(-m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f( m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f( m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	
	// bottom
	glNormal3f(0.0, -1.0, 0.0);
	glTexCoord2f(0.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f( m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f( m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f(-m_size.m_x, -m_size.m_y,  m_size.m_z);
	
	// left
	glNormal3f(-1.0, 0.0, 0.0);
	glTexCoord2f(0.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	
	// right
	glNormal3f(1.0, 0.0, 0.0);
	glTexCoord2f(0.0, 0.0); glVertex3f(m_size.m_x,  -m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f(m_size.m_x,  -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f(m_size.m_x,   m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f(m_size.m_x,   m_size.m_y,  m_size.m_z);
	
	glEnd();

	// render the rest of the hierarchy
	RenderPrimitive::Render ();

	glPopMatrix();
} 



SpherePrimitive::SpherePrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radiusX, dFloat radiusY, dFloat radiusZ, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	m_radiusX = radiusX;
	m_radiusY = radiusY;
	m_radiusZ = radiusZ;
}

//  Draws the rotating sphere
void SpherePrimitive::Render()  const
{
	GLUquadricObj *pObj;

	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glPushMatrix();	
	dMatrix mat (m_matrix);
	mat.m_front = mat.m_front.Scale (m_radiusX);
	mat.m_up    = mat.m_up.Scale (m_radiusY);
	mat.m_right = mat.m_right.Scale (m_radiusZ);
	glMultMatrix(&mat[0][0]);
	
	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, g_ballTexture);
	
	// Get a new Quadric off the stack
	pObj = gluNewQuadric();				
	// Get a new Quadric off the stack
	gluQuadricTexture(pObj, true);						

	gluSphere(pObj, 1.0f, 20, 20);					

	gluDeleteQuadric(pObj);	

	// render the rest of the hierarchy
	RenderPrimitive::Render ();

	glPopMatrix();
} 


CylinderPrimitive::CylinderPrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	m_radius = radius;
	m_height = height;
}



//  Draws the rotating sphere
void CylinderPrimitive::Render()  const
{
	GLUquadricObj *pObj;

	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);


	glPushMatrix();

	// opengl cylinder are aligned along the z axis, we want it along the x axis, we craete a rotation matrix
	// to do the alignment
	dMatrix matrix (dgYawMatrix (3.1416f * 0.5f));
	matrix.m_posit = matrix.RotateVector (dVector (0.0f, 0.0f, -m_height * 0.5f));
	matrix = matrix * m_matrix;
	glMultMatrix(&matrix[0][0]);
	
	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	// Get a new Quadric off the stack
	pObj = gluNewQuadric();				
	gluQuadricTexture(pObj, true);						
	gluCylinder(pObj, m_radius, m_radius, m_height, 20, 2);

	// render the caps
	gluQuadricOrientation(pObj, GLU_INSIDE);
	gluDisk(pObj, 0.0f, m_radius, 20, 1);

 	glTranslatef (0.0f, 0.0f, m_height);
 	gluQuadricOrientation(pObj, GLU_OUTSIDE);
	gluDisk(pObj, 0.0f, m_radius, 20, 1);

	gluDeleteQuadric(pObj);	

	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
} 


CapsulePrimitive::CapsulePrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	m_radius = radius;
	m_height = height - radius * 2.0f;
}



//  Draws the rotating sphere
void CapsulePrimitive::Render()  const
{
	GLUquadricObj *pObj;

	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glPushMatrix();

	// opengl cylinder are aligned along the z axis, we want it along the x axis, we craete a rotation matrix
	// to do the alignment
	dMatrix matrix (dgYawMatrix (3.1416f * 0.5f));
	matrix.m_posit = matrix.RotateVector (dVector (0.0f, 0.0f, -m_height * 0.5f));
	matrix = matrix * m_matrix;

	glMultMatrix(&matrix[0][0]);
	
	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	// Get a new Quadric off the stack
	pObj = gluNewQuadric();				
	gluQuadricTexture(pObj, true);						
	gluCylinder(pObj, m_radius, m_radius, m_height, 20, 2);

	// render the caps
//	gluQuadricOrientation(pObj, GLU_INSIDE);
//	gluDisk(pObj, 0.0f, m_radius, 20, 1);
	gluSphere(pObj, m_radius, 20, 20);

 	glTranslatef (0.0f, 0.0f, m_height);
 //	gluQuadricOrientation(pObj, GLU_OUTSIDE);
//	gluDisk(pObj, 0.0f, m_radius, 20, 1);
	gluSphere(pObj, m_radius, 20, 20);

	gluDeleteQuadric(pObj);	


	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
} 


ConePrimitive::ConePrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	m_radius = radius;
	m_height = height;
}



//  Draws the rotating sphere
void ConePrimitive::Render()  const
{
	GLUquadricObj *pObj;

	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glPushMatrix();

	// Opengl cylinder are aligned along the z axis, we want it along the x axis, we create a rotation matrix
	// to do the alignment
	dMatrix matrix (dgYawMatrix (3.1416f * 0.5f));
	matrix.m_posit = matrix.RotateVector (dVector (0.0f, 0.0f, -m_height * 0.5f));
	matrix = matrix * m_matrix;

	glMultMatrix(&matrix[0][0]);

	
	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	// Get a new Quadric off the stack
	pObj = gluNewQuadric();				
	gluQuadricTexture(pObj, true);						
	gluCylinder(pObj, m_radius, 0.0f, m_height, 20, 2);

	// render the caps
	gluQuadricOrientation(pObj, GLU_INSIDE);
	gluDisk(pObj, 0.0f, m_radius, 20, 1);
  

	gluDeleteQuadric(pObj);	


	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
} 

ChamferCylinderPrimitive::ChamferCylinderPrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radius, dFloat height, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	int i;
	int j;
	int slices = 16;
	int breaks = 16;

	dFloat sliceStep;
	dFloat sliceAngle;

	dFloat breakStep;
//	dFloat breakAngle;
	dFloat textuteScale;


	radius = radius - height * 0.5f;
	if (radius < 0.1f * height) {
		radius = 0.1f * height;
	}

	
	sliceStep = 2.0f * 3.1416f / slices; 
	sliceAngle = 0.0f;

	m_list = glGenLists(1);
	glNewList (m_list, GL_COMPILE);


	breakStep = 2.0f * 3.1416f / breaks;
	dMatrix rot (dgPitchMatrix (breakStep));	

	height *= 0.5f;

	textuteScale = 1.0f;

	for (j = 0; j < slices; j ++) {
		dVector p0 (height * dCos(sliceAngle),             0.0f, radius + height * dSin(sliceAngle));
		dVector p1 (height * dCos(sliceAngle + sliceStep), 0.0f, radius + height * dSin(sliceAngle + sliceStep));

		dVector p0uv (dCos(sliceAngle) * 0.5f, 0.0f, dSin(sliceAngle) * 0.5f);
		dVector p1uv (dCos(sliceAngle + sliceStep) * 0.5f, 0.0f, dSin(sliceAngle + sliceStep) * 0.5f);

		for (i = 0; i < breaks; i ++) {
			dVector q0 (rot.RotateVector (p0));
			dVector q1 (rot.RotateVector (p1));

			dVector q0uv (rot.RotateVector (p0uv));
			dVector q1uv (rot.RotateVector (p1uv));


			glBegin(GL_POLYGON);

			dVector normal ((p0 - q0) * (q1 - q0));
			normal = normal.Scale (1.0f / dSqrt (normal % normal));
			glNormal3f(normal.m_x, normal.m_y, normal.m_z);

			glTexCoord2f(dFloat (i + 0) / breaks, dFloat (j + 0) / slices);
			glVertex3f(p0.m_x, p0.m_y, p0.m_z);

			
			glTexCoord2f(dFloat (i + 1) / breaks, dFloat (j + 0) / slices);
			glVertex3f(p1.m_x, p1.m_y, p1.m_z);

			
			glTexCoord2f(dFloat (i + 1) / breaks, dFloat (j + 1) / slices);
			glVertex3f(q1.m_x, q1.m_y, q1.m_z);

			
			glTexCoord2f(dFloat (i + 0) / breaks, dFloat (j + 1) / slices);
			glVertex3f(q0.m_x, q0.m_y, q0.m_z);
			glEnd();

			p0 = q0;
			p1 = q1;

			p0uv = q0uv;
			p1uv = q1uv;
		}

		sliceAngle += sliceStep;
	}

	glBegin(GL_POLYGON);
	glNormal3f(-1.0f, 0.0f, 0.0f);
	dVector p0 (-height, 0.0f, radius);
	dVector uv0 (0.0, 0.5f, 0.0f);
	for (i = 0; i < breaks; i ++) {
		glTexCoord2f(uv0.m_y + 0.5f, uv0.m_z + 0.5f);
		uv0 = rot.RotateVector (uv0);

		glVertex3f(p0.m_x, p0.m_y, p0.m_z);
		p0 = rot.UnrotateVector (p0);
	}
	glEnd();

	glBegin(GL_POLYGON);
	glNormal3f(1.0f, 0.0f, 0.0f);
	dVector p1 (height, 0.0f, radius);
	dVector uv1 (0.0, 0.5f, 0.0f);
	for (i = 0; i < breaks; i ++) {
		glTexCoord2f(uv1.m_y + 0.5f, uv1.m_z + 0.5f);
		uv1 = rot.RotateVector (uv1);

		glVertex3f(p1.m_x, p1.m_y, p1.m_z);
		p1 = rot.RotateVector (p1);
	}
	glEnd();

	glEndList ();

}



//  Draws the rotating sphere
void ChamferCylinderPrimitive::Render()  const
{
	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	glPushMatrix();
	glMultMatrix(&m_matrix[0][0]);

	glCallList (m_list);

	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
} 


ConvexHullPrimitive::ConvexHullPrimitive(dSceneNode* parent, const dMatrix& matrix, NewtonWorld* nWorld, NewtonCollision* collision, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	// for this geometry we will create a dummy rigid body and use the debug collision to extract 
	// the collision shape of the geometry, then we will destroy the rigid body

	NewtonBody *body;

	m_list = glGenLists(1);
	glNewList (m_list, GL_COMPILE);

	body = NewtonCreateBody (nWorld, collision);
	NewtonBodySetUserData (body, this);

	NewtonBodyForEachPolygonDo (body, GetShapeFromCollision);

	glEndList ();

	NewtonDestroyBody(nWorld, body);
}


void ConvexHullPrimitive::GetShapeFromCollision (const NewtonBody* body, int vertexCount, const dFloat* faceVertex, int id)
{
	int i;
	ConvexHullPrimitive* primitive;

	primitive = (ConvexHullPrimitive*) NewtonBodyGetUserData (body);

	glBegin(GL_POLYGON);

	// calculate the face normal for this polygon
	dVector normal (0.0f, 0.0f, 0.0f);
	dVector q0 (faceVertex[0 * 3 + 0], faceVertex[0 * 3 + 1], faceVertex[0 * 3 + 2]);
	dVector q1 (faceVertex[1 * 3 + 0], faceVertex[1 * 3 + 1], faceVertex[1 * 3 + 2]);
	for (i = 2; i < vertexCount; i ++) {
		dVector q2 (faceVertex[i * 3 + 0], faceVertex[i * 3 + 1], faceVertex[i * 3 + 2]);
		dVector dp0 (q1 - q0);
		dVector dp1 (q2 - q0);
		normal += dp0 * dp1;
	}

	normal = normal.Scale (dSqrt (1.0f / (normal % normal)));

	// submit the polygon to open gl
	glNormal3f(normal.m_x, normal.m_y, normal.m_z);
	for (i = 0; i < vertexCount; i ++) {
		dVector p0 (faceVertex[i * 3 + 0], faceVertex[i * 3 + 1], faceVertex[i * 3 + 2]);
		// calculate pseudo spherical mapping
		dVector tex (p0.Scale (1.0f / dSqrt (p0 % p0)));
		glTexCoord2f(dAsin (tex.m_x) / 3.1416f + 0.5f, dAsin (tex.m_z) / 3.1416f + 0.5f);
		glVertex3f(p0.m_x, p0.m_y, p0.m_z);

	}
	glEnd();
}



void ConvexHullPrimitive::Render() const
{
	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	glPushMatrix();
	glMultMatrix(&m_matrix[0][0]);
	glCallList (m_list);

	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
}
	


TorusPrimitive::TorusPrimitive(dSceneNode* parent, const dMatrix& matrix, dFloat radiusA, dFloat radiusC, int texture)
	:RenderPrimitive (parent, matrix, texture) 
{
	int i;
	int j;
	int k;
	int count;
	int segments;
	dFloat A;
	dFloat C;
	dFloat alpha0;
	dFloat alpha1;
	dFloat beta0;
	dFloat beta1;
	dVector array[4];
  
	A = radiusA;
	C = radiusC;

	m_list = glGenLists(1);
	glNewList (m_list, GL_COMPILE);

	segments = 16;
	// create the upper half of a change
	for (i = 0; i < segments; i ++) {
		beta0 = 2.0f * dFloat (i) * 3.1416 / segments; 
		beta1 = 2.0f * dFloat (i + 1) * 3.1416 / segments; 
		count = 0;
		for (j = 0; j < segments; j ++) {
			alpha0 = 2.0f * dFloat (j) * 3.1416 / segments; 
			alpha1 = 2.0f * dFloat (j + 1) * 3.1416 / segments; 
			array[0].m_x = A * dSin (alpha0);
			array[0].m_y = (C + A * dCos(alpha0)) * dCos (beta0);
			array[0].m_z = (C + A * dCos(alpha0)) * dSin (beta0);

			array[1].m_x = A * dSin (alpha0);
			array[1].m_y = (C + A * dCos(alpha0)) * dCos (beta1);
			array[1].m_z = (C + A * dCos(alpha0)) * dSin (beta1);

			array[2].m_x = A * dSin (alpha1);
			array[2].m_y = (C + A * dCos(alpha1)) * dCos (beta1);
			array[2].m_z = (C + A * dCos(alpha1)) * dSin (beta1);

			array[3].m_x = A * dSin (alpha1);
			array[3].m_y = (C + A * dCos(alpha1)) * dCos (beta0);
			array[3].m_z = (C + A * dCos(alpha1)) * dSin (beta0);

			dVector e0 (array[1] - array[0]);
			dVector e1 (array[2] - array[0]);
			dVector normal (e0 * e1);
			normal = normal.Scale (1.0f / dSqrt( (normal % normal)));

			// submit the polygon to open gl
			glBegin(GL_POLYGON);
			glNormal3f(normal.m_x, normal.m_y, normal.m_z);
			for (k = 0; k < 4; k ++) {
				dVector p0 (array[k].m_x, array[k].m_y, array[k].m_z);
				dVector tex (p0.Scale (1.0f / dSqrt (p0 % p0)));

				glTexCoord2f(dAsin (tex.m_x) / 3.1416f + 0.5f, dAsin (tex.m_z) / 3.1416f + 0.5f);
				glVertex3f(p0.m_x, p0.m_y, p0.m_z);
			}
			glEnd();

		}

	}

	glEndList ();
}


void TorusPrimitive::Render() const
{
	// set the color of the cube's surface
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	
	glPushMatrix();
	glMultMatrix(&m_matrix[0][0]);

	glCallList (m_list);

	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
}



SkyBoxPrimitive::SkyBoxPrimitive(dSceneNode* parent)
	:RenderPrimitive (parent, GetIdentityMatrix())
{
	dFloat boxsize;

	boxsize = 200.0f;

	m_size = dVector (boxsize, boxsize, boxsize);
	LoadTexture ("NewtonSky0001.tga", m_textures[0]);
	LoadTexture ("NewtonSky0002.tga", m_textures[1]);
	LoadTexture ("NewtonSky0003.tga", m_textures[2]);
	LoadTexture ("NewtonSky0004.tga", m_textures[3]);
	LoadTexture ("NewtonSky0005.tga", m_textures[4]);
	LoadTexture ("NewtonSky0006.tga", m_textures[5]);
}

void SkyBoxPrimitive::Render() const
{
	dMatrix matrix;

	// get the model veiMatrix; 
	glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);

	dMatrix skyMatrix (m_matrix);
	skyMatrix.m_posit = matrix.UnrotateVector (matrix.m_posit.Scale (-1.0f));
	skyMatrix.m_posit.m_y += 50.0f; 
	
	glPushMatrix();
	glMultMatrix(&skyMatrix[0][0]);

	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDisable (GL_LIGHTING);

	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);

	dFloat padd;

	padd = 5.0e-3f;

//	uncommenting this will make a perfect sky box texture stitch, unfortunately
//  this option ios not supported by all version of opengl	
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); 
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	
	// the cube will just be drawn as six quads for the sake of simplicity
	// for each face, we specify the quad's normal (for lighting), then
	// specify the quad's 4 vertices's and associated texture coordinates
	// front
	glBindTexture(GL_TEXTURE_2D, m_textures[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f( m_size.m_x,  m_size.m_y, m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f( m_size.m_x, -m_size.m_y, m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, m_size.m_z);
	glEnd();


	// left
	glBindTexture(GL_TEXTURE_2D, m_textures[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glEnd();


	// right
	glBindTexture(GL_TEXTURE_2D, m_textures[4]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(m_size.m_x,   m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(m_size.m_x,   m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(m_size.m_x,  -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(m_size.m_x,  -m_size.m_y,  m_size.m_z);
	glEnd();

		
	// back
	glBindTexture(GL_TEXTURE_2D, m_textures[3]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f( m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f( m_size.m_x, -m_size.m_y, -m_size.m_z);
	glEnd();

	// bottom
	glBindTexture(GL_TEXTURE_2D, m_textures[5]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(0.0, 1.0); glVertex3f( m_size.m_x, -m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f( m_size.m_x, -m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f(-m_size.m_x, -m_size.m_y, -m_size.m_z);
	glEnd();	 	

	// top
	glBindTexture(GL_TEXTURE_2D, m_textures[2]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(0.0, 0.0); glVertex3f( m_size.m_x,  m_size.m_y, -m_size.m_z);
	glTexCoord2f(1.0, 0.0); glVertex3f( m_size.m_x,  m_size.m_y,  m_size.m_z);
	glTexCoord2f(1.0, 1.0); glVertex3f(-m_size.m_x,  m_size.m_y,  m_size.m_z);
	glEnd();	 


	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glEnable (GL_LIGHTING);

	// render the rest of the hierarchy
	glPopMatrix();
	RenderPrimitive::Render ();

}
	
SkyBoxPrimitive::~SkyBoxPrimitive()
{
}
