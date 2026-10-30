//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple 4d vector class
//********************************************************************

// HeightFieldCollision.cpp: implementation of the HeightFieldCollision class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "OpenGlUtil.h"
#include "HeightFieldCollision.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


extern bool debugLinesMode;
extern dInt32 hightMap_debugCount;
extern dVector hightMap_debugLines[1024][2];


static dInt32 debugRayCast = 0;
dVector debugRayCastLines[1024][2];

// comment this define out to see the different in contact generation
#define FILTER_FLAT_EDGES


HeightFieldCollision::HeightFieldCollision(dSceneNode* parent, NewtonWorld* nWorld, NewtonBodyDestructor destructor)
	:RenderPrimitive(parent, GetIdentityMatrix())
{
	dInt32 x;
	dInt32 z;
	dInt32 x0;
	dInt32 x1;
	dInt32 z0;
	dInt32 z1;
	

	// build a simple height field array. Note this could be load it from a texture map file
	for (z = 0; z < HEIGHT_SIZE; z ++) {
		for (x = 0; x < HEIGHT_SIZE; x ++) {
			// add some pseudo Gaussian randomness to the height field
			dFloat r = dFloat (dRand() + dRand()) / dRAND_MAX - 1.0f;
			m_heightField[z][x] = r * 0.125f;

			// add sine wave for rolling terrain
			m_heightField[z][x] += (5.0f * dSin(x/5.0f) + 7.0f * dCos((z)/7.0f)) * 1.5f;

m_heightField[z][x] = 0.0f;
		}
	}

	// make all triangle edge collidable
	memset (m_edgeFlags, 0, sizeof (m_edgeFlags));


	// calculate the min and max of the terrain geometry
	m_minBox = dVector (1.0e10f, 1.0e10f, 1.0e10f);
	m_maxBox = dVector (-1.0e10f, -1.0e10f, -1.0e10f);

	m_list = glGenLists(1);
	glNewList (m_list, GL_COMPILE);

	// create a display list for fast open gl rendering
	glColor3f(1, 1.0, 0.1);         
	for (x = 0; x < HEIGHT_SIZE - 1; x ++) {
//		glBegin(GL_LINES);
		glBegin(GL_TRIANGLE_STRIP);
		for (z = 0; z < HEIGHT_SIZE - 1; z ++) {
			{
				x0  = x + 1;
				z0 = ((z - 1) < 0) ? 0 : z - 1;
				z1 = ((z + 1) > (HEIGHT_SIZE - 1)) ? HEIGHT_SIZE - 1 : z + 1 ;
				dVector v0 (CELL_SIZE * x0, m_heightField[z0][x0], CELL_SIZE * z0);
				dVector v1 (CELL_SIZE * x0, m_heightField[z1][x0], CELL_SIZE * z1);
				dVector z10 (v1 - v0);

				z0 = z;
				x0 = ((x + 1 - 1) < 0) ? 0 : x - 1;
				x1 = ((x + 1 + 1) > (HEIGHT_SIZE - 1)) ? HEIGHT_SIZE - 1 : x + 1 + 1;
				dVector r0 (CELL_SIZE * x0, m_heightField[z0][x0], CELL_SIZE * z0);
				dVector r1 (CELL_SIZE * x1, m_heightField[z0][x0], CELL_SIZE * z0);
				dVector x10 (r1 - r0);
				dVector normal (z10 * x10);
				normal = normal.Scale (dSqrt (1.0f / (normal % normal)));
				
				glTexCoord2f((x + 1) * TEXTURE_SCALE, z * TEXTURE_SCALE);
				glNormal3f(normal.m_x, normal.m_y, normal.m_z);
				glVertex3f (CELL_SIZE * (x+1), m_heightField[z][x + 1], CELL_SIZE * z);

				// get the min and max box size
				m_minBox.m_x = min (CELL_SIZE * (x+1), m_minBox.m_x);
				m_minBox.m_y = min (m_heightField[z][x + 1], m_minBox.m_y);
				m_minBox.m_z = min (CELL_SIZE * z, m_minBox.m_z);
				m_maxBox.m_x = max (CELL_SIZE * (x+1), m_maxBox.m_x);
				m_maxBox.m_y = max (m_heightField[z][x + 1], m_maxBox.m_y);
				m_maxBox.m_z = max (CELL_SIZE * z, m_maxBox.m_z);
			}

			{
				x0  = x;
				z0 = ((z - 1) < 0) ? 0 : z - 1;
				z1 = ((z + 1) > (HEIGHT_SIZE - 1)) ? HEIGHT_SIZE - 1 : z + 1 ;
				dVector v0 (CELL_SIZE * x0, m_heightField[z0][x0], CELL_SIZE * z0);
				dVector v1 (CELL_SIZE * x0, m_heightField[z1][x0], CELL_SIZE * z1);
				dVector z10 (v1 - v0);

				z0 = z;
				x0 = ((x - 1) < 0) ? 0 : x - 1;
				x1 = ((x + 1) > (HEIGHT_SIZE - 1)) ? HEIGHT_SIZE - 1 : x + 1;
				dVector r0 (CELL_SIZE * x0, m_heightField[z0][x0], CELL_SIZE * z0);
				dVector r1 (CELL_SIZE * x1, m_heightField[z0][x1], CELL_SIZE * z0);
				dVector x10 (r1 - r0);
				dVector normal (z10 * x10);
				normal = normal.Scale (dSqrt (1.0f / (normal % normal)));
				
				glTexCoord2f(x * TEXTURE_SCALE, z * TEXTURE_SCALE);
				glNormal3f(normal.m_x, normal.m_y, normal.m_z);
				glVertex3f(CELL_SIZE * x, m_heightField[z][x], CELL_SIZE * z);

				// get the min and max box size
				m_minBox.m_x = min (CELL_SIZE * x, m_minBox.m_x);
				m_minBox.m_y = min (m_heightField[z][x], m_minBox.m_y);
				m_minBox.m_z = min (CELL_SIZE * z, m_minBox.m_z);
				m_maxBox.m_x = max (CELL_SIZE * x, m_maxBox.m_x);
				m_maxBox.m_y = max (m_heightField[z][x], m_maxBox.m_y);
				m_maxBox.m_z = max (CELL_SIZE * z, m_maxBox.m_z);
			}
		}
		glEnd();
	}
	
	glEndList();    

	
	// create a rigid body and collision geometry
	NewtonCollision* collision;
  
	collision = NewtonCreateUserMeshCollision (nWorld, &m_minBox[0], &m_maxBox[0], this, MeshCollisionCollideCallback, UserMeshCollisionRayHitCallback, NULL);

	// create the level rigid body
	m_level = NewtonCreateBody(nWorld, collision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (m_level, this);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonReleaseCollision (nWorld, collision);

	// set the global position of this body
	NewtonBodySetMatrix (m_level, &m_matrix[0][0]); 

	// set the destructor for this object
	NewtonBodySetDestructorCallback (m_level, destructor);


	dVector boxP0; 
	dVector boxP1; 
	// get the position of the aabb of this geometry
	NewtonCollisionCalculateAABB (collision, &m_matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// add some extra padding the world size
	boxP0.m_x -=  10.0f;
	boxP0.m_y -=  10.0f;
	boxP0.m_z -=  10.0f;
	boxP1.m_x +=  10.0f;
	boxP1.m_y += 400.0f;
	boxP1.m_z +=  10.0f;

	// set the world size
	NewtonSetWorldSize (nWorld, &boxP0.m_x, &boxP1.m_x); 
}

HeightFieldCollision::~HeightFieldCollision()
{
}


NewtonBody* HeightFieldCollision::GetRigidBody() const
{
	return m_level;
}

void HeightFieldCollision::Render() const
{
	GLfloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

	glPushMatrix();		
	glMultMatrix(&m_matrix[0][0]);

	// set up the cube's texture
	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, g_floorTexture);

	glCallList (m_list);


	// if in debug mode show last ray cast
	if (debugLinesMode) {
		dInt32 i;
		glDisable (GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
	
		glBegin(GL_LINES);
		glColor3f(0.0f, 1.0f, 1.0f);

		for (i = 0; i < debugRayCast; i ++) {
			glVertex3f (debugRayCastLines[i][0].m_x, debugRayCastLines[i][0].m_y, debugRayCastLines[i][0].m_z);
			glVertex3f (debugRayCastLines[i][1].m_x, debugRayCastLines[i][1].m_y, debugRayCastLines[i][1].m_z);
		}
		glEnd();

		glEnable(GL_LIGHTING);
		glEnable(GL_TEXTURE_2D);
		
	}

	// render the rest of the hierarchy
	RenderPrimitive::Render ();
	glPopMatrix();
}


// calculate the bounding box surrounding a line segment
void HeightFieldCollision::CalculateMinExtend2d (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1)
{
	dFloat x0;
	dFloat x1;
	dFloat z0;
	dFloat z1;

	x0 = min (p0.m_x, p1.m_x) - 1.0e-3f;
	z0 = min (p0.m_z, p1.m_z) - 1.0e-3f;

	x1 = max (p0.m_x, p1.m_x) + 1.0e-3f;
	z1 = max (p0.m_z, p1.m_z) + 1.0e-3f;

	x0 = CELL_SIZE * dFloor (x0 * (1.0f / CELL_SIZE));
	z0 = CELL_SIZE * dFloor (z0 * (1.0f / CELL_SIZE));
	x1 = CELL_SIZE * dFloor (x1 * (1.0f / CELL_SIZE)) + CELL_SIZE;
	z1 = CELL_SIZE * dFloor (z1 * (1.0f / CELL_SIZE)) + CELL_SIZE;

	boxP0.m_x = max (x0, m_minBox.m_x);
	boxP0.m_z = max (z0, m_minBox.m_z);

	boxP1.m_x = min (x1, m_maxBox.m_x);
	boxP1.m_z = min (z1, m_maxBox.m_z);
}


void HeightFieldCollision::CalculateMinExtend3d (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1)
{
	dFloat x0;
	dFloat x1;
	dFloat y0;
	dFloat y1;
	dFloat z0;
	dFloat z1;

	x0 = min (p0.m_x, p1.m_x) - 1.0e-3f;
	y0 = min (p0.m_y, p1.m_y) - 1.0e-3f;
	z0 = min (p0.m_z, p1.m_z) - 1.0e-3f;

	x1 = max (p0.m_x, p1.m_x) + 1.0e-3f;
	y1 = max (p0.m_y, p1.m_y) + 1.0e-3f;
	z1 = max (p0.m_z, p1.m_z) + 1.0e-3f;

	x0 = CELL_SIZE * dFloor (x0 * (1.0f / CELL_SIZE));
	y0 = CELL_SIZE * dFloor (z0 * (1.0f / CELL_SIZE));
	z0 = CELL_SIZE * dFloor (z0 * (1.0f / CELL_SIZE));

	x1 = CELL_SIZE * dFloor (x1 * (1.0f / CELL_SIZE)) + CELL_SIZE;
	y1 = CELL_SIZE * dFloor (y1 * (1.0f / CELL_SIZE)) + CELL_SIZE;
	z1 = CELL_SIZE * dFloor (z1 * (1.0f / CELL_SIZE)) + CELL_SIZE;

	boxP0.m_x = max (x0, m_minBox.m_x);
	boxP0.m_y = max (y0, m_minBox.m_y);
	boxP0.m_z = max (z0, m_minBox.m_z);

	boxP1.m_x = min (x1, m_maxBox.m_x);
	boxP1.m_y = min (z1, m_maxBox.m_y);
	boxP1.m_z = min (z1, m_maxBox.m_z);
	
}


// clip a line segment against a box  
bool HeightFieldCollision::ClipRay2d (dVector& p0, dVector& p1, const dVector& boxP0, const dVector& boxP1) 
{
	dFloat t;
	dFloat tmp0;
	dFloat tmp1;

	// clip against positive x axis
	tmp0 = boxP1.m_x - p0.m_x;
	if (tmp0 > 0.0f) {
		tmp1 = boxP1.m_x - p1.m_x;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p1.m_x = boxP1.m_x;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p1.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		}
	} else {
		tmp1 = boxP1.m_x - p1.m_x;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p0.m_x = boxP1.m_x;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p0.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		} else {
			return false;
		}
	}

	// clip against negative x axis
	tmp0 = boxP0.m_x - p0.m_x;
	if (tmp0 < 0.0f) {
		tmp1 = boxP0.m_x - p1.m_x;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p1.m_x = boxP0.m_x;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p1.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		}
	} else {
		tmp1 = boxP0.m_x - p1.m_x;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p0.m_x = boxP0.m_x;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p0.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		} else {
			return false;
		}
	}
	
	// clip against positive z axis
	tmp0 = boxP1.m_z - p0.m_z;
	if (tmp0 > 0.0f) {
		tmp1 = boxP1.m_z - p1.m_z;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p1.m_z = boxP1.m_z;
			p1.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		}
	} else {
		tmp1 = boxP1.m_z - p1.m_z;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p0.m_z = boxP1.m_z;
			p0.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		} else {
			return false;
		}
	}

	// clip against negative z axis
	tmp0 = boxP0.m_z - p0.m_z;
	if (tmp0 < 0.0f) {
		tmp1 = boxP0.m_z - p1.m_z;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p1.m_z = boxP0.m_z;
			p1.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		}
	} else {
		tmp1 = boxP0.m_z - p1.m_z;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p0.m_z = boxP0.m_z;
			p0.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		} else {
			return false;
		}
	}

	// the line or part of the line segment is contained by the cell
	return true;
}


// calculate the intersection of a ray and a triangle
dFloat HeightFieldCollision::RayCastTriangle (const dVector& p0, const dVector& dp, const dVector& origin, const dVector& e1, const dVector& e2)
{
	dFloat t;
	dFloat b0;
	dFloat b1;
	dFloat b00;
	dFloat b11;
	dFloat a00;
	dFloat a10;
	dFloat a11;
	dFloat det;
	dFloat dot;
	dFloat tol;

	// clip line again first triangle
	dVector normal (e2 * e1);

	dot = normal % dp;
	if (dot <= 1.0e-6f) {
		t = ((origin - p0) % normal) / dot;
		if (t > 0.0f) {
			if (t < 1.0f) {
				dVector q (p0 + dp.Scale (t));
				a00 = e1 % e1;
				a11 = e2 % e2;
				a10 = e1 % e2;
				det = a00 * a11 - a10 * a10;
				// det must be positive and different than zero
				//_ASSERTE (det > 0.0f);
				
				dVector q0p0 (q - origin);
				b0 = q0p0 % e1;
				b1 = q0p0 % e2;

				tol = -det * 1.0e-3f;
				b00 = b0 * a11 - b1 * a10;
				if (b00 >= tol) {
					b11 = b1 * a00 - b0 * a10;
					if (b11 >= tol) {
						if ((b00 + b11) <= (det * 1.001f)) {
							// found a hit return this value
							return t;
						}
					}
				}
			}
		}
	}

	// if it come here the there no intersection
	return 1.2f;
}

// calculate the intersection point of a line segment and the two triangles making a the cell of a heih pam terrain
dFloat HeightFieldCollision::RayCastCell (dInt32 xIndex0, dInt32 zIndex0, const dVector& p0, const dVector& dp, dVector& normalOut)
{
	dFloat t;


	// if debug mode on save the line cell in wire frame
	if (debugLinesMode) {
		if (debugRayCast  < 1000) {
			dVector p00 ((xIndex0 + 0) * CELL_SIZE, m_heightField[zIndex0 + 0][xIndex0 + 0], (zIndex0 + 0) * CELL_SIZE);
			dVector p10 ((xIndex0 + 1) * CELL_SIZE, m_heightField[zIndex0 + 0][xIndex0 + 1], (zIndex0 + 0) * CELL_SIZE);
			dVector p11 ((xIndex0 + 1) * CELL_SIZE, m_heightField[zIndex0 + 1][xIndex0 + 1], (zIndex0 + 1) * CELL_SIZE);
			dVector p01 ((xIndex0 + 0) * CELL_SIZE, m_heightField[zIndex0 + 1][xIndex0 + 0], (zIndex0 + 1) * CELL_SIZE);

			debugRayCastLines[debugRayCast][0] = p00;
			debugRayCastLines[debugRayCast][1] = p10;
			debugRayCast ++;

			debugRayCastLines[debugRayCast][0] = p10;
			debugRayCastLines[debugRayCast][1] = p11;
			debugRayCast ++;

			debugRayCastLines[debugRayCast][0] = p11;
			debugRayCastLines[debugRayCast][1] = p01;
			debugRayCast ++;

			debugRayCastLines[debugRayCast][0] = p01;
			debugRayCastLines[debugRayCast][1] = p00;
			debugRayCast ++;

			debugRayCastLines[debugRayCast][0] = p11;
			debugRayCastLines[debugRayCast][1] = p00;
			debugRayCast ++;
		}
	}

	// get the 3d point at the corner of the cell
	dVector p00 ((xIndex0 + 0) * CELL_SIZE, m_heightField[zIndex0 + 0][xIndex0 + 0], (zIndex0 + 0) * CELL_SIZE);
	dVector p10 ((xIndex0 + 1) * CELL_SIZE, m_heightField[zIndex0 + 0][xIndex0 + 1], (zIndex0 + 0) * CELL_SIZE);
	dVector p11 ((xIndex0 + 1) * CELL_SIZE, m_heightField[zIndex0 + 1][xIndex0 + 1], (zIndex0 + 1) * CELL_SIZE);

	// clip line again first triangle
	dVector e0 (p10 - p00);
	dVector e1 (p11 - p00);

	t = RayCastTriangle (p0, dp, p00, e0, e1);
	if (t < 1.0f) {
		normalOut = e1 * e0;
		return t;
	}

	// clip line against second triangle
	dVector p01 ((xIndex0 + 0) * CELL_SIZE, m_heightField[zIndex0 + 1][xIndex0 + 0], (zIndex0 + 1) * CELL_SIZE);
	dVector e2 (p01 - p00);
	t = RayCastTriangle (p0, dp, p00, e1, e2);
	if (t < 1.0f) {
		normalOut = e2 * e1;
	}
	return t;
}



// determine if a ray segment intersection the height map cell
dFloat  HeightFieldCollision::UserMeshCollisionRayHitCallback (NewtonUserMeshCollisionRayHitDesc* rayDesc)
{
	dInt32 xInc;
	dInt32 zInc;
	dInt32 xIndex0;
	dInt32 zIndex0;
	dFloat t;
	dFloat tx;
	dFloat tz;
	dFloat val;
	dFloat ix0;
	dFloat iz0;
	dFloat scale;
	dFloat invScale;
	
	dFloat stepX;
	dFloat stepZ;
	dVector normalOut;
	HeightFieldCollision *map;



	// set the debug line counter to zero
	debugRayCast = 0;


	// the user data is the pointer to the collision geometry
	map = (HeightFieldCollision*) rayDesc->m_userData;

	dVector q0 (rayDesc->m_p0[0], rayDesc->m_p0[1], rayDesc->m_p0[2]);
	dVector q1 (rayDesc->m_p1[0], rayDesc->m_p1[1], rayDesc->m_p1[2]);

//if (q1.m_y < 0.0f) return q0.m_y / (q0.m_y - q1.m_y);

	dVector boxP0;
	dVector boxP1;

	// calculate the ray bounding box
	map->CalculateMinExtend2d (q0, q1, boxP0, boxP1);

	dVector dq (q1 - q0);
	dVector padding (dq.Scale (CELL_SIZE * 10.0f / (dSqrt (dq % dq) + 1.0e-6f)));
	
	// make sure the line segment crosses the original segment box
	dVector p0 (q0 - padding);
	dVector p1 (q1 + padding);

	// clip the line against the bounding box
	if (map->ClipRay2d (p0, p1, boxP0, boxP1)) {
		dVector dp (p1 - p0);

		scale = CELL_SIZE;
		invScale = (1.0f / CELL_SIZE);
		ix0 = dFloor (p0.m_x * invScale);
		iz0 = dFloor (p0.m_z * invScale);

		// implement a 3ddda line algorithm 
		if (dp.m_x > 0.0f) {
			xInc = 1;
			val = 1.0f / dp.m_x;
			stepX = scale * val;
			tx = (scale * (ix0 + 1.0f) - p0.m_x) * val;
		} else if (dp.m_x < 0.0f) {
			xInc = -1;
			val = -1.0f / dp.m_x;
			stepX = scale * val;
			tx = -(scale * ix0 - p0.m_x) * val;
		} else {
			xInc = 0;
			stepX = 0.0f;
			tx = 1.0e10f;
		}

		if (dp.m_z > 0.0f) {
			zInc = 1;
			val = 1.0f / dp.m_z;
			stepZ = scale * val;
			tz = (scale * (iz0 + 1.0f) - p0.m_z) * val;
		} else if (dp.m_z < 0.0f) {
			zInc = -1;
			val = -1.0f / dp.m_z;
			stepZ = scale * val;
			tz = -(scale * iz0 - p0.m_z) * val;
		} else {
			zInc = 0;
			stepZ = 0.0f;
			tz = 1.0e10f;
		}


		xIndex0 = dInt32 (ix0);
		zIndex0 = dInt32 (iz0);

		// for each cell touched by the line
		do {
			t = map->RayCastCell (xIndex0, zIndex0, q0, dq, normalOut);
			if (t < 1.0f) {
				// bail out at the first intersection and copy the data into the descriptor
				normalOut = normalOut.Scale (1.0f / dSqrt (normalOut % normalOut));
				rayDesc->m_normalOut[0] = normalOut.m_x;
				rayDesc->m_normalOut[1] = normalOut.m_y;
				rayDesc->m_normalOut[2] = normalOut.m_z;
				rayDesc->m_userIdOut = (xIndex0 << 16) + zIndex0;
				return t;
			}

			if (tx < tz) {
				xIndex0 += xInc;
				tx += stepX;
			} else {
				zIndex0 += zInc;
				tz += stepZ;
			}
		} while ((tx <= 1.0f) || (tz <= 1.0f));
	}

	// if no cell was hit, return a large value
	return 1.2f;
}



void  HeightFieldCollision::MeshCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* collideDesc)
{
	dInt32 x;
	dInt32 z;
	dInt32 x0;
	dInt32 x1;
	dInt32 z0;
	dInt32 z1;
	dInt32 index;
	dInt32 faceCount;
	dInt32 vertexIndex;
	dInt32 step;
	dInt32 edgesIndex[MAX_COLLIDING_FACES];

	dVector boxP0;
	dVector boxP1;
	HeightFieldCollision *map;

	// the user data is the pointer to the collision geometry
	map = (HeightFieldCollision*) collideDesc->m_userData;

	dVector p0 (collideDesc->m_boxP0[0], collideDesc->m_boxP0[1], collideDesc->m_boxP0[2]);
	dVector p1 (collideDesc->m_boxP1[0], collideDesc->m_boxP1[1], collideDesc->m_boxP1[2]);

	map->CalculateMinExtend3d (p0, p1, boxP0, boxP1);

	x0 = (dInt32) (boxP0.m_x * (1.0f / CELL_SIZE));
	x1 = (dInt32) (boxP1.m_x * (1.0f / CELL_SIZE));

	z0 = (dInt32) (boxP0.m_z * (1.0f / CELL_SIZE));
	z1 = (dInt32) (boxP1.m_z * (1.0f / CELL_SIZE));


	// initialize the callback data structure
	collideDesc->m_vertexStrideInBytes = sizeof (dVector);
	collideDesc->m_userAttribute = map->m_attribute;
	collideDesc->m_faceIndexCount = map->m_faceIndices;
	collideDesc->m_faceVertexIndex = map->m_indexArray;
	collideDesc->m_vertex = &map->m_collisionVertex[0][0];
  

	// scan the vertices's intersected by the box extend
	vertexIndex = 0;
	for (z = z0; z <= z1; z ++) {
		for (x = x0; x <= x1; x ++) {
			edgesIndex[vertexIndex] = z * HEIGHT_SIZE + x;
			map->m_collisionVertex[vertexIndex] = dVector(CELL_SIZE * x, map->m_heightField[z][x], CELL_SIZE * z);
			
			vertexIndex ++;
			//_ASSERTE (vertexIndex < MAX_COLLIDING_FACES * 2);
		}
	}


	// build a vertex list index list mesh from the vertices's intersected by the extend
	index = 0;
	faceCount = 0;
	vertexIndex = 0;
	step = x1 - x0 + 1;

	for (z = z0; z < z1; z ++) {
		for (x = x0; x < x1; x ++) {
	

			map->m_attribute[faceCount] = (x << 16) + z;
			map->m_faceIndices[faceCount] = 3;

			map->m_indexArray[index + 0] = vertexIndex;
			map->m_indexArray[index + 1] = (vertexIndex + step + 1);
			map->m_indexArray[index + 2] = (vertexIndex + 1);


			index += 3;
			faceCount ++;
	
			map->m_attribute[faceCount] = (x << 16) + z;
			map->m_faceIndices[faceCount] = 3;

			map->m_indexArray[index + 0] = vertexIndex;
			map->m_indexArray[index + 1] = (vertexIndex + step);
			map->m_indexArray[index + 2] = (vertexIndex + step + 1);

			index += 3;
			faceCount ++;
			vertexIndex ++;

			//_ASSERTE (faceCount < MAX_COLLIDING_FACES);
			
		}
		vertexIndex ++;
	}

	collideDesc->m_faceCount = faceCount;

	
	if (debugLinesMode) {
//		uncomment this to see only a specific collision ID
//		dInt32 id = (dInt32)NewtonConvexCollisionGetUserID (NewtonBodyGetCollision (collideDesc->m_objBody));
//		if (id  == 0x100) {
			if (hightMap_debugCount < 1024 * 4 - 256) {
				index = 0;
				for (dInt32 i = 0; i < faceCount; i ++) {
					dInt32 j0;
					dInt32 j1;
					j0 = map->m_indexArray[index + map->m_faceIndices[i] - 1];
					for (dInt32 j = 0; j <  map->m_faceIndices[i]; j ++) {
						j1 = map->m_indexArray[index];
						hightMap_debugLines[hightMap_debugCount][0] = dVector (map->m_collisionVertex[j0]);
						hightMap_debugLines[hightMap_debugCount][1] = dVector (map->m_collisionVertex[j1]);
						hightMap_debugCount ++;
						j0 = j1;
						index ++;
					}
				}
			}
//		}
	}
}
