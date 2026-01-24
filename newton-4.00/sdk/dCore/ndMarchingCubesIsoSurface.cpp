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

#include "ndCoreStdafx.h"
#include "ndSort.h"
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndProfiler.h"
#include "ndMarchingCubesIsoSurface.h"

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/

namespace Original
{
	struct XYZ
	{
		ndFloat32 x;
		ndFloat32 y;
		ndFloat32 z;
	};

	struct TRIANGLE
	{
		XYZ p[3];
	};

	struct GRIDCELL
	{
		XYZ p[8];
		ndFloat32 val[8];
	} ;
	#define ABS ndAbs

	/*
	   Linearly interpolate the position where an isosurface cuts
	   an edge between two vertices, each with their own scalar value
	*/

	//class mpVector : public ndVector
	//{
	//	bool operator<(const ndVector& right) const
	//	{
	//		if (x < right.x)
	//			return true;
	//		else if (x > right.x)
	//			return false;
	//
	//		if (y < right.y)
	//			return true;
	//		else if (y > right.y)
	//			return false;
	//
	//		if (z < right.z)
	//			return true;
	//		else if (z > right.z)
	//			return false;
	//
	//		return false;
	//	}
	//};
	//
	//ndVector LinearInterp(ndVector p1, ndVector p2, float value)
	//{
	//	if (p2 < p1)
	//	{
	//		ndVector temp;
	//		temp = p1;
	//		p1 = p2;
	//		p2 = temp;
	//	}
	//
	//	mpVector p;
	//	if (fabs(p1.val - p2.val) > 0.00001)
	//		p = (mpVector)p1 + ((mpVector)p2 - (mpVector)p1) / (p2.val - p1.val) * (value - p1.val);
	//	else
	//		p = (mpVector)p1;
	//	return p;
	//}

	XYZ VertexInterp(ndFloat32 isolevel, XYZ p1, XYZ p2, ndFloat32 valp1, ndFloat32 valp2)
	{
		ndFloat32 mu;
		XYZ p;

		if (ABS(isolevel - valp1) < 0.00001)
			return(p1);
		if (ABS(isolevel - valp2) < 0.00001)
			return(p2);
		if (ABS(valp1 - valp2) < 0.00001)
			return(p1);
		mu = (isolevel - valp1) / (valp2 - valp1);
		p.x = p1.x + mu * (p2.x - p1.x);
		p.y = p1.y + mu * (p2.y - p1.y);
		p.z = p1.z + mu * (p2.z - p1.z);

		return(p);
	}


	/*
	   Given a grid cell and an isolevel, calculate the triangular
	   facets required to represent the isosurface through the cell.
	   Return the number of triangular facets, the array "triangles"
	   will be loaded up with the vertices at most 5 triangular facets.
		0 will be returned if the grid cell is either totally above
	   of totally below the isolevel.
	*/
	int Polygonise(GRIDCELL grid, ndFloat32 isolevel, TRIANGLE* triangles)
	{
		int i, ntriang;
		int cubeindex;
		XYZ vertlist[12];

		static int edgeTable[256] = 
		{
			0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
			0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
			0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
			0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
			0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
			0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
			0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
			0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
			0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
			0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
			0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
			0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
			0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
			0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
			0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
			0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
			0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
			0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
			0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
			0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
			0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
			0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
			0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
			0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
			0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
			0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
			0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
			0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
			0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
			0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
			0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
			0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0 
		};

		static int triTable[256][16] =
		{ 
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
			{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
			{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
			{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
			{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
			{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
			{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
			{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
			{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
			{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
			{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
			{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
			{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
			{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
			{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
			{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
			{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
			{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
			{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
			{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
			{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
			{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
			{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
			{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
			{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
			{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
			{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
			{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
			{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
			{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
			{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
			{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
			{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
			{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
			{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
			{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
			{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
			{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
			{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
			{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
			{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
			{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
			{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
			{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
			{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
			{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
			{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
			{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
			{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
			{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
			{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
			{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
			{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
			{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
			{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
			{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
			{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
			{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
			{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
			{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
			{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
			{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
			{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
			{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
			{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
			{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
			{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
			{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
			{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
			{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
			{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
			{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
			{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
			{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
			{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
			{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
			{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
			{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
			{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
			{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
			{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
			{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
			{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
			{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
			{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
			{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
			{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
			{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
			{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
			{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
			{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
			{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
			{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
			{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
			{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
			{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
			{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
			{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
			{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
			{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
			{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
			{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
			{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
			{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
			{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
			{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
			{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
			{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
			{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
			{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
			{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
			{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
			{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
			{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
			{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
			{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
			{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
			{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
			{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
			{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
			{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
			{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
			{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
			{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
			{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
			{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
			{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
			{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
			{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
			{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
			{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
			{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
			{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
			{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
			{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
			{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
			{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
			{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
			{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
			{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
			{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
			{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
			{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
			{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
			{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
			{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
			{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
			{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
			{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
			{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
			{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
			{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
			{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
			{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
			{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
			{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
			{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
			{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
			{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
			{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
			{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
			{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
			{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
			{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
			{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
			{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
			{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
			{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
			{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1} 
		};

		/*
		   Determine the index into the edge table which
		   tells us which vertices are inside of the surface
		*/
		cubeindex = 0;
		if (grid.val[0] < isolevel) cubeindex |= 1;
		if (grid.val[1] < isolevel) cubeindex |= 2;
		if (grid.val[2] < isolevel) cubeindex |= 4;
		if (grid.val[3] < isolevel) cubeindex |= 8;
		if (grid.val[4] < isolevel) cubeindex |= 16;
		if (grid.val[5] < isolevel) cubeindex |= 32;
		if (grid.val[6] < isolevel) cubeindex |= 64;
		if (grid.val[7] < isolevel) cubeindex |= 128;

		/* Cube is entirely in/out of the surface */
		if (edgeTable[cubeindex] == 0)
			return(0);

		/* Find the vertices where the surface intersects the cube */
		if (edgeTable[cubeindex] & 1)
			vertlist[0] = VertexInterp(isolevel, grid.p[0], grid.p[1], grid.val[0], grid.val[1]);
		if (edgeTable[cubeindex] & 2)
			vertlist[1] = VertexInterp(isolevel, grid.p[1], grid.p[2], grid.val[1], grid.val[2]);
		if (edgeTable[cubeindex] & 4)
			vertlist[2] = VertexInterp(isolevel, grid.p[2], grid.p[3], grid.val[2], grid.val[3]);
		if (edgeTable[cubeindex] & 8)
			vertlist[3] = VertexInterp(isolevel, grid.p[3], grid.p[0], grid.val[3], grid.val[0]);
		if (edgeTable[cubeindex] & 16)
			vertlist[4] = VertexInterp(isolevel, grid.p[4], grid.p[5], grid.val[4], grid.val[5]);
		if (edgeTable[cubeindex] & 32)
			vertlist[5] = VertexInterp(isolevel, grid.p[5], grid.p[6], grid.val[5], grid.val[6]);
		if (edgeTable[cubeindex] & 64)
			vertlist[6] = VertexInterp(isolevel, grid.p[6], grid.p[7], grid.val[6], grid.val[7]);
		if (edgeTable[cubeindex] & 128)
			vertlist[7] = VertexInterp(isolevel, grid.p[7], grid.p[4], grid.val[7], grid.val[4]);
		if (edgeTable[cubeindex] & 256)
			vertlist[8] = VertexInterp(isolevel, grid.p[0], grid.p[4], grid.val[0], grid.val[4]);
		if (edgeTable[cubeindex] & 512)
			vertlist[9] = VertexInterp(isolevel, grid.p[1], grid.p[5], grid.val[1], grid.val[5]);
		if (edgeTable[cubeindex] & 1024)
			vertlist[10] = VertexInterp(isolevel, grid.p[2], grid.p[6], grid.val[2], grid.val[6]);
		if (edgeTable[cubeindex] & 2048)
			vertlist[11] = VertexInterp(isolevel, grid.p[3], grid.p[7], grid.val[3], grid.val[7]);

		/* Create the triangle */
		ntriang = 0;
		for (i = 0; triTable[cubeindex][i] != -1; i += 3) 
		{
			triangles[ntriang].p[0] = vertlist[triTable[cubeindex][i]];
			triangles[ntriang].p[2] = vertlist[triTable[cubeindex][i + 1]];
			triangles[ntriang].p[1] = vertlist[triTable[cubeindex][i + 2]];
			ntriang++;
		}

		return(ntriang);
	}
}



class ndMarchingCubeIsoSurface::ndIsoCell
{
	public:
	ndVector m_isoValues[8];
};


ndMarchingCubeIsoSurface::ndMarchingCubeIsoSurface(const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize)
	:ndMarchingCubes(gridSize)
	,m_boxP0(boxP0 & ndVector::m_triplexMask)
	,m_boxP1(boxP1 & ndVector::m_triplexMask)
	,m_densityWindow0()
	,m_densityWindow1()
{
}

ndMarchingCubeIsoSurface::~ndMarchingCubeIsoSurface()
{
}

void ndMarchingCubeIsoSurface::GenerateIndexList()
{
	//#define D_LOW_RES_BITS	   1
	//#define D_LOW_RES_FRACTION (1 << D_LOW_RES_BITS)

	//class ndKey_lowX
	//{
	//	public:
	//	ndKey_lowX(void* const) {}
	//	ndInt32 GetKey(const ndVector& point) const
	//	{
	//		ndFloat32 val = (point.m_x * D_LOW_RES_FRACTION);
	//		ndInt32 key = ndInt32(val);
	//		return key & 0xff;
	//	}
	//};
	//
	//class ndKey_highX
	//{
	//public:
	//	ndKey_highX(void* const) {}
	//	ndInt32 GetKey(const ndVector& point) const
	//	{
	//		ndFloat32 val = point.m_x * ndFloat32(D_LOW_RES_FRACTION);
	//		ndInt32 key = ndInt32(val) >> 8;
	//		return key & 0xff;
	//	}
	//};
	//
	//class ndKey_lowY
	//{
	//public:
	//	ndKey_lowY(void* const) {}
	//	ndInt32 GetKey(const ndVector& point) const
	//	{
	//		ndFloat32 val = (point.m_y * D_LOW_RES_FRACTION);
	//		ndInt32 key = ndInt32(val);
	//		return key & 0xff;
	//	}
	//};
	//
	//class ndKey_highY
	//{
	//public:
	//	ndKey_highY(void* const) {}
	//	ndInt32 GetKey(const ndVector& point) const
	//	{
	//		ndFloat32 val = point.m_y * ndFloat32(D_LOW_RES_FRACTION);
	//		ndInt32 key = ndInt32(val) >> 8;
	//		return key & 0xff;
	//	}
	//};
	//
	//class ndKey_lowZ
	//{
	//public:
	//	ndKey_lowZ(void* const) {}
	//	ndInt32 GetKey(const ndVector& point) const
	//	{
	//		ndFloat32 val = (point.m_z * D_LOW_RES_FRACTION);
	//		ndInt32 key = ndInt32(val);
	//		return key & 0xff;
	//	}
	//};
	//
	//class ndKey_highZ
	//{
	//public:
	//	ndKey_highZ(void* const) {}
	//	ndInt32 GetKey(const ndVector& point) const
	//	{
	//		ndFloat32 val = point.m_z * ndFloat32(D_LOW_RES_FRACTION);
	//		ndInt32 key = ndInt32(val) >> 8;
	//		return key & 0xff;
	//	}
	//};
	//
	//ndCountingSort<ndVector, ndKey_lowX, 8>(*m_threadPool, m_meshPoints, m_meshNormals, nullptr, nullptr);
	//if (D_LOW_RES_FRACTION * m_volumeInGrids.m_ix >= 256)
	//{
	//	ndCountingSort<ndVector, ndKey_highX, 8>(*m_threadPool, m_meshPoints, m_meshNormals, nullptr, nullptr);
	//}
	//
	//ndCountingSort<ndVector, ndKey_lowY, 8>(*m_threadPool, m_meshPoints, m_meshNormals, nullptr, nullptr);
	//if (D_LOW_RES_FRACTION * m_volumeInGrids.m_iy >= 256)
	//{
	//	ndCountingSort<ndVector, ndKey_highY, 8>(*m_threadPool, m_meshPoints, m_meshNormals, nullptr, nullptr);
	//}
	//
	//ndCountingSort<ndVector, ndKey_lowZ, 8>(*m_threadPool, m_meshPoints, m_meshNormals, nullptr, nullptr);
	//if (D_LOW_RES_FRACTION * m_volumeInGrids.m_iz >= 256)
	//{
	//	ndCountingSort<ndVector, ndKey_highZ, 8>(*m_threadPool, m_meshPoints, m_meshNormals, nullptr, nullptr);
	//}
	//m_meshIndices.SetCount(m_meshPoints.GetCount());

	//ndInt32 vertexCount = 0;
	//m_meshPoints.PushBack(m_boxP1);
	//for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()) - 1; ++i)
	//{
	//	const ndInt32 index = ndInt32(m_meshPoints[i].m_w);
	//	m_meshIndices[index] = vertexCount;
	//	m_meshPoints[vertexCount] = m_meshPoints[i] & ndVector::m_triplexMask;
	//
	//	const ndGridHash hash(m_meshPoints[i] * ndVector::m_two, 0);
	//	for (i = i + 1; i < ndInt32(m_meshPoints.GetCount()); ++i)
	//	{
	//		const ndGridHash hash1(m_meshPoints[i] * ndVector::m_two, 0);
	//		if (hash.m_gridFullHash != hash1.m_gridFullHash)
	//		{
	//			break;
	//		}
	//		const ndInt32 duplicateIndex = ndInt32(m_meshPoints[i].m_w);
	//		m_meshIndices[duplicateIndex] = vertexCount;
	//	}
	//	--i;
	//	vertexCount++;
	//}
	//m_meshPoints.SetCount(vertexCount);
	//m_meshNormals.SetCount(vertexCount);

	m_meshNormals.SetCount(m_meshPoints.GetCount());
	m_meshIndices.SetCount(m_meshPoints.GetCount());
	// transform to world space
	//auto ApplyScale = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	auto ApplyScale = [this](ndInt32 groupId)
	{
		m_meshNormals[groupId] = ndVector::m_zero;
		m_meshPoints[groupId] = m_meshPoints[groupId] * m_gridSize + m_boxP0;
		m_meshIndices[groupId] = groupId;
	};
	for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()); ++i)
	{
		ApplyScale(i);
	}

	auto CalculateVertexNormals = [this](ndInt32 groupId)
	{
		const ndInt32 index = groupId * 3;
		const ndInt32 id0 = m_meshIndices[index + 0];
		const ndInt32 id1 = m_meshIndices[index + 1];
		const ndInt32 id2 = m_meshIndices[index + 2];

		const ndVector p0(m_meshPoints[id0]);
		const ndVector p1(m_meshPoints[id1]);
		const ndVector p2(m_meshPoints[id2]);

		const ndVector e_10(p1 - p0);
		const ndVector e_20(p2 - p0);
		const ndVector normal(e_10.CrossProduct(e_20) & ndVector::m_triplexMask);
		m_meshNormals[id0] += normal;
		m_meshNormals[id1] += normal;
		m_meshNormals[id2] += normal;
	};

	const ndInt32 triangleCount = ndInt32(m_meshIndices.GetCount()) / 3;
	// need to build an edge list to parallelize this,
	// for now just add the contributions 
	//m_threadPool->ParallelExecute(CalculateVertexNormals, triangleCount, jobStride);
	for (ndInt32 i = 0; i < triangleCount; ++i)
	{
		CalculateVertexNormals(i);
	}

	// Normalize normals.
	for (ndInt32 i = 0; i < ndInt32(m_meshPoints.GetCount()); ++i)
	{
		m_meshNormals[i] = m_meshNormals[i].Normalize();
	}
}

void ndMarchingCubeIsoSurface::GenerateMesh()
{
	const ndVector boxSize(m_boxP1 - m_boxP0);
	const ndVector boxSizeInGrids((boxSize * m_invGridSize).Ceiling().GetInt());

	ndFloat32 high = m_boxP0.m_y;
	auto ReadLayerDensity = [this, &boxSizeInGrids, &high](ndArray<ndReal>& layer)
	{
		ndInt32 row = 0;
		ndFloat32 posit_z = m_boxP0.m_z;
		const ndInt32 stride = boxSizeInGrids.m_ix;
		for (ndInt32 z = 0; z < boxSizeInGrids.m_iz; ++z)
		{
			ndFloat32 posit_x = m_boxP0.m_x;
			for (ndInt32 x = 0; x < boxSizeInGrids.m_ix; ++x)
			{
				const ndVector posit(posit_x, high, posit_z, ndFloat32(0.0f));
				layer[row + x] = GetIsoValue(posit);
				posit_x += m_gridSize.m_x;
			}
			posit_z += m_gridSize.m_z;
			row += stride;
		}
	};

	m_densityWindow0.SetCount(boxSizeInGrids.m_ix * boxSizeInGrids.m_iz);
	m_densityWindow1.SetCount(boxSizeInGrids.m_ix * boxSizeInGrids.m_iz);

	ndFloat32 grid_y0 = ndFloat32(0.0f);
	ndFloat32 grid_y1 = ndFloat32(1.0f);
	ReadLayerDensity(m_densityWindow0);

	static int xxxx;
	for (ndInt32 y = 1; y < (boxSizeInGrids.m_iy - 1); ++y)
	{
		high += m_gridSize.m_y;
		ReadLayerDensity(m_densityWindow1);

		ndInt32 stride = 0;
		ndFloat32 grid_Z0 = ndFloat32(0.0f);
		ndFloat32 grid_Z1 = ndFloat32(1.0f);
		for (ndInt32 z = 0; z < boxSizeInGrids.m_iz - 1; ++z)
		{
			for (ndInt32 x = 0; x < boxSizeInGrids.m_ix - 1; ++x)
			{
				ndIsoCell cell;
				const ndInt32 i0 = stride + x;
				const ndInt32 i1 = stride + x + 1;
				const ndInt32 i2 = boxSizeInGrids.m_ix + stride + x + 1;
				const ndInt32 i3 = boxSizeInGrids.m_ix + stride + x;

				ndFloat32 isoValues[8];
				isoValues[0] = m_densityWindow0[i0];
				isoValues[1] = m_densityWindow0[i1];
				isoValues[2] = m_densityWindow0[i2];
				isoValues[3] = m_densityWindow0[i3];
				isoValues[4] = m_densityWindow1[i0];
				isoValues[5] = m_densityWindow1[i1];
				isoValues[6] = m_densityWindow1[i2];
				isoValues[7] = m_densityWindow1[i3];

				ndInt32 tableIndex = 0;
				for (ndInt32 i = 0; i < 8; ++i)
				{
					//tableIndex |= (cell.m_isoValues[i].m_w <= 0.0f) << i;
					tableIndex |= (isoValues[i] <= -0.0f) << i;
				}

				const ndInt32 edgeStart = m_edgeScan[tableIndex];
				const ndInt32 edgeCount = m_edgeScan[tableIndex + 1] - edgeStart;
				if (edgeCount)
				{
					//if ((xxxx == 0) ||
					//	(xxxx == 1) ||
					//	(xxxx == 2) ||
					//	(xxxx == 3))
					{
						cell.m_isoValues[0] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z0, isoValues[0]);
						cell.m_isoValues[1] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z0, isoValues[1]);
						cell.m_isoValues[2] = ndVector(ndFloat32(x + 1), grid_y0, grid_Z1, isoValues[2]);
						cell.m_isoValues[3] = ndVector(ndFloat32(x + 0), grid_y0, grid_Z1, isoValues[3]);
						cell.m_isoValues[4] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z0, isoValues[4]);
						cell.m_isoValues[5] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z0, isoValues[5]);
						cell.m_isoValues[6] = ndVector(ndFloat32(x + 1), grid_y1, grid_Z1, isoValues[6]);
						cell.m_isoValues[7] = ndVector(ndFloat32(x + 0), grid_y1, grid_Z1, isoValues[7]);

						ndVector vertlist[12];
						for (ndInt32 i = 0; i < edgeCount; ++i)
						{
							const ndEdge& edge = m_edges[edgeStart + i];
							const ndInt32 midPoint = edge.m_midPoint;
							
							const ndVector p0(cell.m_isoValues[edge.m_p0]);
							const ndVector p1(cell.m_isoValues[edge.m_p1]);
							ndAssert((p1.m_w * p0.m_w) <= ndFloat32(0.0f));
							const ndVector p1p0(p1 - p0);
							ndFloat32 param = p0.m_w / (p1.m_w - p0.m_w);
							//param = 0.5f;
							const ndVector p3 (ndVector::m_triplexMask & (p0 - p1p0.Scale(param)));
							vertlist[midPoint] = p3;
						}

						const ndInt32 faceStart = m_facesScan[tableIndex];
						const ndInt32 triangleStart = m_facesScan[tableIndex];
						const ndInt32 triangleCount = m_facesScan[tableIndex + 1] - triangleStart;
						for (ndInt32 i = 0; i < triangleCount; ++i)
						{
							const ndInt32 j0 = m_faces[faceStart + i][0];
							const ndInt32 j1 = m_faces[faceStart + i][1];
							const ndInt32 j2 = m_faces[faceStart + i][2];

							m_meshPoints.PushBack(vertlist[j0]);
							m_meshPoints.PushBack(vertlist[j1]);
							m_meshPoints.PushBack(vertlist[j2]);
						}
					}
					xxxx++;
				}
			}
			stride += boxSizeInGrids.m_ix;
			grid_Z0 += ndFloat32(1.0f);
			grid_Z1 += ndFloat32(1.0f);
		}
		grid_y0 += ndFloat32(1.0f);
		grid_y1 += ndFloat32(1.0f);
		m_densityWindow0.Swap(m_densityWindow1);
	}
	high += m_gridSize.m_y;

	GenerateIndexList();
}
