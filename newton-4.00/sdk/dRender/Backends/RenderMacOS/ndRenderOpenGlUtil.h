/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
	
#ifndef __ND_RENDER_OPENGLUTIL_H__
#define __ND_RENDER_OPENGLUTIL_H__

#include "ndRenderContext.h"

//#define OFFSETOF(s,m) ((size_t)&(((s*)0)->m))
//
//#ifdef D_NEWTON_USE_DOUBLE
//	inline void glMaterialParam(GLenum face, GLenum pname, const ndFloat32 *params)
//	{
//		ndReal tmp[4] = { ndReal(params[0]), ndReal(params[1]), ndReal(params[2]), ndReal(params[3]) };
//		glMaterialfv(face, pname, &tmp[0]);
//	}
//	#define glMultMatrix(x) glMultMatrixd(x)
//	#define glLoadMatrix(x) glMultMatrixd(x)
//	#define glGetFloat(x,y) glGetDoublev(x,(GLdouble *)y) 
//#else 
//	#define glMaterialParam glMaterialfv
//	#define glMultMatrix(x) glMultMatrixf(x)
//	#define glLoadMatrix(x) glMultMatrixf(x)
//	#define glGetFloat(x,y) glGetFloatv(x, (ndReal*)y) 
//#endif

class glUV
{
	public:
	ndReal m_u;
	ndReal m_v;
};

class glVector3
{
	public:
	glVector3()
	{
		m_data[0] = ndReal(0.0f);
		m_data[1] = ndReal(0.0f);
		m_data[2] = ndReal(0.0f);
	}

	glVector3(ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		m_data[0] = ndReal(x);
		m_data[1] = ndReal(y);
		m_data[2] = ndReal(z);
	}

	glVector3(const ndVector& v)
	{
		m_data[0] = ndReal(v[0]);
		m_data[1] = ndReal(v[1]);
		m_data[2] = ndReal(v[2]);
	}

	ndReal& operator[] (ndInt32 i)
	{
		ndAssert(i >= 0);
		ndAssert(i < ndInt32(sizeof(m_data) / sizeof(m_data[0])));
		return m_data[i];
	}

	const ndReal& operator[] (ndInt32 i) const
	{
		ndAssert(i >= 0);
		ndAssert(i < ndInt32(sizeof(m_data) / sizeof(m_data[0])));
		return m_data[i];
	}
	union
	{
		struct
		{
			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
		};
		ndReal m_data[3];
	};
};

class glVector4
{
	public:
	glVector4()
	{
		m_data[0] = ndReal(0.0f);
		m_data[1] = ndReal(0.0f);
		m_data[2] = ndReal(0.0f);
		m_data[3] = ndReal(0.0f);
	}

	glVector4(ndFloat32 x, ndFloat32 y, ndFloat32 z, ndFloat32 w)
	{
		m_data[0] = ndReal(x);
		m_data[1] = ndReal(y);
		m_data[2] = ndReal(z);
		m_data[3] = ndReal(w);
	}

	glVector4(const ndVector& v)
	{
		m_data[0] = ndReal(v[0]);
		m_data[1] = ndReal(v[1]);
		m_data[2] = ndReal(v[2]);
		m_data[3] = ndReal(v[3]);
	}

	ndReal& operator[] (ndInt32 i)
	{
		ndAssert(i >= 0);
		ndAssert(i < ndInt32(sizeof(m_data) / sizeof(m_data[0])));
		return m_data[i];
	}

	const ndReal& operator[] (ndInt32 i) const
	{
		ndAssert(i >= 0);
		ndAssert(i < ndInt32(sizeof(m_data) / sizeof(m_data[0])));
		return m_data[i];
	}

	union
	{
		struct
		{
			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
			ndReal m_w;
		};
		ndReal m_data[4];
	};
};

class glMatrix
{
	public:
	glMatrix()
	{
	}

	glMatrix(const ndMatrix& matrix)
	{
		for (ndInt32 i = 0; i < 4; ++i)
		{
			m_data[i] = matrix[i];
		}
	}

	glVector4& operator[] (ndInt32 i)
	{
		ndAssert(i >= 0);
		ndAssert(i < ndInt32(sizeof(m_data) / sizeof(m_data[0])));
		return m_data[i];
	}

	const glVector4& operator[] (ndInt32 i) const
	{
		ndAssert(i >= 0);
		ndAssert(i < ndInt32(sizeof(m_data) / sizeof(m_data[0])));
		return m_data[i];
	}

	glVector4 m_data[4];
};

class glPointColor
{
	public:
	glVector3 m_point;
	glVector3 m_color;
};

class glPositionNormal
{
	public:
	glVector3 m_posit;
	glVector3 m_normal;
};

class glPositionNormalColor
{
	public:
	glVector3 m_posit;
	glVector3 m_normal;
	glVector3 m_color;
};

class glPositionUV
{
	public:
	glVector3 m_posit;
	glUV m_uv;
};

class glPositionNormalUV : public glPositionNormal
{
	public:
	glUV m_uv;
};

class glSkinVertex : public glPositionNormalUV
{
	public:
	glVector4 m_weighs;
	union 
	{
		GLuint m_boneIndexInt[4];
		ndReal m_boneIndexFloat[4];
	};
};

#endif 

