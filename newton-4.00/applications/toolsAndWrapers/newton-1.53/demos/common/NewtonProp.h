// NewtonProp.h: interface for the NewtonProp class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NEWTONPROP_H__891E229C_C442_46B5_8197_6D7C84717C8F__INCLUDED_)
#define AFX_NEWTONPROP_H__891E229C_C442_46B5_8197_6D7C84717C8F__INCLUDED_


#include <stdafx.h>
#include "OpenGlUtil.h"
#include "dSceneNode.h"


class NewtonProp : public dSceneNode  
{
	public:
	NewtonProp (dSceneNode* parent, const char* fileName, NewtonWorld* nWorld, const dMatrix& matrix);
	virtual ~NewtonProp();

	private:
	static void Destroy (const NewtonBody* body);
	static void ApplyGravityForce (const NewtonBody* body);
	static void SetTransform (const NewtonBody* body, const dFloat* matrix);
};

#endif 
