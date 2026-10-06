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

#ifndef ND_CONTACT_CALLBACK_H_
#define ND_CONTACT_CALLBACK_H_
		  
class ndApplicationMaterial : public ndMaterial
{
	public:
	ndApplicationMaterial();
	ndApplicationMaterial(const ndApplicationMaterial& src);
	virtual ~ndApplicationMaterial();

	virtual ndApplicationMaterial* Clone() const
	{
		return new ndApplicationMaterial(*this);
	}

	virtual void OnContactCallback(const ndContact* const joint, ndFloat32 timestep) const;
	virtual bool OnAabbOverlap(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const;
	virtual bool OnAabbOverlap(const ndContact* const joint, ndFloat32 timestep, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const;
};

class ndMaterialHash
{
	public:
	ndMaterialHash()
		:m_key(0)
	{
	}

	ndMaterialHash(ndUnsigned32 low, ndUnsigned32 high)
		:m_lowKey(ndUnsigned32(ndMin(low, high)))
		,m_highKey(ndUnsigned32(ndMax(low, high)))
	{
	}

	bool operator<(const ndMaterialHash& other) const
	{
		return (m_key < other.m_key);
	}

	bool operator>(const ndMaterialHash& other) const
	{
		return (m_key > other.m_key);
	}

	union
	{
		ndUnsigned64 m_key;
		class
		{
			public:
			ndUnsigned32 m_lowKey;
			ndUnsigned32 m_highKey;
		};
	};
};

class ndMaterialGraph: public ndTree<ndApplicationMaterial*, ndMaterialHash, ndContainersFreeListAlloc<ndMaterialGraph*>>
{
	public:
	ndMaterialGraph();
	~ndMaterialGraph();
	ndNode* GetNode(ndUnsigned32 id0, ndUnsigned32 id1) const;
};

class ndContactCallback: public ndContactNotify
{
	public: 
	ndContactCallback();
	virtual ~ndContactCallback() override;
	virtual ndApplicationMaterial& RegisterMaterial(const ndApplicationMaterial& material, ndUnsigned32 id0, ndUnsigned32 id1);

	bool HasMaterial(ndUnsigned32 id0, ndUnsigned32 id1) const;
	virtual ndMaterial* GetMaterial(ndUnsigned32 id0, ndUnsigned32 id1) const;
	virtual ndMaterial* GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const override;

	private:
	virtual bool OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep) const override;
	virtual void OnContactCallback(const ndContact* const contactJoint, ndFloat32 timestep) const override;
	virtual bool OnAabbOverlap(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) const override;
	virtual bool OnCompoundSubShapeOverlap(const ndContact* const contactJoint, ndFloat32 timestep, const ndShapeInstance* const instance0, const ndShapeInstance* const instance1) const override;
	
	ndMaterialGraph m_materialGraph;
	ndApplicationMaterial m_defaultMaterial;
};

#endif
