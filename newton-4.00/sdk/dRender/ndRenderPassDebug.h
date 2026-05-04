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
#ifndef __ND_RENDER_PASS_DEBUG_H__
#define __ND_RENDER_PASS_DEBUG_H__

#include "ndRenderPass.h"

class ndRenderPrimitive;

class ndRenderPassDebug : public ndRenderPass
{
	public:
	class ndPointColor
	{
		public:
		ndVector m_point;
		ndVector m_color;
	};

	class ndPointNormalColor
	{
		public:
		ndVector m_point;
		ndVector m_normal;
		ndVector m_color;
	};

	class ndRuntimeLine
	{
		public:
		ndVector m_p0;
		ndVector m_p1;
		ndVector m_color;
	};

	class ndDebugOptions
	{
		public:
		ndDebugOptions()
		{
			memset(this, 0, sizeof(ndDebugOptions));
		}

		bool m_showContacts;
		bool m_showBodyAABB;
		bool m_showBodyFrame;
		bool m_showBroadPhase;
		bool m_showCentreOfMass;
		bool m_showContactsForce;
		bool m_showJointDebugInfo;
		bool m_showModelsDebugInfo;
		bool m_showStaticMeshCollidingFaces;
	};

	ndRenderPassDebug(ndRender* const owner, ndWorld* const world);
	~ndRenderPassDebug();

	const ndArray<ndPointColor>& GetLines() const;
	const ndArray<ndPointColor>& GetPoints() const;
	const ndArray<ndPointNormalColor>& GetTriangles() const;

	const ndDebugOptions& GetDebugDisplayOptions() const;
	void SetDebugDisplayOptions(const ndDebugOptions& options);

	void ClearRuntimeLines();
	void SwapRuntimeLinesBuffers();
	void AddRuntimeLine(const ndVector& p0, const ndVector& p1, const ndVector& color);

	protected:
	class ndCallback;
	void GenerateBodyAABB();
	void GenerateContacts();
	void GenerateBroadphase();
	void GenerateBodyFrames();
	void GenerateJointsDebug();
	void GenerateModelsDebug();
	void GenerateCenterOfMass();
	void GenerateContactForce();
	virtual void RenderScene() override;
	
	ndDebugOptions m_options;
	ndArray<ndPointColor> m_debugLines;
	ndArray<ndPointColor> m_debugPoints;
	ndArray<ndPointNormalColor> m_debugTriangles;
	ndSharedPtr<ndRenderPrimitive> m_renderLinesPrimitive;
	ndSharedPtr<ndRenderPrimitive> m_renderPointsPrimitive;
	ndSharedPtr<ndRenderPrimitive> m_renderTrianglePrimitive;

	ndArray<ndRuntimeLine> m_runtimeLines;
	ndArray<ndRuntimeLine> m_runtimeRenderLines;
	ndSpinLock m_runtimeLineLock;
	ndSpinLock m_runtimeAddLineLock;

	ndWorld* m_world;
};

#endif