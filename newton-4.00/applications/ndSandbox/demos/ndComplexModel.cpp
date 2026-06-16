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

#include "ndSandboxStdafx.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"

#define ND_EXCAVATOR_ENGINE_OMEGA		ndFloat32 (100.0f)
#define ND_EXCAVATOR_CAMERA_DISTANCE	ndFloat32 (-15.0f)

namespace ndExcavator
{
	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "implements a medium complexity articulated model");
			scene->Print(color, "'c' change player camera");
			scene->Print(color, "'a' turn left");
			scene->Print(color, "'d' turn right");
			scene->Print(color, "'w' moves forward");
			scene->Print(color, "'s' moves backward");
			scene->Print(color, "'right mouse click' move arm");
			scene->Print(color, "'middle mouse click' rotate bucket");
			scene->Print(color, "'left and right click' rotate cabin");
		}
	};

	enum ndBodyPartType
	{
		m_chassis = ndDemoContactCallback::m_default + 1,
		m_thread,
		m_roller,
	};
	
	class ExcavatorController: public ndModelNotify
	{
		public:
		ExcavatorController(
			ndDemoEntityManager* const scene,
			ndModelArticulation* const model, 
			ndSharedPtr<ndRenderSceneNode>& visualMesh)
			:ndModelNotify()
			,m_scene(scene)
			,m_mouseX(ndFloat32(0.0f))
			,m_mouseY(ndFloat32(0.0f))
			,m_armAngle(ndFloat32(0.0f))
			,m_armAngle0(ndFloat32(0.0f))
			,m_armPosit_x(ndFloat32(0.0f))
			,m_armPosit_y(ndFloat32(0.0f))
			,m_engineOmega(ndFloat32(0.0f))
			,m_engineTurnRateOmega(ndFloat32(0.0f))
			,m_backTrackingArm(8)
		{
			SetModel(model);

			// set the egine node
			m_engineNode = model->FindByName("engine");

			// find the bucket joint
			m_bucketJoint = (ndJointHinge*)*model->FindByName("bucket")->m_joint;

			// find the arm effector
			m_armEffector = (ndIkSwivelPositionEffector*)*model->FindByName("base_arm02")->m_joint;
			ndVector localPosit(m_armEffector->GetLocalTargetPosition());
			m_armAngle = ndFloat32(0.0f);
			m_armPosit_y = localPosit.m_y;
			m_armAngle0 = ndAtan2(localPosit.m_z, localPosit.m_x);
			m_armPosit_x = ndSqrt(localPosit.m_x * localPosit.m_x + localPosit.m_z * localPosit.m_z);

			// set the Camera node
			ndAssert(visualMesh->FindByName("cameraPivot"));
			ndSharedPtr<ndRenderSceneNode> cameraPivotNode(visualMesh->FindByName("cameraPivot")->GetSharedPtr());

			ndVector cameraPivot(ndVector::m_zero);
			cameraPivot.m_y = ndFloat32(2.5f);
			ndRender* const renderer = *m_scene->GetRenderer();
			m_cameraNode = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, ND_EXCAVATOR_CAMERA_DISTANCE));
			cameraPivotNode->AddChild(m_cameraNode);
		}

		bool OnContactGeneration(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1)
		{
			const ndModelArticulation* const articulation = GetModel()->GetAsModelArticulation();
			return articulation->PairCollide(body0, body1);
		}

		void UpdateEngine(ndFloat32 timestep)
		{
			// reset the motor matrix to align with the chassis matrix
			ndJointDoubleHinge* const engine = (ndJointDoubleHinge*)*m_engineNode->m_joint;
			const ndMatrix matrix(engine->GetLocalMatrix0().OrthoInverse() * engine->GetLocalMatrix1() * engine->GetBody1()->GetMatrix());
			engine->GetBody0()->SetMatrixNoSleep(matrix);

			//integrate forward motion angle;
			ndFloat32 fowardAngle = engine->GetAngle0();
			engine->SetTargetAngle0(fowardAngle + m_engineOmega * timestep);

			// integrate turn rate angle
			ndFloat32 turnAngle = engine->GetAngle1();
			engine->SetTargetAngle1(turnAngle + m_engineTurnRateOmega * timestep);
		}

		// update the model physics every sub step.
		void Update(ndFloat32 timestep) override
		{
			ndModelNotify::Update(timestep);
			UpdateEngine(timestep);
		}

		void ApplyBucketControl()
		{
			ndFloat32 mouseX;
			ndFloat32 mouseY;
			m_scene->GetMousePosition(mouseX, mouseY);

			// rotate the bucket
			if (m_scene->GetMouseKeyState(2))
			{
				ndFloat32 bucketAngleStep = ndFloat32(2.0f) * ndDegreeToRad;
				if ((mouseY - m_mouseY) > ndFloat32(0.001f))
				{
					m_bucketJoint->GetBody0()->SetSleepState(false);
					ndFloat32 angle = ndMin(m_bucketJoint->GetTargetAngle() + bucketAngleStep, ndFloat32(130.0f) * ndDegreeToRad);
					m_bucketJoint->SetTargetAngle(angle);
				}
				else if ((mouseY - m_mouseY) < ndFloat32(-0.001f))
				{
					m_bucketJoint->GetBody0()->SetSleepState(false);
					ndFloat32 angle = ndMax (m_bucketJoint->GetTargetAngle() - bucketAngleStep, ndFloat32 (-80.0f) * ndDegreeToRad);
					m_bucketJoint->SetTargetAngle(angle);
				}
			}
		}

		void ApplyArmControl()
		{
			ndFloat32 mouseX;
			ndFloat32 mouseY;
			m_scene->GetMousePosition(mouseX, mouseY);

			ndFloat32 step_x = 0.0f;
			ndFloat32 step_y = 0.0f;
			bool rotateCabine = false;

			if (m_scene->GetMouseKeyState(0) && m_scene->GetMouseKeyState(1))
			{
				// rotate cabin
				ndFloat32 cabinOmega = ndFloat32(1.0f) * ndDegreeToRad;
				if ((mouseX - m_mouseX) > ndFloat32(0.001f))
				{
					rotateCabine = true;
					m_armAngle = ndAnglesAdd(m_armAngle, cabinOmega);
				}
				else if ((mouseX - m_mouseX) < ndFloat32(-0.001f))
				{
					rotateCabine = true;
					m_armAngle = ndAnglesAdd(m_armAngle, -cabinOmega);
				}
			}
			else if (m_scene->GetMouseKeyState(1))
			{
				// moving arm in the plane of the IK chain
				if ((mouseX - m_mouseX) > ndFloat32(0.001f))
				{
					step_x = 0.05f;
				}
				else if ((mouseX - m_mouseX) < ndFloat32(-0.001f))
				{
					step_x = -0.05f;
				}
				
				if ((mouseY - m_mouseY) > ndFloat32(0.001f))
				{
					step_y = -0.05f;
				}
				else if ((mouseY - m_mouseY) < ndFloat32(-0.001f))
				{
					step_y = 0.05f;
				}
			}

			if (rotateCabine || (step_x != 0.0f) || (step_y != 0.0f))
			{
				m_armPosit_x += step_x;
				m_armPosit_y += step_y;

				ndFloat32 angle = m_armAngle0 + m_armAngle;
				ndFloat32 x = m_armPosit_x * ndCos(angle);
				ndFloat32 z = m_armPosit_x * ndSin(angle);

				ndVector posit(x, m_armPosit_y, z, ndFloat32(1.0f));
				if (m_armEffector->TestWorkSpaceViolation(posit))
				{
					// prevent arm from get stucked in a violation by back tracking
					for (ndInt32 i = 0; i < m_backTrackingArm.GetCount(); ++i)
					{
						posit = ndVector(m_backTrackingArm[i]);
						if (!m_armEffector->TestWorkSpaceViolation(posit))
						{
							x = posit.m_x;
							z = posit.m_z;
							m_armPosit_y = posit.m_y;
							m_armPosit_x = ndSqrt(x * x + z * z);
							break;
						}
					}
				}
				m_armEffector->GetBody0()->SetSleepState(false);
				m_armEffector->SetLocalTargetPosition(ndVector(x, m_armPosit_y, z, ndFloat32(1.0f)));

				// save position for backtracking if get stuck
				for (ndInt32 i = m_backTrackingArm.GetCount() - 1; i > 0; --i)
				{
					m_backTrackingArm[i] = m_backTrackingArm[i - 1];
				}
				m_backTrackingArm[0] = ndVector(x, m_armPosit_y, z, ndFloat32(1.0f));
			}
		}

		void ApplyEngineInputs()
		{
			ndBodyDynamic* const engine = m_engineNode->m_body->GetAsBodyDynamic();

			ndRender* const renderer = *m_scene->GetRenderer();
			ndSharedPtr<ndRenderSceneNode> camera(renderer->GetCamera());
			if (camera == m_cameraNode)
			{
				ndFloat32 turnSign = ndFloat32(1.0f);
				m_engineOmega = ndFloat32(0.0f);
				if (m_scene->GetKeyState(ImGuiKey_W))
				{
					turnSign = ndFloat32(0.75f);
					m_engineOmega = -ND_EXCAVATOR_ENGINE_OMEGA;
					engine->SetSleepState(false);
				}
				else if (m_scene->GetKeyState(ImGuiKey_S))
				{
					turnSign = ndFloat32(-0.75f);
					m_engineOmega = ND_EXCAVATOR_ENGINE_OMEGA;
					engine->SetSleepState(false);
				}

				m_engineTurnRateOmega = ndFloat32(0.0f);
				if (m_scene->GetKeyState(ImGuiKey_A))
				{
					m_engineTurnRateOmega = -ND_EXCAVATOR_ENGINE_OMEGA * turnSign;
					engine->SetSleepState(false);
				}
				else if (m_scene->GetKeyState(ImGuiKey_D))
				{
					m_engineTurnRateOmega = ND_EXCAVATOR_ENGINE_OMEGA * turnSign;
					engine->SetSleepState(false);
				}
			}
		}

		// apply model control at the step rate 
		void PostTransformUpdate(ndFloat32) override
		{
			// apply aggressive sleep is the model is moving too slow
			GetModel()->SetSleep(ndFloat32(0.71f), ndFloat32(0.71f), ndFloat32(1.5f), ndFloat32(2.25f));

			// apply engine inputs
			ApplyEngineInputs();

			// apply the bucke controll
			ApplyBucketControl();

			// apply the bucke controll
			ApplyArmControl();

			m_scene->GetMousePosition(m_mouseX, m_mouseY);
		}

		ndSharedPtr<ndRenderSceneNode> GetCamera()
		{
			return m_cameraNode;
		}

		ndWeakPtr<ndJointHinge> m_bucketJoint;
		ndWeakPtr<ndDemoEntityManager> m_scene;
		ndSharedPtr<ndRenderSceneNode> m_cameraNode;
		ndWeakPtr<ndIkSwivelPositionEffector> m_armEffector;
		ndWeakPtr<ndModelArticulation::ndNode> m_engineNode;

		ndFloat32 m_mouseX;
		ndFloat32 m_mouseY;
		ndFloat32 m_armAngle;
		ndFloat32 m_armAngle0;
		ndFloat32 m_armPosit_x;
		ndFloat32 m_armPosit_y;
		ndFloat32 m_engineOmega;
		ndFloat32 m_engineTurnRateOmega;

		ndFixSizeArray<ndVector, 8> m_backTrackingArm;
	};

	class ExcavatorThreadFloorMaterial : public ndApplicationMaterial
	{
		public:
		ExcavatorThreadFloorMaterial()
			:ndApplicationMaterial()
		{
		}

		ExcavatorThreadFloorMaterial(const ExcavatorThreadFloorMaterial& src)
			:ndApplicationMaterial(src)
		{
			m_restitution = ndFloat32(0.1f);
			m_staticFriction0 = ndFloat32(0.8f);
			m_staticFriction1 = ndFloat32(0.8f);
			m_dynamicFriction0 = ndFloat32(0.8f);
			m_dynamicFriction1 = ndFloat32(0.8f);
		}

		ndApplicationMaterial* Clone() const override
		{
			return new ExcavatorThreadFloorMaterial(*this);
		}

		// this material process the contacts generated by the thread links and the backgrount
		virtual void OnContactCallback(const ndContact* const joint, ndFloat32) const override
		{
			// we will only use the first contact point for each thread
			ndContactPointList& contactPoints = (ndContactPointList&)joint->GetContactPoints();
			ndContactPointList::ndNode* next;
			for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst()->GetNext(); contactPointsNode; contactPointsNode = next)
			{
				next = contactPointsNode->GetNext();
				contactPoints.Remove(contactPointsNode);
			}
		}
	};

	class ExcavatorThreadRollerMaterial : public ndApplicationMaterial
	{
		public:
		ExcavatorThreadRollerMaterial()
			:ndApplicationMaterial()
		{
			// disable lateral friction direction
			m_flags = m_flags & ~m_friction0Enable;

			// set high friction and low restitution
			m_restitution = ndFloat32(0.1f);
			m_staticFriction0 = ndFloat32(1.0f);
			m_staticFriction1 = ndFloat32(1.0f);
			m_dynamicFriction0 = ndFloat32(1.0f);
			m_dynamicFriction1 = ndFloat32(1.0f);
		}

		ExcavatorThreadRollerMaterial(const ExcavatorThreadFloorMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const override
		{
			return new ExcavatorThreadRollerMaterial(*this);
		}

		// this material process the contatact generated by the thread links and the backgrount
		virtual void OnContactCallback(const ndContact* const joint, ndFloat32) const override
		{
			ndContactPointList& contactPoints = (ndContactPointList&)joint->GetContactPoints();
			ndAssert(contactPoints.GetCount() == 1);
			ndContactMaterial& contact = contactPoints.GetFirst()->GetInfo();
			ndBodyKinematic* const roller = joint->GetBody0()->GetCollisionShape().GetShape()->GetAsShapeChamferCylinder() ? joint->GetBody0() : joint->GetBody1();
			ndAssert(roller->GetCollisionShape().GetShape()->GetAsShapeChamferCylinder());
			const ndMatrix& matrix = roller->GetMatrix();
			contact.RotateTangentDirections(matrix.m_front);
		}

		//bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const override
		bool OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance&, const ndShapeInstance&) const override
		{
			//ndAssert(0);
			return true;
		}
	};

	ndSharedPtr<ndModelNotify> CreateExcavator(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		// load the ndMesh physics asset
		ndMeshLoader loader;
		loader.LoadMesh(ndGetWorkingFileName("excavatorPhysics.nd"));

		// Get the location on the map 
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		loader.m_mesh->SetMatrix(loader.m_mesh->GetMatrix() * matrix);

		// make a articulated physics model 
		ndSharedPtr<ndModel> excavatorModel(new ndModelArticulation());
		ndModelArticulation* const excavator = excavatorModel->GetAsModelArticulation();
		excavator->Deserialize(*loader.m_mesh);

		// create a graphic object and add it to the scene for visualization
		ndRender* const renderer = *scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, ndGetWorkingFileName("")));

		const ndMesh* const rootMesh = *loader.m_mesh;
		auto BindPhysicsAndGraphics = [scene, excavator, rootMesh, &sceneMesh](ndModelArticulation::ndNode* const node)
		{
			if (!excavator->IsCloseLoop(node))
			{
				// find the visual node this body control by name. 
				const ndMatrix matrix(node->m_body->GetMatrix());
				ndRenderSceneNode* const visualEntityPtr = sceneMesh->FindByClosestMatch(node->m_name);
				ndAssert(visualEntityPtr);
				ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *sceneMesh) ? sceneMesh : visualEntityPtr->GetSharedPtr());
	
				// add a rigid body with notification callback
				const ndMesh* const meshNode = rootMesh->FindByClosestMatch(node->m_name);
				ndAssert(meshNode);

				const ndMesh* parentMeshNode = meshNode->GetParent();
				while (parentMeshNode && !parentMeshNode->GetRigidBody())
				{
					parentMeshNode = parentMeshNode->GetParent();
				}
				ndBodyKinematic* const parentBody = parentMeshNode ? excavator->FindByName(parentMeshNode->GetName().GetStr())->m_body->GetAsBodyKinematic() : nullptr;
			
				ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
				node->m_body->SetNotifyCallback(notify);

				// set the body type for optimizing collision contacts
				ndShapeInstance& collision = node->m_body->GetAsBodyKinematic()->GetCollisionShape();
				collision.m_shapeMaterial.m_userId = m_chassis;
				if (collision.GetShape()->GetAsShapeChamferCylinder())
				{
					collision.m_shapeMaterial.m_userId = m_roller;
				}
				else if ((node->m_name.Find("leftThread") != -1) || (node->m_name.Find("rightThread") != -1))
				{
					collision.m_shapeMaterial.m_userId = m_thread;
				}
			}
		};
		excavator->NodeIterator(BindPhysicsAndGraphics);

		// a more explicit way to add the visual modifier
		auto AddGraphicsModiers = [sceneMesh](ndMesh* const node)
		{
			ndSharedPtr<ndMeshTransformModifier> modifier(node->GetModifier());
			if (modifier)
			{
				ndAssert(modifier->m_owner);
				ndAssert(modifier->m_target);
				ndRenderSceneNode* const owner = sceneMesh->FindByName(modifier->m_owner->GetName());
				ndRenderSceneNode* const target = sceneMesh->FindByName(modifier->m_target->GetName());
				ndAssert(owner);
				ndAssert(target);

				if (strcmp(modifier->ClassName(), ndMeshTransformModifierLookAt::StaticClassName()) == 0)
				{
					ndSharedPtr<ndRenderTransformModifier> renderModifier(new ndRenderTransformModifierLookAtNode(owner, target));
					owner->SetTransformModifier(renderModifier);
				}
				else if (strcmp(modifier->ClassName(), ndMeshTransformModifierTwoLinksIK::StaticClassName()) == 0)
				{
					const ndMeshTransformModifierTwoLinksIK* const modifierIk = (ndMeshTransformModifierTwoLinksIK*) *modifier;
					ndRenderSceneNode* const link = sceneMesh->FindByName(modifierIk->m_childLink->GetName());
					ndAssert(link);
					ndSharedPtr<ndRenderTransformModifier> renderModifier(new ndRenderTransformModifierTwoLinksIK(owner, link, target, modifierIk->m_solutionSign));
					owner->SetTransformModifier(renderModifier);
				}
				else
				{
					ndAssert(0);
				}
			}
		};
		loader.m_mesh->NodeIterator(AddGraphicsModiers);
		
		// using a model articulation for this vehicle
		ndSharedPtr<ndModelNotify> controller(new ExcavatorController(scene, excavator, sceneMesh));
		excavatorModel->SetNotifyCallback(controller);

		// add physics model to world, and graphics model to scene
		ndWorld* const world = scene->GetWorld();
		scene->AddEntity(sceneMesh);
		world->AddModel(excavatorModel);

		return controller;
	}
}
using namespace ndExcavator;

//void ndComplexModelObtimizedERxperiment(ndDemoEntityManager* const scene)
//{
//	//ndSharedPtr<ndBody> mapBody(BuildPlayground(scene));
//	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
//	//ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));
//
//	// add a help menu
//	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend());
//	scene->SetDemoHelp(demoHelper);
//
//	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
//	
//	ndMaterial* const defaulMaterial = callback->GetMaterial(ndDemoContactCallback::m_default, ndDemoContactCallback::m_default);
//	defaulMaterial->m_staticFriction0 = ndFloat32(1.0f);
//	defaulMaterial->m_staticFriction1 = ndFloat32(1.0f);
//	defaulMaterial->m_dynamicFriction0 = ndFloat32(1.0f);
//	defaulMaterial->m_dynamicFriction1 = ndFloat32(1.0f);
//
//	// this material prune extra contact from the thread links and ground 
//	ExcavatorThreadFloorMaterial material0;
//	callback->RegisterMaterial(material0, m_thread, ndDemoContactCallback::m_default);
//	
//	// another material to remove the friction from the thread and roller contacts
//	ExcavatorThreadRollerMaterial material1;
//	callback->RegisterMaterial(material1, m_thread, m_roller);
//	
//	ndMatrix matrix(ndGetIdentityMatrix());
//	ndSharedPtr<ndModelNotify> controller (CreateExcavator(scene, matrix));
//	
//	const ndInt32 stacks = 3;
//	ndMatrix matrix1(ndGetIdentityMatrix());
//	for (ndInt32 i = 0; i < stacks; ++i)
//	{
//		for (ndInt32 j = 0; j < stacks; ++j)
//		{
//			matrix1.m_posit.m_x = 25.0f + ndFloat32(i - stacks / 2) * 12.0f;
//			matrix1.m_posit.m_z = ndFloat32(j - stacks / 2) * 12.0f;
//			//AddPlanks(scene, matrix1, 10.0f, 4);
//			AddLumberYard(scene, matrix1, 4.0f, 10);
//		}
//	}
//
//	ExcavatorController* const playerController = (ExcavatorController*)*controller;
//	ndRender* const renderer = *scene->GetRenderer();
//	renderer->SetCamera(playerController->GetCamera());
//}

void ndComplexModel(ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> mapBody(BuildPlayground(scene));
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	//ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));

	// add a help menu
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();

	// set defualt matrial frition to maximum
	ndMaterial* const defaulMaterial = callback->GetMaterial(ndDemoContactCallback::m_default, ndDemoContactCallback::m_default);
	defaulMaterial->m_staticFriction0 = ndFloat32 (1.0f);
	defaulMaterial->m_staticFriction1 = ndFloat32(1.0f);
	defaulMaterial->m_dynamicFriction0 = ndFloat32(1.0f);
	defaulMaterial->m_dynamicFriction1 = ndFloat32(1.0f);

	// this material prune extra contact from the thread links and ground 
	ExcavatorThreadFloorMaterial material0;
	callback->RegisterMaterial(material0, m_thread, ndDemoContactCallback::m_default);

	// another material to remove the friction from the thread and roller contacts
	ExcavatorThreadRollerMaterial material1;
	callback->RegisterMaterial(material1, m_thread, m_roller);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndModelNotify> controller(CreateExcavator(scene, matrix));

	const ndInt32 stacks = 1;
	//const ndInt32 stacks = 3; //need multi threaded for this
	ndMatrix matrix1(ndGetIdentityMatrix());
	for (ndInt32 i = 0; i < stacks; ++i)
	{
		for (ndInt32 j = 0; j < stacks; ++j)
		{
			matrix1.m_posit.m_x = 25.0f + ndFloat32(i - stacks / 2) * 12.0f;
			matrix1.m_posit.m_z = ndFloat32(j - stacks / 2) * 12.0f;
			AddLumberYard(scene, matrix1, 4.0f, 10);
		}
	}

	ExcavatorController* const playerController = (ExcavatorController*)*controller;
	ndRender* const renderer = *scene->GetRenderer();
	renderer->SetCamera(playerController->GetCamera());
}