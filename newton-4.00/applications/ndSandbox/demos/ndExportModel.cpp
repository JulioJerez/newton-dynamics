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

static ndSharedPtr<ndModel> LoadAndBindModel(ndDemoEntityManager* const scene, const ndMatrix& location, const char* const fileName)
{
    ndMeshLoader loader;
    loader.LoadMesh(ndGetWorkingFileName(fileName).GetStr());

    // make an articulated from the loaded mesh
    ndSharedPtr<ndModel> model(new ndModelArticulation());
    model->GetAsModelArticulation()->Deserialize(*loader.m_mesh);

    // make a hierarchical render mesh and add to the render scene
    ndRender* const renderer = *scene->GetRenderer();
    ndSharedPtr<ndRenderSceneNode> sceneMesh(ndRenderMeshLoader::CreateRenderSceneMesh(renderer, *loader.m_mesh, ndGetWorkingFileName("")));
    scene->AddEntity(sceneMesh);

    // set the matrix location to both visual and physic
    const ndModelArticulation::ndNode* const rootNode = model->GetAsModelArticulation()->GetRoot();
    const ndMatrix matrix(rootNode ? rootNode->m_body->GetMatrix() * location : location);
    sceneMesh->SetTransform(matrix);
    sceneMesh->SetTransform(matrix);
    model->GetAsModelArticulation()->SetTransform(matrix);

    // Bind application data to the model, 
    // this could be a render mesh or something else. 
    // For this demo it is ndRenderSceneNode mesh
    const ndMesh* const rootMesh = *loader.m_mesh;
    auto BindApplicationData = [scene, rootMesh, &sceneMesh](ndModelArticulation::ndNode* const node)
    {
        const ndMesh* const meshNode = rootMesh->FindByClosestMatch(node->m_name);
        ndAssert(meshNode);

        // find the visual node this body control by name. 
        const ndMatrix matrix(node->m_body->GetMatrix());
        ndRenderSceneNode* const visualEntityPtr = sceneMesh->FindByClosestMatch(meshNode->GetName()); 
        ndAssert(visualEntityPtr);
        ndSharedPtr<ndRenderSceneNode> visualEntity((visualEntityPtr == *sceneMesh) ? sceneMesh : visualEntityPtr->GetSharedPtr());

        // add a rigid body with notification callback
        ndBodyKinematic* const parentBody = node->GetParent() ? node->GetParent()->m_body->GetAsBodyKinematic() : nullptr;
        ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, visualEntity, parentBody));
        node->m_body->SetNotifyCallback(notify);
    };
    model->GetAsModelArticulation()->NodeIterator(BindApplicationData);

    return model;
}

namespace ndBoxTricycle
{
    void BoxTricycle(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 diameter)
    {
        ndMatrix matrix(ndGetIdentityMatrix());
        ndModelArticulation* const carModel = new ndModelArticulation();
        ndSharedPtr<ndModel>model(carModel);

        auto MakePrimitive = [scene](const ndMatrix& matrix, const ndShapeInstance& shape, ndFloat32 mass)
        {
            ndSharedPtr<ndBody> body(new ndBodyDynamic());
            body->SetMatrix(matrix);
            body->GetAsBodyDynamic()->SetCollisionShape(shape);
            body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);
            return body;
        };

        ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(diameter * 2.0f, 0.5f * diameter, 1.5f * diameter)));
        ndSharedPtr<ndBody> rootBody(MakePrimitive(matrix, **shape, mass));
        ndModelArticulation::ndNode* rootNode = carModel->AddRootBody(rootBody);

        // add two roller wheels
        {
            ndSharedPtr<ndShapeInstance>rollerShape(new ndShapeInstance(new ndShapeChamferCylinder(0.25f * diameter, 0.25f * diameter)));
            {
                // add a roller
                ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * matrix);
                rollerMatrix.m_posit.m_x -= diameter * 0.9f;
                rollerMatrix.m_posit.m_z -= diameter * 0.8f;
                rollerMatrix.m_posit.m_y -= diameter * 0.25f;
                ndSharedPtr<ndBody> rollerBody(MakePrimitive(rollerMatrix, **rollerShape, mass * 0.125f));

                const ndMatrix rollerPin(rollerMatrix);
                ndSharedPtr<ndJointBilateralConstraint> rollerAxle(new ndJointHinge(rollerPin, rollerBody->GetAsBodyDynamic(), rootBody->GetAsBodyDynamic()));
                carModel->AddLimb(rootNode, rollerBody, rollerAxle);
            }

            {
                // add another roller
                ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * matrix);
                rollerMatrix.m_posit.m_x -= diameter * 0.9f;
                rollerMatrix.m_posit.m_z += diameter * 0.8f;
                rollerMatrix.m_posit.m_y -= diameter * 0.25f;

                ndSharedPtr<ndBody> rollerBody(MakePrimitive(rollerMatrix, **rollerShape, mass * 0.125f));
                const ndMatrix rollerPin(rollerMatrix);
                ndSharedPtr<ndJointBilateralConstraint> rollerAxle(new ndJointHinge(rollerPin, rollerBody->GetAsBodyDynamic(), rootBody->GetAsBodyDynamic()));
                carModel->AddLimb(rootNode, rollerBody, rollerAxle);
            }

            {
                // add a wheel
                ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * matrix);
                rollerMatrix.m_posit.m_x += diameter * 1.0f;
                rollerMatrix.m_posit.m_y -= diameter * 0.25f;
                ndSharedPtr<ndBody> rollerBody(MakePrimitive(rollerMatrix, **rollerShape, mass * 0.125f));

                ndWheelDescriptor desc;
                const ndMatrix rollerPin(rollerMatrix);
                ndSharedPtr<ndJointBilateralConstraint> wheelAxle(new ndJointWheel(rollerPin, rollerBody->GetAsBodyDynamic(), rootBody->GetAsBodyDynamic(), desc));
                carModel->AddLimb(rootNode, rollerBody, wheelAxle);
            }
        }

        // we now export the model as a ndMesh
        model->GetAsModelArticulation()->SaveNdMesh(ndGetWorkingFileName("boxTricycle.nd").GetStr());

        // test the exported model
        ndPhysicsWorld* const world = scene->GetWorld();
        ndSharedPtr<ndModel> testModel(LoadAndBindModel(scene, location, "boxTricycle.nd"));
        world->AddModel(testModel);
    }
};

// Material ragdoll : désactive les collisions internes
namespace ndDaveRagdoll
{
    ndModelArticulation* CreateRagdoll(ndDemoEntityManager* const scene)
    {
        ndMatrix m_bassin_matrixLocal(ndGetIdentityMatrix());

        ndMatrix m_colonne_matrixLocal(ndRollMatrix(90.0f * ndDegreeToRad));
        m_colonne_matrixLocal.m_posit.m_y = 0.75f * 0.5f;

        ndMatrix m_head_matrixLocal(ndGetIdentityMatrix());
        m_head_matrixLocal.m_posit.m_x = 1.25f;

        // right arm transforms
        ndMatrix m_epaule_R_matrixLocal(ndYawMatrix(-45.0f * ndDegreeToRad));
        m_epaule_R_matrixLocal.m_posit.m_x = 0.65f;

        ndMatrix m_bras_R_matrixLocal(ndYawMatrix(-120.0f * ndDegreeToRad));
        m_bras_R_matrixLocal.m_posit.m_x = 0.8f;

        ndMatrix m_avant_bras_R_matrixLocal(ndRollMatrix(-30.0f * ndDegreeToRad));
        m_avant_bras_R_matrixLocal.m_posit.m_x = 1.0f;

        ndMatrix m_hand_R_matrixLocal(ndGetIdentityMatrix());
        m_hand_R_matrixLocal.m_posit.m_x = 1.0f;

        // left arm transforms
        ndMatrix m_epaule_L_matrixLocal(ndYawMatrix(45.0f * ndDegreeToRad));
        m_epaule_L_matrixLocal.m_posit.m_x = 0.65f;

        ndMatrix m_bras_L_matrixLocal(ndYawMatrix(120.0f * ndDegreeToRad));
        m_bras_L_matrixLocal.m_posit.m_x = 0.8f;

        ndMatrix m_avant_bras_L_matrixLocal(ndRollMatrix(-30.0f * ndDegreeToRad));
        m_avant_bras_L_matrixLocal.m_posit.m_x = 1.0f;

        ndMatrix m_hand_L_matrixLocal(ndGetIdentityMatrix());
        m_hand_L_matrixLocal.m_posit.m_x = 1.0f;

        // left Leg Transforms
        ndMatrix m_hip_L_matrixLocal(ndPitchMatrix(-10.0f * ndDegreeToRad) * ndRollMatrix(-10.0f * ndDegreeToRad) * ndYawMatrix(90.0f * ndDegreeToRad));
        m_hip_L_matrixLocal.m_posit.m_y = -0.4f;
        m_hip_L_matrixLocal.m_posit.m_z = -0.1f;

        ndMatrix m_cuisse_L_matrixLocal(ndRollMatrix(-80.0f * ndDegreeToRad));
        m_cuisse_L_matrixLocal.m_posit.m_x = 0.45f;

        ndMatrix m_tibia_L_matrixLocal(ndYawMatrix(15.0f * ndDegreeToRad));
        m_tibia_L_matrixLocal.m_posit.m_x = 1.0f;

        ndMatrix m_pied_L_matrixLocal(ndGetIdentityMatrix());
        m_pied_L_matrixLocal.m_posit.m_x = 1.15f;

        // right Leg Transforms                    
        ndMatrix m_hip_R_matrixLocal(ndPitchMatrix(10.0f * ndDegreeToRad) * ndRollMatrix(-10.0f * ndDegreeToRad) * ndYawMatrix(-90.0f * ndDegreeToRad));
        m_hip_R_matrixLocal.m_posit.m_y = -0.4f;
        m_hip_R_matrixLocal.m_posit.m_z =  0.1f;

        ndMatrix m_cuisse_R_matrixLocal(ndRollMatrix(-80.0f * ndDegreeToRad));
        m_cuisse_R_matrixLocal.m_posit.m_x = 0.45f;

        ndMatrix m_tibia_R_matrixLocal(ndYawMatrix(-15.0f * ndDegreeToRad));
        m_tibia_R_matrixLocal.m_posit.m_x = 1.0f;

        ndMatrix m_pied_R_matrixLocal(ndGetIdentityMatrix());
        m_pied_R_matrixLocal.m_posit.m_x = 1.15f;

        auto MakePrimitive = [](const ndMatrix& matrix, const ndShapeInstance& shape)
        {
            ndFloat32 mass = 1.0f;
            ndSharedPtr<ndBody> body(new ndBodyDynamic());

            const ndVector com(matrix.TransformVector(shape.GetLocalMatrix().m_posit));
            body->SetMatrix(matrix);
            body->SetCentreOfMass(com);
            body->GetAsBodyDynamic()->SetCollisionShape(shape);
            body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);
            return body;
        };

        auto RootCreateCapsule = [MakePrimitive](const ndMatrix& matrix, ndFloat32 radius, ndFloat32 height)
        {
            ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCapsule(radius, radius, height)));
            ndMatrix localMatrix(ndRollMatrix(90.0f * ndDegreeToRad));
            shape->SetLocalMatrix(localMatrix);
            return MakePrimitive(matrix, **shape);
        };

        auto CreateCapsule = [&scene, MakePrimitive](const ndMatrix& matrix, ndFloat32 radius, ndFloat32 height)
        {
            ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeCapsule(radius, radius, height)));
            ndMatrix localMatrix(ndGetIdentityMatrix());
            localMatrix.m_posit.m_x = height * 0.5f;
            shape->SetLocalMatrix(localMatrix);
            return MakePrimitive(matrix, **shape);
        };

        auto CreateBox = [MakePrimitive](const ndMatrix& matrix, ndFloat32 sx, ndFloat32 sy, ndFloat32 sz)
        {
            ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeBox(sx, sy, sz)));
            return MakePrimitive(matrix, **shape);
        };

        ndFloat32 capsuleRadius = 0.125f;

        ndMatrix bassinMatrix(m_bassin_matrixLocal);
        ndSharedPtr<ndBody> bassinBody (RootCreateCapsule(bassinMatrix, capsuleRadius, 0.75f));

        ndMatrix colonneMatrix (m_colonne_matrixLocal * bassinMatrix);
        ndSharedPtr<ndBody> colonneBody (CreateCapsule(colonneMatrix, capsuleRadius, 1.25f));
        
        ndMatrix headMatrix (m_head_matrixLocal * colonneMatrix);
        ndSharedPtr<ndBody> headBody (CreateCapsule(headMatrix, capsuleRadius, 0.5f));

        // right arm
        ndMatrix epauleRMatrix (m_epaule_R_matrixLocal * colonneMatrix);
        ndSharedPtr<ndBody> epauleRBody (CreateCapsule(epauleRMatrix, capsuleRadius, 0.75f));
        
        ndMatrix brasRMatrix (m_bras_R_matrixLocal * epauleRMatrix);
        ndSharedPtr<ndBody> brasRBody (CreateCapsule(brasRMatrix, capsuleRadius, 1.0f));
        
        ndMatrix avantbrasRMatrix (m_avant_bras_R_matrixLocal * brasRMatrix);
        ndSharedPtr<ndBody> avantbrasRBody (CreateCapsule(avantbrasRMatrix, capsuleRadius, 0.8f));
        
        ndMatrix handRMatrix = m_hand_R_matrixLocal * avantbrasRMatrix;
        ndSharedPtr<ndBody> handRBody(CreateBox(handRMatrix, 0.25f, 0.25f, 0.125f));

        // left arm
        ndMatrix epauleLMatrix(m_epaule_L_matrixLocal* colonneMatrix);
        ndSharedPtr<ndBody> epauleLBody(CreateCapsule(epauleLMatrix, capsuleRadius, 0.75f));

        ndMatrix brasLMatrix (m_bras_L_matrixLocal * epauleLMatrix);
        ndSharedPtr<ndBody> brasLBody (CreateCapsule(brasLMatrix, capsuleRadius, 1.0f));

        ndMatrix avantbrasLMatrix (m_avant_bras_L_matrixLocal * brasLMatrix);
        ndSharedPtr<ndBody> avantbrasLBody (CreateCapsule(avantbrasLMatrix, capsuleRadius, 0.8f));

        ndMatrix handLMatrix (m_hand_L_matrixLocal * avantbrasLMatrix);
        ndSharedPtr<ndBody> handLBody(CreateBox(handLMatrix, 0.25f, 0.25f, 0.125f));

        // left leg
        ndMatrix hipLMatrix (m_hip_L_matrixLocal * bassinMatrix);
        ndSharedPtr<ndBody> hipLBody (CreateCapsule(hipLMatrix, capsuleRadius, 0.5f));
        
        ndMatrix cuisseLMatrix (m_cuisse_L_matrixLocal * hipLMatrix);
        ndSharedPtr<ndBody> cuisseLBody (CreateCapsule(cuisseLMatrix, capsuleRadius, 1.0f));
        
        ndMatrix tibiaLMatrix (m_tibia_L_matrixLocal * cuisseLMatrix);
        ndSharedPtr<ndBody> tibiaLBody (CreateCapsule(tibiaLMatrix, capsuleRadius, 1.0f));
        
        ndMatrix piedLMatrix (m_pied_L_matrixLocal * tibiaLMatrix);
        ndSharedPtr<ndBody> piedLBody (CreateBox(piedLMatrix, 0.13f, 0.4f, 0.75f));
         
        // right Leg
        ndMatrix hipRMatrix (m_hip_R_matrixLocal * bassinMatrix);
        ndSharedPtr<ndBody> hipRBody (CreateCapsule(hipRMatrix, capsuleRadius, 0.5f));
        
        ndMatrix cuisseRMatrix (m_cuisse_R_matrixLocal * hipRMatrix);
        ndSharedPtr<ndBody> cuisseRBody (CreateCapsule(cuisseRMatrix, capsuleRadius, 1.0f));
        
        ndMatrix tibiaRMatrix (m_tibia_R_matrixLocal * cuisseRMatrix);
        ndSharedPtr<ndBody> tibiaRBody (CreateCapsule(tibiaRMatrix, capsuleRadius, 1.0f));
        
        ndMatrix piedRMatrix = m_pied_R_matrixLocal * tibiaRMatrix;
        ndSharedPtr<ndBody> piedRBody = CreateBox(piedRMatrix, 0.13f, 0.4f, 0.75f);
        

        //// === Liste des bodies ===
        //m_bodypartlist =
        //{
        //    bassinBody, colonneBody, headBody,
        //    epauleLBody, brasLBody, avantbrasLBody, handLBody,
        //    epauleRBody, brasRBody, avantbrasRBody, handRBody,
        //    hipLBody, cuisseLBody, tibiaLBody, piedLBody, orteilLBody,
        //    hipRBody, cuisseRBody, tibiaRBody, piedRBody, orteilRBody
        //};
        //
        //std::vector<ndFloat32> bodypartMassweigh =
        //{
        //    3.0f, //bassinBody, 
        //    2.0f, //colonneBody, 
        //    0.5f, //headBody,
        //    1.0f, //epauleLBody, 
        //    1.0f, //brasLBody, 
        //    1.0f, //avantbrasLBody, 
        //    1.0f, //handLBody,
        //    1.0f, //epauleRBody, 
        //    1.0f, //brasRBody, 
        //    1.0f, //avantbrasRBody, 
        //    1.0f, //handRBody,
        //    2.0f, //hipLBody, 
        //    1.0f, //cuisseLBody, 
        //    1.0f, //tibiaLBody, 
        //    1.0f, //piedLBody, 
        //    1.0f, //orteilLBody,
        //    2.0f, //hipRBody, 
        //    1.0f, //cuisseRBody, 
        //    1.0f, //tibiaRBody, 
        //    1.0f, //piedRBody, 
        //    1.0f, //orteilRBody
        //};

        //ndPhysicsWorld* const world = scene->GetWorld();
        //// === Material ragdoll ===
        //ndContactCallback* callback = (ndContactCallback*)world->GetContactNotify();
        //DGRagdollMaterial ragdollMat;
        //callback->RegisterMaterial(ragdollMat, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

        //for (auto& body : m_bodypartlist)
        //{
        //    ndShapeMaterial mat = body->GetAsBodyDynamic()->GetCollisionShape().GetMaterial();
        //    mat.m_userId = ndDemoContactCallback::m_modelPart;
        //    body->GetAsBodyDynamic()->GetCollisionShape().SetMaterial(mat);
        //}
        //NormalizeMassDistribution(bodypartMassweigh, 80.0f);
        
        ndModelArticulation::ndNode* nextRootTemp1 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp2 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp3 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp = nullptr;

        ndModelArticulation* const model = new ndModelArticulation();
        ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(bassinBody);

        { // colonne
            const ndMatrix tmp1(ndYawMatrix(-90.0f * ndDegreeToRad) * colonneBody->GetMatrix());
     
            ndJointHinge* const joint = new ndJointHinge(tmp1, colonneBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-25.0f * ndDegreeToRad, 0.1f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 10.0f, 0.5f);
            
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp = model->AddLimb(modelRootNode, colonneBody, jointPtr);
            nextRootTemp3 = nextRootTemp;
        }

        { // head
            const ndMatrix tmp1(ndYawMatrix(-90.0f * ndDegreeToRad) * headBody->GetMatrix());
        
            ndJointHinge* const joint = new ndJointHinge(tmp1, headBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-120.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 25.0f, 0.5f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp = model->AddLimb(nextRootTemp, headBody, jointPtr);
        }

        // right arm
        { // epaule_R
            ndMatrix tmp1(epauleRBody->GetMatrix());

            ndJointHinge* const joint = new ndJointHinge(tmp1, epauleRBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 10.0f, 0.5f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp3, epauleRBody, jointPtr);
        }

        { // bras_R
            ndMatrix tmp1(brasRBody->GetMatrix());

            ndJointDoubleHinge* const joint = new ndJointDoubleHinge(tmp1, brasRBody->GetAsBodyKinematic(), epauleRBody->GetAsBodyKinematic());
            joint->SetLimitState0(true);
            joint->SetLimits0(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint->SetAsSpringDamper0(0.01f, 50.0f, 10.0f);

            joint->SetLimitState1(true);
            joint->SetLimits1(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint->SetAsSpringDamper1(0.01f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, brasRBody, jointPtr);
        }

        { // avantbras_R arm
            ndMatrix tmp1(avantbrasRBody->GetMatrix());
            tmp1.m_front = tmp1.m_right.Scale(1.0f);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndJointHinge* const joint = new ndJointHinge(tmp1, avantbrasRBody->GetAsBodyKinematic(), brasRBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-120.0f * ndDegreeToRad, 15.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.1f, 0.0f, 5.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, avantbrasRBody, jointPtr);
        }

        { // hand_R
            ndMatrix tmp1(handRBody->GetMatrix());

            ndJointHinge* const joint = new ndJointHinge(tmp1, handRBody->GetAsBodyKinematic(), avantbrasRBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-45.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, handRBody, jointPtr);
        }

        // left arm
        { // epaule_L
            ndMatrix tmp1(epauleLBody->GetMatrix());

            ndJointHinge* const joint = new ndJointHinge(tmp1, epauleLBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 10.0f, 0.5f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp3, epauleLBody, jointPtr);
        }

        { // bras_L
            ndMatrix tmp1(brasLBody->GetMatrix());

            ndJointDoubleHinge* const joint = new ndJointDoubleHinge(tmp1, brasLBody->GetAsBodyKinematic(), epauleLBody->GetAsBodyKinematic());
            joint->SetLimitState0(true);
            joint->SetLimits0(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint->SetAsSpringDamper0(0.01f, 50.0f, 10.0f);
            
            joint->SetLimitState1(true);
            joint->SetLimits1(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint->SetAsSpringDamper1(0.01f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, brasLBody, jointPtr);
        }

        { // avantbras_L = arm
            ndMatrix tmp1(avantbrasLBody->GetMatrix());
            tmp1.m_front = tmp1.m_right.Scale(-1.0f);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndJointHinge* const joint = new ndJointHinge(tmp1, avantbrasLBody->GetAsBodyKinematic(), brasLBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-15.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.1f, 0.0f, 5.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, avantbrasLBody, jointPtr);
        }

        { // hand_L
            ndMatrix tmp1(handLBody->GetMatrix());

            ndJointHinge* const joint = new ndJointHinge(tmp1, handLBody->GetAsBodyKinematic(), avantbrasLBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-45.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, handLBody, jointPtr);
        }

        // left leg
        { // hip_L
            ndMatrix tmp1(hipLBody->GetMatrix());

            ndJointHinge* const joint = new ndJointHinge(tmp1, hipLBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 25.0f, 1.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(modelRootNode, hipLBody, jointPtr);
        }

        { // cuisse_L
            ndMatrix tmp1(cuisseLBody->GetMatrix());
            tmp1.m_up = tmp1.m_right;
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndJointDoubleHinge* const joint = new ndJointDoubleHinge(tmp1, cuisseLBody->GetAsBodyKinematic(), hipLBody->GetAsBodyKinematic());
            joint->SetLimitState0(true);
            joint->SetLimits0(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint->SetAsSpringDamper0(0.005f, 50.0f, 10.0f);

            joint->SetLimitState1(true);
            joint->SetLimits1(-10.0f * ndDegreeToRad, 70.0f * ndDegreeToRad);
            joint->SetAsSpringDamper1(0.005f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, cuisseLBody, jointPtr);
        }

        { // tibia_L
            ndMatrix tmp1(tibiaLBody->GetMatrix());
            ndSwap(tmp1.m_up, tmp1.m_front);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndJointHinge* const joint = new ndJointHinge(tmp1, tibiaLBody->GetAsBodyKinematic(), cuisseLBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-5.0f * ndDegreeToRad, 150.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 2.5f, 0.25f);
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, tibiaLBody, jointPtr);
        }

        { // pied_L
            ndMatrix tmp1(piedLBody->GetMatrix());
            tmp1.m_front = tmp1.m_right;
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndShapeInstance& instance = piedLBody->GetAsBodyKinematic()->GetCollisionShape();
            ndMatrix footMatrix(instance.GetLocalMatrix());
            footMatrix.m_posit.m_z += 0.15f;
            instance.SetLocalMatrix(footMatrix);

            ndJointDoubleHinge* const joint = new ndJointDoubleHinge(tmp1, piedLBody->GetAsBodyKinematic(), tibiaLBody->GetAsBodyKinematic());

            joint->SetLimitState0(true);
            joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
            joint->SetAsSpringDamper0(0.01f, 0.0f, 10.0f);

            joint->SetLimitState1(true);
            joint->SetLimits1(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
            joint->SetAsSpringDamper1(0.01f, 0.0f, 5.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, piedLBody, jointPtr);
        }

        // right leg
        { // hip_R
            ndMatrix tmp1(hipRBody->GetMatrix());
            
            ndJointHinge* const joint = new ndJointHinge(tmp1, hipRBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 25.0f, 1.0f);
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(modelRootNode, hipRBody, jointPtr);
        }

        { // cuisse_R
            ndMatrix tmp1(cuisseRBody->GetMatrix());
            tmp1.m_up = tmp1.m_right.Scale (-1.0f);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndJointDoubleHinge* const joint = new ndJointDoubleHinge(tmp1, cuisseRBody->GetAsBodyKinematic(), hipRBody->GetAsBodyKinematic());
            joint->SetLimitState0(true);
            joint->SetLimits0(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint->SetAsSpringDamper0(0.005f, 50.0f, 10.0f);

            joint->SetLimitState1(true);
            joint->SetLimits1(-70.0f * ndDegreeToRad, 10.0f * ndDegreeToRad);
            joint->SetAsSpringDamper1(0.005f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, cuisseRBody, jointPtr);
        }

        { // tibia_R
            ndMatrix tmp1(tibiaRBody->GetMatrix());
            ndSwap(tmp1.m_up, tmp1.m_front);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndJointHinge* const joint = new ndJointHinge(tmp1, tibiaRBody->GetAsBodyKinematic(), cuisseRBody->GetAsBodyKinematic());
            joint->SetLimitState(true);
            joint->SetLimits(-150.0f * ndDegreeToRad, 5.0f * ndDegreeToRad);
            joint->SetAsSpringDamper(0.01f, 2.5f, 0.25f);
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, tibiaRBody, jointPtr);
        }

        { // pied_R
            ndMatrix tmp1(piedRBody->GetMatrix());
            tmp1.m_front = tmp1.m_right;
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            ndShapeInstance& instance = piedRBody->GetAsBodyKinematic()->GetCollisionShape();
            ndMatrix footMatrix(instance.GetLocalMatrix());
            footMatrix.m_posit.m_z -= 0.15f;
            instance.SetLocalMatrix(footMatrix);

            ndJointDoubleHinge* const joint = new ndJointDoubleHinge(tmp1, piedRBody->GetAsBodyKinematic(), tibiaRBody->GetAsBodyKinematic());

            joint->SetLimitState0(true);
            joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
            joint->SetAsSpringDamper0(0.01f, 0.0f, 10.0f);

            joint->SetLimitState1(true);
            joint->SetLimits1(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
            joint->SetAsSpringDamper1(0.01f, 0.0f, 5.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, piedRBody, jointPtr);
        }

        return model;
    }

    void RagDoll(ndDemoEntityManager* const scene, const ndMatrix& location)
    {
        // make a physic rag doll for using as template
        ndSharedPtr<ndModel> model (CreateRagdoll(scene));

        // we now export the model as a ndMesh
        model->GetAsModelArticulation()->SaveNdMesh(ndGetWorkingFileName("daveRagdoll.nd").GetStr());

        // test the exported model
        ndPhysicsWorld* const world = scene->GetWorld();
        ndSharedPtr<ndModel> testModel(LoadAndBindModel(scene, location, "daveRagdoll.nd"));
        world->AddModel(testModel);
    }
};

namespace ndBasicRagdoll
{
    class ndDefinition
    {
        public:
        enum ndjointType
        {
            m_root,
            m_hinge,
            m_spherical,
            m_doubleHinge,
            m_effector
        };

        struct ndDampData
        {
            ndDampData()
                :m_spring(0.0f)
                , m_damper(0.25f)
                , m_regularizer(0.025f)
            {
            }

            ndDampData(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
                :m_spring(spring)
                ,m_damper(damper)
                ,m_regularizer(regularizer)
            {
            }

            ndFloat32 m_spring;
            ndFloat32 m_damper;
            ndFloat32 m_regularizer;
        };

        struct ndJointLimits
        {
            ndFloat32 m_minTwistAngle;
            ndFloat32 m_maxTwistAngle;
            ndFloat32 m_coneAngle;
        };

        struct ndOffsetFrameMatrix
        {
            ndFloat32 m_pitch;
            ndFloat32 m_yaw;
            ndFloat32 m_roll;
        };

        char m_boneName[32];
        ndjointType m_limbType;
        ndFloat32 m_massWeight;
        ndJointLimits m_jointLimits;
        ndOffsetFrameMatrix m_frameBasics;
        ndDampData m_coneSpringData;
        ndDampData m_twistSpringData;
    };

    static ndDefinition ragdollDefinition[] =
    {
        { "root", ndDefinition::m_root, 1.0f, {}, {} },

        { "lowerback", ndDefinition::m_spherical, 1.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
        { "upperback", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },

        { "lowerneck", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
        { "upperneck", ndDefinition::m_spherical, 1.0f,{ -60.0f, 60.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },

        { "lclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
        { "lhumerus", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
        { "lradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

        { "rclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
        { "rhumerus", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
        { "rradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

        { "rhipjoint", ndDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
        { "rfemur", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
        { "rtibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

        { "lhipjoint", ndDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
        { "lfemur", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
        { "ltibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

        { "", ndDefinition::m_root,{},{} },
    };

    ndModelArticulation* CreateRagdoll()
    {
        ndMeshLoader loader;
        loader.LoadMesh(ndGetWorkingFileName("ragdoll.nd"));

        auto CreateBodyPart = [](const ndMesh* const meshNode)
        {
            ndSharedPtr<ndShapeInstance> shape(((ndMesh*)meshNode)->CreateCollisionFromChildren());
            const ndMatrix matrix(meshNode->CalculateGlobalMatrix());
            ndSharedPtr<ndBody> body(new ndBodyDynamic());
            body->SetMatrix(matrix);
            body->GetAsBodyDynamic()->SetCollisionShape(**shape);
            body->GetAsBodyDynamic()->SetMassMatrix(1.0f, **shape);
            return body;
        };

        auto ConnectBodyParts = [](ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody, const ndDefinition& definition)
        {
            ndMatrix matrix(childBody->GetMatrix());
            ndDefinition::ndOffsetFrameMatrix frameAngle(definition.m_frameBasics);
            ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

            ndDefinition::ndJointLimits jointLimits(definition.m_jointLimits);

            switch (definition.m_limbType)
            {
                case ndDefinition::m_spherical:
                {
                    ndJointSpherical* const joint = new ndJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBody);
                    joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
                    joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
                    joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
                    return (ndJointBilateralConstraint*)joint;
                }

                case ndDefinition::m_hinge:
                {
                    ndJointHinge* const joint = new ndJointHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
                    joint->SetLimitState(true);
                    joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
                    joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
                    return (ndJointBilateralConstraint*)joint;
                }

                case ndDefinition::m_doubleHinge:
                {
                    ndJointDoubleHinge* const joint = new ndJointDoubleHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
                    joint->SetLimitState0(true);
                    joint->SetLimitState1(true);
                    joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
                    joint->SetLimits1(-45.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
                    joint->SetAsSpringDamper0(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
                    joint->SetAsSpringDamper1(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
                    return (ndJointBilateralConstraint*)joint;
                }

                default:
                    ndAssert(0);
            }
            return (ndJointBilateralConstraint*)nullptr;
        };


        ndModelArticulation* const ragdoll = new ndModelArticulation;
        
        ndFixSizeArray<const ndMesh*, 256> stack;
        ndFixSizeArray<ndModelArticulation::ndNode*, 256> parents;
        ndMesh* const mesh(loader.m_mesh->FindByName(ragdollDefinition[0].m_boneName));

        stack.PushBack(mesh);
        parents.PushBack(nullptr);
        
        while (stack.GetCount())
        {
            const ndMesh* const meshNode = stack.Pop();
            ndModelArticulation::ndNode* parentNode = parents.Pop();
            const char* const name = meshNode->GetName().GetStr();
            for (ndInt32 i = 0; ragdollDefinition[i].m_boneName[0]; ++i)
            {
                const ndDefinition& definition = ragdollDefinition[i];
                if (strcmp(definition.m_boneName, name) == 0)
                {
                    ndSharedPtr<ndBody> body(CreateBodyPart(meshNode));
                    if (!parentNode)
                    {
                        parentNode = ragdoll->AddRootBody(body);
                        parentNode->m_name = meshNode->GetName();
                    }
                    else
                    {
                        ndBodyDynamic* const parentBody = parentNode->m_body->GetAsBodyDynamic();
                        
                        //connect this body part to its parentBody with a rag doll joint
                        ndSharedPtr<ndJointBilateralConstraint> joint(ConnectBodyParts(body->GetAsBodyDynamic(), parentBody, definition));
                        
                        // add this child body to the rad doll model.
                        parentNode = ragdoll->AddLimb(parentNode, body, joint);
                    }
                    break;
                }
            }
        
            const ndList<ndSharedPtr<ndMesh>>& children = meshNode->GetChildren();
            for (ndList<ndSharedPtr<ndMesh>>::ndNode* node = children.GetFirst(); node; node = node->GetNext())
            {
                const ndMesh* const child = *node->GetInfo();
                stack.PushBack(child);
                parents.PushBack(parentNode);
            }
        }
        //CalculateMassDistribution(ragdoll, ndFloat32(100.0f));

        return ragdoll;
    }

    void RagDoll(ndDemoEntityManager* const scene, const ndMatrix& location)
    {
        // make a physic rag doll for using as template
        ndSharedPtr<ndModel> model(CreateRagdoll());
        
        // we now export the model as a ndMesh
        model->GetAsModelArticulation()->SaveNdMesh(ndGetWorkingFileName("basicRagdoll.nd").GetStr());

        // test the exported model
        ndPhysicsWorld* const world = scene->GetWorld();
        ndSharedPtr<ndModel> testModel(LoadAndBindModel(scene, location, "basicRagdoll.nd"));
        world->AddModel(testModel);
    }
};

namespace ndExcavator
{
#if 0
    class ndExcavatorController : public ndModelNotify
    {
        public:
        enum ndBodyPartType
        {
            m_chassis = 1,
            m_thread,
            m_roller,
        };

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
                scene->Print(color, "'right and middle mouse click' rotate cabin");
            }
        };

        ndExcavatorController(
            ndDemoEntityManager* const scene, ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndModel>& model, ndSharedPtr<ndRenderSceneNode>& visualMesh)
            :ndModelNotify()
            ,m_scene(scene)
            ,m_engineNode(nullptr)
            ,m_armEffector(nullptr)
            ,m_bucketJoint(nullptr)
            ,m_cameraNode(nullptr)
            ,m_mouseX(ndFloat32(0.0f))
            ,m_mouseY(ndFloat32(0.0f))
            ,m_targetOmega(ndFloat32(0.0f))
            ,m_turnRateOmega(ndFloat32(0.0f))
            ,m_armAngle(ndFloat32(0.0f))
            ,m_armAngle0(ndFloat32(0.0f))
            ,m_armPosit_x(ndFloat32(0.0f))
            ,m_armPosit_y(ndFloat32(0.0f))
            ,m_backTrackingArm(8)
        {
            ndModelArticulation* const articulation = model->GetAsModelArticulation();
            ndAssert(articulation);

            // build the major vehicle components, each major component build its sub parts
            MakeChassis(articulation, mesh, visualMesh);

            // add the cabin and boom mechanism
            MakeCabinAndUpperBody(articulation, mesh, visualMesh);

            // add the roller and diffrenticl gear system
            MakeLeftTrack(articulation, mesh, visualMesh);
            MakeRightTrack(articulation, mesh, visualMesh);

            // add the tracks
            MakeThread(articulation, "leftThread", mesh, visualMesh);
            MakeThread(articulation, "rightThread", mesh, visualMesh);
        }

        ndSharedPtr<ndRenderSceneNode> GetCamera()
        {
            return m_cameraNode;
        }

        static ndSharedPtr<ndModelNotify> CreateExcavator(ndDemoEntityManager* const scene, const ndMatrix& location, ndRenderMeshLoader& mesh)
        {
            // crate a model and set the root transform
            ndSharedPtr<ndRenderSceneNode> vehicleMesh(mesh.m_renderMesh->Clone());
            ndMatrix matrix(location);
            matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);

            ndMatrix saveMatrix(mesh.m_mesh->GetMatrix());
            mesh.m_mesh->SetMatrix(saveMatrix * matrix);

            // using a model articulation for this vehicle
            ndSharedPtr<ndModel> vehicleModel(new ndModelArticulation());
            ndSharedPtr<ndModelNotify> controller(new ndExcavatorController(scene, mesh.m_mesh, vehicleModel, vehicleMesh));
            vehicleModel->SetNotifyCallback(controller);

            ndWorld* const world = scene->GetWorld();
            scene->AddEntity(vehicleMesh);
            world->AddModel(vehicleModel);

            // restore mesh Matrix 
            mesh.m_mesh->SetMatrix(saveMatrix);
            return controller;
        }

        private:
        // set a body pat type later usage in the concact filter generatior
        void SetBodyType(ndShapeInstance* const collision, ndBodyPartType type)
        {
            collision->m_shapeMaterial.m_userId = type;
        }

        bool OnContactGeneration(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) override
        {
            // here the application can use filter to determine what body parts should collide.
            // this greatly improves performance because since articulated models in general 
            // do not self collide, but occationaly some parts do collide. 
            const ndShapeInstance& shape0 = body0->GetCollisionShape();
            const ndShapeInstance& shape1 = body1->GetCollisionShape();

            ndBodyPartType type0 = ndBodyPartType(shape0.m_shapeMaterial.m_userId);
            ndBodyPartType type1 = ndBodyPartType(shape1.m_shapeMaterial.m_userId);

            bool collide = ((type0 == m_roller) && (type1 == m_thread)) || ((type0 == m_thread) && (type1 == m_roller));
            return collide;
        }

        void MakeChassis(
            ndModelArticulation* const articulation,
            ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
        {
            //ndMesh* const chassisMesh = mesh->FindByName("base");
            //ndAssert(chassisMesh);
            //
            //// create the collision and the world matrix
            //ndMatrix matrix(chassisMesh->CalculateGlobalMatrix());
            //ndSharedPtr<ndShapeInstance> collision(chassisMesh->CreateCollisionFromChildren());
            //
            //// set a body pat type later usage in the concact filter generatior
            //SetBodyType(*collision, m_chassis);
            //
            //// find the visual mesh for thsi body
            //ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(chassisMesh->GetName());
            //ndSharedPtr<ndRenderSceneNode> visualMesh((visualMeshNode == *visualMeshRoot) ? visualMeshRoot : visualMeshRoot->GetSharedPtr());
            //
            //ndFloat32 chassisMass = ndFloat32(4000.0f);
            //
            //// create the rigid that represent the chassis
            //ndSharedPtr<ndBody> chassisBody(new ndBodyDynamic());
            //chassisBody->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh, nullptr));
            //chassisBody->SetMatrix(matrix);
            //chassisBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
            //chassisBody->GetAsBodyDynamic()->SetMassMatrix(chassisMass, **collision);
            //
            //articulation->AddRootBody(chassisBody);
            //
            //// add the motor
            //AddLocomotion(articulation);
            //
            ////add a camera 
            //ndAssert(visualMeshRoot->FindByName("cameraPivot"));
            //ndSharedPtr<ndRenderSceneNode> cameraPivotNode(visualMeshRoot->FindByName("cameraPivot")->GetSharedPtr());

            //ndVector cameraPivot(ndVector::m_zero);
            //cameraPivot.m_y = ndFloat32(2.5f);
            //ndRender* const renderer = *m_scene->GetRenderer();
            //m_cameraNode = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, ND_EXCAVATOR_CAMERA_DISTANCE));
            //cameraPivotNode->AddChild(m_cameraNode);
        }

        ndSharedPtr<ndBody> MakeBodyPart(
            ndMesh* const childMesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
            ndSharedPtr<ndShapeInstance> collision,
            ndBodyKinematic* const parentBody,
            ndFloat32 mass)
        {
            // calculate matrix
            const ndMatrix matrix(childMesh->CalculateGlobalMatrix());

            // find the visual mesh
            ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(childMesh->GetName());
            ndSharedPtr<ndRenderSceneNode> visualMesh(visualMeshNode->GetSharedPtr());

            ndSharedPtr<ndBody> body(new ndBodyDynamic());
            body->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh, parentBody));
            body->SetMatrix(matrix);
            body->GetAsBodyDynamic()->SetCollisionShape(**collision);
            body->GetAsBodyDynamic()->SetMassMatrix(mass, **collision);
            return body;
        }

        ndSharedPtr<ndBody> MakeBodyPart(
            ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
            ndBodyKinematic* const parentbody,
            const char* const name, ndFloat32 mass)
        {
            ndMesh* const meshNode = mesh->FindByName(name);
            ndAssert(meshNode);

            // build collision mesh
            ndSharedPtr<ndShapeInstance> collision(meshNode->CreateCollisionFromChildren());
            return MakeBodyPart(meshNode, visualMeshRoot, collision, parentbody, mass);
        }

        void MakeCabinAndUpperBody(
            ndModelArticulation* const articulation,
            ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
        {
            ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

            ndSharedPtr<ndBody> cabinBody(MakeBodyPart(mesh, visualMeshRoot, rootNode->m_body->GetAsBodyDynamic(), "EngineBody", 400.0f));
            SetBodyType(&cabinBody->GetAsBodyDynamic()->GetCollisionShape(), m_chassis);

            // set the center of mass of engine
            const ndMatrix hingeFrame(cabinBody->GetMatrix());
            ndVector com(hingeFrame.m_front.Scale(ndFloat32(5.0f)));
            cabinBody->SetCentreOfMass(com);

            ndSharedPtr<ndJointBilateralConstraint> cabinPivot(new ndJointHinge(hingeFrame, cabinBody->GetAsBodyDynamic(), rootNode->m_body->GetAsBodyDynamic()));
            ndModelArticulation::ndNode* const cabinNode = articulation->AddLimb(rootNode, cabinBody, cabinPivot);

            // add arm0.
            ndSharedPtr<ndBody> bodyArm0(MakeBodyPart(mesh, visualMeshRoot, cabinBody->GetAsBodyKinematic(), "Boom", 50.0f));
            SetBodyType(&bodyArm0->GetAsBodyDynamic()->GetCollisionShape(), m_chassis);
            const ndMatrix matrixArm0(ndYawMatrix(ndFloat32(90.0f) * ndDegreeToRad) * bodyArm0->GetMatrix());
            ndSharedPtr<ndJointBilateralConstraint> jointArm0(new ndJointHinge(matrixArm0, bodyArm0->GetAsBodyDynamic(), cabinBody->GetAsBodyDynamic()));
            ndModelArticulation::ndNode* const armNode0 = articulation->AddLimb(cabinNode, bodyArm0, jointArm0);

            // add arm1.
            ndSharedPtr<ndBody> bodyArm1(MakeBodyPart(mesh, visualMeshRoot, bodyArm0->GetAsBodyKinematic(), "arm02", 50.0f));
            SetBodyType(&bodyArm1->GetAsBodyDynamic()->GetCollisionShape(), m_chassis);
            const ndMatrix matrixArm1(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * bodyArm1->GetMatrix());
            ndSharedPtr<ndJointBilateralConstraint> jointArm1(new ndJointHinge(matrixArm1, bodyArm1->GetAsBodyDynamic(), bodyArm0->GetAsBodyDynamic()));
            ndModelArticulation::ndNode* const armNode1 = articulation->AddLimb(armNode0, bodyArm1, jointArm1);

            // add bucket
            ndSharedPtr<ndBody> bodyBucket(MakeBodyPart(mesh, visualMeshRoot, bodyArm1->GetAsBodyKinematic(), "bucket", 40.0f));
            SetBodyType(&bodyBucket->GetAsBodyDynamic()->GetCollisionShape(), m_chassis);
            const ndMatrix matrixBucket(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * bodyBucket->GetMatrix());
            ndSharedPtr<ndJointBilateralConstraint> jointBucket(new ndJointHinge(matrixBucket, bodyBucket->GetAsBodyDynamic(), bodyArm1->GetAsBodyDynamic()));
            articulation->AddLimb(armNode1, bodyBucket, jointBucket);
            m_bucketJoint = (ndJointHinge*)*jointBucket;
            m_bucketJoint->SetAsSpringDamper(ndFloat32(0.1f), ndFloat32(2000.0f), ndFloat32(50.0f));

            // add an effector to move the arm
            ndMatrix baseFrame(ndGetIdentityMatrix());
            baseFrame.m_posit = hingeFrame.m_posit;

            m_armEffector = new ndIkSwivelPositionEffector(
                baseFrame, rootNode->m_body->GetAsBodyDynamic(),
                matrixBucket.m_posit, bodyArm1->GetAsBodyDynamic());
            m_armEffector->SetSwivelMode(false);
            ndSharedPtr<ndJointBilateralConstraint> effector(m_armEffector);
            articulation->AddCloseLoop(effector);

            // calculate the work space.
            ndVector arm1Len(jointBucket->CalculateGlobalMatrix0().m_posit - jointArm1->CalculateGlobalMatrix0().m_posit);
            ndVector arm0Len(jointArm1->CalculateGlobalMatrix0().m_posit - jointArm0->CalculateGlobalMatrix0().m_posit);
            ndVector cabinLen(jointArm0->CalculateGlobalMatrix0().m_posit - cabinPivot->CalculateGlobalMatrix0().m_posit);
            ndFloat32 l1 = ndSqrt(arm1Len.DotProduct(arm1Len).GetScalar());
            ndFloat32 l0 = ndSqrt(arm0Len.DotProduct(arm0Len).GetScalar()) + ndSqrt(cabinLen.DotProduct(cabinLen).GetScalar());
            ndFloat32 minRadios = (l0 - l1) * ndFloat32(1.1f);
            ndFloat32 maxRadios = (l0 + l1) * ndFloat32(0.9f);
            m_armEffector->SetWorkSpaceConstraints(minRadios, maxRadios);

            // calculate arm parameters		
            ndVector localPosit(m_armEffector->GetLocalTargetPosition());
            m_armAngle = ndFloat32(0.0f);
            m_armPosit_y = localPosit.m_y;
            m_armAngle0 = ndAtan2(localPosit.m_z, localPosit.m_x);
            m_armPosit_x = ndSqrt(localPosit.m_x * localPosit.m_x + localPosit.m_z * localPosit.m_z);
        }

        void LinkTires(ndModelArticulation* const articulation,
            ndModelArticulation::ndNode* const master, ndModelArticulation::ndNode* const slave)
        {
            const ndShapeInstance& slaveShape = slave->m_body->GetAsBodyKinematic()->GetCollisionShape();
            const ndShapeInstance& masterShape = master->m_body->GetAsBodyKinematic()->GetCollisionShape();

            auto GetRadios = [](const ndShapeInstance& shape)
                {
                    //ndShapeCompound* const compoundTire = ((ndShape*)shape.GetShape())->GetAsShapeCompound();
                    //const ndShapeCompound::ndTreeArray& array = compoundTire->GetTree();
                    //ndShapeCompound::ndTreeArray::Iterator it(compoundTire->GetTree());
                    //
                    //ndFloat32 radios = ndFloat32(0.0f);
                    //for (it.Begin(); it; it++)
                    //{
                    //	ndShapeInstance* const subShape = compoundTire->GetShapeInstance(it.GetNode());
                    //	ndAssert (subShape->GetShape()->GetAsShapeChamferCylinder());
                    //	const ndShapeInfo info(subShape->GetShapeInfo());
                    //	radios = ndMax(radios, info.m_chamferCylinder.m_radius);
                    //}
                    const ndShapeInfo info(shape.GetShapeInfo());
                    //return info.m_chamferCylinder.m_radius;
                    return 1.0f;
                };
            ndFloat32 slaveRadio = GetRadios(slaveShape);
            ndFloat32 masterRadio = GetRadios(masterShape);

            ndMatrix pinMatrix0;
            ndMatrix pinMatrix1;
            const ndJointBilateralConstraint* const joint = *master->m_joint;
            joint->CalculateGlobalMatrix(pinMatrix0, pinMatrix1);

            ndFloat32 ratio = slaveRadio / masterRadio;
            ndSharedPtr<ndJointBilateralConstraint> link(new ndJointGear(
                ratio, pinMatrix0[0], slave->m_body->GetAsBodyDynamic(),
                pinMatrix1[0].Scale(ndFloat32(-1.0f)), master->m_body->GetAsBodyDynamic()));

            articulation->AddCloseLoop(link);
        }

        ndModelArticulation::ndNode* MakeRollerTire(
            ndModelArticulation* const articulation,
            ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
            const char* const name)
        {
            ndMesh* const node = mesh->FindByName(name);
            ndAssert(node);
            ndSharedPtr<ndShapeInstance> tireCollision(node->CreateCollisionChamferCylinder());
            SetBodyType(*tireCollision, m_roller);

            //ndFloat32 threadThickness = ndFloat32(0.18f);
            //ndFloat32 threadThickness = ndFloat32(0.0f);
            //ndVector padd(ndVector::m_zero);
            //padd.m_y = threadThickness;
            //padd.m_z = threadThickness;
            //tireCollision->SetScale (tireCollision->GetScale() + padd);

            tireCollision->SetLocalMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * tireCollision->GetLocalMatrix());

            ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();
            ndSharedPtr<ndBody> tireBody(MakeBodyPart(node, visualMeshRoot, tireCollision,
                rootNode->m_body->GetAsBodyKinematic(), ndFloat32(30.0f)));

            const ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * tireBody->GetMatrix());
            ndSharedPtr<ndJointBilateralConstraint> rollerPivot(new ndJointRoller(rollerMatrix, tireBody->GetAsBodyDynamic(), rootNode->m_body->GetAsBodyDynamic()));
            ndJointRoller* const rollerJoint = (ndJointRoller*)*rollerPivot;
            rollerJoint->SetAsSpringDamperPosit(ndFloat32(0.01f), ndFloat32(2000.0f), ndFloat32(50.0f));

            ndModelArticulation::ndNode* const rollerLimb = articulation->AddLimb(rootNode, tireBody, rollerPivot);
            return rollerLimb;
        }

        void MakeLeftTrack(
            ndModelArticulation* const articulation,
            ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
        {
            ndModelArticulation::ndNode* const leftTire_0 = MakeRollerTire(articulation, mesh, visualMeshRoot, "leftGear");
            ndModelArticulation::ndNode* const leftTire_7 = MakeRollerTire(articulation, mesh, visualMeshRoot, "leftFrontRoller");
            ndAssert(leftTire_0);
            ndAssert(leftTire_7);
            LinkTires(articulation, leftTire_0, leftTire_7);
            MakeRollerTire(articulation, mesh, visualMeshRoot, "leftSupportRoller");

            for (ndInt32 i = 0; i < 3; ++i)
            {
                char name[64];
                snprintf(name, 63, "leftRoller%d", i);
                ndModelArticulation::ndNode* const rollerTire = MakeRollerTire(articulation, mesh, visualMeshRoot, name);
                ndAssert(rollerTire);
                LinkTires(articulation, leftTire_0, rollerTire);
            }

            // link traction tire to the engine using a differential gear
            ndMatrix engineMatrix;
            ndMatrix chassisMatrix;
            ndBodyDynamic* const tire = leftTire_0->m_body->GetAsBodyDynamic();
            ndBodyDynamic* const engine = m_engineNode->m_body->GetAsBodyDynamic();
            m_engineNode->m_joint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
            const ndMatrix tireMatrix(tire->GetMatrix());

            ndSharedPtr<ndJointBilateralConstraint> axel(
                new ndMultiBodyVehicleDifferentialAxle(
                    engineMatrix.m_front.Scale(ndFloat32(-1.0f)), engineMatrix.m_up, engine,
                    tireMatrix.m_right.Scale(ND_EXCAVATOR_GEAR_GAIN), tire));

            articulation->AddCloseLoop(axel);
        }

        void MakeRightTrack(
            ndModelArticulation* const articulation,
            ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
        {
            ndModelArticulation::ndNode* const rightTire_0 = MakeRollerTire(articulation, mesh, visualMeshRoot, "rightGear");
            ndModelArticulation::ndNode* const rightTire_7 = MakeRollerTire(articulation, mesh, visualMeshRoot, "rightFrontRoller");
            ndAssert(rightTire_0);
            ndAssert(rightTire_7);
            LinkTires(articulation, rightTire_0, rightTire_7);
            MakeRollerTire(articulation, mesh, visualMeshRoot, "rightSupportRoller");

            for (int i = 0; i < 3; ++i)
            {
                char name[64];
                snprintf(name, 63, "rightRoller%d", i);
                ndModelArticulation::ndNode* const rollerTire = MakeRollerTire(articulation, mesh, visualMeshRoot, name);
                ndAssert(rollerTire);
                LinkTires(articulation, rightTire_0, rollerTire);
            }

            // link traction tire to the engine using a differential gear
            ndMatrix engineMatrix;
            ndMatrix chassisMatrix;
            ndBodyDynamic* const tire = rightTire_0->m_body->GetAsBodyDynamic();
            ndBodyDynamic* const engine = m_engineNode->m_body->GetAsBodyDynamic();
            m_engineNode->m_joint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
            const ndMatrix tireMatrix(tire->GetMatrix());

            ndSharedPtr<ndJointBilateralConstraint> axel(
                new ndMultiBodyVehicleDifferentialAxle(
                    engineMatrix.m_front.Scale(ndFloat32(1.0f)), engineMatrix.m_up, engine,
                    tireMatrix.m_right.Scale(-ND_EXCAVATOR_GEAR_GAIN), tire));

            articulation->AddCloseLoop(axel);
        }

        void MakeThread(ndModelArticulation* const articulation,
            const char* const baseName,
            ndSharedPtr<ndMesh>& mesh,
            ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
        {
            ndFixSizeArray<ndMesh*, 256> stack;
            ndFixSizeArray<ndMesh*, 256> linkArray;
            stack.PushBack(*mesh);

            // get all the thread links in order.
            while (stack.GetCount())
            {
                ndMesh* const node = stack.Pop();
                if (node->GetName().Find(baseName) != -1)
                {
                    linkArray.PushBack(node);
                    ndInt32 index = 0;
                    for (ndInt32 i = linkArray.GetCount() - 2; i >= 0; --i)
                    {
                        if (linkArray[i]->GetName() > node->GetName())
                        {
                            linkArray[i + 1] = linkArray[i];
                            index = i;
                        }
                        else
                        {
                            break;
                        }
                    }
                    linkArray[index] = node;
                }

                for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = node->GetChildren().GetFirst(); child; child = child->GetNext())
                {
                    stack.PushBack(*child->GetInfo());
                }
            }

            // make the collision shape. 
            ndSharedPtr<ndShapeInstance> threadCollision(linkArray[0]->CreateCollisionFromChildren());
            SetBodyType(*threadCollision, m_thread);

            ndFloat32 threadLinkMass = ndFloat32(8.0f);
            ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();
            ndSharedPtr<ndBody> linkBody(MakeBodyPart(linkArray[0], visualMeshRoot, threadCollision, rootNode->m_body->GetAsBodyKinematic(), threadLinkMass));

            ndMatrix planeMatrix(linkBody->GetMatrix());
            ndVector planePivot(planeMatrix.m_posit);
            ndVector planeNornal(planeMatrix.m_up);
            ndSharedPtr<ndJointBilateralConstraint> linkJoint(new ndJointPlane(
                planePivot, planeNornal, linkBody->GetAsBodyDynamic(),
                rootNode->m_body->GetAsBodyDynamic()));

            ndModelArticulation::ndNode* const firstLink = articulation->AddLimb(rootNode, linkBody, linkJoint);

            // connent all threads planks with hinge joint
            ndFloat32 linkDamper = ndFloat32(5.0f);
            ndFloat32 linkRegularizer = ndFloat32(0.15f);
            ndModelArticulation::ndNode* linkNode0 = firstLink;
            for (ndInt32 i = 1; i < linkArray.GetCount(); ++i)
            {
                ndSharedPtr<ndBody> body(MakeBodyPart(linkArray[i], visualMeshRoot, threadCollision, rootNode->m_body->GetAsBodyDynamic(), threadLinkMass));
                ndMatrix hingeMatrix(ndRollMatrix(90.0f * ndDegreeToRad) * body->GetMatrix());
                ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(hingeMatrix, body->GetAsBodyKinematic(), linkNode0->m_body->GetAsBodyKinematic()));
                ((ndJointHinge*)*joint)->SetAsSpringDamper(linkRegularizer, ndFloat32(0.0f), linkDamper);
                ndModelArticulation::ndNode* const firstLink1 = articulation->AddLimb(linkNode0, body, joint);
                linkNode0 = firstLink1;
            }

            ndMatrix hingeMatrix(ndRollMatrix(90.0f * ndDegreeToRad) * firstLink->m_body->GetMatrix());
            ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(hingeMatrix, linkNode0->m_body->GetAsBodyKinematic(), firstLink->m_body->GetAsBodyKinematic()));
            ((ndJointHinge*)*joint)->SetAsSpringDamper(linkRegularizer, ndFloat32(0.0f), linkDamper);
            articulation->AddCloseLoop(joint);
        }

        void AddLocomotion(ndModelArticulation* const articulation)
        {
            ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

            ndSharedPtr<ndShapeInstance> motorCollision(new ndShapeInstance(new ndShapeCylinder(ndFloat32(0.25f), ndFloat32(0.25f), ndFloat32(0.75f))));
            SetBodyType(*motorCollision, m_chassis);

            ndMatrix engineMatrix(rootNode->m_body->GetMatrix());
            engineMatrix = ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * engineMatrix;
            engineMatrix.m_posit.m_y += 1.0f;

            // make engine body
            ndFloat32 mass = ndFloat32(50.0f);
            ndFloat32 radius = ndFloat32(1.0f);
            ndFloat32 inertia = ndFloat32(2.0f / 5.0f) * mass * radius * radius;

            ndSharedPtr<ndBody> motorBody(new ndBodyDynamic());
            motorBody->SetMatrix(engineMatrix);
            motorBody->GetAsBodyDynamic()->SetCollisionShape(**motorCollision);
            motorBody->GetAsBodyDynamic()->SetMassMatrix(inertia, inertia, inertia, mass);

            ndMatrix engineAxis;
            engineAxis.m_front = engineMatrix.m_front;
            engineAxis.m_up = engineMatrix.m_right;
            engineAxis.m_right = engineAxis.m_front.CrossProduct(engineAxis.m_up);
            engineAxis.m_posit = engineMatrix.m_posit;

            ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointDoubleHinge(engineAxis, motorBody->GetAsBodyKinematic(), rootNode->m_body->GetAsBodyKinematic()));
            ((ndJointDoubleHinge*)*joint)->SetAsSpringDamper0(0.1f, 1500.0f, 10.0f);
            ((ndJointDoubleHinge*)*joint)->SetAsSpringDamper1(0.1f, 1500.0f, 10.0f);

            m_engineNode = articulation->AddLimb(rootNode, motorBody, joint);
        }

        void Update(ndFloat32 timestep) override
        {
            ndModelNotify::Update(timestep);

            ndJointDoubleHinge* const engine = (ndJointDoubleHinge*)*m_engineNode->m_joint;

            // reset the engine carcase transform
            const ndMatrix matrix(engine->GetLocalMatrix0().OrthoInverse() * engine->GetLocalMatrix1() * engine->GetBody1()->GetMatrix());
            engine->GetBody0()->SetMatrixNoSleep(matrix);

            // integrate tyurn rate angle
            ndFloat32 turnAngle = engine->GetAngle0();
            engine->SetTargetAngle0(turnAngle + m_turnRateOmega * timestep);

            // integrate the joints angle;
            ndFloat32 fowardAngle = engine->GetAngle1();
            engine->SetTargetAngle1(fowardAngle + m_targetOmega * timestep);
        }

        void PostTransformUpdate(ndFloat32)
        {
            GetModel()->SetSleep(ndFloat32(0.71f), ndFloat32(0.71f), ndFloat32(1.5f), ndFloat32(2.25f));

            ndBodyDynamic* const engine = m_engineNode->m_body->GetAsBodyDynamic();

            ndRender* const renderer = *m_scene->GetRenderer();
            ndSharedPtr<ndRenderSceneNode> camera(renderer->GetCamera());
            if (camera == m_cameraNode)
            {
                ndFloat32 turnSign = ndFloat32(1.0f);
                m_targetOmega = ndFloat32(0.0f);
                if (m_scene->GetKeyState(ImGuiKey_W))
                {
                    turnSign = ndFloat32(0.75f);
                    m_targetOmega = -ND_EXCAVATOR_ENGINE_OMEGA;
                    engine->SetSleepState(false);
                }
                else if (m_scene->GetKeyState(ImGuiKey_S))
                {
                    turnSign = ndFloat32(-0.75f);
                    m_targetOmega = ND_EXCAVATOR_ENGINE_OMEGA;
                    engine->SetSleepState(false);
                }

                m_turnRateOmega = ndFloat32(0.0f);
                if (m_scene->GetKeyState(ImGuiKey_A))
                {
                    m_turnRateOmega = -ND_EXCAVATOR_ENGINE_OMEGA * turnSign;
                    engine->SetSleepState(false);
                }
                else if (m_scene->GetKeyState(ImGuiKey_D))
                {
                    m_turnRateOmega = ND_EXCAVATOR_ENGINE_OMEGA * turnSign;
                    engine->SetSleepState(false);
                }
            }

            ndFloat32 mouseX;
            ndFloat32 mouseY;
            m_scene->GetMousePosition(mouseX, mouseY);

            // control cabine rotation angle
            if (m_scene->GetMouseKeyState(1) || m_scene->GetMouseKeyState(2))
            {
                ndFloat32 step_x = 0;
                ndFloat32 step_y = 0;

                // moving arm in the plane of the IK chain
                if (m_scene->GetMouseKeyState(1) && !m_scene->GetMouseKeyState(2))
                {
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

                // rotate the cabine
                if (m_scene->GetMouseKeyState(1) && m_scene->GetMouseKeyState(2))
                {
                    ndFloat32 cabinOmega = ndFloat32(1.0f) * ndDegreeToRad;
                    if ((mouseX - m_mouseX) > ndFloat32(0.001f))
                    {
                        m_armAngle = ndAnglesAdd(m_armAngle, cabinOmega);
                    }
                    else if ((mouseX - m_mouseX) < ndFloat32(-0.001f))
                    {
                        m_armAngle = ndAnglesAdd(m_armAngle, -cabinOmega);
                    }
                }

                // rotate the bucket
                if (m_scene->GetMouseKeyState(2) && !m_scene->GetMouseKeyState(1))
                {
                    ndFloat32 bucketAngleStep = ndFloat32(2.0f) * ndDegreeToRad;
                    if ((mouseY - m_mouseY) > ndFloat32(0.001f))
                    {
                        m_bucketJoint->SetTargetAngle(m_bucketJoint->GetTargetAngle() + bucketAngleStep);
                    }
                    else if ((mouseY - m_mouseY) < ndFloat32(-0.001f))
                    {
                        m_bucketJoint->SetTargetAngle(m_bucketJoint->GetTargetAngle() - bucketAngleStep);
                    }
                }

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
                engine->SetSleepState(false);
                m_armEffector->SetLocalTargetPosition(ndVector(x, m_armPosit_y, z, ndFloat32(1.0f)));

                // save position for backtracking if get stuck
                for (ndInt32 i = m_backTrackingArm.GetCount() - 1; i > 0; --i)
                {
                    m_backTrackingArm[i] = m_backTrackingArm[i - 1];
                }
                m_backTrackingArm[0] = ndVector(x, m_armPosit_y, z, ndFloat32(1.0f));
            }

            m_mouseX = mouseX;
            m_mouseY = mouseY;
        }

        ndDemoEntityManager* m_scene;
        ndModelArticulation::ndNode* m_engineNode;
        ndIkSwivelPositionEffector* m_armEffector;
        ndJointHinge* m_bucketJoint;
        ndSharedPtr<ndRenderSceneNode> m_cameraNode;
        ndFloat32 m_mouseX;
        ndFloat32 m_mouseY;
        ndFloat32 m_targetOmega;
        ndFloat32 m_turnRateOmega;

        ndFloat32 m_armAngle;
        ndFloat32 m_armAngle0;
        ndFloat32 m_armPosit_x;
        ndFloat32 m_armPosit_y;
        ndFixSizeArray<ndVector, 8> m_backTrackingArm;
    };
#endif

    static void MakeChassis(
        ndModelArticulation* const articulation,
        ndSharedPtr<ndMesh>& mesh,
        ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
    {
        ndMesh* const chassisMesh = mesh->FindByName("base");
        ndAssert(chassisMesh);
        
        // create the collision and the world matrix
        ndMatrix matrix(chassisMesh->CalculateGlobalMatrix());
        ndSharedPtr<ndShapeInstance> collision(chassisMesh->CreateCollisionFromChildren());
        
        //// set a body pat type later usage in the concact filter generatior
        //SetBodyType(*collision, m_chassis);
        //
        // find the visual mesh for thsi body
        ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(chassisMesh->GetName());
        ndSharedPtr<ndRenderSceneNode> visualMesh((visualMeshNode == *visualMeshRoot) ? visualMeshRoot : visualMeshRoot->GetSharedPtr());
        
        ndFloat32 chassisMass = ndFloat32(4000.0f);
        
        // create the rigid that represent the chassis
        ndSharedPtr<ndBody> chassisBody(new ndBodyDynamic());
        //chassisBody->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh, nullptr));
        chassisBody->SetMatrix(matrix);
        chassisBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
        chassisBody->GetAsBodyDynamic()->SetMassMatrix(chassisMass, **collision);
        
        articulation->AddRootBody(chassisBody);
        
        //// add the motor
        //AddLocomotion(articulation);
        //
        ////add a camera 
        //ndAssert(visualMeshRoot->FindByName("cameraPivot"));
        //ndSharedPtr<ndRenderSceneNode> cameraPivotNode(visualMeshRoot->FindByName("cameraPivot")->GetSharedPtr());

        //ndVector cameraPivot(ndVector::m_zero);
        //cameraPivot.m_y = ndFloat32(2.5f);
        //ndRender* const renderer = *m_scene->GetRenderer();
        //m_cameraNode = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, ND_EXCAVATOR_CAMERA_DISTANCE));
        //cameraPivotNode->AddChild(m_cameraNode);
    }

    void MakeModel(ndDemoEntityManager* const scene, const ndMatrix& location)
    //static ndSharedPtr<ndModelNotify> CreateExcavator(ndDemoEntityManager* const scene, const ndMatrix& location, ndRenderMeshLoader& mesh)
    {
        ndRenderMeshLoader loader(*scene->GetRenderer());
        loader.LoadMesh(ndGetWorkingFileName("excavator.nd"));

        // create a model and set the root transform
        //ndSharedPtr<ndRenderSceneNode> vehicleMesh(mesh.m_renderMesh->Clone());
        //ndSharedPtr<ndRenderSceneNode>MeshToRenderSceneNode(ndGetPath(fbxPathMeshName));
        //ndMatrix matrix(location);
        //matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
        //
        //ndMatrix saveMatrix(mesh.m_mesh->GetMatrix());
        //mesh.m_mesh->SetMatrix(saveMatrix * matrix);
        //
        // using a model articulation for this vehicle
        ndModelArticulation* const excavator = new ndModelArticulation();
        ndSharedPtr<ndModel> vehicleModel(excavator);
        //ndSharedPtr<ndModelNotify> controller(new ndExcavatorController(scene, mesh.m_mesh, vehicleModel, vehicleMesh));
        //vehicleModel->SetNotifyCallback(controller);

        MakeChassis(excavator, loader.m_mesh, loader.m_renderMesh);

        excavator->GetAsModelArticulation()->SaveNdMesh(ndGetWorkingFileName("ndExcavatorPhysics.nd").GetStr());
        
        ndWorld* const world = scene->GetWorld();
        ndSharedPtr<ndModel> testModel(LoadAndBindModel(scene, location, "ndExcavatorPhysics.nd"));
        world->AddModel(testModel);
    }
};

void ndExportModel(ndDemoEntityManager* const scene)
{
    ndSharedPtr<ndBody> floor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

    ndMatrix origin(ndGetIdentityMatrix());
    origin.m_posit.m_y = 3.0f;
    origin.m_posit.m_z = 2.0f;
    //ndBoxTricycle::BoxTricycle(scene, origin, 100.0f, 0.75f);

    origin.m_posit.m_z = 0.0f;
    //ndBasicRagdoll::RagDoll(scene, origin);

    origin.m_posit.m_z = -3.0f;
    //ndDaveRagdoll::RagDoll(scene, origin);

    origin.m_posit.m_x = 10.0f;
    origin.m_posit.m_z = 0.0f;
    ndExcavator::MakeModel(scene, origin);

    ndQuaternion rot;
    origin.m_posit.m_x -= 8.0f;
    origin.m_posit.m_y = 2.0f;
    origin.m_posit.m_z = 0.0f;
    scene->SetCameraMatrix(rot, origin.m_posit);
}
