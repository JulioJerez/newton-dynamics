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
    ndModelArticulation* const articulation = model->GetAsModelArticulation();
    const ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();
    const ndMatrix matrix(rootNode ? rootNode->m_body->GetMatrix() * location : location);
    sceneMesh->SetTransform(matrix);
    sceneMesh->SetTransform(matrix);
    model->GetAsModelArticulation()->SetTransform(matrix);

    // Bind application data to the model, 
    // this could be a render mesh or something else. 
    // For this demo it is ndRenderSceneNode mesh
    const ndMesh* const rootMesh = *loader.m_mesh;
    auto BindApplicationData = [scene, articulation, rootNode, rootMesh, &sceneMesh](ndModelArticulation::ndNode* const node)
    {
        if (!articulation->IsCloseLoop(node))
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
        }
    };
    articulation->NodeIterator(BindApplicationData);

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

    ndSharedPtr<ndBody> MakeBodyPart(ndMesh* const childMesh, ndSharedPtr<ndShapeInstance> collision, ndFloat32 mass)
    {
        // calculate matrix
        const ndMatrix matrix(childMesh->CalculateGlobalMatrix());

        // find the visual mesh
        ndSharedPtr<ndBody> body(new ndBodyDynamic());
        body->SetMatrix(matrix);
        body->GetAsBodyDynamic()->SetCollisionShape(**collision);
        body->GetAsBodyDynamic()->SetMassMatrix(mass, **collision);
        return body;
    }

    ndSharedPtr<ndBody> MakeBodyPart(ndSharedPtr<ndMesh>& mesh, const char* const name, ndFloat32 mass)
    {
        ndMesh* const meshNode = mesh->FindByName(name);
        ndAssert(meshNode);

        // build collision mesh
        ndSharedPtr<ndShapeInstance> collision(meshNode->CreateCollisionFromChildren());
        return MakeBodyPart(meshNode, collision, mass);
    }

    void AddEngine(ndModelArticulation* const articulation)
    {
        ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

        ndSharedPtr<ndShapeInstance> motorCollision(new ndShapeInstance(new ndShapeCylinder(ndFloat32(0.25f), ndFloat32(0.25f), ndFloat32(0.75f))));
        //SetBodyType(*motorCollision, m_chassis);

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

        ndModelArticulation::ndNode* const engineNode = articulation->AddLimb(rootNode, motorBody, joint);
        engineNode->m_name = "engine";
    }

    void MakeChassis(ndModelArticulation* const articulation, ndSharedPtr<ndMesh>& mesh)
    {
        ndMesh* const chassisMesh = mesh->FindByName("base");
        ndAssert(chassisMesh);
        
        // create the collision and the world matrix
        ndMatrix matrix(chassisMesh->CalculateGlobalMatrix());
        ndSharedPtr<ndShapeInstance> collision(chassisMesh->CreateCollisionFromChildren());
        
        ndFloat32 chassisMass = ndFloat32(4000.0f);
        
        // create the rigid that represent the chassis
        ndSharedPtr<ndBody> chassisBody(new ndBodyDynamic());
        chassisBody->SetMatrix(matrix);
        chassisBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
        chassisBody->GetAsBodyDynamic()->SetMassMatrix(chassisMass, **collision);
        
        ndModelArticulation::ndNode* const rootNode = articulation->AddRootBody(chassisBody);
        rootNode->m_name = chassisMesh->GetName();
        
        // add the motor
        AddEngine(articulation);
    }

    void MakeCabinAndUpperBody(ndModelArticulation* const articulation, ndSharedPtr<ndMesh>& mesh)
    {
        ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

        ndSharedPtr<ndBody> cabinBody(MakeBodyPart(mesh, "EngineBody", 400.0f));

        // set the center of mass of engine
        const ndMatrix hingeFrame(cabinBody->GetMatrix());
        ndVector com(hingeFrame.m_front.Scale(ndFloat32(5.0f)));
        cabinBody->SetCentreOfMass(com);

        ndSharedPtr<ndJointBilateralConstraint> cabinPivot(new ndJointHinge(hingeFrame, cabinBody->GetAsBodyDynamic(), rootNode->m_body->GetAsBodyDynamic()));
        ndModelArticulation::ndNode* const cabinNode = articulation->AddLimb(rootNode, cabinBody, cabinPivot);
        cabinNode->m_name = "EngineBody";

        // add arm0.
        ndSharedPtr<ndBody> bodyArm0(MakeBodyPart(mesh, "Boom", 50.0f));
        const ndMatrix matrixArm0(ndYawMatrix(ndFloat32(90.0f) * ndDegreeToRad) * bodyArm0->GetMatrix());
        ndSharedPtr<ndJointBilateralConstraint> jointArm0(new ndJointHinge(matrixArm0, bodyArm0->GetAsBodyDynamic(), cabinBody->GetAsBodyDynamic()));
        ndModelArticulation::ndNode* const armNode0 = articulation->AddLimb(cabinNode, bodyArm0, jointArm0);
        armNode0->m_name = "Boom";
        
        // add arm1.
        ndSharedPtr<ndBody> bodyArm1(MakeBodyPart(mesh, "arm02", 50.0f));
        const ndMatrix matrixArm1(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * bodyArm1->GetMatrix());
        ndSharedPtr<ndJointBilateralConstraint> jointArm1(new ndJointHinge(matrixArm1, bodyArm1->GetAsBodyDynamic(), bodyArm0->GetAsBodyDynamic()));
        ndModelArticulation::ndNode* const armNode1 = articulation->AddLimb(armNode0, bodyArm1, jointArm1);
        armNode1->m_name = "arm02";
        
        // add bucket
        ndSharedPtr<ndBody> bodyBucket(MakeBodyPart(mesh, "bucket", 40.0f));
        const ndMatrix matrixBucket(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * bodyBucket->GetMatrix());
        ndSharedPtr<ndJointBilateralConstraint> jointBucket(new ndJointHinge(matrixBucket, bodyBucket->GetAsBodyDynamic(), bodyArm1->GetAsBodyDynamic()));
        ndModelArticulation::ndNode* const bucket = articulation->AddLimb(armNode1, bodyBucket, jointBucket);
        bucket->m_name = "bucket";
        ((ndJointHinge*)*jointBucket)->SetAsSpringDamper(ndFloat32(0.1f), ndFloat32(2000.0f), ndFloat32(50.0f));
        
        // add an effector to move the arm
        ndMatrix baseFrame(ndGetIdentityMatrix());
        baseFrame.m_posit = hingeFrame.m_posit;
        
        ndIkSwivelPositionEffector* const armEffector = new ndIkSwivelPositionEffector(
            baseFrame, rootNode->m_body->GetAsBodyDynamic(),
            matrixBucket.m_posit, bodyArm1->GetAsBodyDynamic());
        armEffector->SetSwivelMode(false);
        ndSharedPtr<ndJointBilateralConstraint> effector(armEffector);
        articulation->AddCloseLoop(effector, "buckEffector");
        
        // calculate the work space.
        ndVector arm1Len(jointBucket->CalculateGlobalMatrix0().m_posit - jointArm1->CalculateGlobalMatrix0().m_posit);
        ndVector arm0Len(jointArm1->CalculateGlobalMatrix0().m_posit - jointArm0->CalculateGlobalMatrix0().m_posit);
        ndVector cabinLen(jointArm0->CalculateGlobalMatrix0().m_posit - cabinPivot->CalculateGlobalMatrix0().m_posit);
        ndFloat32 l1 = ndSqrt(arm1Len.DotProduct(arm1Len).GetScalar());
        ndFloat32 l0 = ndSqrt(arm0Len.DotProduct(arm0Len).GetScalar()) + ndSqrt(cabinLen.DotProduct(cabinLen).GetScalar());
        ndFloat32 minRadios = (l0 - l1) * ndFloat32(1.1f);
        ndFloat32 maxRadios = (l0 + l1) * ndFloat32(0.9f);
        armEffector->SetWorkSpaceConstraints(minRadios, maxRadios);
        
        // calculate arm parameters		
        //ndVector localPosit(armEffector->GetLocalTargetPosition());
        //m_armAngle = ndFloat32(0.0f);
        //m_armPosit_y = localPosit.m_y;
        //m_armAngle0 = ndAtan2(localPosit.m_z, localPosit.m_x);
        //m_armPosit_x = ndSqrt(localPosit.m_x * localPosit.m_x + localPosit.m_z * localPosit.m_z);
    }

    void MakeModel(ndDemoEntityManager* const scene, const ndMatrix& location)
    {
        ndRenderMeshLoader loader(*scene->GetRenderer());
        loader.LoadMesh(ndGetWorkingFileName("excavator.nd"));

loader.m_mesh->SetMatrix(ndGetIdentityMatrix());

        // using a model articulation for this vehicle
        ndModelArticulation* const excavator = new ndModelArticulation();
        ndSharedPtr<ndModel> vehicleModel(excavator);

        // for more readability break construction into sub function
        MakeChassis(excavator, loader.m_mesh);

        // add the cabin and boom mechanism
        MakeCabinAndUpperBody(excavator, loader.m_mesh);

        excavator->GetAsModelArticulation()->Serialize(*loader.m_mesh);
        loader.SaveMesh(ndGetWorkingFileName("ndExcavatorPhysics.nd"));
        
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
