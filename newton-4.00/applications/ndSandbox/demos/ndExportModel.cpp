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


namespace ndSimpleBoxCar
{
    ndModelArticulation* CreateBoxCarModel(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
    {
        auto MakePrimitive = [scene](const ndMatrix& matrix, const ndShapeInstance& shape, ndFloat32 mass)
        {
            ndSharedPtr<ndBody> body(new ndBodyDynamic());
            body->SetMatrix(matrix);
            body->GetAsBodyDynamic()->SetCollisionShape(shape);
            body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);
            return body;
        };

        ndPhysicsWorld* const world = scene->GetWorld();
        ndRender* const render = *scene->GetRenderer();

        ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(diameter * 2.0f, 0.5f * diameter, 1.5f * diameter)));
        ndRenderPrimitive::ndDescriptor descriptor(render);
        descriptor.m_collision = shape;
        descriptor.m_mapping = ndRenderPrimitive::m_box;
        descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

        ndModelArticulation* const carModel = new ndModelArticulation();

        ndMatrix matrix(ndGetIdentityMatrix());
        matrix.m_posit = FindFloor(*world, origin, 200.0f);
        matrix.m_posit.m_y += diameter * 1.5f;

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
        return carModel;
    }

    void BindApplicationData(
        ndDemoEntityManager* const scene,
        ndMeshLoader& meshLoader,
        ndModelArticulation* const model)
    {
        ndRender* const render = *scene->GetRenderer();

        const ndMesh* const rootMesh = *meshLoader.m_mesh;
        for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
        {
            // find th emesh node
            const ndMesh* const meshNode = rootMesh->FindByClosestMatch(node->m_name);
            ndAssert(meshNode);

            // create a graphic primitive for visualization
            ndSharedPtr<ndMeshShapeInstance> primitive(meshNode->GetPrimitive());
            ndRenderPrimitive::ndDescriptor descriptor(render);
            descriptor.m_collision = ndSharedPtr<ndShapeInstance>(primitive->CreateObject());
            descriptor.m_mapping = ndRenderPrimitive::m_box;
            descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
            ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

            const ndMatrix matrix(node->m_body->GetMatrix());
            ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
            entity->SetPrimitiveMatrix(meshNode->GetGeometryMatrix());
            entity->SetPrimitive(mesh);
            scene->AddEntity(entity);

            // add a rigid body notification callback
            ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, entity));
            ((ndDemoEntityNotify*)*notify)->ResetEntityTransform(matrix);
            node->m_body->SetNotifyCallback(notify);
        }
    }

    void BoxCarModel(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
    {
        ndPhysicsWorld* const world = scene->GetWorld();

        // we first create an articulated model.
        ndSharedPtr<ndModel>model(CreateBoxCarModel(scene, origin, mass, diameter));

        // test if the model is valid
        //world->AddModel(ndSharedPtr<ndModel>(model));

        // we now export the model as a ndMesh
        model->GetAsModelArticulation()->SaveNdMesh(ndGetWorkingFileName("boxCar.nd").GetStr());

        // we now load the ndMesh
        ndMeshLoader loader;
        //ndRenderMeshLoader loader(*scene->GetRenderer());
        loader.LoadMesh(ndGetWorkingFileName("boxCar.nd").GetStr());

        // make an articulated from the loaded mesh
        ndSharedPtr<ndModel> articulation(new ndModelArticulation());
        articulation->GetAsModelArticulation()->Deserialize(*loader.m_mesh);

        // set the matrix location
        ndMatrix matrix(ndGetIdentityMatrix());
        matrix.m_posit.m_y = 2.0f;
        articulation->GetAsModelArticulation()->SetTransform(matrix);

        // Bind application data to the model
        BindApplicationData(scene, loader, articulation->GetAsModelArticulation());

        // add the loaded model to the world
        world->AddModel(articulation);
    }
};

// Material ragdoll : désactive les collisions internes
namespace ndDaveRagdoll
{
    ndModelArticulation* CreateDaveRagdoll(ndDemoEntityManager* const scene)
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
        ndMatrix m_hip_L_matrixLocal(ndRollMatrix(-10.0f * ndDegreeToRad) * ndYawMatrix(90.0f * ndDegreeToRad));
        m_hip_L_matrixLocal.m_posit.m_y = -0.4f;
        m_hip_L_matrixLocal.m_posit.m_z = -0.1f;

        ndMatrix m_cuisse_L_matrixLocal(ndRollMatrix(-80.0f * ndDegreeToRad));
        m_cuisse_L_matrixLocal.m_posit.m_x = 0.45f;

        // right Leg Transforms                    
        ndMatrix m_hip_R_matrixLocal(ndRollMatrix(-10.0f * ndDegreeToRad) * ndYawMatrix(-90.0f * ndDegreeToRad));
        m_hip_R_matrixLocal.m_posit.m_y = -0.4f;
        m_hip_R_matrixLocal.m_posit.m_z =  0.1f;

        ndMatrix m_cuisse_R_matrixLocal(ndRollMatrix(-80.0f * ndDegreeToRad));
        m_cuisse_R_matrixLocal.m_posit.m_x = 0.45f;

        ndMatrix m_tibia_L_matrixLocal(ndGetIdentityMatrix());
        m_tibia_L_matrixLocal.m_front = ndVector(0.99999994f, 0.0f, 0.0f, 0.0f);
        m_tibia_L_matrixLocal.m_up = ndVector(0.0f, 0.99999994f, 0.0f, 0.0f);
        m_tibia_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_tibia_L_matrixLocal.m_posit = ndVector(-1.72500002f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_tibia_R_matrixLocal(ndGetIdentityMatrix());
        m_tibia_R_matrixLocal.m_front = ndVector(0.99999994f, 0.0f, 0.0f, 0.0f);
        m_tibia_R_matrixLocal.m_up = ndVector(0.0f, 0.99999994f, 0.0f, 0.0f);
        m_tibia_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_tibia_R_matrixLocal.m_posit = ndVector(-1.72500002f, 0.0f, 0.0f, 1.0f);

        ndMatrix m_pied_L_matrixLocal(ndGetIdentityMatrix());
        m_pied_L_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_pied_L_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_pied_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_pied_L_matrixLocal.m_posit = ndVector(-0.75f, 0.0f, 0.150000006f, 1.0f);

        ndMatrix m_pied_R_matrixLocal(ndGetIdentityMatrix());
        m_pied_R_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_pied_R_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_pied_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_pied_R_matrixLocal.m_posit = ndVector(-0.75f, 0.0f, 0.150000006f, 1.0f);

        ndMatrix m_orteille_L_matrixLocal(ndGetIdentityMatrix());
        m_orteille_L_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_orteille_L_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_orteille_L_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_orteille_L_matrixLocal.m_posit = ndVector(0.0f, 0.0f, 0.5f, 1.0f);

        ndMatrix m_orteille_R_matrixLocal(ndGetIdentityMatrix());
        m_orteille_R_matrixLocal.m_front = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
        m_orteille_R_matrixLocal.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
        m_orteille_R_matrixLocal.m_right = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
        m_orteille_R_matrixLocal.m_posit = ndVector(0.0f, 0.0f, 0.5f, 1.0f);

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
        
        //ndMatrix tibiaLMatrix = m_tibia_L_matrixLocal * cuisseLMatrix;
        //ndSharedPtr<ndBody> tibiaLBody = CreateCapsule(1.25f, tibiaLMatrix);
        //
        //ndMatrix piedLMatrix = m_pied_L_matrixLocal * tibiaLMatrix;
        //ndSharedPtr<ndBody> piedLBody = CreateBox(0.13f, 0.4f, 0.75f, piedLMatrix);
        //
        //ndMatrix orteilLMatrix = m_orteille_L_matrixLocal * piedLMatrix;
        //ndSharedPtr<ndBody> orteilLBody = CreateBox(0.125f, 0.4f, 0.25f, orteilLMatrix);
        // 
        // right Leg
        ndMatrix hipRMatrix (m_hip_R_matrixLocal * bassinMatrix);
        ndSharedPtr<ndBody> hipRBody (CreateCapsule(hipRMatrix, capsuleRadius, 0.5f));
        
        ndMatrix cuisseRMatrix (m_cuisse_R_matrixLocal * hipRMatrix);
        ndSharedPtr<ndBody> cuisseRBody (CreateCapsule(cuisseRMatrix, capsuleRadius, 1.0f));
        
        //ndMatrix tibiaRMatrix = m_tibia_R_matrixLocal * cuisseRMatrix;
        //ndSharedPtr<ndBody> tibiaRBody = CreateCapsule(1.25f, tibiaRMatrix);
        //
        //ndMatrix piedRMatrix = m_pied_R_matrixLocal * tibiaRMatrix;
        //ndSharedPtr<ndBody> piedRBody = CreateBox(0.13f, 0.4f, 0.75f, piedRMatrix);
        //
        //ndMatrix orteilRMatrix = m_orteille_R_matrixLocal * piedRMatrix;
        //ndSharedPtr<ndBody> orteilRBody = CreateBox(0.125f, 0.4f, 0.25f, orteilRMatrix);

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

        // === Tes joints (je les laisse tels quels, ils étaient déjà bons) ===
        // (tu peux les remettre exactement comme dans ton code original)
        ndJointHinge* joint1 = nullptr;
        ndJointHinge* joint2 = nullptr;
        ndJointHinge* joint3 = nullptr;
        ndJointHinge* joint4 = nullptr;
        ndJointDoubleHinge* joint5 = nullptr;
        ndJointDoubleHinge* joint6 = nullptr;
        ndJointHinge* joint7 = nullptr;
        ndJointHinge* joint8 = nullptr;
        ndJointHinge* joint9 = nullptr;
        ndJointHinge* joint10 = nullptr;
        ndJointHinge* joint11 = nullptr;
        ndJointHinge* joint12 = nullptr;
        ndJointHinge* joint13 = nullptr;
        ndJointHinge* joint14 = nullptr;
        ndJointDoubleHinge* joint15 = nullptr;
        ndJointDoubleHinge* joint16 = nullptr;
        ndJointHinge* joint17 = nullptr;
        ndJointHinge* joint18 = nullptr;
        ndJointHinge* joint19 = nullptr;
        ndJointHinge* joint20 = nullptr;
        
        ndModelArticulation::ndNode* nextRootTemp1 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp2 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp3 = nullptr;
        ndModelArticulation::ndNode* nextRootTemp = nullptr;

        ndModelArticulation* const model = new ndModelArticulation();
        ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(bassinBody);

        { // colonne
            const ndMatrix tmp1(ndYawMatrix(-90.0f * ndDegreeToRad) * colonneBody->GetMatrix());
     
            joint1 = new ndJointHinge(tmp1, colonneBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint1->SetLimitState(true);
            joint1->SetLimits(-25.0f * ndDegreeToRad, 0.1f * ndDegreeToRad);
            joint1->SetAsSpringDamper(0.01f, 10.0f, 0.5f);
            
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint1);
            nextRootTemp = model->AddLimb(modelRootNode, colonneBody, jointPtr);
            nextRootTemp3 = nextRootTemp;
        }

        { // head
            const ndMatrix tmp1(ndYawMatrix(-90.0f * ndDegreeToRad) * headBody->GetMatrix());
        
            joint2 = new ndJointHinge(tmp1, headBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            joint2->SetLimitState(true);
            joint2->SetLimits(-120.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint2->SetAsSpringDamper(0.01f, 25.0f, 0.5f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint2);
            nextRootTemp = model->AddLimb(nextRootTemp, headBody, jointPtr);
        }

        // right arm
        { // epaule_R
            ndMatrix tmp1(epauleRBody->GetMatrix());

            joint14 = new ndJointHinge(tmp1, epauleRBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            joint14->SetLimitState(true);
            joint14->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint14->SetAsSpringDamper(0.01f, 10.0f, 0.5f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint14);
            nextRootTemp2 = model->AddLimb(nextRootTemp3, epauleRBody, jointPtr);
        }

        { // bras_R
            ndMatrix tmp1(brasRBody->GetMatrix());
            //tmp1.m_up = tmp1.m_right;
            //tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            joint16 = new ndJointDoubleHinge(tmp1, brasRBody->GetAsBodyKinematic(), epauleRBody->GetAsBodyKinematic());
            joint16->SetLimitState0(true);
            joint16->SetLimits0(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint16->SetAsSpringDamper0(0.005f, 50.0f, 10.0f);

            joint16->SetLimitState1(true);
            joint16->SetLimits1(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint16->SetAsSpringDamper1(0.005f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint16);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, brasRBody, jointPtr);
        }

        { // avantbras_R arm
            ndMatrix tmp1(avantbrasRBody->GetMatrix());
            tmp1.m_front = tmp1.m_right.Scale(1.0f);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            joint18 = new ndJointHinge(tmp1, avantbrasRBody->GetAsBodyKinematic(), brasRBody->GetAsBodyKinematic());
            joint18->SetLimitState(true);
            joint18->SetLimits(-120.0f * ndDegreeToRad, 5.0f * ndDegreeToRad);
            joint18->SetAsSpringDamper(0.1f, 0.0f, 5.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint18);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, avantbrasRBody, jointPtr);
        }

        { // hand_R
            ndMatrix tmp1(handRBody->GetMatrix());

            joint20 = new ndJointHinge(tmp1, handRBody->GetAsBodyKinematic(), avantbrasRBody->GetAsBodyKinematic());
            joint20->SetLimitState(true);
            joint20->SetLimits(-45.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint20);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, handRBody, jointPtr);
        }

        // left arm
        { // epaule_L
            ndMatrix tmp1(epauleLBody->GetMatrix());

            joint13 = new ndJointHinge(tmp1, epauleLBody->GetAsBodyKinematic(), colonneBody->GetAsBodyKinematic());
            joint13->SetLimitState(true);
            joint13->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint13->SetAsSpringDamper(0.01f, 10.0f, 0.5f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint13);
            nextRootTemp1 = model->AddLimb(nextRootTemp3, epauleLBody, jointPtr);
        }

        { // bras_L
            ndMatrix tmp1(brasLBody->GetMatrix());
            //tmp1.m_up = tmp1.m_right.Scale(-1.0f);
            //tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            joint15 = new ndJointDoubleHinge(tmp1, brasLBody->GetAsBodyKinematic(), epauleLBody->GetAsBodyKinematic());
            joint15->SetLimitState0(true);
            joint15->SetLimits0(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint15->SetAsSpringDamper0(0.005f, 50.0f, 10.0f);
            
            joint15->SetLimitState1(true);
            joint15->SetLimits1(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint15->SetAsSpringDamper1(0.005f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint15);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, brasLBody, jointPtr);
        }

        { // avantbras_L = arm
            ndMatrix tmp1(avantbrasLBody->GetMatrix());
            tmp1.m_front = tmp1.m_right.Scale(-1.0f);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            joint17 = new ndJointHinge(tmp1, avantbrasLBody->GetAsBodyKinematic(), brasLBody->GetAsBodyKinematic());
            joint17->SetLimitState(true);
            joint17->SetLimits(-5.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint17->SetAsSpringDamper(0.1f, 0.0f, 5.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint17);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, avantbrasLBody, jointPtr);
        }

        { // hand_L
            ndMatrix tmp1(handLBody->GetMatrix());
            joint19 = new ndJointHinge(tmp1, handLBody->GetAsBodyKinematic(), avantbrasLBody->GetAsBodyKinematic());
            joint19->SetLimitState(true);
            joint19->SetLimits(-45.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint19);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, handLBody, jointPtr);
        }

        // left leg
        { // hip_L
            ndMatrix tmp1(hipLBody->GetMatrix());

            joint3 = new ndJointHinge(tmp1, hipLBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint3->SetLimitState(true);
            joint3->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint3->SetAsSpringDamper(0.01f, 25.0f, 1.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint3);
            nextRootTemp1 = model->AddLimb(modelRootNode, hipLBody, jointPtr);
        }

        { // cuisse_L
            ndMatrix tmp1(cuisseLBody->GetMatrix());
            tmp1.m_up = tmp1.m_right;
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            joint5 = new ndJointDoubleHinge(tmp1, cuisseLBody->GetAsBodyKinematic(), hipLBody->GetAsBodyKinematic());
            joint5->SetLimitState0(true);
            joint5->SetLimits0(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint5->SetAsSpringDamper0(0.005f, 50.0f, 10.0f);

            joint5->SetLimitState1(true);
            joint5->SetLimits1(-70.0f * ndDegreeToRad, 10.0f * ndDegreeToRad);
            joint5->SetAsSpringDamper1(0.005f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint5);
            nextRootTemp1 = model->AddLimb(nextRootTemp1, cuisseLBody, jointPtr);
        }

        // right leg
        { // hip_R
            ndMatrix tmp1(hipRBody->GetMatrix());
            
            joint4 = new ndJointHinge(tmp1, hipRBody->GetAsBodyKinematic(), bassinBody->GetAsBodyKinematic());
            joint4->SetLimitState(true);
            joint4->SetLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
            joint4->SetAsSpringDamper(0.01f, 25.0f, 1.0f);
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint4);
            nextRootTemp2 = model->AddLimb(modelRootNode, hipRBody, jointPtr);
        }

        { // cuisse_R
            ndMatrix tmp1(cuisseRBody->GetMatrix());
            tmp1.m_up = tmp1.m_right.Scale (-1.0f);
            tmp1.m_right = tmp1.m_front.CrossProduct(tmp1.m_up);

            joint6 = new ndJointDoubleHinge(tmp1, cuisseRBody->GetAsBodyKinematic(), hipRBody->GetAsBodyKinematic());
            joint6->SetLimitState0(true);
            joint6->SetLimits0(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
            joint6->SetAsSpringDamper0(0.005f, 50.0f, 10.0f);

            joint6->SetLimitState1(true);
            joint6->SetLimits1(-70.0f * ndDegreeToRad, 10.0f * ndDegreeToRad);
            joint6->SetAsSpringDamper1(0.005f, 50.0f, 10.0f);

            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint6);
            nextRootTemp2 = model->AddLimb(nextRootTemp2, cuisseRBody, jointPtr);
        }
#if 0
        { // tibia_L
            ndMatrix tmp1(tibiaLBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.7f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint7 = new ndJointHinge(tmp1, tibiaLBody->GetAsBodyKinematic(), cuisseLBody->GetAsBodyKinematic());
            joint7->SetLimitState(true);
            joint7->SetLimits(-165.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint7->SetAsSpringDamper(0.01f, 2.5f, 0.25f);
            //
            //m_jointlist.push_back(joint7);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint7);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, tibiaLBody, jointPtr);
        }

        { // tibia_R
            ndMatrix tmp1(tibiaRBody->GetMatrix());
            tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1] + 0.7f, tmp1[3][2], 1.0f); // offset
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;

            joint8 = new ndJointHinge(tmp1, tibiaRBody->GetAsBodyKinematic(), cuisseRBody->GetAsBodyKinematic());
            joint8->SetLimitState(true);
            joint8->SetLimits(-165.0f * ndDegreeToRad, 2.0f * ndDegreeToRad);
            joint8->SetAsSpringDamper(0.01f, 2.5f, 0.25f);
            //
            //m_jointlist.push_back(joint8);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint8);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, tibiaRBody, jointPtr);
        }

        { // pied_L
            ndMatrix tmp1(piedLBody->GetMatrix());
            // Dave, this offset not right
            //tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.15f, 1.0f); // offset
            tmp1[3].m_z += 0.15f;
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;

            joint9 = new ndJointHinge(tmp1, piedLBody->GetAsBodyKinematic(), tibiaLBody->GetAsBodyKinematic());
            joint9->SetLimitState(true);

            // Dave, allow the fee to rotate fond and back
            //joint9->SetLimits(-2.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            joint9->SetLimits(-30.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            joint9->SetAsSpringDamper(0.01f, 25.0f, 1.0f);

            //m_jointlist.push_back(joint9);
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint9);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, piedLBody, jointPtr);
        }

        { // pied_R
            ndMatrix tmp1(piedRBody->GetMatrix());
            // Dave, this offset not right
            //tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.15f, 1.0f); // offset
            tmp1[3].m_z += 0.15f;
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;

            // Dave, flip the matrix cause the feel to rotate back
            //tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;

            joint10 = new ndJointHinge(tmp1, piedRBody->GetAsBodyKinematic(), tibiaRBody->GetAsBodyKinematic());
            joint10->SetLimitState(true);

            // Dave, allow the fee to rotate fond and back
            //joint10->SetLimits(-2.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            joint10->SetLimits(-30.0f * ndDegreeToRad, 65.0f * ndDegreeToRad);
            joint10->SetAsSpringDamper(0.01f, 25.0f, 1.0f);

            //m_jointlist.push_back(joint10);
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint10);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, piedRBody, jointPtr);
        }

        { // orteille_R
            ndMatrix tmp1(orteilRBody->GetMatrix());
            //tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.125f, 1.0f); // offset
            tmp1[3].m_z += 0.1f;
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //tmp1 = ndYawMatrix(-180.0f * ndDegreeToRad) * tmp1;

            joint12 = new ndJointHinge(tmp1, orteilRBody->GetAsBodyKinematic(), piedRBody->GetAsBodyKinematic());
            joint12->SetLimitState(true);
            joint12->SetLimits(-15.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
            joint12->SetAsSpringDamper(0.01f, 100.0f, 5.0f);
            //
            //m_jointlist.push_back(joint12);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint12);
            nextRootTemp2 = m_model->AddLimb(nextRootTemp2, orteilRBody, jointPtr);
        }

        { // orteille_L
            ndMatrix tmp1(orteilLBody->GetMatrix());
            //tmp1[3] = ndVector(tmp1[3][0], tmp1[3][1], tmp1[3][2] - 0.125f, 1.0f); // offset
            tmp1[3].m_z += 0.1f;
            tmp1 = ndRollMatrix(90.0f * ndDegreeToRad) * tmp1;
            //
            joint11 = new ndJointHinge(tmp1, orteilLBody->GetAsBodyKinematic(), piedLBody->GetAsBodyKinematic());
            joint11->SetLimitState(true);
            joint11->SetLimits(-15.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
            joint11->SetAsSpringDamper(0.01f, 100.0f, 5.0f);
            //
            //m_jointlist.push_back(joint11);
            //
            ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint11);
            nextRootTemp1 = m_model->AddLimb(nextRootTemp1, orteilLBody, jointPtr);
        }
#endif
        return model;
    }

    void BindApplicationData(
        ndDemoEntityManager* const scene,
        ndMeshLoader& meshLoader,
        ndModelArticulation* const model)
    {
        ndRender* const render = *scene->GetRenderer();

        const ndMesh* const rootMesh = *meshLoader.m_mesh;
        for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
        {
            // find the mesh node
            const ndMesh* const meshNode = rootMesh->FindByClosestMatch(node->m_name);
            ndAssert(meshNode);

            // create a graphic primitive for visualization
            ndSharedPtr<ndMeshShapeInstance> primitive(meshNode->GetPrimitive());
            ndRenderPrimitive::ndDescriptor descriptor(render);
            descriptor.m_collision = ndSharedPtr<ndShapeInstance>(primitive->CreateObject());
            descriptor.m_mapping = ndRenderPrimitive::m_box;
            descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
            ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

            const ndMatrix matrix(node->m_body->GetMatrix());
            ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
            entity->SetPrimitiveMatrix(meshNode->GetGeometryMatrix());
            entity->SetPrimitive(mesh);
            scene->AddEntity(entity);

            // add a rigid body notification callback
            ndSharedPtr<ndBodyNotify> notify(new ndDemoEntityNotify(scene, entity));
            ((ndDemoEntityNotify*)*notify)->ResetEntityTransform(matrix);
            node->m_body->SetNotifyCallback(notify);
        }
    }

    void RagDoll(ndDemoEntityManager* const scene, const ndVector& origin)
    {
        ndPhysicsWorld* world = scene->GetWorld();

        // make a physic rag doll for using as template
        ndSharedPtr<ndModel> model (CreateDaveRagdoll(scene));

        // we now export the model as a ndMesh
        model->GetAsModelArticulation()->SaveNdMesh(ndGetWorkingFileName("daveRagdoll.nd").GetStr());

        ndMeshLoader loader;
        //ndRenderMeshLoader loader(*scene->GetRenderer());
        loader.LoadMesh(ndGetWorkingFileName("daveRagdoll.nd").GetStr());

        // make an articulated from the loaded mesh
        ndSharedPtr<ndModel> articulation(new ndModelArticulation());
        articulation->GetAsModelArticulation()->Deserialize(*loader.m_mesh);

        // set the matrix location
        ndMatrix matrix(ndGetIdentityMatrix());
        matrix.m_posit = origin;
        articulation->GetAsModelArticulation()->SetTransform(matrix);

        // Bind application data to the model
        BindApplicationData(scene, loader, articulation->GetAsModelArticulation());

        // add the loaded model to the world
        world->AddModel(articulation);
    }
};

void ndExportModel(ndDemoEntityManager* const scene)
{
    ndSharedPtr<ndBody> floor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

    ndVector origin(ndVector::m_wOne);
    origin.m_y = 2.0f;
    //ndSimpleBoxCar::BoxCarModel(scene, origin, 100.0f, 0.75f);

    origin.m_x += 3.0f;
    ndDaveRagdoll::RagDoll(scene, origin);

    ndQuaternion rot;
    origin.m_x -= 5.0f;
    origin.m_y = 2.0f;
    scene->SetCameraMatrix(rot, origin);
}
