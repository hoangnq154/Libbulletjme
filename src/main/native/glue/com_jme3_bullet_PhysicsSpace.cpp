/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Normen Hansen
 */
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "com_jme3_bullet_PhysicsSpace.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "jmeUserInfo.h"
#include <vector>

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getWorldType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getWorldType
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);

    btDynamicsWorldType type = pWorld->getWorldType();
    return (jint) type;
}

struct RecoverResult {
	bool hasPenetration;
	btVector3 normal;
	btVector3 pointWorld;
	btScalar penetration_distance; // Negative mean penetration
	int other_compound_shape_index;
	const btCollisionObject *other_collision_object;
	int local_shape_most_recovered;

	RecoverResult() :
		hasPenetration(false),
		normal(0, 0, 0),
		pointWorld(0, 0, 0),
		penetration_distance(1e20),
		other_compound_shape_index(0),
		other_collision_object(nullptr),
		local_shape_most_recovered(0) {}
};


struct GodotDeepPenetrationContactResultCallback : public btManifoldResult {
	btVector3 m_pointNormalWorld;
	btVector3 m_pointWorld;
	btScalar m_penetration_distance;
	int m_other_compound_shape_index;

	GodotDeepPenetrationContactResultCallback(const btCollisionObjectWrapper *body0Wrap, const btCollisionObjectWrapper *body1Wrap) :
			btManifoldResult(body0Wrap, body1Wrap),
			m_penetration_distance(0),
			m_other_compound_shape_index(0) {}

	void reset() {
		m_penetration_distance = 0;
	}

	bool hasHit() {
		return m_penetration_distance < 0;
	}

	virtual void addContactPoint(const btVector3 &normalOnBInWorld, const btVector3 &pointInWorldOnB, btScalar depth){
        if (m_penetration_distance > depth) { // Has penetration?

                const bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
                m_penetration_distance = depth;
                m_other_compound_shape_index = isSwapped ? m_index0 : m_index1;
                m_pointWorld = isSwapped ? (pointInWorldOnB + (normalOnBInWorld * depth)) : pointInWorldOnB;

                m_pointNormalWorld = isSwapped ? normalOnBInWorld * -1 : normalOnBInWorld;
            }
	}
};

RecoverResult *recoverResult = nullptr;
btVoronoiSimplexSolver *gjk_simplex_solver = new btVoronoiSimplexSolver();
btGjkEpaPenetrationDepthSolver *gjk_epa_pen_solver = new btGjkEpaPenetrationDepthSolver();

bool RFP_convex_convex_test(const btConvexShape * p_shapeA, const btConvexShape * p_shapeB, btCollisionObject * p_objectB, int p_shapeId_A, int p_shapeId_B, const btTransform & p_transformA, const btTransform & p_transformB, btScalar p_recover_movement_scale, btVector3 & r_delta_recover_movement, RecoverResult * r_recover_result)
{
	// Initialize GJK input
	btGjkPairDetector::ClosestPointInput gjk_input;
	gjk_input.m_transformA = p_transformA;
	// Avoid repeat penetrations
	gjk_input.m_transformA.getOrigin() += r_delta_recover_movement;
	gjk_input.m_transformB = p_transformB;

	// Perform GJK test
	btPointCollector result;
	btGjkPairDetector gjk_pair_detector(p_shapeA, p_shapeB, gjk_simplex_solver, gjk_epa_pen_solver);
	gjk_pair_detector.getClosestPoints(gjk_input, result, nullptr);
	if (0 > result.m_distance) {
		// Has penetration
		r_delta_recover_movement += result.m_normalOnBInWorld * (result.m_distance * -1 * p_recover_movement_scale);

		if (r_recover_result) {
			if (result.m_distance < r_recover_result->penetration_distance) {
				r_recover_result->hasPenetration = true;
				r_recover_result->local_shape_most_recovered = p_shapeId_A;
				r_recover_result->other_collision_object = p_objectB;
				r_recover_result->other_compound_shape_index = p_shapeId_B;
				r_recover_result->penetration_distance = result.m_distance;
				r_recover_result->pointWorld = result.m_pointInWorld;
				r_recover_result->normal = result.m_normalOnBInWorld;
			}
		}
		return true;
	}
	return false;
}

bool RFP_convex_world_test(jmePhysicsSpace* space, const btConvexShape * p_shapeA, const btCollisionShape * p_shapeB, btCollisionObject * p_objectA, btCollisionObject * p_objectB, int p_shapeId_A, int p_shapeId_B, const btTransform & p_transformA, const btTransform & p_transformB, btScalar p_recover_movement_scale, btVector3 & r_delta_recover_movement, RecoverResult * r_recover_result)
{

    btDiscreteDynamicsWorld *dynamicsWorld = space->getDynamicsWorld();
    btDispatcher *dispatcher = dynamicsWorld->getDispatcher();
	/// Contact test

	btTransform tA(p_transformA);
	// Avoid repeat penetrations
	tA.getOrigin() += r_delta_recover_movement;

	btCollisionObjectWrapper obA(nullptr, p_shapeA, p_objectA, tA, -1, p_shapeId_A);
	btCollisionObjectWrapper obB(nullptr, p_shapeB, p_objectB, p_transformB, -1, p_shapeId_B);

	btCollisionAlgorithm *algorithm = dispatcher->findAlgorithm(&obA, &obB, nullptr, BT_CONTACT_POINT_ALGORITHMS);
	if (algorithm) {
		GodotDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
		//discrete collision detection query
		algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);

		algorithm->~btCollisionAlgorithm();
		dispatcher->freeCollisionAlgorithm(algorithm);

		if (contactPointResult.hasHit()) {
			r_delta_recover_movement += contactPointResult.m_pointNormalWorld * (contactPointResult.m_penetration_distance * -1 * p_recover_movement_scale);
			if (r_recover_result) {
				if (contactPointResult.m_penetration_distance < r_recover_result->penetration_distance) {
					r_recover_result->hasPenetration = true;
					r_recover_result->local_shape_most_recovered = p_shapeId_A;
					r_recover_result->other_collision_object = p_objectB;
					r_recover_result->other_compound_shape_index = p_shapeId_B;
					r_recover_result->penetration_distance = contactPointResult.m_penetration_distance;
					r_recover_result->pointWorld = contactPointResult.m_pointWorld;
					r_recover_result->normal = contactPointResult.m_pointNormalWorld;
				}
			}
			return true;
		}
	}
	return false;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    RFPConvexConvexTest
 * Signature: (JJJJIILcom/jme3/math/Transform;Lcom/jme3/math/Transform;FLcom/jme3/math/Vector3f;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_PhysicsSpace_RFPConvexConvexTest
  (JNIEnv *pEnv, jclass, jlong spaceId, jlong p_shapeA, jlong p_shapeB, jlong p_objectB, jint p_shapeId_A, jint p_shapeId_B, jobject p_transformA_jni, jobject p_transformB_jni, jfloat p_recover_movement_scale, jobject r_delta_recover_movement_jni){

    if(recoverResult)
        delete recoverResult;
    recoverResult = new RecoverResult();

    jmePhysicsSpace * const
                pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",JNI_FALSE)

    btTransform p_transformA,p_transformB;btVector3 inScale;
    btVector3 r_delta_recover_movement;
    jmeBulletUtil::convert(pEnv,p_transformA_jni,&p_transformA,&inScale);
    jmeBulletUtil::convert(pEnv,p_transformB_jni,&p_transformB,&inScale);
    jmeBulletUtil::convert(pEnv,r_delta_recover_movement_jni,&r_delta_recover_movement);

    return RFP_convex_convex_test(reinterpret_cast<btConvexShape *>(p_shapeA),reinterpret_cast<btConvexShape *>(p_shapeB),
    reinterpret_cast<btCollisionObject *>(p_objectB), p_shapeId_A,p_shapeId_B, p_transformA, p_transformB,p_recover_movement_scale,r_delta_recover_movement ,recoverResult);

//    return false;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    RFPConvexWorldTest
 * Signature: (JJJJIILcom/jme3/math/Transform;Lcom/jme3/math/Transform;FLcom/jme3/math/Vector3f;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_PhysicsSpace_RFPConvexWorldTest
  (JNIEnv *pEnv, jclass, jlong spaceId,jlong p_shapeA, jlong p_shapeB, jlong p_objectA, jlong p_objectB, jint p_shapeId_A, jint p_shapeId_B, jobject p_transformA_jni, jobject p_transformB_jni , jfloat p_recover_movement_scale, jobject r_delta_recover_movement_jni){

    if(recoverResult)
            delete recoverResult;
    recoverResult = new RecoverResult();

    jmePhysicsSpace * const
                    pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
        NULL_CHK(pEnv, pSpace, "The physics space does not exist.",JNI_FALSE)

    btTransform p_transformA,p_transformB;btVector3 inScale;
    btVector3 r_delta_recover_movement;
    jmeBulletUtil::convert(pEnv,p_transformA_jni,&p_transformA,&inScale);
    jmeBulletUtil::convert(pEnv,p_transformB_jni,&p_transformB,&inScale);
    jmeBulletUtil::convert(pEnv,r_delta_recover_movement_jni,&r_delta_recover_movement);

    return RFP_convex_world_test(pSpace,reinterpret_cast<btConvexShape *>(p_shapeA), reinterpret_cast<btCollisionShape *>(p_shapeB),
    reinterpret_cast<btCollisionObject *>(p_objectA),reinterpret_cast<btCollisionObject *>(p_objectB),p_shapeId_A,p_shapeId_B, p_transformA, p_transformB,p_recover_movement_scale,r_delta_recover_movement ,recoverResult);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultHasPenetration
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultHasPenetration
  (JNIEnv *pEnv, jclass, jlong){
    return recoverResult->hasPenetration;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultNormal
 * Signature: (J)Lcom/jme3/math/Vector3f;
 */
JNIEXPORT void  JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultNormal
  (JNIEnv *pEnv, jclass, jlong spaceId, jobject storeVector){
    jmeBulletUtil::convert(pEnv, &recoverResult->normal, storeVector);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultPointWorld
 * Signature: (J)Lcom/jme3/math/Vector3f;
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultPointWorld
  (JNIEnv *pEnv, jclass, jlong spaceId, jobject storeVector){
        jmeBulletUtil::convert(pEnv, &recoverResult->pointWorld, storeVector);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultPenetrationDistance
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultPenetrationDistance
  (JNIEnv *, jclass, jlong){
    return (jfloat)recoverResult->penetration_distance;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultOtherCompoundShapeIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultOtherCompoundShapeIndex
  (JNIEnv *, jclass, jlong){
    return (jint)recoverResult->other_compound_shape_index;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultOtherCollisionObject
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultOtherCollisionObject
  (JNIEnv *, jclass, jlong){
    return (jlong)(recoverResult->other_collision_object);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getRecoverResultLocalShapeMostRecovered
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getRecoverResultLocalShapeMostRecovered
  (JNIEnv *, jclass, jlong){
    return (jint)recoverResult->local_shape_most_recovered;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addAction
(JNIEnv *pEnv, jclass, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHK(pEnv, pAction, "The action object does not exist.",)

    pWorld->addAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addCharacterObject
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    ASSERT_CHK(pEnv, pUser->m_jmeSpace == NULL,);
    pUser->m_jmeSpace = pSpace;

    pWorld->addCollisionObject(pCollisionObject,
            btBroadphaseProxy::CharacterFilter,
            btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addConstraintC
 * Signature: (JJZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addConstraintC
(JNIEnv *pEnv, jclass, jlong spaceId, jlong constraintId,
        jboolean disableCollisions) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",)
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    pWorld->addConstraint(pConstraint, bool(disableCollisions));
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addRigidBody
 * Signature: (JJII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addRigidBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong rigidBodyId, jint proxyGroup,
        jint proxyMask) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    jmeUserPointer const pUser = (jmeUserPointer) pBody->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    ASSERT_CHK(pEnv, pUser->m_jmeSpace == NULL,);
    pUser->m_jmeSpace = pSpace;

    pWorld->addRigidBody(pBody, proxyGroup, proxyMask);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    countManifolds
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_countManifolds
(JNIEnv *pEnv, jclass, jlong spaceId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);
    const btDispatcher * const pDispatcher = pWorld->getDispatcher();
    NULL_CHK(pEnv, pDispatcher, "The dispatcher does not exist.", 0);

    int result = pDispatcher->getNumManifolds();
    return result;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    createPhysicsSpace
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;II)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_createPhysicsSpace
(JNIEnv *pEnv, jobject object, jobject minVector, jobject maxVector,
        jint broadphaseType, jint numSolvers) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, minVector, "The min vector does not exist.", 0)
    btVector3 min;
    jmeBulletUtil::convert(pEnv, minVector, &min);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, maxVector, "The max vector does not exist.", 0)
    btVector3 max;
    jmeBulletUtil::convert(pEnv, maxVector, &max);
    EXCEPTION_CHK(pEnv, 0);

    jmePhysicsSpace * const
            pSpace = new jmePhysicsSpace(pEnv, object); //dance003
#if BT_THREADSAFE
    ASSERT_CHK(pEnv, numSolvers >= 1, 0);
    ASSERT_CHK(pEnv, numSolvers <= BT_MAX_THREAD_COUNT, 0);
    pSpace->createMultiThreadedSpace(min, max, (int) broadphaseType,
            (int) numSolvers);
#else
    ASSERT_CHK(pEnv, numSolvers == 1, 0);
    pSpace->createPhysicsSpace(min, max, (int) broadphaseType);
#endif // BT_THREADSAFE

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_getGravity
(JNIEnv *pEnv, jclass, jlong spaceId, jobject storeVector) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);
    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& gravity = pWorld->getGravity();
    jmeBulletUtil::convert(pEnv, &gravity, storeVector);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getManifoldByIndex
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_getManifoldByIndex
(JNIEnv *pEnv, jclass, jlong spaceId, jint manifoldIndex) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);
    btDispatcher * const pDispatcher = pWorld->getDispatcher();
    NULL_CHK(pEnv, pDispatcher, "The dispatcher does not exist.", 0);
    int index = int(manifoldIndex);

    const btPersistentManifold * const
            pManifold = pDispatcher->getManifoldByIndexInternal(index);
    return reinterpret_cast<jlong> (pManifold);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getNumConstraints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getNumConstraints
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);

    int count = pWorld->getNumConstraints();
    return (jint) count;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getSolverInfo
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_getSolverInfo
(JNIEnv *pEnv, jclass, jlong spaceId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", 0);

    btContactSolverInfo *pInfo = &pWorld->getSolverInfo();
    return reinterpret_cast<jlong> (pInfo);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    isCcdWithStaticOnly
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_PhysicsSpace_isCcdWithStaticOnly
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", JNI_FALSE);
    const btDiscreteDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", JNI_FALSE);

    bool result = pWorld->getCcdWithStaticOnly();
    return result;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    isSpeculativeContactRestitution
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_PhysicsSpace_isSpeculativeContactRestitution
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", JNI_FALSE);
    const btDiscreteDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.", JNI_FALSE);

    bool result = pWorld->getApplySpeculativeContactRestitution();
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeAction
(JNIEnv *pEnv, jclass, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHK(pEnv, pAction, "The action object does not exist.",)

    pWorld->removeAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeCharacterObject
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    ASSERT_CHK(pEnv, pUser->m_jmeSpace == pSpace,);
    pUser->m_jmeSpace = NULL;

    pWorld->removeCollisionObject(pCollisionObject);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeConstraint
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeConstraint
(JNIEnv *pEnv, jclass, jlong spaceId, jlong constraintId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The constraint does not exist.",)
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    pWorld->removeConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeRigidBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeRigidBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong rigidBodyId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    jmeUserPointer const pUser = (jmeUserPointer) pBody->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    ASSERT_CHK(pEnv, pUser->m_jmeSpace == pSpace,);
    pUser->m_jmeSpace = NULL;

    pWorld->removeRigidBody(pBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setCcdWithStaticOnly
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setCcdWithStaticOnly
(JNIEnv *pEnv, jclass, jlong spaceId, jboolean setting) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);
    btDiscreteDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    bool enable = bool(setting);
    pWorld->setCcdWithStaticOnly(enable);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setGravity
(JNIEnv *pEnv, jclass, jlong spaceId, jobject gravityVector) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    NULL_CHK(pEnv, gravityVector, "The gravity vector does not exist.",)

    btVector3 gravity;
    jmeBulletUtil::convert(pEnv, gravityVector, &gravity);
    EXCEPTION_CHK(pEnv,);

    pWorld->setGravity(gravity);
}

/// This class is required to implement custom collision behaviour in the broadphase
struct GodotFilterCallback : public btOverlapFilterCallback {
	public:
	static bool test_collision_filters(uint32_t body0_collision_layer, uint32_t body0_collision_mask, uint32_t body1_collision_layer, uint32_t body1_collision_mask){
	    return body0_collision_layer & body1_collision_mask || body1_collision_layer & body0_collision_mask;
	}

	// return true when pairs need collision
	virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const{
        return GodotFilterCallback::test_collision_filters(proxy0->m_collisionFilterGroup, proxy0->m_collisionFilterMask, proxy1->m_collisionFilterGroup, proxy1->m_collisionFilterMask);
	}
};

struct RecoverPenetrationBroadPhaseCallback : public btBroadphaseAabbCallback {
private:
	btDbvtVolume bounds;

	const btCollisionObject *self_collision_object;
	uint32_t collision_layer;
	uint32_t collision_mask;

	struct CompoundLeafCallback : btDbvt::ICollide {
	private:
		RecoverPenetrationBroadPhaseCallback *parent_callback;
		btCollisionObject *collision_object;

	public:
		CompoundLeafCallback(RecoverPenetrationBroadPhaseCallback *p_parent_callback, btCollisionObject *p_collision_object) :
			parent_callback(p_parent_callback),
			collision_object(p_collision_object) {
		}

		void Process(const btDbvtNode *leaf) {
			BroadphaseResult result;
			result.collision_object = collision_object;
			result.compound_child_index = leaf->dataAsInt;
			parent_callback->results->push_back(result);
		}
	};

public:
	struct BroadphaseResult {
		btCollisionObject *collision_object;
		int compound_child_index;
	};

	std::vector<BroadphaseResult> *results;

public:
	RecoverPenetrationBroadPhaseCallback(const btCollisionObject *p_self_collision_object, uint32_t p_collision_layer, uint32_t p_collision_mask, btVector3 p_aabb_min, btVector3 p_aabb_max) :
		self_collision_object(p_self_collision_object),
		collision_layer(p_collision_layer),
		collision_mask(p_collision_mask) {
		bounds = btDbvtVolume::FromMM(p_aabb_min, p_aabb_max);
	}

	virtual ~RecoverPenetrationBroadPhaseCallback() {}

	virtual bool process(const btBroadphaseProxy *proxy) {
		btCollisionObject *co = static_cast<btCollisionObject *>(proxy->m_clientObject);
		if (co->getInternalType() <= btCollisionObject::CO_RIGID_BODY) {
			if (self_collision_object != proxy->m_clientObject && GodotFilterCallback::test_collision_filters(collision_layer, collision_mask, proxy->m_collisionFilterGroup, proxy->m_collisionFilterMask)) {
				if (co->getCollisionShape()->isCompound()) {
					const btCompoundShape *cs = static_cast<btCompoundShape *>(co->getCollisionShape());

					if (cs->getNumChildShapes() > 1) {
						const btDbvt *tree = cs->getDynamicAabbTree();
						if(tree == nullptr)
						    return true;

						// Transform bounds into compound shape local space
						const btTransform other_in_compound_space = co->getWorldTransform().inverse();
						const btMatrix3x3 abs_b = other_in_compound_space.getBasis().absolute();
						const btVector3 local_center = other_in_compound_space(bounds.Center());
						const btVector3 local_extent = bounds.Extents().dot3(abs_b[0], abs_b[1], abs_b[2]);
						const btVector3 local_aabb_min = local_center - local_extent;
						const btVector3 local_aabb_max = local_center + local_extent;
						const btDbvtVolume local_bounds = btDbvtVolume::FromMM(local_aabb_min, local_aabb_max);

						// Test collision against compound child shapes using its AABB tree
						CompoundLeafCallback compound_leaf_callback(this, co);
						tree->collideTV(tree->m_root, local_bounds, compound_leaf_callback);
					}
					else {
						// If there's only a single child shape then there's no need to search any more, we know which child overlaps
						BroadphaseResult result;
						result.collision_object = co;
						result.compound_child_index = 0;
						results->push_back(result);
					}
				}
				else {
					BroadphaseResult result;
					result.collision_object = co;
					result.compound_child_index = -1;
					results->push_back(result);
				}
				return true;
			}
		}
		return false;
	}
};


std::vector<RecoverPenetrationBroadPhaseCallback::BroadphaseResult> aabbTest_results;

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    aabbTest
 * Signature: (JJIILcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_aabbTest
  (JNIEnv *pEnv, jclass, jlong spaceId, jlong collision_objectId, jint collision_layer, jint collision_mask, jobject aabb_min, jobject aabb_max){

    jmePhysicsSpace * const
                pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
        NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
        btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
        NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);
        ASSERT_CHK(pEnv, pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD,);

        btCollisionObject * const
                    pCollisionObject = reinterpret_cast<btCollisionObject *> (collision_objectId);
            NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)


        btVector3 p_aabb_min , p_aabb_max;
            jmeBulletUtil::convert(pEnv, aabb_min, &p_aabb_min);
            jmeBulletUtil::convert(pEnv, aabb_max, &p_aabb_max);
            EXCEPTION_CHK(pEnv,);

        RecoverPenetrationBroadPhaseCallback recover_broad_result(pCollisionObject, collision_layer, collision_mask , p_aabb_min, p_aabb_max);
        aabbTest_results.clear();
        recover_broad_result.results = &aabbTest_results;
        pWorld->getBroadphase()->aabbTest(p_aabb_min, p_aabb_max, recover_broad_result);


}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getAABBTestResultCount
 * Signature: (J)V
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getAABBTestResultCount
  (JNIEnv *, jclass, jlong){
       return (jint)aabbTest_results.size();
  }

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getAABBTestResultCollisionObject
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_getAABBTestResultCollisionObject
  (JNIEnv *, jclass, jlong, jint idx){
        return (jlong)aabbTest_results[idx].collision_object;

  }

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getAABBTestResultCompoundChildIndex
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getAABBTestResultCompoundChildIndex
  (JNIEnv *, jclass, jlong, jint idx){
        return (jint)aabbTest_results[idx].compound_child_index;
  }


/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setSolverType
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setSolverType
(JNIEnv *pEnv, jclass, jlong spaceId, jint solverType) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);
    ASSERT_CHK(pEnv, pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD,);

    btConstraintSolver *pConstraintSolver;
    btMLCPSolverInterface *pMLCP;
    switch (solverType) {
        case 0: // SI
            pConstraintSolver = new btSequentialImpulseConstraintSolver(); //dance006
            break;
        case 1: // Dantzig
            pMLCP = new btDantzigSolver(); // TODO leak
            pConstraintSolver = new btMLCPSolver(pMLCP); //dance006
            break;
        case 2: // Lemke
            pMLCP = new btLemkeSolver(); // TODO leak
            pConstraintSolver = new btMLCPSolver(pMLCP); //dance006
            break;
        case 3: // PGS
            pMLCP = new btSolveProjectedGaussSeidel(); // TODO leak
            pConstraintSolver = new btMLCPSolver(pMLCP); //dance006
            break;
        case 4: // NNCG
            pConstraintSolver = new btNNCGConstraintSolver(); //dance006
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The solver type is out of range.");
            return;
    }

    btConstraintSolver *pOldSolver = pWorld->getConstraintSolver();
    pWorld->setConstraintSolver(pConstraintSolver);
    delete pOldSolver; //dance006
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setSpeculativeContactRestitution
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setSpeculativeContactRestitution
(JNIEnv *pEnv, jclass, jlong spaceId, jboolean setting) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);
    btDiscreteDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    bool enable = (bool)setting;
    pWorld->setApplySpeculativeContactRestitution(enable);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    stepSimulation
 * Signature: (JFIFZZZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_stepSimulation
(JNIEnv *pEnv, jclass, jlong spaceId, jfloat tpf, jint maxSteps,
        jfloat accuracy, jboolean enableContactEndedCallback,
        jboolean enableContactProcessedCallback,
        jboolean enableContactStartedCallback) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    pSpace->stepSimulation(tpf, maxSteps, accuracy, enableContactEndedCallback,
            enableContactProcessedCallback, enableContactStartedCallback);
}


struct GodotKinClosestConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback {
public:
	//const BulletRigidBody3D *m_self_object;
	//const VSet<RID> *m_exclude;
	const bool m_infinite_inertia = false;

	GodotKinClosestConvexResultCallback(const btVector3 &convexFromWorld, const btVector3 &convexToWorld, bool p_infinite_inertia) :
			btCollisionWorld::ClosestConvexResultCallback(convexFromWorld, convexToWorld),
			m_infinite_inertia(p_infinite_inertia) {}

	virtual bool needsCollision(btBroadphaseProxy *proxy0) const{
	    const bool needs = GodotFilterCallback::test_collision_filters(m_collisionFilterGroup, m_collisionFilterMask, proxy0->m_collisionFilterGroup, proxy0->m_collisionFilterMask);
        	if (needs) {
        		btCollisionObject *btObj = static_cast<btCollisionObject *>(proxy0->m_clientObject);
        		//BulletCollisionObject3D *gObj = static_cast<BulletCollisionObject3D *>(btObj->getUserPointer());

//        		if (gObj == m_self_object) {
//        			return false;
//        		} else {
//        			// A kinematic body can't be stopped by a rigid body since the mass of kinematic body is infinite
//        			if (m_infinite_inertia && !btObj->isStaticOrKinematicObject()) {
//        				return false;
//        			}
//
//        			if (gObj->getType() == BulletCollisionObject3D::TYPE_AREA) {
//        				return false;
//        			}
//
//        			if (m_self_object->has_collision_exception(gObj) || gObj->has_collision_exception(m_self_object)) {
//        				return false;
//        			}
//
//        			if (m_exclude->has(gObj->get_rid())) {
//        				return false;
//        			}
//        		}
        		return true;
        	} else {
        		return false;
        	}
	}
};

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    sweepTestClosestResultCallback
 * Signature: (JLcom/jme3/math/Transform;Lcom/jme3/math/Transform;JLcom/jme3/bullet/collision/PhysicsSweepTestResult;F)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_sweepTestClosestResultCallback
  (JNIEnv *pEnv, jclass, jlong shapeId, jobject from, jobject to, jlong spaceId,
           jobject result, jfloat allowedCcdPenetration){


 }