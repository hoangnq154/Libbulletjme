/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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

#include "com_jme3_bullet_SoftBodyWorldInfo.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

/*
 * Author: Dokthar
 */
#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    createSoftBodyWorldInfo
     * Signature: ()J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_createSoftBodyWorldInfo
    (JNIEnv *env, jobject object) {
        jmeClasses::initJavaClasses(env);
        btSoftBodyWorldInfo* worldInfo = new btSoftBodyWorldInfo();
        return reinterpret_cast<jlong> (worldInfo);
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    getAirDensity
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_getAirDensity
    (JNIEnv *env, jobject object, jlong worldId) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.", 0);

        return world->air_density;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    getGravity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_getGravity
    (JNIEnv *env, jobject object, jlong worldId, jobject gravity) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        NULL_CHECK(gravity, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &world->m_gravity, gravity);
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    getMaxDisplacement
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_getMaxDisplacement
    (JNIEnv *env, jobject object, jlong worldId) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.", 0);

        return world->m_maxDisplacement;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    getWaterDensity
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_getWaterDensity
    (JNIEnv *env, jobject object, jlong worldId) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.", 0);

        return world->water_density;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    getWaterNormal
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_getWaterNormal
    (JNIEnv *env, jobject object, jlong worldId, jobject normal) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        NULL_CHECK(normal, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &world->water_normal, normal);
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    getWaterOffset
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_SoftBodyWorldInfo_getWaterOffset
    (JNIEnv *env, jobject object, jlong worldId) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.", 0);

        return world->water_offset;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setAirDensity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_setAirDensity
    (JNIEnv *env, jobject object, jlong worldId, jfloat value) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        world->air_density = value;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setGravity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_setGravity
    (JNIEnv *env, jobject object, jlong worldId, jobject gravity) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        NULL_CHECK(gravity, "The gravity vector does not exist.",);
        jmeBulletUtil::convert(env, gravity, &world->m_gravity);
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setMaxDisplacement
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_setMaxDisplacement
    (JNIEnv *env, jobject object, jlong worldId, jfloat value) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        world->m_maxDisplacement = value;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setSoftBodyWorldInfo
     * Signature: (JJ)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_SoftBodyWorldInfo_setSoftBodyWorldInfo
    (JNIEnv *env, jobject object, jlong worldId, jlong copyId) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The target btSoftBodyWorldInfo does not exist.", 0);

        btSoftBodyWorldInfo* copy = reinterpret_cast<btSoftBodyWorldInfo*> (copyId);
        NULL_CHECK(copy, "The source btSoftBodyWorldInfo does not exist.", 0);

        if (world != copy) {
            world->air_density = copy->air_density;
            world->water_density = copy->water_density;
            world->water_offset = copy->water_offset;
            world->m_maxDisplacement = copy->m_maxDisplacement;
            world->water_normal = copy->water_normal;
            world->m_broadphase = copy->m_broadphase; // <- <!> pointer variable
            world->m_dispatcher = copy->m_dispatcher; // <- <!> pointer variable
            world->m_gravity = copy->m_gravity;
            world->m_sparsesdf = copy->m_sparsesdf;
        }
        return 0;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setWaterDensity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_setWaterDensity
    (JNIEnv *env, jobject object, jlong worldId, jfloat value) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        world->water_density = value;
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setWaterNormal
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_setWaterNormal
    (JNIEnv *env, jobject object, jlong worldId, jobject normal) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        NULL_CHECK(normal, "The normal vector does not exist.",);
        jmeBulletUtil::convert(env, normal, &world->water_normal);
    }

    /*
     * Class:     com_jme3_bullet_SoftBodyWorldInfo
     * Method:    setWaterOffset
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_SoftBodyWorldInfo_setWaterOffset
    (JNIEnv *env, jobject object, jlong worldId, jfloat value) {
        btSoftBodyWorldInfo* world = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(world, "The btSoftBodyWorldInfo does not exist.",);

        world->water_offset = value;
    }

#ifdef __cplusplus
}
#endif