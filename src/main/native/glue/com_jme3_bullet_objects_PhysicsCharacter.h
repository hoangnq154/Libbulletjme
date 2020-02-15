/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_objects_PhysicsCharacter */

#ifndef _Included_com_jme3_bullet_objects_PhysicsCharacter
#define _Included_com_jme3_bullet_objects_PhysicsCharacter
#ifdef __cplusplus
extern "C" {
#endif
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_NONE
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_NONE 0L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_01
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_01 1L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_02
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_02 2L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_03
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_03 4L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_04
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_04 8L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_05
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_05 16L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_06
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_06 32L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_07
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_07 64L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_08
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_08 128L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_09
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_09 256L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_10
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_10 512L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_11
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_11 1024L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_12
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_12 2048L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_13
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_13 4096L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_14
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_14 8192L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_15
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_15 16384L
#undef com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_16
#define com_jme3_bullet_objects_PhysicsCharacter_COLLISION_GROUP_16 32768L
/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    createCharacterObject
 * Signature: (JJF)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_createCharacterObject
  (JNIEnv *, jobject, jlong, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    createGhostObject
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_createGhostObject
  (JNIEnv *, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    finalizeNativeCharacter
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_finalizeNativeCharacter
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getAngularDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getAngularDamping
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getAngularVelocity
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getFallSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getFallSpeed
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getGravity
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getJumpSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getJumpSpeed
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getLinearDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getLinearDamping
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getLinearVelocity
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getMaxPenetrationDepth
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getMaxPenetrationDepth
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getMaxSlope
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getMaxSlope
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getStepHeight
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getStepHeight
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getUpDirection
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getUpDirection
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getWalkOffset
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getWalkOffset
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    isUsingGhostSweepTest
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_isUsingGhostSweepTest
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    jump
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_jump
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    onGround
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_onGround
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    reset
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_reset
  (JNIEnv *, jobject, jlong, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setAngularDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setAngularDamping
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setAngularVelocity
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setCharacterFlags
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setCharacterFlags
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setFallSpeed
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setFallSpeed
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setGravity
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setJumpSpeed
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setJumpSpeed
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setLinearDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setLinearDamping
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setLinearVelocity
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setMaxPenetrationDepth
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setMaxPenetrationDepth
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setMaxSlope
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setMaxSlope
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setStepHeight
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setStepHeight
  (JNIEnv *, jobject, jlong, jfloat);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setUp
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setUp
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setUseGhostSweepTest
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setUseGhostSweepTest
  (JNIEnv *, jobject, jlong, jboolean);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setWalkDirection
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setWalkDirection
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    warp
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_warp
  (JNIEnv *, jobject, jlong, jobject);

#ifdef __cplusplus
}
#endif
#endif
