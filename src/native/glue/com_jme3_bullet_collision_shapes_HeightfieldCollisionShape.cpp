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
#include "com_jme3_bullet_collision_shapes_HeightfieldCollisionShape.h"
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * A btHeightfieldTerrainShape whose heights are represented by btScalars.
     * Access is provided to its height data.
     */
    ATTRIBUTE_ALIGNED16(class)
    HeightfieldShape : public btHeightfieldTerrainShape {
        public:
        BT_DECLARE_ALIGNED_ALLOCATOR();

        HeightfieldShape(int heightStickWidth, int heightStickLength,
                const void *pHeightfieldData, btScalar heightScale,
                btScalar minHeight, btScalar maxHeight, int upAxis,
                bool flipQuadEdges)
                : btHeightfieldTerrainShape(heightStickWidth,
                heightStickLength, pHeightfieldData, heightScale, minHeight,
                maxHeight, upAxis, PHY_FLOAT, flipQuadEdges) {
        }

        const btScalar * getHeightData() {
            return m_heightfieldDataFloat;
        }
    };

    /*
     * Class:     com_jme3_bullet_collision_shapes_HeightfieldCollisionShape
     * Method:    createShape2
     * Signature: (IILjava/nio/FloatBuffer;FFFIZZZZ)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_HeightfieldCollisionShape_createShape2
    (JNIEnv *env, jobject object, jint heightStickWidth, jint heightStickLength,
            jobject floatBuffer, jfloat heightScale, jfloat minHeight,
            jfloat maxHeight, jint upAxis, jboolean flipQuadEdges,
            jboolean flipTriangleWinding, jboolean useDiamond,
            jboolean useZigzag) {
        jmeClasses::initJavaClasses(env);

        NULL_CHECK(floatBuffer, "The heightfield buffer does not exist.", 0);
        const jfloat * const pHeights
                = (jfloat *) env->GetDirectBufferAddress(floatBuffer);
        NULL_CHECK(pHeights, "The heightfield buffer is not direct.", 0);

        HeightfieldShape *pShape;
#ifdef BT_USE_DOUBLE_PRECISION
        const int numHeights = heightStickLength * heightStickWidth;
        btScalar * const pDpHeights = new btScalar[numHeights];
        for (int i = 0; i < numHeights; ++i) {
            pDpHeights[i] = pHeights[i];
        }
        pShape = new HeightfieldShape(heightStickWidth, heightStickLength,
                pDpHeights, heightScale, minHeight, maxHeight, upAxis,
                flipQuadEdges);
#else
        pShape = new HeightfieldShape(heightStickWidth, heightStickLength,
                pHeights, heightScale, minHeight, maxHeight, upAxis,
                flipQuadEdges);
#endif

        pShape->setFlipTriangleWinding(flipTriangleWinding);
        pShape->setUseDiamondSubdivision(useDiamond);
        pShape->setUseZigzagSubdivision(useZigzag);

        return reinterpret_cast<jlong> (pShape);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_HeightfieldCollisionShape
     * Method:    finalizeNative
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_HeightfieldCollisionShape_finalizeNative
    (JNIEnv *env, jobject object, jlong shapeId) {
#ifdef BT_USE_DOUBLE_PRECISION
        HeightfieldShape *pShape
                = reinterpret_cast<HeightfieldShape *> (shapeId);
        if (pShape != NULL) {
            const btScalar *pDpHeights = pShape->getHeightData();
            delete[] pDpHeights;
        }
#endif
    }

#ifdef __cplusplus
}
#endif
