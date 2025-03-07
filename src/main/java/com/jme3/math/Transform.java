/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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
package com.jme3.math;

import com.jme3.util.TempVars;

/**
 * A 3-D coordinate transform composed of translation, rotation, and scaling.
 * The order of application is: scale, then rotate, then translate.
 *
 * <p>Started July 16, 2004
 *
 * @author Jack Lindamood
 * @author Joshua Slack
 */
public final class Transform implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;
    /**
     * Shared instance of the identity transform. Do not modify!
     */
    public static final Transform IDENTITY = new Transform();
    /**
     * Rotation component.
     */
    private Quaternion rot = new Quaternion();
    /**
     * Translation component: an offset for each local axis.
     */
    private Vector3f translation = new Vector3f();
    /**
     * Scaling component: a scale factor for each local axis.
     */
    private Vector3f scale = new Vector3f(1, 1, 1);

    /**
     * Instantiates a coordinate transform without scaling.
     *
     * @param translation the desired translation (not null, unaffected)
     * @param rot the desired rotation (not null, unaffected)
     */
    public Transform(Vector3f translation, Quaternion rot) {
        this.translation.set(translation);
        this.rot.set(rot);
    }

    /**
     * Instantiates a coordinate transform with scaling.
     *
     * @param translation the desired translation (not null, unaffected)
     * @param rot the desired rotation (not null, unaffected)
     * @param scale the desired scaling (not null, unaffected)
     */
    public Transform(Vector3f translation, Quaternion rot, Vector3f scale) {
        this(translation, rot);
        this.scale.set(scale);
    }

    /**
     * Instantiates an identity transform: no translation, no rotation, and no
     * scaling.
     */
    public Transform() {
        this(Vector3f.ZERO, Quaternion.IDENTITY);
    }

    /**
     * Sets the rotation component to the argument.
     *
     * @param rot the desired rotation value (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Transform setRotation(Quaternion rot) {
        this.rot.set(rot);
        return this;
    }

    /**
     * Sets the translation component to the argument.
     *
     * @param trans the desired offsets (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Transform setTranslation(Vector3f trans) {
        this.translation.set(trans);
        return this;
    }

    /**
     * Returns the translation component.
     *
     * @return the pre-existing instance (not null)
     */
    public Vector3f getTranslation() {
        return translation;
    }

    /**
     * Sets the scaling component to the argument.
     *
     * @param scale the desired scale factors (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Transform setScale(float scale) {
        this.scale.set(scale, scale, scale);
        return this;
    }

    /**
     * Returns the scaling component.
     *
     * @return the pre-existing instance (not null)
     */
    public Vector3f getScale() {
        return scale;
    }

    /**
     * Returns the rotation component.
     *
     * @return the pre-existing instance (not null)
     */
    public Quaternion getRotation() {
        return rot;
    }

    /**
     * Combines with the argument and returns the (modified) current instance.
     *
     * @param parent the parent transform (not null, unaffected unless it's
     *     <code>this</code>)
     * @return the (modified) current instance (for chaining)
     */
    public Transform combineWithParent(Transform parent) {
        //applying parent scale to local scale
        scale.multLocal(parent.scale);
        //applying parent rotation to local rotation.
        parent.rot.mult(rot, rot);
        //applying parent scale to local translation.
        translation.multLocal(parent.scale);
        //applying parent rotation to local translation, then applying parent translation to local translation.
        //Note that parent.rot.multLocal(translation) doesn't modify "parent.rot" but "translation"
        parent.rot
                .multLocal(translation)
                .addLocal(parent.translation);

        return this;
    }

    /**
     * Transforms the specified coordinates and returns the result in
     * <code>store</code>. If the <code>store</code> is null, a new Vector3f is
     * created to hold the value. Either way, the current instance is
     * unaffected, unless <code>store</code> is its translation or scaling.
     *
     * @param in the coordinates to transform (not null, unaffected)
     * @param store storage for the result (modified if not null)
     * @return the transformed coordinates (either <code>store</code> or a new
     *     Vector3f)
     */
    public Vector3f transformVector(final Vector3f in, Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }

        // multiply with scale first, then rotate, finally translate (cf.
        // Eberly)
        return rot.mult(store.set(in).multLocal(scale), store).addLocal(translation);
    }

    /**
     * Applies the inverse transform to the specified coordinates and returns
     * the result in <code>store</code>. If the <code>store</code> is null, a
     * new Vector3f is created to hold the value. Either way, the current
     * instance is unaffected, unless <code>store</code> is its translation or
     * scaling.
     *
     * @param in the coordinates to transform (not null, unaffected unless it's
     *     <code>store</code>)
     * @param store storage for the result (modified if not null)
     * @return the transformed coordinates (either <code>store</code> or a new
     *     Vector3f)
     */
    public Vector3f transformInverseVector(final Vector3f in, Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }

        // The author of this code should've looked above and taken the inverse of that,
        // but for some reason, they didn't.
//        in.subtract(translation, store).divideLocal(scale);
//        rot.inverse().mult(store, store);
        in.subtract(translation, store);
        rot.inverse().mult(store, store);
        store.divideLocal(scale);

        return store;
    }

    /**
     * Creates an equivalent transform matrix. The current instance is
     * unaffected.
     *
     * @return a new Matrix4f
     */
    public Matrix4f toTransformMatrix() {
        return toTransformMatrix(null);
    }

    /**
     * Converts to an equivalent transform matrix. The current instance is
     * unaffected.
     *
     * @param store storage for the result (modified if not null)
     * @return a transform matrix (either <code>store</code> or a new Matrix4f)
     */
    public Matrix4f toTransformMatrix(Matrix4f store) {
        if (store == null) {
            store = new Matrix4f();
        }
        store.setTranslation(translation);
        rot.toTransformMatrix(store);
        store.setScale(scale);
        return store;
    }

    /**
     * Sets the current instance from a transform matrix. Any shear in the
     * matrix is lost -- in other words, it may not be possible to recreate the
     * original matrix from the result.
     *
     * @param mat the input matrix (not null, unaffected)
     */
    public void fromTransformMatrix(Matrix4f mat) {
        TempVars vars = TempVars.get();
        translation.set(mat.toTranslationVector(vars.vect1));
        rot.set(mat.toRotationQuat(vars.quat1));
        scale.set(mat.toScaleVector(vars.vect2));
        vars.release();
    }

    /**
     * Returns the inverse. The current instance is unaffected.
     *
     * @return a new Transform
     */
    public Transform invert() {
        Transform t = new Transform();
        t.fromTransformMatrix(toTransformMatrix().invertLocal());
        return t;
    }

    /**
     * Sets the current instance to the identity transform: translation=(0,0,0)
     * scaling=(1,1,1) rotation=(0,0,0,1).
     */
    public void loadIdentity() {
        translation.set(0, 0, 0);
        scale.set(1, 1, 1);
        rot.set(0, 0, 0, 1);
    }

    /**
     * Returns a hash code. If two transforms have identical values, they
     * will have the same hash code. The current instance is unaffected.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 89 * hash + rot.hashCode();
        hash = 89 * hash + translation.hashCode();
        hash = 89 * hash + scale.hashCode();
        return hash;
    }

    /**
     * Tests for exact equality with the argument, distinguishing -0 from 0. If
     * {@code obj} is null, false is returned. Either way, the current instance
     * is unaffected.
     *
     * @param obj the object to compare (may be null, unaffected)
     * @return true if {@code this} and {@code obj} have identical values,
     *     otherwise false
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Transform other = (Transform) obj;
        return this.translation.equals(other.translation)
                && this.scale.equals(other.scale)
                && this.rot.equals(other.rot);
    }

    /**
     * Returns a string representation of the transform, which is unaffected.
     * For example, the identity transform is represented by:
     * <pre>
     * Transform[ 0.0, 0.0, 0.0]
     * [ 0.0, 0.0, 0.0, 1.0]
     * [ 1.0 , 1.0, 1.0]
     * </pre>
     *
     * @return the string representation (not null, not empty)
     */
    @Override
    public String toString() {
        return getClass().getSimpleName()
                + "[ " + translation.x + ", " + translation.y + ", " + translation.z + "]\n"
                + "[ " + rot.x + ", " + rot.y + ", " + rot.z + ", " + rot.w + "]\n"
                + "[ " + scale.x + " , " + scale.y + ", " + scale.z + "]";
    }

    /**
     * Creates a copy. The current instance is unaffected.
     *
     * @return a new instance, equivalent to the current one
     */
    @Override
    public Transform clone() {
        try {
            Transform tq = (Transform) super.clone();
            tq.rot = rot.clone();
            tq.scale = scale.clone();
            tq.translation = translation.clone();
            return tq;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
