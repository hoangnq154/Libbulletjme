/*
 * Copyright (c) 2009-2019 jMonkeyEngine
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
package com.jme3.bullet.objects;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.CollisionFlag;
import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.math.Vector3f;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision object for simplified character simulation, based on Bullet's
 * btKinematicCharacterController.
 *
 * @author normenhansen
 */
public class PhysicsCharacter extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsCharacter.class.getName());
    /**
     * default gravity vector -- differs from that of
     * btKinematicCharacterController!
     */
    final private static Vector3f defaultGravity = new Vector3f(0f, -29.4f, 0f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_Y}
     */
    final private static Vector3f unitY = new Vector3f(0f, 1f, 0f);
    // *************************************************************************
    // fields

    /**
     * copy of the maximum amount of vertical movement without jumping or
     * falling (in physics-space units)
     */
    private float stepHeight;
    /**
     * Unique identifier of the btKinematicCharacterController (as opposed to
     * the collision object, which is a btPairCachingGhostObject). Constructors
     * are responsible for setting this to a non-zero value. The ID might change
     * if the character gets rebuilt.
     */
    private long characterId = 0L;
    /**
     * temporary storage for one vector per thread
     */
    final private static ThreadLocal<Vector3f> threadTmpVector
            = new ThreadLocal<Vector3f>() {
        @Override
        protected Vector3f initialValue() {
            return new Vector3f();
        }
    };
    // *************************************************************************
    // constructors

    /**
     * Instantiate a responsive character with the specified CollisionShape and
     * step height.
     *
     * @param shape the desired shape (not null, convex, alias created)
     * @param stepHeight the maximum amount of vertical movement without jumping
     * or falling (in physics-space units)
     */
    public PhysicsCharacter(ConvexShape shape, float stepHeight) {
        Validate.nonNull(shape, "shape");

        super.setCollisionShape(shape);
        this.stepHeight = stepHeight;
        buildObject();
        /*
         * The default gravity for a Bullet btKinematicCharacterController
         * is (0,0,-29.4), which makes no sense for JME.
         * So override the default.
         */
        setGravity(defaultGravity);
        setUp(unitY);

        assert isContactResponse();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read this character's angular damping.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getAngularDamping() {
        return getAngularDamping(characterId);
    }

    /**
     * Copy this character's angular velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the velocity vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f getAngularVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getAngularVelocity(characterId, result);
        return result;
    }

    /**
     * Read the ID of the btKinematicCharacterController. Used internally.
     *
     * @return the unique identifier (not zero)
     */
    public long getControllerId() {
        assert characterId != 0L;
        return characterId;
    }

    /**
     * Read this character's maximum fall speed (terminal velocity).
     *
     * @return the speed (in physics-space units per second)
     */
    public float getFallSpeed() {
        return getFallSpeed(characterId);
    }

    /**
     * Copy this character's gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector (in physics-space units per second
     * squared, direction opposite the "up" vector, either storeResult or a new
     * vector, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getGravity(characterId, result);
        return result;
    }

    /**
     * Read this character's jump speed.
     *
     * @return the speed (in physics-space units per second)
     */
    public float getJumpSpeed() {
        return getJumpSpeed(characterId);
    }

    /**
     * Read this character's linear damping.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getLinearDamping() {
        return getLinearDamping(characterId);
    }

    /**
     * Copy the linear velocity of this character's center.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector (either storeResult or a new vector, not null)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getLinearVelocity(characterId, result);
        return result;
    }

    /**
     * Read this character's maximum penetration depth.
     *
     * @return the depth (in physics-space units)
     */
    public float getMaxPenetrationDepth() {
        return getMaxPenetrationDepth(characterId);
    }

    /**
     * Read this character's maximum slope angle.
     *
     * @return the angle relative to the horizontal (in radians)
     */
    public float getMaxSlope() {
        return getMaxSlope(characterId);
    }

    /**
     * Read this character's step height.
     *
     * @return the maximum amount of vertical movement without jumping or
     * falling (in physics-space units)
     */
    public float getStepHeight() {
        assert stepHeight == getStepHeight(characterId);
        return stepHeight;
    }

    /**
     * Copy this character's "up" direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (in physics-space coordinates, in the direction
     * opposite the gravity vector, either storeResult or a new vector, not
     * null)
     */
    public Vector3f getUpDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getUpDirection(characterId, result);
        return result;
    }

    /**
     * Copy the character's walk offset.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an offset vector (either storeResult or a new vector, not null)
     */
    public Vector3f getWalkDirection(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getWalkOffset(characterId, result);
        return result;
    }

    /**
     * Test whether the ghost's convexSweepTest is in use.
     *
     * @return true if using the ghost's test, otherwise false
     */
    public boolean isUsingGhostSweepTest() {
        return isUsingGhostSweepTest(characterId);
    }

    /**
     * Jump in the "up" direction.
     */
    public void jump() {
        jump(translateIdentity);
    }

    /**
     * Jump in the specified direction.
     *
     * @param dir desired jump direction (not null, unaffected) or (0,0,0) to
     * jump in the "up" direction
     */
    public void jump(Vector3f dir) {
        jump(characterId, dir);
    }

    /**
     * Test whether this character is on the ground.
     *
     * @return true if on the ground, otherwise false
     */
    public boolean onGround() {
        return onGround(characterId);
    }

    /**
     * Reset this character, including its velocity.
     *
     * @param space (not null)
     */
    public void reset(PhysicsSpace space) {
        long spaceId = space.nativeId();
        reset(characterId, spaceId);
    }

    /**
     * Alter this character's angular damping.
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=0)
     */
    public void setAngularDamping(float damping) {
        setAngularDamping(characterId, damping);
    }

    /**
     * Alter this character's angular velocity.
     *
     * @param angularVelocity the desired angular velocity vector (not null,
     * unaffected)
     */
    public void setAngularVelocity(Vector3f angularVelocity) {
        setAngularVelocity(characterId, angularVelocity);
    }

    /**
     * Apply the specified CollisionShape to this character. Note that the
     * character should not be in any PhysicsSpace while changing shape; the
     * character gets rebuilt on the physics side.
     *
     * @param collisionShape the shape to apply (not null, convex, alias
     * created) TODO declare as ConvexShape
     */
    @Override
    public void setCollisionShape(CollisionShape collisionShape) {
        assert collisionShape.isConvex();

        super.setCollisionShape(collisionShape);
        if (hasAssignedNativeObject()) {
            long objectId = nativeId();
            long shapeId = collisionShape.nativeId();
            attachCollisionShape(objectId, shapeId);
        } else {
            buildObject();
        }
    }

    /**
     * Enable/disable this character's contact response.
     *
     * @param newState true to respond to contact forces, false to ignore them
     * (default=true)
     */
    public void setContactResponse(boolean newState) {
        long objectId = nativeId();
        int flags = getCollisionFlags(objectId);
        if (newState) {
            flags &= ~CollisionFlag.NO_CONTACT_RESPONSE;
        } else {
            flags |= CollisionFlag.NO_CONTACT_RESPONSE;
        }
        setCollisionFlags(objectId, flags);
    }

    /**
     * Alter this character's maximum fall speed (terminal velocity).
     *
     * @param fallSpeed the desired speed (in physics-space units per second,
     * default=55)
     */
    public void setFallSpeed(float fallSpeed) {
        setFallSpeed(characterId, fallSpeed);
    }

    /**
     * Alter the character's gravitational acceleration without altering its
     * "up" vector.
     *
     * @param downwardAcceleration the desired downward acceleration (in
     * physics-space units per second squared, not null, unaffected,
     * default=29.4)
     */
    public void setGravity(float downwardAcceleration) {
        Vector3f gVector = threadTmpVector.get();
        getUpDirection(gVector);
        gVector.multLocal(-downwardAcceleration);
        setGravity(gVector);
    }

    /**
     * Alter this character's gravitational acceleration. This may also alter
     * its "up" vector.
     *
     * @param gravity the desired acceleration vector (in physics-space units
     * per second squared, not null, unaffected, default=(0,-29.4,0))
     */
    public void setGravity(Vector3f gravity) {
        setGravity(characterId, gravity);
    }

    /**
     * Alter this character's jump speed.
     *
     * @param jumpSpeed the desired speed (in physics-space units per second,
     * default=10)
     */
    public void setJumpSpeed(float jumpSpeed) {
        setJumpSpeed(characterId, jumpSpeed);
    }

    /**
     * Alter this character's linear damping.
     *
     * @param damping the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=0)
     */
    public void setLinearDamping(float damping) {
        setLinearDamping(characterId, damping);
    }

    /**
     * Alter the linear velocity of this character's center.
     *
     * @param velocity the desired velocity vector (not null)
     */
    public void setLinearVelocity(Vector3f velocity) {
        setLinearVelocity(characterId, velocity);
    }

    /**
     * Alter this character's maximum penetration depth.
     *
     * @param depth the desired depth (in physics-space units, default=0.2)
     */
    public void setMaxPenetrationDepth(float depth) {
        setMaxPenetrationDepth(characterId, depth);
    }

    /**
     * Alter this character's maximum slope angle.
     *
     * @param slopeRadians the desired angle relative to the horizontal (in
     * radians, default=Pi/4)
     */
    public void setMaxSlope(float slopeRadians) {
        setMaxSlope(characterId, slopeRadians);
    }

    /**
     * Directly alter this character's location. (Same as
     * {@link #warp(com.jme3.math.Vector3f)}).)
     *
     * @param location the desired location (not null, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        warp(location);
    }

    /**
     * Alter this character's step height.
     *
     * @param height the desired maximum amount of vertical movement without
     * jumping or falling (in physics-space units, default=1)
     */
    public void setStepHeight(float height) {
        this.stepHeight = height;
        setStepHeight(characterId, height);
    }

    /**
     * Alter which convexSweepTest is used.
     *
     * @param useGhostSweepTest true to use the ghost's test, false to use the
     * world's test (default=true)
     */
    public void setSweepTest(boolean useGhostSweepTest) {
        setUseGhostSweepTest(characterId, useGhostSweepTest);
    }

    /**
     * Alter this character's "up" direction. This may also alter its gravity
     * vector.
     *
     * @param direction the desired direction (not null, not zero, unaffected,
     * default=(0,1,0))
     */
    public void setUp(Vector3f direction) {
        Validate.nonZero(direction, "direction");
        setUp(characterId, direction);
    }

    /**
     * Alter this character's walk offset. The offset must be perpendicular to
     * the "up" direction. It will continue to be applied until altered again.
     *
     * @param offset the desired location increment for each physics tick (in
     * physics-space coordinates, not null, unaffected, default=(0,0,0))
     */
    public void setWalkDirection(Vector3f offset) {
        setWalkDirection(characterId, offset);
    }

    /**
     * Directly alter the location of this character's center.
     *
     * @param location the desired physics location (not null, unaffected)
     */
    public void warp(Vector3f location) {
        warp(characterId, location);
    }
    // *************************************************************************
    // Object methods

    /**
     * Finalize this physics character just before it is destroyed. Should be
     * invoked only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        try {
            finalizeNativeCharacter(characterId);
        } finally {
            super.finalize();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Create the configured objects in Bullet.
     */
    private void buildObject() {
        if (!hasAssignedNativeObject()) {
            long objectId = createGhostObject();
            setNativeId(objectId);
            assert getInternalType(objectId) == PcoType.ghost :
                    getInternalType(objectId);
            logger2.log(Level.FINE, "Creating {0}.", this);

            initUserPointer();
        }

        long objectId = nativeId();
        setCharacterFlags(objectId);

        CollisionShape shape = getCollisionShape();
        assert shape.isConvex();
        long shapeId = shape.nativeId();
        attachCollisionShape(objectId, shapeId);

        if (characterId != 0L) {
            logger2.log(Level.FINE, "Clearing {0}.", this);
            finalizeNativeCharacter(characterId);
        }
        characterId = createCharacterObject(objectId, shapeId, stepHeight);
        assert characterId != 0L;
        logger2.log(Level.FINE, "Creating {0}.", this);
    }
    // *************************************************************************
    // native methods

    native private long createCharacterObject(long ghostId, long shapeId,
            float stepHeight);

    native private long createGhostObject();

    native private void finalizeNativeCharacter(long controllerId);

    native private float getAngularDamping(long controllerId);

    native private void getAngularVelocity(long controllerId,
            Vector3f storeVector);

    native private float getFallSpeed(long controllerId);

    native private void getGravity(long controllerId, Vector3f storeVector);

    native private float getJumpSpeed(long controllerId);

    native private float getLinearDamping(long controllerId);

    native private void getLinearVelocity(long controllerId,
            Vector3f storeVector);

    native private float getMaxPenetrationDepth(long controllerId);

    native private float getMaxSlope(long controllerId);

    native private float getStepHeight(long controllerId);

    native private void getUpDirection(long controllerId, Vector3f storeVector);

    native private void getWalkOffset(long controllerId, Vector3f storeVector);

    native private boolean isUsingGhostSweepTest(long controllerId);

    native private void jump(long controllerId, Vector3f direction);

    native private boolean onGround(long controllerId);

    native private void reset(long controllerId, long spaceId);

    native private void setAngularDamping(long controllerId, float damping);

    native private void setAngularVelocity(long controllerId,
            Vector3f angularVelocity);

    native private void setCharacterFlags(long ghostId);

    native private void setFallSpeed(long controllerId, float fallSpeed);

    native private void setGravity(long controllerId, Vector3f gravity);

    native private void setJumpSpeed(long controllerId, float jumpSpeed);

    native private void setLinearDamping(long controllerId, float damping);

    native private void setLinearVelocity(long controllerId, Vector3f velocity);

    native private void setMaxPenetrationDepth(long controllerId, float depth);

    native private void setMaxSlope(long controllerId, float slopeRadians);

    native private void setStepHeight(long controllerId, float height);

    native private void setUp(long controllerId, Vector3f direction);

    native private void setUseGhostSweepTest(long controllerId,
            boolean useGhostSweepTest);

    native private void setWalkDirection(long controllerId, Vector3f direction);

    native private void warp(long controllerId, Vector3f location);
}
