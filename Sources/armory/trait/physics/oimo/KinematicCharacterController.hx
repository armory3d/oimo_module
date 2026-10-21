package armory.trait.physics.oimo;

import iron.data.MeshData;
import iron.math.Quat;
import iron.math.Vec4;
import iron.object.MeshObject;
import iron.object.Transform;

import oimo.collision.geometry.*;
import oimo.common.Mat3;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.RigidBodyType;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

class KinematicCharacterController extends iron.Trait {
    var ready: Bool = false;
    public var onReady: Void->Void = null;

    public var id: Int = 0;
    public var physics: PhysicsWorld = null;
    public var transform: Transform = null;
    public var body: oimo.dynamics.rigidbody.RigidBody = null;
    public var shape: Shape = null;
    public var group: Int = 1;
    public var mask: Int = 1;

    public var mass: Float;
    public var friction: Float;
    public var restitution: Float;
    public var collisionMargin: Float;
    public var animated: Bool;
    public var jumpSpeed: Float;
    var shapeType: ControllerShape;

    // Requested movement for the next fixed step, as a displacement vector.
    var walkDirection: Vec4 = new Vec4();

    // Gravity / jump state.
    var gravity: Float = 9.81;
    var gravityEnabled: Bool = true;
    var fallSpeed: Float = 55.0;
    var verticalVelocity: Float = 0.0;
    var maxSlope: Float = Math.PI / 4;
    var maxSlopeCos: Float = Math.cos(Math.PI / 4);
    var grounded: Bool = false;
    var upInterpolate: Bool = false;

    // Reusable objects for transform synchronization.
    var tempPos: Vec3 = new Vec3();
    var tempQuat: oimo.common.Quat = new oimo.common.Quat();
    var tempIronQuat: Quat = new Quat();

    // Reusable objects for sweep / slide movement.
    var movePosition: Vec4 = new Vec4();
    var moveRemaining: Vec4 = new Vec4();
    var moveDirection: Vec4 = new Vec4();
    var moveNormal: Vec4 = new Vec4();
    var moveFrom: Vec4 = new Vec4();
    var moveTo: Vec4 = new Vec4();

    // Oimo's broad-phase sweep test ignores the convex margin, so very short
    // sweeps can miss targets that are already within the shape's margin.
    // Probe a little further than the requested movement and clip the hit.
    static inline var MOVE_SKIN: Float = 0.001;
    static inline var MOVE_BACK_OFF: Float = 0.001;
    static inline var MIN_SWEEP_DISTANCE: Float = 1.0;
    static inline var MOVE_EPSILON: Float = 1e-6;
    static inline var MAX_MOVE_ITERATIONS: Int = 6;
    static inline var GROUND_SNAP_DISTANCE: Float = 0.5;

    public function new(mass = 1.0, shapeType: ControllerShape = ControllerShape.Capsule, jumpSpeed = 8.0, friction = 0.5, restitution = 0.0, collisionMargin = 0.0, animated = false, group = 1, mask = 1) {
        super();

        this.mass = mass;
        this.shapeType = shapeType;
        this.jumpSpeed = jumpSpeed;
        this.friction = friction;
        this.restitution = restitution;
        this.collisionMargin = collisionMargin;
        this.animated = animated;
        this.group = group;
        this.mask = mask;

        id = PhysicsWorld.nextId++;

        notifyOnAdd(init);
        notifyOnLateUpdate(lateUpdate);
        notifyOnRemove(removeFromWorld);
    }

    inline function withMargin(f: Float): Float {
        return f + f * collisionMargin;
    }

    inline function zUpRotation(): Mat3 {
        return new Mat3(1, 0, 0, 0, 0, -1, 0, 1, 0);
    }

    public function notifyOnReady(f: Void->Void) {
        onReady = f;
        if (ready && onReady != null) onReady();
    }

    public function init() {
        if (ready) return;

        transform = object.transform;
        transform.buildMatrix();
        physics = PhysicsWorld.active;
        if (physics == null) {
            trace("KinematicCharacterController: PhysicsWorld is not active.");
            return;
        }

        shape = createShape();
        if (shape == null) return;

        var bodyConfig = new RigidBodyConfig();
        bodyConfig.type = RigidBodyType.KINEMATIC;
        bodyConfig.position.init(transform.worldx(), transform.worldy(), transform.worldz());
        bodyConfig.autoSleep = false;
        bodyConfig.linearDamping = 0;
        bodyConfig.angularDamping = 0;

        body = new oimo.dynamics.rigidbody.RigidBody(bodyConfig);
        body.addShape(shape);
        body.userData = this;

        tempIronQuat.fromMat(transform.world);
        tempQuat.init(tempIronQuat.x, tempIronQuat.y, tempIronQuat.z, tempIronQuat.w);
        body.setOrientation(tempQuat);

        physics.addKinematicCharacterController(this);

        ready = true;
        if (onReady != null) onReady();
    }

    function createShape(): Shape {
        var shapeConfig = new ShapeConfig();

        if (shapeType == ControllerShape.Box) {
            shapeConfig.geometry = new BoxGeometry(new Vec3(
                withMargin(transform.dim.x) * 0.5,
                withMargin(transform.dim.y) * 0.5,
                withMargin(transform.dim.z) * 0.5
            ));
        }
        else if (shapeType == ControllerShape.Sphere) {
            var width = transform.dim.x;
            if (transform.dim.y > width) width = transform.dim.y;
            if (transform.dim.z > width) width = transform.dim.z;
            shapeConfig.geometry = new SphereGeometry(withMargin(width) * 0.5);
        }
        else if (shapeType == ControllerShape.Capsule) {
            var height = transform.dim.z - transform.dim.x;
            if (height < 0) height = 0;
            shapeConfig.geometry = new CapsuleGeometry(
                withMargin(transform.dim.x) * 0.5,
                withMargin(height) * 0.5
            );
            shapeConfig.rotation = zUpRotation();
        }
        else if (shapeType == ControllerShape.Cylinder) {
            shapeConfig.geometry = new CylinderGeometry(
                withMargin(transform.dim.x) * 0.5,
                withMargin(transform.dim.z) * 0.5
            );
            shapeConfig.rotation = zUpRotation();
        }
        else if (shapeType == ControllerShape.Cone) {
            shapeConfig.geometry = new ConeGeometry(
                withMargin(transform.dim.x) * 0.5,
                withMargin(transform.dim.z) * 0.5
            );
            shapeConfig.rotation = zUpRotation();
        }
        else if (shapeType == ControllerShape.ConvexHull) {
            if (!Std.isOfType(object, MeshObject)) {
                trace("KinematicCharacterController: ConvexHull requires a MeshObject.");
                return null;
            }

            var mesh: MeshObject = cast object;
            var md: MeshData = mesh.data;
            if (md == null || md.geom == null || md.geom.positions == null) {
                trace("KinematicCharacterController: mesh data is not available.");
                return null;
            }

            var positions = md.geom.positions.values;
            if (positions == null || positions.length == 0) {
                trace("KinematicCharacterController: mesh positions are empty.");
                return null;
            }

            var sx = transform.scale.x * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
            var sy = transform.scale.y * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
            var sz = transform.scale.z * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
            var verts: Array<Vec3> = [];

            for (i in 0...Std.int(positions.length / 4)) {
                verts.push(new Vec3(
                    positions[i * 4    ] * sx,
                    positions[i * 4 + 1] * sy,
                    positions[i * 4 + 2] * sz
                ));
            }

            shapeConfig.geometry = new ConvexHullGeometry(verts);
        }
        else {
            trace("KinematicCharacterController: unsupported shape type " + shapeType);
            return null;
        }

        var volume = shapeConfig.geometry._volume;
        shapeConfig.density = mass > 0 && volume > 0 ? mass / volume : 1.0;
        shapeConfig.friction = friction;
        shapeConfig.restitution = restitution;
        shapeConfig.collisionGroup = group;
        shapeConfig.collisionMask = mask;

        var s = new Shape(shapeConfig);
        s.setIsTrigger(true);
        return s;
    }

    function lateUpdate() {
        if (!ready || body == null || transform == null) return;

        if (object.animation != null || animated) {
            syncTransform();
        }
        else {
            body.getPositionTo(tempPos);
            body.getOrientationTo(tempQuat);

            transform.loc.set(tempPos.x, tempPos.y, tempPos.z, 1.0);
            transform.rot.set(tempQuat.x, tempQuat.y, tempQuat.z, tempQuat.w);

            if (object.parent != null) {
                var parentTransform = object.parent.transform;
                transform.loc.x -= parentTransform.worldx();
                transform.loc.y -= parentTransform.worldy();
                transform.loc.z -= parentTransform.worldz();
            }

            transform.buildMatrix();
        }
    }

    public function syncTransform() {
        if (body == null || transform == null) return;

        transform.buildMatrix();

        tempPos.init(transform.worldx(), transform.worldy(), transform.worldz());
        tempIronQuat.fromMat(transform.world);
        tempQuat.init(tempIronQuat.x, tempIronQuat.y, tempIronQuat.z, tempIronQuat.w);

        body.setPosition(tempPos);
        body.setOrientation(tempQuat);
        body.wakeUp();

        // A teleport or animated pose invalidates the accumulated vertical state.
        verticalVelocity = 0.0;
        grounded = false;
    }

    public function fixedUpdate(dt: Float) {
        if (!ready || body == null || physics == null || transform == null) return;
        moveCharacter(dt);
    }

    // Oimo's broad-phase sweep test uses only the core of a convex shape, so
    // it can reject short sweeps that would already overlap the shape's margin.
    function sweepProbeExtension(): Float {
        var extension = MIN_SWEEP_DISTANCE;
        var maxDim = Math.max(transform.dim.x, Math.max(transform.dim.y, transform.dim.z));
        maxDim = withMargin(maxDim);
        if (maxDim > extension) extension = maxDim;
        return extension;
    }

    function moveCharacter(dt: Float) {
        body.getPositionTo(tempPos);
        body.getOrientationTo(tempQuat);
        tempIronQuat.set(tempQuat.x, tempQuat.y, tempQuat.z, tempQuat.w);

        movePosition.set(tempPos.x, tempPos.y, tempPos.z, 1.0);

        var wasGrounded = grounded;

        // Horizontal request plus the current vertical motion.
        moveRemaining.set(walkDirection.x, walkDirection.y, walkDirection.z, 0.0);

        if (gravityEnabled) {
            verticalVelocity -= gravity * dt;
            if (verticalVelocity < -fallSpeed) verticalVelocity = -fallSpeed;
        }

        moveRemaining.z += verticalVelocity * dt;

        grounded = false;
        slideMove();

        // Keep contact while walking down slopes or small steps.
        if (!grounded && wasGrounded && verticalVelocity <= 0.0) {
            if (probeGround(GROUND_SNAP_DISTANCE)) {
                grounded = true;
                verticalVelocity = 0.0;
            }
        }

        if (grounded && verticalVelocity < 0.0) verticalVelocity = 0.0;

        tempPos.init(movePosition.x, movePosition.y, movePosition.z);
        body.setPosition(tempPos);
        body.wakeUp();
    }

    function slideMove() {
        if (moveRemaining.length() <= MOVE_EPSILON) return;

        var extension = sweepProbeExtension();
        var iterations = 0;
        while (iterations < MAX_MOVE_ITERATIONS) {
            iterations++;

            // Resolve penetration first: an overlapped start makes the GJK cast
            // report "no hit", which would let the character tunnel through.
            var overlap = physics.overlapBody(body, movePosition, tempIronQuat, group, mask, true);
            if (overlap != null) {
                moveNormal.set(overlap.normal.x, overlap.normal.y, overlap.normal.z, 0.0);
                if (moveNormal.length() > MOVE_EPSILON) {
                    moveNormal.normalize();
                    var push = overlap.depth + MOVE_SKIN;

                    movePosition.addf(
                        moveNormal.x * push,
                        moveNormal.y * push,
                        moveNormal.z * push
                    );

                    if (moveNormal.z >= maxSlopeCos && verticalVelocity <= 0.0) {
                        grounded = true;
                        verticalVelocity = 0.0;
                    }

                    var into = moveRemaining.dot(moveNormal);
                    if (into < 0.0) {
                        moveRemaining.addf(
                            -moveNormal.x * into,
                            -moveNormal.y * into,
                            -moveNormal.z * into
                        );
                    }
                    blockClimb(moveNormal);
                }
                continue;
            }

            var remainingLength = moveRemaining.length();
            if (remainingLength <= MOVE_EPSILON) break;

            moveDirection.set(
                moveRemaining.x / remainingLength,
                moveRemaining.y / remainingLength,
                moveRemaining.z / remainingLength,
                0.0
            );

            // Start slightly behind the current position so an exact touch does
            // not make GJK reject the cast as already overlapping.
            var totalProbe = MOVE_BACK_OFF + remainingLength + extension;
            moveFrom.set(
                movePosition.x - moveDirection.x * MOVE_BACK_OFF,
                movePosition.y - moveDirection.y * MOVE_BACK_OFF,
                movePosition.z - moveDirection.z * MOVE_BACK_OFF,
                1.0
            );
            moveTo.set(
                moveFrom.x + moveDirection.x * totalProbe,
                moveFrom.y + moveDirection.y * totalProbe,
                moveFrom.z + moveDirection.z * totalProbe,
                1.0
            );

            var sweep = physics.sweepBody(body, moveFrom, moveTo, tempIronQuat, group, mask, true);
            if (sweep == null) {
                movePosition.add(moveRemaining);
                break;
            }

            var hitDistance = sweep.hitFraction * totalProbe - MOVE_BACK_OFF;
            if (hitDistance > remainingLength) {
                // The closest target is beyond this step's movement.
                movePosition.add(moveRemaining);
                break;
            }

            moveNormal.set(sweep.normal.x, sweep.normal.y, sweep.normal.z, 0.0);
            if (moveNormal.length() <= MOVE_EPSILON) {
                // Degenerate normal: stop rather than risk moving through.
                break;
            }
            moveNormal.normalize();

            var walkable = moveNormal.z >= maxSlopeCos;
            if (walkable && verticalVelocity <= 0.0) {
                grounded = true;
                verticalVelocity = 0.0;
            }
            else if (moveNormal.z < 0.0 && verticalVelocity > 0.0) {
                // Ceiling: stop upward movement.
                verticalVelocity = 0.0;
            }

            var fraction = hitDistance <= 0.0 ? 0.0 : hitDistance / remainingLength;
            if (fraction > 1.0) fraction = 1.0;

            movePosition.addf(
                moveRemaining.x * fraction,
                moveRemaining.y * fraction,
                moveRemaining.z * fraction
            );

            // Leave a small gap so the next sweep does not start touching.
            movePosition.addf(
                moveNormal.x * MOVE_SKIN,
                moveNormal.y * MOVE_SKIN,
                moveNormal.z * MOVE_SKIN
            );

            moveRemaining.mult(1.0 - fraction);
            var into = moveRemaining.dot(moveNormal);
            if (into < 0.0) {
                moveRemaining.addf(
                    -moveNormal.x * into,
                    -moveNormal.y * into,
                    -moveNormal.z * into
                );
            }
            blockClimb(moveNormal);
        }
    }

    // On surfaces steeper than the slope limit, keep the slide but discard any
    // uphill part so the character cannot climb vertical faces.
    function blockClimb(normal: Vec4) {
        if (normal.z >= maxSlopeCos) return;
        if (verticalVelocity > 0.0) return;
        if (moveRemaining.z > 0.0) moveRemaining.z = 0.0;
    }

    function probeGround(maxDistance: Float): Bool {
        if (maxDistance <= MOVE_EPSILON) return false;

        var extension = sweepProbeExtension();
        var totalProbe = MOVE_BACK_OFF + maxDistance + extension;

        moveFrom.set(movePosition.x, movePosition.y, movePosition.z + MOVE_BACK_OFF, 1.0);
        moveTo.set(movePosition.x, movePosition.y, movePosition.z + MOVE_BACK_OFF - totalProbe, 1.0);

        var sweep = physics.sweepBody(body, moveFrom, moveTo, tempIronQuat, group, mask, true);
        if (sweep == null) return false;

        var hitDistance = sweep.hitFraction * totalProbe - MOVE_BACK_OFF;
        if (hitDistance > maxDistance) return false;

        moveNormal.set(sweep.normal.x, sweep.normal.y, sweep.normal.z, 0.0);
        if (moveNormal.length() <= MOVE_EPSILON) return false;
        moveNormal.normalize();

        if (moveNormal.z < maxSlopeCos) return false;

        if (hitDistance < 0.0) hitDistance = 0.0;

        movePosition.z -= hitDistance;
        movePosition.addf(
            moveNormal.x * MOVE_SKIN,
            moveNormal.y * MOVE_SKIN,
            moveNormal.z * MOVE_SKIN
        );
        return true;
    }

    public function canJump(): Bool {
        return grounded;
    }

    public function onGround(): Bool {
        return grounded;
    }

    public function setJumpSpeed(speed: Float) {
        jumpSpeed = speed > 0.0 ? speed : 0.0;
    }

    public function setFallSpeed(speed: Float) {
        fallSpeed = speed > 0.0 ? speed : 0.0;
    }

    public function setMaxSlope(angleRadians: Float) {
        if (angleRadians < 0.0) angleRadians = 0.0;
        else if (angleRadians > Math.PI / 2) angleRadians = Math.PI / 2;
        maxSlope = angleRadians;
        maxSlopeCos = Math.cos(angleRadians);
    }

    public function getMaxSlope(): Float {
        return maxSlope;
    }

    public function setMaxJumpHeight(height: Float) {
        if (gravity > 0.0) jumpSpeed = Math.sqrt(2.0 * gravity * height);
    }

    public function setWalkDirection(direction: Vec4) {
        if (direction == null) {
            walkDirection.set(0.0, 0.0, 0.0, 1.0);
            return;
        }

        walkDirection.set(direction.x, direction.y, direction.z, 1.0);
    }

    public function setUpInterpolate(value: Bool) {
        upInterpolate = value;
    }

    public function jump() {
        if (!ready || body == null) return;

        verticalVelocity = jumpSpeed;
        grounded = false;
        body.wakeUp();
    }

    public function removeFromWorld() {
        if (physics != null) physics.removeKinematicCharacterController(this);

        physics = null;
        body = null;
        shape = null;
        ready = false;
    }

    public function setGroup(group: Int) {
        if (this.group == group) return;
        this.group = group;
        if (physics != null) physics.updateKinematicCharacterController(this);
        else if (shape != null) shape.setCollisionGroup(group);
    }

    public function setMask(mask: Int) {
        if (this.mask == mask) return;
        this.mask = mask;
        if (physics != null) physics.updateKinematicCharacterController(this);
        else if (shape != null) shape.setCollisionMask(mask);
    }

    public function getOverlappingBodies(): Array<RigidBody> {
        return physics != null ? physics.getOverlappingBodies(this) : [];
    }

    public function activate() {
        if (body != null) body.wakeUp();
    }

    public function disableGravity() {
        gravityEnabled = false;
    }

    public function enableGravity() {
        gravityEnabled = true;
    }

    public function setGravity(value: Float) {
        gravity = value;
    }

    public function setActivationState(state: ControllerActivationState) {
        if (body != null) body.wakeUp();
    }

    public function setFriction(friction: Float) {
        this.friction = friction;
        if (shape != null) shape.setFriction(friction);
    }
}

enum abstract ControllerShape(Int) from Int to Int {
    var Box = 0;
    var Sphere = 1;
    var Capsule = 5;
    var Cylinder = 6;
    var Cone = 4;
    var ConvexHull = 2;
}

enum abstract ControllerActivationState(Int) from Int to Int {
    var Active = 1;
    var NoDeactivation = 4;
    var NoSimulation = 5;
}
