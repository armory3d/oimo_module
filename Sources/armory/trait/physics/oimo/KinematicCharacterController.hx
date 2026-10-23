package armory.trait.physics.oimo;

import armory.trait.physics.oimo.PhysicsWorld.ConvexHit;
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
    public var ready: Bool = false;
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

    var walkDirection: Vec4 = new Vec4();
    var up: Vec4 = new Vec4(0.0, 0.0, 1.0, 0.0);
    var stepHeight: Float = 0.5;
    var gravity: Float = 9.8 * 3.0;
    var fallSpeed: Float = 55.0;
    var verticalVelocity: Float = 0.0;
    var verticalOffset: Float = 0.0;
    var maxSlope: Float = Math.PI / 4;
    var maxSlopeCos: Float = Math.cos(Math.PI / 4);
    var slopeTan: Float = 1.0;
    var currentStepOffset: Float = 0.0;
    var grounded: Bool = false;
    var stepBlocked: Bool = false;
    var footRadius: Float = 0.0;
    var lastDirection: Vec4 = new Vec4(0.0, 0.0, 0.0, 0.0);
    var maxJumpHeight: Float = 0.0;
    var upInterpolate: Bool = true;
    var defaultJumpSpeed: Float = 8.0;
    var jumpAxis: Vec4 = new Vec4(0.0, 0.0, 1.0, 0.0);

    var tempPos: Vec3 = new Vec3();
    var tempQuat: oimo.common.Quat = new oimo.common.Quat();
    var tempIronQuat: Quat = new Quat();

    var movePosition: Vec4 = new Vec4();
    var moveRemaining: Vec4 = new Vec4();
    var moveDirection: Vec4 = new Vec4();
    var moveNormal: Vec4 = new Vec4();
    var moveFrom: Vec4 = new Vec4();
    var moveTo: Vec4 = new Vec4();

    var planeNormals: Array<Vec4> = [new Vec4(), new Vec4(), new Vec4()];
    var planeCount: Int = 0;

    var fallMotion: Vec4 = new Vec4();
    var fallNormal: Vec4 = new Vec4();
    var lastValidPosition: Vec4 = new Vec4();
    var lastValidOk: Bool = false;

    static inline var CONTACT_CLEARANCE: Float = 0.01;
    static inline var MOVE_EPSILON: Float = 1e-6;
    static inline var NORMAL_EPSILON: Float = 0.0001;
    // Slack on the slope limit, in cosine units. 0.007 is ~0.01 rad at 45
    // degrees (Godot uses the same angle slack): a surface sitting exactly on
    // the limit must not flip between "floor" and "wall" from tick to tick.
    static inline var SLOPE_EPSILON: Float = 0.007;
    static inline var MAX_MOVE_ITERATIONS: Int = 10;
    static inline var MAX_FALL_ITERATIONS: Int = 3;
    static inline var MAX_SLIDE_PLANES: Int = 3;
    static inline var RECOVER_ITERATIONS: Int = 4;

    public function new(mass = 1.0, shapeType: ControllerShape = ControllerShape.Capsule, jumpSpeed = 8.0, friction = 0.5, restitution = 0.0, collisionMargin = 0.0, animated = false, group = 1, mask = 1) {
        super();

        this.mass = mass;
        this.shapeType = shapeType;
        this.jumpSpeed = jumpSpeed;
        this.defaultJumpSpeed = jumpSpeed;
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

        var width = transform.dim.x > transform.dim.y ? transform.dim.x : transform.dim.y;
        footRadius = withMargin(width) * 0.5;

        jumpAxis.set(up.x, up.y, up.z, 0.0);

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
            var width = transform.dim.x;
            if (transform.dim.y > width) width = transform.dim.y;
            if (transform.dim.z > width) width = transform.dim.z;
            shapeConfig.geometry = new SphereGeometry(withMargin(width) * 0.5);
            trace("KinematicCharacterController: unsupported shape type " + shapeType + ". Fallinback to SPHERE." );
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
        verticalOffset = 0.0;
        currentStepOffset = 0.0;
        grounded = false;
        stepBlocked = false;
        lastDirection.set(0.0, 0.0, 0.0, 0.0);
        lastValidOk = false;
    }

    public function fixedUpdate(dt: Float) {
        if (!ready || body == null || physics == null || transform == null) return;
        moveCharacter(dt);
    }

    inline function upDot(v: Vec4): Float {
        return up.x * v.x + up.y * v.y + up.z * v.z;
    }

    inline function upShift(v: Vec4, distance: Float): Void {
        v.x += up.x * distance;
        v.y += up.y * distance;
        v.z += up.z * distance;
    }

    inline function horizontalLength(v: Vec4): Float {
        var along = upDot(v);
        var x = v.x - up.x * along;
        var y = v.y - up.y * along;
        var z = v.z - up.z * along;
        return Math.sqrt(x * x + y * y + z * z);
    }

    function moveCharacter(dt: Float) {
        body.getPositionTo(tempPos);
        body.getOrientationTo(tempQuat);
        tempIronQuat.set(tempQuat.x, tempQuat.y, tempQuat.z, tempQuat.w);

        movePosition.set(tempPos.x, tempPos.y, tempPos.z, 1.0);

        lastValidOk = recoverPenetration();
        lastValidPosition.set(movePosition.x, movePosition.y, movePosition.z, 1.0);

        // Same vertical velocity integration and clamping as Bullet.
        verticalVelocity -= gravity * dt;
        if (verticalVelocity > jumpSpeed) verticalVelocity = jumpSpeed;
        if (verticalVelocity < -fallSpeed) verticalVelocity = -fallSpeed;
        verticalOffset = verticalVelocity * dt;

        var startX = movePosition.x;
        var startY = movePosition.y;
        var startZ = movePosition.z;

        currentStepOffset = 0.0;

        if (verticalVelocity > 0.0) {
            // Rising: apply the jump displacement along the jump axis, stopped only by a ceiling.
            var rise = riseAlong(jumpAxis, verticalOffset);
            stepForwardAndStrafe();
            // A ceiling ends the jump: cancel the upward velocity instead of pressing
            // into it for the rest of the ascent.
            if (rise < verticalOffset - MOVE_EPSILON) {
                verticalVelocity = 0.0;
                verticalOffset = 0.0;
            }
            grounded = false;
        }
        else {
            var walked = stepForwardAndStrafe();
            // part of the walk direction, not its full length.
            var walkLength = horizontalLength(walkDirection);
            var steppedUp = false;

            if (stepBlocked && walked < walkLength - MOVE_EPSILON) {
                var walkedX = movePosition.x;
                var walkedY = movePosition.y;
                var walkedZ = movePosition.z;

                movePosition.set(startX, startY, startZ, 1.0);
                var rise = riseAlong(up, stepHeight);

                if (rise > MOVE_EPSILON) {
                    var stepped = stepForwardAndStrafe();
                    if (stepped > walked + MOVE_EPSILON) {
                        currentStepOffset = rise;
                        steppedUp = true;
                    }
                    else {
                        movePosition.set(walkedX, walkedY, walkedZ, 1.0);
                    }
                }
            }

            var dx = movePosition.x - startX;
            var dy = movePosition.y - startY;
            var dz = movePosition.z - startZ;
            var alongUp = up.x * dx + up.y * dy + up.z * dz;
            dx -= up.x * alongUp;
            dy -= up.y * alongUp;
            dz -= up.z * alongUp;
            var horizontalDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);

            stepDown(dt, horizontalDistance, steppedUp);
        }

        if (!recoverPenetration() && lastValidOk) {
            movePosition.set(lastValidPosition.x, lastValidPosition.y, lastValidPosition.z, 1.0);
            recoverPenetration();
            grounded = false;
        }

        tempPos.init(movePosition.x, movePosition.y, movePosition.z);
        body.setPosition(tempPos);
        body.wakeUp();
    }

    // Moves the character along `dir` by up to `distance`, stopping short of ceilings.
    // Returns the distance actually travelled.
    function riseAlong(dir: Vec4, distance: Float): Float {
        if (distance <= MOVE_EPSILON) return 0.0;

        moveFrom.set(movePosition.x, movePosition.y, movePosition.z, 1.0);
        moveTo.set(
            moveFrom.x + dir.x * distance,
            moveFrom.y + dir.y * distance,
            moveFrom.z + dir.z * distance,
            1.0
        );

        // Unfiltered: only ceilings can shorten the rise, slopes and walls are classified afterwards.
        var sweep = physics.sweepBody(body, moveFrom, moveTo, tempIronQuat, group, mask, true);
        var travelled = distance;
        if (sweep != null) {
            moveNormal.set(sweep.normal.x, sweep.normal.y, sweep.normal.z, 0.0);
            if (moveNormal.length() > MOVE_EPSILON) {
                moveNormal.normalize();
                if (upDot(moveNormal) < -NORMAL_EPSILON) {
                    var hitDistance = sweep.hitFraction * distance - CONTACT_CLEARANCE;
                    if (hitDistance < 0.0) hitDistance = 0.0;
                    if (hitDistance < travelled) travelled = hitDistance;
                }
            }
        }

        movePosition.x += dir.x * travelled;
        movePosition.y += dir.y * travelled;
        movePosition.z += dir.z * travelled;
        return travelled;
    }

    function recoverPenetration(): Bool {
        var iterations = 0;
        while (iterations < RECOVER_ITERATIONS) {
            iterations++;

            var overlap = physics.overlapBody(body, movePosition, tempIronQuat, group, mask, true);
            if (overlap == null) return true;
            if (overlap.depth <= CONTACT_CLEARANCE) return true;

            moveNormal.set(overlap.normal.x, overlap.normal.y, overlap.normal.z, 0.0);
            if (moveNormal.length() <= MOVE_EPSILON) return false;
            moveNormal.normalize();

            var push = overlap.depth + CONTACT_CLEARANCE;
            movePosition.addf(moveNormal.x * push, moveNormal.y * push, moveNormal.z * push);
        }

        var overlap = physics.overlapBody(body, movePosition, tempIronQuat, group, mask, true);
        return overlap == null || overlap.depth <= CONTACT_CLEARANCE;
    }

    function stepForwardAndStrafe(): Float {
        moveRemaining.set(walkDirection.x, walkDirection.y, walkDirection.z, 0.0);

        var along = upDot(walkDirection);
        var hx = walkDirection.x - up.x * along;
        var hy = walkDirection.y - up.y * along;
        var hz = walkDirection.z - up.z * along;
        var hl = Math.sqrt(hx * hx + hy * hy + hz * hz);
        if (hl > MOVE_EPSILON) lastDirection.set(hx / hl, hy / hl, hz / hl, 0.0);

        return slideMove();
    }

    function slideMove(): Float {
        var startX = movePosition.x;
        var startY = movePosition.y;
        var startZ = movePosition.z;

        stepBlocked = false;
        planeCount = 0;

        var iterations = 0;
        while (iterations < MAX_MOVE_ITERATIONS) {
            iterations++;

            var remainingLength = moveRemaining.length();
            if (remainingLength <= MOVE_EPSILON) break;

            moveDirection.set(
                moveRemaining.x / remainingLength,
                moveRemaining.y / remainingLength,
                moveRemaining.z / remainingLength,
                0.0
            );

            moveFrom.set(movePosition.x, movePosition.y, movePosition.z, 1.0);
            moveTo.set(
                moveFrom.x + moveRemaining.x,
                moveFrom.y + moveRemaining.y,
                moveFrom.z + moveRemaining.z,
                1.0
            );

            var sweep = physics.sweepBody(body, moveFrom, moveTo, tempIronQuat, group, mask, true, 0.0, up);
            if (sweep == null) {
                movePosition.x += moveRemaining.x;
                movePosition.y += moveRemaining.y;
                movePosition.z += moveRemaining.z;
                break;
            }

            moveNormal.set(sweep.normal.x, sweep.normal.y, sweep.normal.z, 0.0);
            if (moveNormal.length() <= MOVE_EPSILON) break;
            moveNormal.normalize();

            var hitDistance = sweep.hitFraction * remainingLength;
            if (hitDistance < 0.0) hitDistance = 0.0;
            else if (hitDistance > remainingLength) hitDistance = remainingLength;

            movePosition.x += moveDirection.x * hitDistance;
            movePosition.y += moveDirection.y * hitDistance;
            movePosition.z += moveDirection.z * hitDistance;

            var into = moveNormal.dot(moveDirection);
            if (into < 0.0) {
                movePosition.addf(
                    moveNormal.x * CONTACT_CLEARANCE,
                    moveNormal.y * CONTACT_CLEARANCE,
                    moveNormal.z * CONTACT_CLEARANCE
                );

                rememberPlane();
            }

            if (upDot(moveNormal) < maxSlopeCos - SLOPE_EPSILON) stepBlocked = true;

            moveRemaining.mult(1.0 - hitDistance / remainingLength);

            for (i in 0...planeCount) {
                var plane = planeNormals[i];
                var along = moveRemaining.dot(plane);
                if (along < 0.0) {
                    moveRemaining.addf(
                        -plane.x * along,
                        -plane.y * along,
                        -plane.z * along
                    );
                }
            }
            blockClimb(moveNormal);

            if (moveRemaining.dot(walkDirection) <= 0.0) break;
        }

        var dx = movePosition.x - startX;
        var dy = movePosition.y - startY;
        var dz = movePosition.z - startZ;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    function rememberPlane() {
        if (planeCount >= MAX_SLIDE_PLANES) return;

        for (i in 0...planeCount) {
            if (planeNormals[i].dot(moveNormal) > 0.999) return;
        }

        var plane = planeNormals[planeCount];
        plane.set(moveNormal.x, moveNormal.y, moveNormal.z, 0.0);
        planeCount++;
    }

    function blockClimb(normal: Vec4) {
        if (upDot(normal) >= maxSlopeCos - SLOPE_EPSILON) return;
        if (verticalVelocity > 0.0) return;

        var uphill = upDot(moveRemaining);
        if (uphill > 0.0) {
            moveRemaining.addf(-up.x * uphill, -up.y * uphill, -up.z * uphill);
        }
    }

    function sweepDown(drop: Float, ahead: Float): ConvexHit {
        moveFrom.set(
            movePosition.x + lastDirection.x * ahead,
            movePosition.y + lastDirection.y * ahead,
            movePosition.z + lastDirection.z * ahead,
            1.0
        );
        moveTo.set(moveFrom.x, moveFrom.y, moveFrom.z, 1.0);
        upShift(moveTo, -drop);

        return physics.sweepBody(body, moveFrom, moveTo, tempIronQuat, group, mask, true, maxSlopeCos - SLOPE_EPSILON, up);
    }

    function dropDown(distance: Float): Bool {
        if (distance <= MOVE_EPSILON) return false;

        fallMotion.set(-up.x * distance, -up.y * distance, -up.z * distance, 0.0);

        var hit = false;
        var iterations = 0;
        while (iterations < MAX_FALL_ITERATIONS) {
            iterations++;

            var remainingLength = fallMotion.length();
            if (remainingLength <= MOVE_EPSILON) break;

            moveDirection.set(
                fallMotion.x / remainingLength,
                fallMotion.y / remainingLength,
                fallMotion.z / remainingLength,
                0.0
            );

            moveFrom.set(movePosition.x, movePosition.y, movePosition.z, 1.0);
            moveTo.set(
                moveFrom.x + fallMotion.x,
                moveFrom.y + fallMotion.y,
                moveFrom.z + fallMotion.z,
                1.0
            );

            // Unfiltered: any surface ends the fall, walkable or not.
            var sweep = physics.sweepBody(body, moveFrom, moveTo, tempIronQuat, group, mask, true);
            if (sweep == null) {
                movePosition.x += fallMotion.x;
                movePosition.y += fallMotion.y;
                movePosition.z += fallMotion.z;
                break;
            }

            hit = true;
            moveNormal.set(sweep.normal.x, sweep.normal.y, sweep.normal.z, 0.0);
            if (moveNormal.length() <= MOVE_EPSILON) break;
            moveNormal.normalize();
            fallNormal.set(moveNormal.x, moveNormal.y, moveNormal.z, 0.0);

            var hitDistance = sweep.hitFraction * remainingLength;
            if (hitDistance < 0.0) hitDistance = 0.0;
            else if (hitDistance > remainingLength) hitDistance = remainingLength;
            movePosition.addf(
                moveDirection.x * hitDistance,
                moveDirection.y * hitDistance,
                moveDirection.z * hitDistance
            );

            var into = moveNormal.dot(moveDirection);
            if (into < 0.0) {
                movePosition.addf(
                    moveNormal.x * CONTACT_CLEARANCE,
                    moveNormal.y * CONTACT_CLEARANCE,
                    moveNormal.z * CONTACT_CLEARANCE
                );
            }

            fallMotion.mult(1.0 - hitDistance / remainingLength);
            var along = fallMotion.dot(moveNormal);
            if (along < 0.0) {
                fallMotion.addf(
                    -moveNormal.x * along,
                    -moveNormal.y * along,
                    -moveNormal.z * along
                );
            }
            if (upDot(fallMotion) >= -MOVE_EPSILON) break;
        }

        return hit;
    }

    function stepDown(dt: Float, horizontalDistance: Float, steppedUp: Bool) {
        if (verticalVelocity > 0.0) {
            grounded = false;
            return;
        }

        var wasGrounded = grounded;

        var gravityDrop = -verticalVelocity * dt;
        if (gravityDrop < 0.0) gravityDrop = 0.0;

        var slopeDrop = horizontalDistance > 0.0 ? horizontalDistance * slopeTan : 0.0;
        var snap = gravityDrop > slopeDrop ? gravityDrop : slopeDrop;
        if (snap > stepHeight) snap = stepHeight;

        var reach = currentStepOffset + snap + CONTACT_CLEARANCE;
        if (!wasGrounded) reach = gravityDrop + CONTACT_CLEARANCE;

        var sweep = sweepDown(reach, 0.0);
        var supportDistance = sweep == null ? -1.0 : sweep.hitFraction * reach;

        if (steppedUp || supportDistance < 0.0) {
            var aheadSweep = sweepDown(reach, footRadius);
            if (aheadSweep != null) {
                var aheadDistance = aheadSweep.hitFraction * reach;
                if (supportDistance < 0.0 || aheadDistance < supportDistance) supportDistance = aheadDistance;
            }
        }

        if (supportDistance >= 0.0) {
            var travel = supportDistance - CONTACT_CLEARANCE;
            if (travel > 0.0) upShift(movePosition, -travel);

            verticalVelocity = 0.0;
            verticalOffset = 0.0;
            grounded = true;
        }
        else {
            var stopped = dropDown(currentStepOffset + gravityDrop);
            if (stopped && upDot(fallNormal) >= maxSlopeCos - SLOPE_EPSILON) {
                // A walkable surface ended the fall even though the ground probe
                // missed it (an edge-on triangle, or a dropped cast): land on it
                // instead of falling through it.
                verticalVelocity = 0.0;
                verticalOffset = 0.0;
                grounded = true;
            }
            else {
                grounded = false;
            }
        }

        currentStepOffset = 0.0;
    }

    public function canJump(): Bool {
        return onGround();
    }

    public function onGround(): Bool {
        return grounded;
    }

    public function setJumpSpeed(speed: Float) {
        jumpSpeed = speed > 0.0 ? speed : 0.0;
        defaultJumpSpeed = jumpSpeed;
    }

    public function setFallSpeed(speed: Float) {
        fallSpeed = speed > 0.0 ? speed : 0.0;
    }

    public function setMaxSlope(angleRadians: Float) {
        if (angleRadians < 0.0) angleRadians = 0.0;
        else if (angleRadians > Math.PI / 2) angleRadians = Math.PI / 2;
        maxSlope = angleRadians;
        maxSlopeCos = Math.cos(angleRadians);
        slopeTan = Math.tan(angleRadians > 1.5 ? 1.5 : angleRadians);
    }

    public function getMaxSlope(): Float {
        return maxSlope;
    }

    public function setMaxJumpHeight(height: Float) {
        maxJumpHeight = height;
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

    public function jump(v: Vec4 = null) {
        if (!ready || body == null) return;

        if (v == null || v.x * v.x + v.y * v.y + v.z * v.z <= MOVE_EPSILON) {
            jumpSpeed = defaultJumpSpeed;
            jumpAxis.set(up.x, up.y, up.z, 0.0);
        }
        else {
            jumpSpeed = v.length();
            jumpAxis.set(v.x, v.y, v.z, 0.0);
            jumpAxis.normalize();
        }

        verticalVelocity = jumpSpeed;
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
        gravity = 0.0;
    }

    public function enableGravity() {
        if (physics == null) return;
        // Armory's Bullet wrapper uses three times the world gravity.
        gravity = Math.abs(upDot(physics.getGravity())) * 3.0;
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
