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

    // Reusable objects for transform synchronization.
    var tempPos: Vec3 = new Vec3();
    var tempQuat: oimo.common.Quat = new oimo.common.Quat();
    var tempIronQuat: Quat = new Quat();

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
    }

    public function fixedUpdate(dt: Float) {
        // TODO: character movement
    }

    public function canJump(): Bool {
        return false;
    }

    public function onGround(): Bool {
        return false;
    }

    public function setJumpSpeed(speed: Float) {
        jumpSpeed = speed;
    }

    public function setFallSpeed(speed: Float) {

    }

    public function setMaxSlope(angleRadians: Float) {

    }

    public function getMaxSlope(): Float {
        return 0.0;
    }

    public function setMaxJumpHeight(height: Float) {

    }

    public function setWalkDirection(direction: Vec4) {

    }

    public function setUpInterpolate(value: Bool) {

    }

    public function jump() {

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

    }

    public function enableGravity() {

    }

    public function setGravity(gravity: Float) {

    }

    public function setActivationState(state: ControllerActivationState) {

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
