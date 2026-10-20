package armory.trait.physics.oimo;

import iron.math.Vec4;

class KinematicCharacterController extends iron.Trait {
    var ready: Bool = false;
    public var onReady: Void->Void = null;

    public function new(mass = 1.0, shape: ControllerShape = ControllerShape.Capsule, jumpSpeed = 8.0, friction = 0.5, restitution = 0.0, collisionMargin = 0.0, animated = false, group = 1, mask = 1) {
        super();
    }

    public function notifyOnReady(f: Void->Void) {
        onReady = f;
        if (ready && onReady != null) onReady();
    }

    public function init() {

    }

    public function canJump(): Bool {
        return false;
    }

    public function onGround(): Bool {
        return false;
    }

    public function setJumpSpeed(speed: Float) {

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

    }

    public function setGroup(group: Int) {

    }

    public function setMask(mask: Int) {

    }

    public function getOverlappingBodies(): Array<RigidBody> {
        return [];
    }

    public function activate() {

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

    }

    public function syncTransform() {

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