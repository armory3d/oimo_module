package armory.trait.physics.oimo;

import armory.trait.physics.oimo.CharacterQueries.CharacterContacts;
import armory.trait.physics.oimo.CharacterQueries.CharacterSweep;
import iron.data.MeshData;
import iron.math.Quat;
import iron.math.Vec4;
import iron.object.MeshObject;
import iron.object.Transform;
import iron.system.Time;

import oimo.collision.geometry.*;
import oimo.common.Mat3;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.RigidBodyType;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

/**
	Kinematic character controller with the public API of Armory's Bullet one (JS/Krom signatures).

	The step structure follows Bullet's btKinematicCharacterController; recovery and sliding follow
    Bullet, Godot's CharacterBody3D and Jolt's CharacterVirtual.
**/
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
	public var interpolate: Bool;

	static inline var SKIN: Float = 0.01;             // Gap kept from surfaces after a move
	static inline var RECOVER_SLOP: Float = 0.005;    // Overlap left alone, Oimo's contact slop
	static inline var RECOVER_PASSES: Int = 4;
	static inline var SLIDE_ITERATIONS: Int = 4;
	static inline var MIN_MOVE: Float = 1e-5;
	static inline var MIN_INTO: Float = 0.2;          // Caps the skin back-off on grazing hits
	static inline var SLOPE_EPSILON: Float = 0.007;
	static inline var GROUND_TOLERANCE: Float = 0.05; // Walkable ground this far past a steep contact still holds
	static inline var STEP_MIN_GAIN: Float = 0.5;     // Share of the blocked walk a step-up must win back
	static inline var CREASE_MAX_COS: Float = 0.984;  // Creases only between surfaces over ~10 degrees apart, Jolt's value

	var shapeType: ControllerShape;
	var queries: CharacterQueries = null;

	// Settings, Bullet's defaults
	var up: Vec4 = new Vec4(0.0, 0.0, 1.0, 0.0);
	var stepHeight: Float = 0.5;
	var jumpSpeed: Float;
	var gravity: Float = 9.8 * 3.0;
	var fallSpeed: Float = 55.0;
	var maxSlope: Float = Math.PI / 4;
	var maxSlopeCos: Float = Math.cos(Math.PI / 4);
	// Stored for API parity; Bullet doesn't use them either.
	var maxJumpHeight: Float = 0.0;
	var upInterpolate: Bool = true;

	// State
	var walkDirection: Vec4 = new Vec4(0.0, 0.0, 0.0, 0.0);
	var verticalVelocity: Float = 0.0;
	var grounded: Bool = false;
	var scaleX: Float = 1.0;
	var scaleY: Float = 1.0;
	var scaleZ: Float = 1.0;
	var previousPosition: Vec4 = new Vec4(); // Body positions of the last two steps, for interpolation
	var currentPosition: Vec4 = new Vec4();

	// Shape extents from the body origin: down to the feet, and out to the side
	var halfHeight: Float = 0.0;
	var footRadius: Float = 0.0;

	// Working pose during a step
	var position: Vec4 = new Vec4();
	var orientation: Quat = new Quat();

	// What the last slide hit
	var slideCeiling: Bool = false;
	var slideLanded: Bool = false;
	var slideStepCandidate: Bool = false;
	var slideWallNormal: Vec4 = new Vec4();

	// Scratch
	var sweep: CharacterSweep = new CharacterSweep();
	var contacts: CharacterContacts = new CharacterContacts();
	var motion: Vec4 = new Vec4();
	var originalMotion: Vec4 = new Vec4();
	var target: Vec4 = new Vec4();
	var normal: Vec4 = new Vec4();
	var previousNormal: Vec4 = new Vec4();
	var hasPrevious: Bool = false;
	var candidate: Vec4 = new Vec4();
	var push: Vec4 = new Vec4();
	var walkStart: Vec4 = new Vec4();
	var walkEnd: Vec4 = new Vec4();
	var walkUnit: Vec4 = new Vec4();
	var tempPos: Vec3 = new Vec3();
	var tempQuat: oimo.common.Quat = new oimo.common.Quat();

	public function new(mass = 1.0, shapeType: ControllerShape = ControllerShape.Capsule, jumpSpeed = 8.0, friction = 0.5, restitution = 0.0, collisionMargin = 0.0, animated = false, group = 1, mask = 1, interpolate = false) {
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
		this.interpolate = interpolate;

		id = PhysicsWorld.nextId++;

		notifyOnAdd(init);
		notifyOnLateUpdate(lateUpdate);
		notifyOnRemove(removeFromWorld);
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
		queries = physics.characterQueries;

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

		orientation.fromMat(transform.world);
		tempQuat.init(orientation.x, orientation.y, orientation.z, orientation.w);
		body.setOrientation(tempQuat);

		updateExtents();
		scaleX = transform.scale.x;
		scaleY = transform.scale.y;
		scaleZ = transform.scale.z;
		currentPosition.set(transform.worldx(), transform.worldy(), transform.worldz(), 1.0);
		previousPosition.setFrom(currentPosition);

		physics.addKinematicCharacterController(this);

		ready = true;
		if (onReady != null) onReady();
	}

	inline function withMargin(f: Float): Float {
		return f + f * collisionMargin;
	}

	inline function zUpRotation(): Mat3 {
		return new Mat3(1, 0, 0, 0, 0, -1, 0, 1, 0);
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
			shapeConfig.geometry = createConvexHull();
			if (shapeConfig.geometry == null) return null;
		}
		else {
			if (shapeType != ControllerShape.Sphere) {
				trace("KinematicCharacterController: unsupported shape type " + shapeType + ". Falling back to Sphere.");
			}
			var width = Math.max(transform.dim.x, Math.max(transform.dim.y, transform.dim.z));
			shapeConfig.geometry = new SphereGeometry(withMargin(width) * 0.5);
		}

		var volume = shapeConfig.geometry._volume;
		shapeConfig.density = mass > 0 && volume > 0 ? mass / volume : 1.0;
		shapeConfig.friction = friction;
		shapeConfig.restitution = restitution;
		shapeConfig.collisionGroup = group;
		shapeConfig.collisionMask = mask;

		return new Shape(shapeConfig);
	}

	function createConvexHull(): ConvexHullGeometry {
		if (!Std.isOfType(object, MeshObject)) {
			trace("KinematicCharacterController: ConvexHull requires a MeshObject.");
			return null;
		}

		var mesh: MeshObject = cast object;
		var md: MeshData = mesh.data;
		var positions = md != null && md.geom != null && md.geom.positions != null ? md.geom.positions.values : null;
		if (positions == null || positions.length == 0) {
			trace("KinematicCharacterController: mesh positions are not available.");
			return null;
		}

		var sx = transform.scale.x * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
		var sy = transform.scale.y * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
		var sz = transform.scale.z * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
		var verts: Array<Vec3> = [];
		for (i in 0...Std.int(positions.length / 4)) {
			verts.push(new Vec3(positions[i * 4] * sx, positions[i * 4 + 1] * sy, positions[i * 4 + 2] * sz));
		}
		return new ConvexHullGeometry(verts);
	}

	// Feet and side distances from the body origin, from the shape's local bounds.
	function updateExtents() {
		var aabb = new Aabb();
		shape._geom._computeAabb(aabb, shape._localTransform);
		var min = aabb.getMin();
		var max = aabb.getMax();
		halfHeight = -min.z;
		footRadius = Math.max(Math.max(-min.x, max.x), Math.max(-min.y, max.y));
	}

	function lateUpdate() {
		if (!ready || body == null || transform == null) return;

		if (object.animation != null || animated) {
			syncTransform();
			return;
		}

		if (interpolate) {
			var t = Time.fixedStepInterpolation;
			if (t < 0.0) t = 0.0;
			else if (t > 1.0) t = 1.0;
			transform.loc.set(
				previousPosition.x + (currentPosition.x - previousPosition.x) * t,
				previousPosition.y + (currentPosition.y - previousPosition.y) * t,
				previousPosition.z + (currentPosition.z - previousPosition.z) * t,
				1.0
			);
		}
		else {
			transform.loc.set(currentPosition.x, currentPosition.y, currentPosition.z, 1.0);
		}
		body.getOrientationTo(tempQuat);
		transform.rot.set(tempQuat.x, tempQuat.y, tempQuat.z, tempQuat.w);

		if (object.parent != null) {
			var parentTransform = object.parent.transform;
			transform.loc.x -= parentTransform.worldx();
			transform.loc.y -= parentTransform.worldy();
			transform.loc.z -= parentTransform.worldz();
		}

		transform.buildMatrix();
	}

	// Moves the body to the object's transform, like Bullet's. Keeps the vertical velocity and ground state.
	public function syncTransform() {
		if (body == null || transform == null) return;

		transform.buildMatrix();

		currentPosition.set(transform.worldx(), transform.worldy(), transform.worldz(), 1.0);
		previousPosition.setFrom(currentPosition); // Don't interpolate across a teleport
		tempPos.init(currentPosition.x, currentPosition.y, currentPosition.z);
		orientation.fromMat(transform.world);
		tempQuat.init(orientation.x, orientation.y, orientation.z, orientation.w);

		body.setPosition(tempPos);
		body.setOrientation(tempQuat);

		if (scaleX != transform.scale.x || scaleY != transform.scale.y || scaleZ != transform.scale.z) setScale();
		activate();
	}

	// Rebuilds the shape for the object's new scale, like Bullet's setLocalScaling.
	function setScale() {
		var newShape = createShape();
		if (newShape == null) return;
		body.removeShape(shape);
		shape = newShape;
		body.addShape(shape);
		updateExtents();
		scaleX = transform.scale.x;
		scaleY = transform.scale.y;
		scaleZ = transform.scale.z;
	}

	public function fixedUpdate(dt: Float) {
		if (!ready || body == null || physics == null || transform == null) return;

		body.getPositionTo(tempPos);
		body.getOrientationTo(tempQuat);
		position.set(tempPos.x, tempPos.y, tempPos.z, 1.0);
		orientation.set(tempQuat.x, tempQuat.y, tempQuat.z, tempQuat.w);

		recover();

		// Bullet's vertical velocity integration and clamping
		verticalVelocity -= gravity * dt;
		if (verticalVelocity > jumpSpeed) verticalVelocity = jumpSpeed;
		if (verticalVelocity < -fallSpeed) verticalVelocity = -fallSpeed;

		var wasGrounded = grounded;
		if (verticalVelocity > 0.0) {
			grounded = false;
			rise(verticalVelocity * dt);
		}

		walk(grounded);

		if (verticalVelocity <= 0.0) settle(verticalVelocity * -dt, wasGrounded);

		tempPos.init(position.x, position.y, position.z);
		body.setPosition(tempPos);
		previousPosition.setFrom(currentPosition);
		currentPosition.setFrom(position);
	}

	// Pushes the character out of what it overlaps, up to a skin gap. Every contact counts, minus
	// what this pass already pushed along its normal, so a corner settles instead of bouncing.
	function recover() {
		for (pass in 0...RECOVER_PASSES) {
			var count = queries.overlap(body, position, orientation, group, mask, contacts);
			if (count == 0) return;

			push.set(0.0, 0.0, 0.0, 0.0);
			for (i in 0...count) {
				var depth = contacts.depths[i];
				if (depth <= RECOVER_SLOP) continue;
				var n = contacts.normals[i];
				var needed = depth + SKIN - push.dot(n);
				if (needed > 0.0) push.addf(n.x * needed, n.y * needed, n.z * needed);
			}
			if (push.x == 0.0 && push.y == 0.0 && push.z == 0.0) return;

			position.addf(push.x, push.y, push.z);
		}
	}

	function rise(distance: Float) {
		motion.set(up.x * distance, up.y * distance, up.z * distance, 0.0);
		slide(false);
		// A ceiling ends the jump instead of pressing into it for the rest of the ascent.
		if (slideCeiling) verticalVelocity = 0.0;
	}

	function walk(onGround: Bool) {
		motion.set(walkDirection.x, walkDirection.y, walkDirection.z, 0.0);
		var length = motion.length();
		if (length < MIN_MOVE) return;

		walkUnit.set(walkDirection.x / length, walkDirection.y / length, walkDirection.z / length, 0.0);
		walkStart.setFrom(position);
		slide(true);

		if (onGround && slideStepCandidate) {
			walkEnd.setFrom(position);
			if (!stepUp(length)) position.setFrom(walkEnd);
		}
	}

	// Retries a blocked walk from up to stepHeight higher. Succeeds only when it wins back most of
	// the blocked distance and lands on walkable ground higher than the start.
	function stepUp(walkLength: Float): Bool {
		var walked = progress();
		var blocked = walkLength - walked;
		if (blocked < MIN_MOVE) return false;

		// Up: a skin above stepHeight, so a step exactly that tall is cleared, not grazed.
		position.setFrom(walkStart);
		var lift = stepHeight + SKIN * 2.0;
		offsetUp(target, lift);
		var raised = sweepTo(target) ? sweep.fraction * lift - SKIN : lift;
		if (raised < MIN_MOVE) return false;
		moveUp(raised);

		// Forward
		motion.set(walkDirection.x, walkDirection.y, walkDirection.z, 0.0);
		slide(true);
		if (progress() < walked + blocked * STEP_MIN_GAIN) return false;

		// Down onto the step
		var reach = raised + SKIN;
		offsetUp(target, -reach);
		if (!sweepTo(target) || !isWalkable(sweep.faceNormal)) return false;
		var drop = sweep.fraction * reach - SKIN;
		if (raised - drop < SKIN) return false;
		if (drop > 0.0) moveUp(-drop);
		return true;
	}

	// Distance walked along the walk direction since walkStart.
	inline function progress(): Float {
		return (position.x - walkStart.x) * walkUnit.x + (position.y - walkStart.y) * walkUnit.y + (position.z - walkStart.z) * walkUnit.z;
	}

	// Ground snap, or a fall when there is no ground within reach.
	function settle(drop: Float, wasGrounded: Bool) {
		if (wasGrounded && snapToGround(Math.max(stepHeight, drop) + SKIN)) {
			grounded = true;
			verticalVelocity = 0.0;
			return;
		}

		motion.set(-up.x * drop, -up.y * drop, -up.z * drop, 0.0);
		slide(false);
		grounded = slideLanded;
		if (grounded) verticalVelocity = 0.0;
	}

	// Like Bullet's step down: drops onto walkable ground up to `reach` below.
	function snapToGround(reach: Float): Bool {
		offsetUp(target, -reach);
		if (!sweepTo(target)) return false;

		var firstDistance = sweep.fraction * reach;
		if (!isWalkable(sweep.faceNormal)) {
			// The first surface below is too steep. Walkable ground right past it still holds the
			// character (standing against a wall or the foot of a steep slope); ground further
			// down doesn't (standing on the steep slope itself).
			if (!queries.sweep(body, position, target, orientation, group, mask, up, maxSlopeCos - SLOPE_EPSILON, up, sweep)) return false;
			if (sweep.fraction * reach > firstDistance + GROUND_TOLERANCE) return false;
		}

		var drop = firstDistance - SKIN;
		if (drop > 0.0) moveUp(-drop);
		return true;
	}

	// Collide and slide along `motion`, leaving a skin gap at every hit. With `walking`, steep
	// surfaces are treated as vertical walls so they never lift or sink the character, low ones
	// are flagged for a step-up, and walkable slopes keep the horizontal speed.
	function slide(walking: Bool) {
		slideCeiling = false;
		slideLanded = false;
		slideStepCandidate = false;
		originalMotion.setFrom(motion);
		hasPrevious = false;

		for (iteration in 0...SLIDE_ITERATIONS) {
			var length = motion.length();
			if (length < MIN_MOVE) break;

			target.set(position.x + motion.x, position.y + motion.y, position.z + motion.z, 1.0);
			if (!sweepTo(target)) {
				position.addf(motion.x, motion.y, motion.z);
				break;
			}

			var dx = motion.x / length;
			var dy = motion.y / length;
			var dz = motion.z / length;
			normal.setFrom(sweep.normal);

			// Back off along the motion so the gap to the surface is SKIN.
			var into = -(normal.x * dx + normal.y * dy + normal.z * dz);
			var advance = sweep.fraction * length;
			if (into > 0.0) advance -= SKIN / Math.max(into, MIN_INTO);
			if (advance < 0.0) advance = 0.0;
			position.addf(dx * advance, dy * advance, dz * advance);
			motion.mult(1.0 - advance / length);

			if (isWalkable(sweep.faceNormal)) slideLanded = true;
			if (-up.dot(normal) >= maxSlopeCos - SLOPE_EPSILON) slideCeiling = true;
			if (walking && !isWalkable(normal)) flattenWall();

			var keepSpeed = walking && isWalkable(normal);
			var horizontalBefore = keepSpeed ? horizontalLength(motion) : 0.0;
			clipMotion();
			if (keepSpeed) {
				var horizontalAfter = horizontalLength(motion);
				if (horizontalAfter > MIN_MOVE) motion.mult(horizontalBefore / horizontalAfter);
			}
			if (motion.dot(originalMotion) <= 0.0) break;
		}
	}

	// Treats the steep surface hit as a vertical wall, and flags it for a step-up when it is low.
	function flattenWall() {
		var along = up.dot(normal);
		candidate.set(normal.x - up.x * along, normal.y - up.y * along, normal.z - up.z * along, 0.0);
		var horizontal = candidate.length();
		if (horizontal <= 0.1) return;

		normal.set(candidate.x / horizontal, candidate.y / horizontal, candidate.z / horizontal, 0.0);
		slideWallNormal.setFrom(normal);
		if (isLowObstacle(sweep.position)) slideStepCandidate = true;
	}

	// Slides the remaining motion along the hit surface, like Bullet's
	// updateTargetPositionBasedOnCollision and Godot's slide. When that would run back into the
	// previous surface of this slide (an inward corner), it follows the edge between the two.
	function clipMotion() {
		var along = motion.dot(normal);
		if (along < 0.0) motion.addf(-normal.x * along, -normal.y * along, -normal.z * along);

		// Nearly parallel surfaces (bumps on one wall) have no useful crease: their edge runs
		// vertically and would stop the slide. Like Jolt, only a real corner uses it.
		if (hasPrevious && previousNormal.dot(normal) < CREASE_MAX_COS && motion.dot(previousNormal) < -MIN_MOVE) {
			candidate.crossvecs(previousNormal, normal);
			var length = candidate.length();
			if (length > MIN_MOVE) {
				candidate.mult(1.0 / length);
				var crease = motion.dot(candidate);
				motion.set(candidate.x * crease, candidate.y * crease, candidate.z * crease, 0.0);
			}
			else {
				motion.set(0.0, 0.0, 0.0, 0.0);
			}
		}

		previousNormal.setFrom(normal);
		hasPrevious = true;
	}

	// True when the wall hit at `contact` ends below stepHeight: a probe ray towards the wall, just
	// above step height, finds nothing within the character's reach.
	function isLowObstacle(contact: Vec4): Bool {
		var feet = up.dot(position) - halfHeight;
		if (up.dot(contact) - feet > stepHeight + SKIN) return false;

		offsetUp(candidate, feet + stepHeight + SKIN - up.dot(position));
		var reach = footRadius + SKIN * 4.0;
		target.set(
			candidate.x - slideWallNormal.x * reach,
			candidate.y - slideWallNormal.y * reach,
			candidate.z - slideWallNormal.z * reach,
			1.0
		);
		return queries.rayFraction(candidate, target, group, mask, body) < 0.0;
	}

	// Unfiltered sweep of the character from `position` to `to`, into `sweep`.
	inline function sweepTo(to: Vec4): Bool {
		return queries.sweep(body, position, to, orientation, group, mask, null, 0.0, up, sweep);
	}

	// `out` = `position` moved `distance` along up.
	inline function offsetUp(out: Vec4, distance: Float) {
		out.set(position.x + up.x * distance, position.y + up.y * distance, position.z + up.z * distance, 1.0);
	}

	inline function moveUp(distance: Float) {
		position.addf(up.x * distance, up.y * distance, up.z * distance);
	}

	inline function horizontalLength(v: Vec4): Float {
		var along = up.dot(v);
		var x = v.x - up.x * along;
		var y = v.y - up.y * along;
		var z = v.z - up.z * along;
		return Math.sqrt(x * x + y * y + z * z);
	}

	inline function isWalkable(n: Vec4): Bool {
		return up.dot(n) >= maxSlopeCos - SLOPE_EPSILON;
	}

	public function canJump(): Bool {
		return onGround();
	}

	public function onGround(): Bool {
		return grounded;
	}

	public function setJumpSpeed(jumpSpeed: Float) {
		this.jumpSpeed = jumpSpeed;
	}

	public function setFallSpeed(fallSpeed: Float) {
		this.fallSpeed = fallSpeed;
	}

	public function setMaxSlope(slopeRadians: Float) {
		maxSlope = slopeRadians;
		maxSlopeCos = Math.cos(slopeRadians);
	}

	public function getMaxSlope(): Float {
		return maxSlope;
	}

	public function setMaxJumpHeight(maxJumpHeight: Float) {
		this.maxJumpHeight = maxJumpHeight;
	}

	// Displacement per fixed step, like Bullet's.
	public function setWalkDirection(walkDirection: Vec4) {
		if (walkDirection == null) this.walkDirection.set(0.0, 0.0, 0.0, 0.0);
		else this.walkDirection.set(walkDirection.x, walkDirection.y, walkDirection.z, 0.0);
	}

	public function setUpInterpolate(value: Bool) {
		upInterpolate = value;
	}

	public function jump() {
		verticalVelocity = jumpSpeed;
		grounded = false;
		activate();
	}

	public function removeFromWorld() {
		if (physics != null) physics.removeKinematicCharacterController(this);

		physics = null;
		queries = null;
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
		// Like Armory's Bullet wrapper: three times the world gravity.
		gravity = Math.abs(physics.getGravity().z) * 3.0;
	}

	public function setGravity(f: Float) {
		gravity = f;
	}

	public function setActivationState(newState: Int) {
		activate();
	}

	public function setFriction(f: Float) {
		friction = f;
		if (shape != null) shape.setFriction(f);
	}
}

enum abstract ControllerShape(Int) from Int to Int {
	var Box = 0;
	var Sphere = 1;
	var Capsule = 5;
	var Cylinder = 4;
	var Cone = 3;
	var ConvexHull = 2;
}

enum abstract ControllerActivationState(Int) from Int to Int {
	var Active = 1;
	var NoDeactivation = 4;
	var NoSimulation = 5;
}
