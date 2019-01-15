package armory.trait.physics.oimo;

#if arm_oimo

import iron.Trait;
import iron.math.Vec4;
import iron.object.Transform;
import iron.object.MeshObject;

class RigidBody extends Trait {

	var shape:Shape;
	public var physics:PhysicsWorld;
	public var transform:Transform = null;

	public var mass:Float;
	public var friction:Float;
	public var restitution:Float;
	public var collisionMargin:Float;
	public var linearDamping:Float;
	public var angularDamping:Float;
	public var animated:Bool;

	public var body:oimo.dynamics.rigidbody.RigidBody = null;
	public var ready = false;

	static var nextId = 0;
	public var id = 0;

	public var onReady:Void->Void = null;

	static var v1 = new oimo.common.Vec3();
	static var v2 = new oimo.common.Vec3();
	static var q1 = new oimo.common.Quat();

	public function new(shape = Shape.Box, mass = 1.0, friction = 0.5, restitution = 0.0, group = 1,
						params:Array<Float> = null, flags:Array<Bool> = null) {
		super();

		this.shape = shape;
		this.mass = mass;
		this.friction = friction;
		this.restitution = restitution;
		// this.group = group;

		if (params == null) params = [0.04, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0];
		if (flags == null) flags = [false, false, false];

		this.linearDamping = params[0];
		this.angularDamping = params[1];
		// this.linearFactors = [params[2], params[3], params[4]];
		// this.angularFactors = [params[5], params[6], params[7]];
		// this.collisionMargin = params[8];
		// this.deactivationParams = [params[9], params[10], params[11]];
		this.animated = flags[0];
		// this.trigger = flags[1];
		// this.ccd = flags[2];

		notifyOnAdd(init);
	}
	
	inline function withMargin(f:Float) {
		return f - f * collisionMargin;
	}

	public function notifyOnReady(f:Void->Void) {
		onReady = f;
		if (ready) onReady();
	}

	public function init() {
		if (ready) return;
		ready = true;
		
		transform = object.transform;
		physics = armory.trait.physics.PhysicsWorld.active;
		
		var shapeConfig = new oimo.dynamics.rigidbody.ShapeConfig();
		shapeConfig.friction = friction;
		shapeConfig.restitution = restitution;
		shapeConfig.density = mass > 0 ? mass : 1.0;

		if (shape == Shape.Box) {
			v1.init(withMargin(transform.dim.x) / 2, withMargin(transform.dim.y) / 2, withMargin(transform.dim.z) / 2);
			shapeConfig.geometry = new oimo.collision.geometry.BoxGeometry(
				v1
			);
		}
		else if (shape == Shape.Sphere) {
			shapeConfig.geometry = new oimo.collision.geometry.SphereGeometry(
				withMargin(transform.dim.x) / 2
			);
		}
		else if (shape == Shape.ConvexHull || shape == Shape.Mesh) {
			var md = cast(object, MeshObject).data;
			var positions = md.geom.positions;
			var sx = transform.scale.x * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
			var sy = transform.scale.y * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
			var sz = transform.scale.z * (1.0 - collisionMargin) * md.scalePos * (1 / 32767);
			var verts:Array<oimo.common.Vec3> = [];
			for (i in 0...Std.int(positions.length / 4)) {
				verts.push(new oimo.common.Vec3(
					positions[i * 4    ] * sx,
					positions[i * 4 + 1] * sy,
					positions[i * 4 + 2] * sz
				));
			}
			shapeConfig.geometry = new oimo.collision.geometry.ConvexHullGeometry(
				verts
			);
		}
		else if (shape == Shape.Cone) {
			// TODO: fix axis
			shapeConfig.geometry = new oimo.collision.geometry.ConeGeometry(
				withMargin(transform.dim.x) / 2, // Radius
				withMargin(transform.dim.y) / 2 // Half-height
			);
		}
		else if (shape == Shape.Cylinder) {
			// TODO: fix axis
			shapeConfig.geometry = new oimo.collision.geometry.CylinderGeometry(
				withMargin(transform.dim.x) / 2, // Radius
				withMargin(transform.dim.y) / 2 // Half-height
			);
		}
		else if (shape == Shape.Capsule) {
			// TODO: fix axis
			shapeConfig.geometry = new oimo.collision.geometry.CapsuleGeometry(
				withMargin(transform.dim.x) / 2, // Radius
				withMargin(transform.dim.y) / 2 // Half-height
			);
		}

		var bodyConfig = new oimo.dynamics.rigidbody.RigidBodyConfig();
		bodyConfig.type = mass > 0 ? oimo.dynamics.rigidbody.RigidBodyType.DYNAMIC : oimo.dynamics.rigidbody.RigidBodyType.STATIC;
		bodyConfig.position.init(transform.worldx(), transform.worldy(), transform.worldz());
		body = new oimo.dynamics.rigidbody.RigidBody(bodyConfig);
		q1.init(transform.rot.x, transform.rot.y, transform.rot.z, transform.rot.w);
		body.setOrientation(q1);
		body.addShape(new oimo.dynamics.rigidbody.Shape(shapeConfig));
		body.setLinearDamping(this.linearDamping);
		body.setAngularDamping(this.angularDamping);
		body.userData = this;

		id = nextId++;

		physics.addRigidBody(this);
		notifyOnRemove(removeFromWorld);

		if (onReady != null) onReady();
	}

	function physicsUpdate() {
		if (!ready) return;
		if (object.animation != null || animated) {
			syncTransform();
		}
		else {
			var p = body.getPosition();
			var q = body.getOrientation();
			transform.loc.set(p.x, p.y, p.z);
			transform.rot.set(q.x, q.y, q.z, q.w);
			if (object.parent != null) {
				var ptransform = object.parent.transform;
				transform.loc.x -= ptransform.worldx();
				transform.loc.y -= ptransform.worldy();
				transform.loc.z -= ptransform.worldz();
			}
			transform.buildMatrix();
		}
	}

	public function removeFromWorld() {
		if (physics != null) physics.removeRigidBody(this);
	}

	public function activate() {
		body.wakeUp();
	}

	public function disableGravity() {
	}

	public function setActivationState(newState:Int) {
	}

	public function applyImpulse(impulse:Vec4, loc:Vec4 = null) {
		activate();
		if (loc == null) loc = transform.loc;
		v1.init(impulse.x, impulse.y, impulse.z);
		v2.init(loc.x, loc.y, loc.z);
		body.applyImpulse(v1, v2);
	}

	public function setLinearFactor(x:Float, y:Float, z:Float) {
	}

	public function setAngularFactor(x:Float, y:Float, z:Float) {
		v1.init(x, y, z);
		body.setRotationFactor(v1);
	}

	public function getLinearVelocity():Vec4 {
		var v = body.getLinearVelocity();
		return new Vec4(v.x, v.y, v.z);
	}

	public function setLinearVelocity(x:Float, y:Float, z:Float) {
		v1.init(x, y, z);
		body.setLinearVelocity(v1);
	}

	public function getAngularVelocity():Vec4 {
		var v = body.getAngularVelocity();
		return new Vec4(v.x, v.y, v.z);
	}

	public function setAngularVelocity(x:Float, y:Float, z:Float) {
		v1.init(x, y, z);
		body.setAngularVelocity(v1);
	}

	public function setFriction(f:Float) {
	}

	public function syncTransform() {
		v1.init(transform.worldx(), transform.worldy(), transform.worldz());
		body.setPosition(v1);
		q1.init(transform.rot.x, transform.rot.y, transform.rot.z, transform.rot.w);
		body.setOrientation(q1);
		activate();
	}
}

@:enum abstract Shape(Int) from Int to Int {
	var Box = 0;
	var Sphere = 1;
	var ConvexHull = 2;
	var Mesh = 3;
	var Cone = 4;
	var Cylinder = 5;
	var Capsule = 6;
	var Terrain = 7;
}

@:enum abstract ActivationState(Int) from Int to Int {
	var Active = 1;
	var NoDeactivation = 4;
	var NoSimulation = 5;
}

#end
