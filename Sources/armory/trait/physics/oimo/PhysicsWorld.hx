package armory.trait.physics.oimo;

#if arm_oimo
import iron.Trait;
import iron.system.Time;
import iron.math.Vec4;
import iron.math.Quat;

import oimo.collision.geometry.RayCastHit;
import oimo.common.Vec3;
import oimo.dynamics.callback.RayCastClosest;
import oimo.dynamics.rigidbody.Shape;

class Hit {
	public var rb: RigidBody;
	public var pos: Vec4;
	public var normal: Vec4;
	public function new(rb: RigidBody, pos: Vec4, normal: Vec4) {
		this.rb = rb;
		this.pos = pos;
		this.normal = normal;
	}
}

class ConvexHit {
	public var pos: Vec4;
	public var normal: Vec4;
	public var hitFraction: Float;
	public function new(pos: Vec4, normal: Vec4, hitFraction: Float){
		this.pos = pos;
		this.normal = normal;
		this.hitFraction = hitFraction;
	}
}

class ContactPair {
	public var a: Int;
	public var b: Int;
	public var posA: Vec4;
	public var posB: Vec4;
	public var nor: Vec4;
	public var impulse: Float;
	public function new(a: Int, b: Int) {
		this.a = a;
		this.b = b;
	}
}

class PhysicsWorld extends Trait {
	#if arm_debug
	public static var physTime = 0.0;
	#end

	public static var active: PhysicsWorld = null;
	public var world: oimo.dynamics.World;
	public var rbMap: Map<Int, RigidBody>;
	public var timeScale: Float = 1.0;
	var preUpdates: Array<Void->Void> = null;
	public var hitPointWorld = new Vec4();
	public var hitNormalWorld = new Vec4();
	public var rayCastResult: RayCastClosestWithMask;
	var contacts: Array<ContactPair>;

	var debugDrawHelper: DebugDrawHelper = null;

	public function new(timeScale = 1.0, maxSteps = 10, solverIterations = 10, fixedStep = 1 / 60, debugDrawMode: DebugDrawMode = NoDebug) {
		super();

		this.timeScale = timeScale;
		Time.initFixedStep(fixedStep);

		if (active == null) {
			createPhysics();
		}
		else {
			for (rb in active.rbMap) { @:privateAccess try { active.removeRigidBody(rb); } catch(e: haxe.Exception) { trace(e.message); } }
			this.world = active.world;
		}

		rbMap = new Map();
		active = this;
		rayCastResult = new RayCastClosestWithMask();
		contacts = [];

		// Ensure physics are updated first in the fixedUpdate list
		_fixedUpdate = [fixedUpdate];
		@:privateAccess iron.App.traitFixedUpdates.insert(0, fixedUpdate);

		setDebugDrawMode(debugDrawMode);
	}

	// TODO
	public function reset() {
		trace("TODO");
	}

	function createPhysics() {
		var g: kha.arrays.Float32Array = iron.Scene.active.raw.gravity;
		var gravity:Vec3 = g == null ? new Vec3(0, 0, -9.81) : new Vec3(g[0], g[1], g[2]);
		world = new oimo.dynamics.World(oimo.collision.broadphase.BroadPhaseType._BVH, gravity);
	}

	public function setGravity(v: Vec4) {
		world.setGravity(new Vec3(v.x, v.y, v.z));
	}

	public function getGravity(): Vec4 {
		var g: Vec3 = world.getGravity();
		return new Vec4(g.x, g.y, g.z);
	}

	public function addRigidBody(body: RigidBody) {
		if (rbMap.exists(body.id)) return;
		world.addRigidBody(body.body);
		rbMap.set(body.id, body);
	}

	// TODO
	public function addPhysicsConstraint(constraint: PhysicsConstraint) {
		trace("TODO");
	}

	public function removeRigidBody(body: RigidBody) {
		if (!rbMap.exists(body.id)) return;
		if (world != null) world.removeRigidBody(body.body);
		rbMap.remove(body.id);
	}

	// TODO
	public function removePhysicsConstraint(constraint: PhysicsConstraint) {
		trace("TODO");
	}

	public function getContacts(body: RigidBody): Array<RigidBody> {
		if (contacts.length == 0) return null;
		var res: Array<RigidBody> = [];
		for (c in contacts) {
			if (c.a == body.id) res.push(rbMap.get(c.b));
			else if (c.b == body.id) res.push(rbMap.get(c.a));
		}
		return res;
	}

	public function getContactPairs(body: RigidBody): Array<ContactPair> {
		if (contacts.length == 0) return null;
		var res: Array<ContactPair> = [];
		for (c in contacts) {
			if (c.a == body.id) res.push(c);
			else if (c.b == body.id) res.push(c);
		}
		return res;
	}

	// TODO
	public function findBody(id: Int): RigidBody {
		trace("TODO");
		return null;
	}

	public function fixedUpdate() {
		var ts: Float = Time.scale * timeScale;
		if (ts == 0.0) return;

		#if arm_debug
		var startTime: Float = kha.Scheduler.realTime();
		#end

		if (preUpdates != null) for (f in preUpdates) f();

		world.step(Time.fixedStep * ts);
		updateContacts();
		for (rb in rbMap) { @:privateAccess try { rb.physicsUpdate(); } catch(e: haxe.Exception) { trace(e.message); } }

		#if arm_debug
		physTime = kha.Scheduler.realTime() - startTime;
		#end
	}

	function updateContacts() {
		contacts.resize(0);

		var contact_list = world.getContactManager().getContactList();
		while(contact_list != null) {
			var b1: RigidBody = cast(contact_list._b1.userData, RigidBody);
			var b2: RigidBody = cast(contact_list._b2.userData, RigidBody);
			var cp: ContactPair = new ContactPair(b1.id, b2.id);
			for (pt in contact_list.getManifold().getPoints()) {
				if (pt.getDepth() > 0) {
					var posA: Vec3 = pt.getPosition1();
					var posB: Vec3 = pt.getPosition2();
					var nor: Vec3  = contact_list.getManifold().getNormal();
					cp.posA = new Vec4(posA.x, posA.y, posA.z);
					cp.posB = new Vec4(posB.x, posB.y, posB.z);
					cp.nor  = new Vec4(nor.x,  nor.y,  nor.z );
					cp.impulse = pt.getNormalImpulse();
					contacts.push(cp);
					break;
				}
			}
			contact_list = contact_list.getNext();
		}
	}

	// TODO
	public function pickClosest(inputX: Float, inputY: Float, mask: Int = 0xFFFFFFFF): RigidBody {
		trace("TODO");
		return null;
	}

	public function rayCast(from: Vec4, to: Vec4, group: Int = 0x00000001, mask: Int = 0xFFFFFFFF): Hit {
		var hitInfo: Hit = null;

		rayCastResult.clear();
		rayCastResult.group = group;
		rayCastResult.mask = mask;

		world.rayCast(new Vec3(from.x, from.y, from.z), new Vec3(to.x, to.y, to.z), rayCastResult);

		if (rayCastResult.hit) {
			var rb: RigidBody = cast(rayCastResult.shape._rigidBody.userData, RigidBody);
			hitPointWorld.set(rayCastResult.position.x, rayCastResult.position.y, rayCastResult.position.z);
			hitNormalWorld.set(rayCastResult.normal.x, rayCastResult.normal.y, rayCastResult.normal.z);
			hitInfo = new Hit(rb, hitPointWorld, hitNormalWorld);
		}

		if (getDebugDrawMode() & DrawRaycast != 0) debugDrawHelper.rayCast({
			from: from,
			to: to,
			hasHit: rayCastResult.hit,
			hitPoint: hitPointWorld.clone(),
			hitNormal: hitNormalWorld.clone()
		});

		return hitInfo;
	}

	// TODO
	public function convexSweepTest(rb: RigidBody, from: Vec4, to: Vec4, rotation: Quat, group: Int = 0x00000001, mask = 0xFFFFFFFF): ConvexHit {
		trace("TODO");
		return null;
	}

	public function notifyOnPreUpdate(f: Void->Void) {
		if (preUpdates == null) preUpdates = [];
		preUpdates.push(f);
	}

	public function removePreUpdate(f: Void->Void) {
		preUpdates.remove(f);
	}

	public function setDebugDrawMode(debugDrawMode: DebugDrawMode) {
		if (debugDrawHelper == null) {
			if (debugDrawMode == NoDebug) return;
		}

		debugDrawHelper = new DebugDrawHelper(this, debugDrawMode);
		world.setDebugDraw(debugDrawHelper);
	}

	public inline function getDebugDrawMode(): DebugDrawMode {
		if (debugDrawHelper == null) return NoDebug;
		return debugDrawHelper.getDebugMode();
	}
}

private class RayCastClosestWithMask extends RayCastClosest {
    public var group: Int;
    public var mask: Int;

    public function new(group: Int = 0x00000001, mask: Int = 0xFFFFFFFF) {
        super();
        this.group = group;
        this.mask = mask;
	}

	override public function process(shape: Shape, hit: RayCastHit): Void {
		if ((mask & shape.getCollisionGroup() != 0) && (shape.getCollisionMask() & group != 0)) super.process(shape, hit);
	}
}

enum abstract DebugDrawMode(Int) from Int to Int {
	var NoDebug: Int = 0;
	var DrawWireframe: Int = 1;
	var DrawAABB: Int = 1 << 1;

	var DrawContactPoint: Int = 1 << 3;

	// var DisableSleeping: Int = 1 << 4;

	var DrawJoints: Int = 1 << 11;
	var DrawJointLimits: Int = 1 << 12;

	// var DrawNormals: Int = 1 << 14; // Not available in Oimo
	var DrawBases: Int = 1 << 15;
	var DrawRaycast: Int = 1 << 16;

	@:op(~A) public inline function bitwiseNegate(): DebugDrawMode {
		return ~this;
	}

	@:op(A & B) public inline function bitwiseAND(other: DebugDrawMode): DebugDrawMode {
		return this & other;
	}

	@:op(A | B) public inline function bitwiseOR(other: DebugDrawMode): DebugDrawMode {
		return this | other;
	}
}
#end
