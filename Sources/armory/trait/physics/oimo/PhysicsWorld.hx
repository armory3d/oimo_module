package armory.trait.physics.oimo;

#if arm_oimo

import iron.Trait;
import iron.system.Time;
import iron.math.Vec4;
import iron.math.RayCaster;

import oimo.collision.geometry.RayCastHit;
import oimo.common.Vec3;
import oimo.dynamics.callback.RayCastClosest;
import oimo.dynamics.rigidbody.Shape;

class Hit {
	public var rb: RigidBody;
	public var pos: Vec4;
	public var normal: Vec4;
	public function new(rb: RigidBody, pos: Vec4, normal: Vec4){
		this.rb = rb;
		this.pos = pos;
		this.normal = normal;
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
	var preUpdates: Array<Void->Void> = null;
	static inline var timeStep: Float = 1 / 60;
	static inline var fixedStep: Float = 1 / 60;
	public var hitPointWorld = new Vec4();
	public var rayCastResult: RayCastMask;
	var contacts: Array<ContactPair>;
	public var pause: Bool = false;
	
	public function new() {
		super();

		if (active == null) {
			createPhysics();
		}
		else {
			for (rb in active.rbMap) removeRigidBody(rb);
			this.world = active.world;
		}

		rbMap = new Map();
		active = this;
		rayCastResult = new RayCastMask();
		contacts = [];

		// Ensure physics are updated first in the lateUpdate list
		_lateUpdate = [lateUpdate];
		@:privateAccess iron.App.traitLateUpdates.insert(0, lateUpdate);
	}

	function createPhysics() {
		var g = iron.Scene.active.raw.gravity;
		var gravity = g == null ? new Vec3(0, 0, -9.81) : new Vec3(g[0], g[1], g[2]);
		world = new oimo.dynamics.World(oimo.collision.broadphase.BroadPhaseType._BVH, gravity);
	}

	public function setGravity(v: Vec4) {
		world.setGravity(new Vec3(v.x, v.y, v.z));
	}

	function getGravity(): Vec4 {
		var g = world.getGravity();
		return new Vec4(g.x, g.y, g.z);
	}

	public function addRigidBody(body: RigidBody) {
		world.addRigidBody(body.body);
		rbMap.set(body.id, body);
	}

	public function removeRigidBody(body: RigidBody) {
		if (world != null) world.removeRigidBody(body.body);
		rbMap.remove(body.id);
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

	public function lateUpdate() {
		var ts = Time.scale;
		if (ts == 0.0) return;

		#if arm_debug
		var startTime = kha.Scheduler.realTime();
		#end

		if (preUpdates != null) for (f in preUpdates) f();

		if (!pause) {
			world.step(timeStep * ts);
			updateContacts();
			for (rb in rbMap) @:privateAccess rb.physicsUpdate();
		}

		#if arm_debug
		physTime = kha.Scheduler.realTime() - startTime;
		#end
	}

	function updateContacts() {
		contacts.resize(0);

		var contact_list = world.getContactManager().getContactList();
		while(contact_list != null) {
			var b1 = cast (contact_list._b1.userData, RigidBody);
			var b2 = cast (contact_list._b2.userData, RigidBody);
			var cp = new ContactPair(b1.id, b2.id);
			for (pt in contact_list.getManifold().getPoints()) {
				if (pt.getDepth() > 0) {
					var posA = pt.getPosition1();
					var posB = pt.getPosition2();
					var nor  = contact_list.getManifold().getNormal();
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

	public function pickClosest(inputX: Float, inputY: Float): RigidBody {
		return null;
	}

	public function rayCast(from: Vec4, to: Vec4, group: Int = 0x00000001, mask: Int = 0xFFFFFFFF): Hit {
		rayCastResult.clear();
		rayCastResult.group = group;
		rayCastResult.mask = mask;

		world.rayCast(new Vec3(from.x, from.y, from.z), new Vec3(to.x, to.y, to.z), rayCastResult);
		
		if (rayCastResult.hit) {
			var rb: RigidBody = cast (rayCastResult.shape._rigidBody.userData, RigidBody);
			var pos: Vec3 = rayCastResult.position;
			var normal: Vec3 = rayCastResult.normal;
			return new Hit(rb, new Vec4(pos.x, pos.y, pos.z), new Vec4(normal.x, normal.y, normal.z));
		}
		
		return null;
	}

	public function notifyOnPreUpdate(f: Void -> Void) {
		if (preUpdates == null) preUpdates = [];
		preUpdates.push(f);
	}

	public function removePreUpdate(f: Void -> Void) {
		preUpdates.remove(f);
	}
}

private class RayCastMask extends RayCastClosest {
    public var group: Int;
    public var mask: Int;

    public function new(group: Int = 0x00000001, mask: Int = 0xFFFFFFFF) {
        super();
        this.group = group;
        this.mask = mask;
	}

	override public function process(shape: Shape, hit: RayCastHit):Void {
		if (hit.fraction < fraction) {
            if ((mask & shape.getCollisionGroup() != 0) && (shape.getCollisionMask() & group != 0)) {
                this.shape = shape;
                this.hit = true;
                fraction = hit.fraction;
                position.copyFrom(hit.position);
                normal.copyFrom(hit.normal);
            }
		}
	}
}

#end
