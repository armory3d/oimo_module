package armory.trait.physics.oimo;

#if arm_oimo

import iron.Trait;
import iron.system.Time;
import iron.math.Vec4;
import iron.math.RayCaster;
import iron.data.SceneFormat;

class ContactPair {
	public var a:Int;
	public var b:Int;
	public var posA:Vec4;
	public var posB:Vec4;
	public var nor:Vec4;
	public var impulse:Float;
	public function new(a:Int, b:Int) {
		this.a = a;
		this.b = b;
	}
}

class PhysicsWorld extends Trait {

	#if arm_debug
	public static var physTime = 0.0;
	#end

	public static var active:PhysicsWorld = null;
	public var world:oimo.dynamics.World;
	public var rbMap:Map<Int, RigidBody>;
	var preUpdates:Array<Void->Void> = null;
	static inline var timeStep = 1 / 60;
	static inline var fixedStep = 1 / 60;
	public var hitPointWorld = new Vec4();
	public var rayCastResult:oimo.dynamics.callback.RayCastClosest;
	var contacts:Array<ContactPair>;
	public var pause = false;
	
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
		rayCastResult = new oimo.dynamics.callback.RayCastClosest();
		contacts = [];

		// Ensure physics are updated first in the lateUpdate list
		_lateUpdate = [lateUpdate];
		@:privateAccess iron.App.traitLateUpdates.insert(0, lateUpdate);
	}

	function createPhysics() {
		var g = iron.Scene.active.raw.gravity;
		var gravity = g == null ? new oimo.common.Vec3(0, 0, -9.81) : new oimo.common.Vec3(g[0], g[1], g[2]);
		world = new oimo.dynamics.World(oimo.collision.broadphase.BroadPhaseType._BVH, gravity);
	}

	public function addRigidBody(body:RigidBody) {
		world.addRigidBody(body.body);
		rbMap.set(body.id, body);
	}

	public function removeRigidBody(body:RigidBody) {
		if (world != null) world.removeRigidBody(body.body);
		rbMap.remove(body.id);
	}

	public function getContacts(body:RigidBody):Array<RigidBody> {
		if (contacts.length == 0) return null;
		var res:Array<RigidBody> = [];
		for (c in contacts) {
			if (c.a == body.id) res.push(rbMap.get(c.b));
			else if (c.b == body.id) res.push(rbMap.get(c.a));
		}
		return res;
	}

	public function getContactPairs(body:RigidBody):Array<ContactPair> {
		if (contacts.length == 0) return null;
		var res:Array<ContactPair> = [];
		for (c in contacts) {
			if (c.a == body.id) res.push(c);
			else if (c.b == body.id) res.push(c);
		}
		return res;
	}

	public function lateUpdate() {
		#if arm_debug
		var startTime = kha.Scheduler.realTime();
		#end

		if (preUpdates != null) for (f in preUpdates) f();

		if (!pause) {
			world.step(timeStep);
			updateContacts();
			for (rb in rbMap) @:privateAccess rb.physicsUpdate();
		}

		#if arm_debug
		physTime = kha.Scheduler.realTime() - startTime;
		#end
	}

	function updateContacts() {
		contacts = [];

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

	public function pickClosest(inputX:Float, inputY:Float):RigidBody {
		return null;
	}

	public function rayCast(from:Vec4, to:Vec4):RigidBody {
		rayCastResult.clear();
		world.rayCast(new oimo.common.Vec3(from.x, from.y, from.z), new oimo.common.Vec3(to.x, to.y, to.z), rayCastResult);
		if (rayCastResult.shape!= null) {
			return cast (rayCastResult.shape._rigidBody.userData, RigidBody);
		}
		else {
			return null;
		}
	}

	public function notifyOnPreUpdate(f:Void->Void) {
		if (preUpdates == null) preUpdates = [];
		preUpdates.push(f);
	}

	public function removePreUpdate(f:Void->Void) {
		preUpdates.remove(f);
	}
}

#end
