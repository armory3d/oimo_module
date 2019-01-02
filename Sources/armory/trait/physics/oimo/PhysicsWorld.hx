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
		var res:Array<RigidBody> = [];
		return res;
	}

	public function getContactPairs(body:RigidBody):Array<ContactPair> {
		var res:Array<ContactPair> = [];
		return res;
	}

	public function lateUpdate() {
		#if arm_debug
		var startTime = kha.Scheduler.realTime();
		#end

		if (preUpdates != null) for (f in preUpdates) f();

		if (!pause) {
			world.step(timeStep);
			for (rb in rbMap) @:privateAccess rb.physicsUpdate();
		}

		#if arm_debug
		physTime = kha.Scheduler.realTime() - startTime;
		#end
	}

	public function pickClosest(inputX:Float, inputY:Float):RigidBody {
		return null;
	}

	public function rayCast(from:Vec4, to:Vec4):RigidBody {
		return null;
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
