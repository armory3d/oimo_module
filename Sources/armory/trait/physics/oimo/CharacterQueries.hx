package armory.trait.physics.oimo;

#if arm_oimo
import iron.math.Quat;
import iron.math.Vec4;

import oimo.collision.geometry.Aabb;
import oimo.collision.geometry.ConvexGeometry;
import oimo.collision.geometry.GeometryType;
import oimo.collision.geometry.RayCastHit;
import oimo.collision.geometry.StaticMeshGeometry;
import oimo.collision.narrowphase.ConvexOverlapHit;
import oimo.common.Mat3;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.dynamics.callback.AabbTestCallback;
import oimo.dynamics.callback.ConvexOverlapCallback;
import oimo.dynamics.callback.RayCastCallback;
import oimo.dynamics.callback.RayCastClosest;
import oimo.dynamics.rigidbody.RigidBodyType;
import oimo.dynamics.rigidbody.Shape;
import oimo.m.M;

typedef OimoBody = oimo.dynamics.rigidbody.RigidBody;

/**
	The allocation-free world queries `KinematicCharacterController` moves with.
	All of them ignore the character's own body and trigger shapes.
**/
@:allow(armory.trait.physics.oimo)
class CharacterQueries {
	// Lets a character resting on a surface (a skin gap away) still see it.
	static inline var OVERLAP_MARGIN = 0.02;
	// Sweep hits closer than this count as the same distance (GJK's precision).
	static inline var SWEEP_TIE_DISTANCE = 0.0002;

	var world: oimo.dynamics.World;

	var sweepCallback = new SweepCallback();
	var contactCollector = new ContactCollector();
	var rayCallback = new RayCallback();
	var bodiesCollector = new BodiesCollector();

	var shapeTransform = new Transform();
	var bodyTransform = new Transform();
	var rotation = new oimo.common.Quat();
	var origin = new Vec3();
	var translation = new Vec3();
	var rayTo = new Vec3();
	var aabb = new Aabb();
	var corner = new Vec3();
	var margin = new Vec3(OVERLAP_MARGIN, OVERLAP_MARGIN, OVERLAP_MARGIN);

	function new(world: oimo.dynamics.World) {
		this.world = world;
	}

	/**
		Sweeps `body`'s shape from `from` to `to`. With an `axis`, only hits whose face normal
		has `dot(axis, faceNormal) >= minDot` count, checked per mesh triangle like Bullet's
		character sweeps. Of hits at the same distance (an edge shared by two triangles), the
		face most aligned with `up` wins.
	**/
	function sweep(body: OimoBody, from: Vec4, to: Vec4, orientation: Quat, group: Int, mask: Int, axis: Vec4, minDot: Float, up: Vec4, out: CharacterSweep): Bool {
		out.hit = false;
		out.fraction = 1.0;
		var convex = placeShape(body, from, orientation);
		if (convex == null) return false;

		translation.init(to.x - from.x, to.y - from.y, to.z - from.z);
		var length = translation.length();
		if (length <= 0.0) return false;

		var cb = sweepCallback;
		cb.reset(group, mask, body, axis, minDot, up, SWEEP_TIE_DISTANCE / length);
		world.convexCast(convex, shapeTransform, translation, cb);
		if (!cb.hit) return false;

		out.hit = true;
		out.fraction = cb.fraction;
		out.position.set(cb.position.x, cb.position.y, cb.position.z, 1.0);
		out.normal.set(cb.normal.x, cb.normal.y, cb.normal.z, 0.0);
		out.faceNormal.set(cb.faceNormal.x, cb.faceNormal.y, cb.faceNormal.z, 0.0);
		return true;
	}

	/** Collects the overlaps of `body`'s shape at `position`: one contact per triangle or shape. **/
	function overlap(body: OimoBody, position: Vec4, orientation: Quat, group: Int, mask: Int, out: CharacterContacts): Int {
		out.count = 0;
		var convex = placeShape(body, position, orientation);
		if (convex == null) return 0;

		contactCollector.reset(group, mask, body, out);
		world.convexOverlap(convex, shapeTransform, contactCollector);
		contactCollector.contacts = null;
		return out.count;
	}

	/** Fraction along `from` -> `to` of the closest hit, or -1 when nothing is hit. **/
	function rayFraction(from: Vec4, to: Vec4, group: Int, mask: Int, ignoreBody: OimoBody): Float {
		rayCallback.reset(group, mask, ignoreBody);
		origin.init(from.x, from.y, from.z);
		rayTo.init(to.x, to.y, to.z);
		world.rayCast(origin, rayTo, rayCallback);
		return rayCallback.hit ? rayCallback.fraction : -1.0;
	}

	/**
		Rigid bodies whose bounds overlap the character's, static ones included. Like
		Bullet's ghost object pair cache, this is an AABB test, not an exact overlap.
	**/
	function overlappingBodies(character: KinematicCharacterController, out: Array<RigidBody>) {
		var shape = character.shape;
		shape._geom._computeAabb(aabb, shape._transform);
		aabb.getMinTo(corner);
		aabb.setMin(corner.subEq(margin));
		aabb.getMaxTo(corner);
		aabb.setMax(corner.addEq(margin));

		bodiesCollector.ignoreBody = character.body;
		bodiesCollector.result = out;
		world.aabbTest(aabb, bodiesCollector);
		bodiesCollector.result = null;
	}

	// Puts `body`'s shape at `position` into shapeTransform; null when it isn't convex.
	function placeShape(body: OimoBody, position: Vec4, orientation: Quat): ConvexGeometry {
		var shape = body.getShapeList();
		if (shape == null || !Std.isOfType(shape._geom, ConvexGeometry)) return null;

		origin.init(position.x, position.y, position.z);
		rotation.init(orientation.x, orientation.y, orientation.z, orientation.w);
		bodyTransform.setPosition(origin);
		bodyTransform.setOrientation(rotation);
		M.transform_mul(shapeTransform, shape._localTransform, bodyTransform);
		return cast shape._geom;
	}
}

/** Result of `CharacterQueries.sweep`. Reused by the caller; valid until the next sweep. **/
class CharacterSweep {
	public var hit: Bool = false;
	public var fraction: Float = 1.0;
	public var position: Vec4 = new Vec4();
	/** Contact normal, pointing towards the swept shape. **/
	public var normal: Vec4 = new Vec4();
	/** The hit triangle's face normal turned towards the swept shape; the contact normal for other shapes. **/
	public var faceNormal: Vec4 = new Vec4();

	public function new() {}
}

/**
	Result of `CharacterQueries.overlap`: one contact per overlapping triangle or shape, the
	deepest ones when there are more than `CAPACITY`.
**/
class CharacterContacts {
	public static inline var CAPACITY = 16;

	public var count: Int = 0;
	/** Push-out directions, pointing towards the character. **/
	public var normals: Array<Vec4> = [for (i in 0...CAPACITY) new Vec4()];
	public var depths: Array<Float> = [for (i in 0...CAPACITY) 0.0];

	public function new() {}
}

// Shapes a character query skips: its own body, triggers, and filtered-out groups.
private inline function skipShape(shape: Shape, ignoreBody: OimoBody, group: Int, mask: Int): Bool {
	return shape._rigidBody == ignoreBody || shape._isTrigger
		|| (mask & shape.getCollisionGroup()) == 0 || (shape.getCollisionMask() & group) == 0;
}

// Closest sweep hit, with Bullet's per-triangle "axis + minimum dot" filter.
private class SweepCallback extends RayCastCallback {
	static inline var DOT_EPSILON = 0.0001;

	public var hit: Bool = false;
	public var fraction: Float = 1.0;
	public var position = new Vec3();
	public var normal = new Vec3();
	public var faceNormal = new Vec3();

	var group: Int = 1;
	var mask: Int = 1;
	var ignoreBody: OimoBody = null;
	var useAxis: Bool = false;
	var axisX: Float = 0.0;
	var axisY: Float = 0.0;
	var axisZ: Float = 0.0;
	var minDot: Float = 0.0;
	var upX: Float = 0.0;
	var upY: Float = 0.0;
	var upZ: Float = 1.0;
	var tieFraction: Float = 0.0;

	var face = new Vec3();
	var rotation = new Mat3();

	public function new() {
		super();
	}

	public function reset(group: Int, mask: Int, ignoreBody: OimoBody, axis: Vec4, minDot: Float, up: Vec4, tieFraction: Float) {
		hit = false;
		fraction = 1.0;
		this.group = group;
		this.mask = mask;
		this.ignoreBody = ignoreBody;
		useAxis = axis != null;
		if (useAxis) {
			axisX = axis.x;
			axisY = axis.y;
			axisZ = axis.z;
		}
		this.minDot = minDot;
		upX = up.x;
		upY = up.y;
		upZ = up.z;
		this.tieFraction = tieFraction;
	}

	override public function process(shape: Shape, hit: RayCastHit): Void {
		if (this.hit && hit.fraction > fraction + tieFraction) return;
		if (skipShape(shape, ignoreBody, group, mask)) return;

		// Oimo's convex cast normal points away from the swept shape.
		var nx = -hit.normal.x;
		var ny = -hit.normal.y;
		var nz = -hit.normal.z;

		if (hit.triangleIndex >= 0 && shape._geom._type == GeometryType._STATIC_MESH) {
			var mesh: StaticMeshGeometry = cast shape._geom;
			mesh.getTriangleNormal(hit.triangleIndex, face);
			shape._transform.getRotationTo(rotation);
			face.mulMat3Eq(rotation);
			// Triangles are two-sided: use the side facing the swept shape.
			if (face.x * nx + face.y * ny + face.z * nz < 0) face.negateEq();
		}
		else {
			face.init(nx, ny, nz);
		}

		if (useAxis && axisX * face.x + axisY * face.y + axisZ * face.z < minDot - DOT_EPSILON) return;

		// At a shared edge both triangles hit at the same distance: keep the more upward face, so
		// the edge of a floor or step counts as ground no matter which triangle came first.
		if (this.hit && hit.fraction > fraction - tieFraction
			&& upX * face.x + upY * face.y + upZ * face.z <= upX * faceNormal.x + upY * faceNormal.y + upZ * faceNormal.z) return;

		this.hit = true;
		if (hit.fraction < fraction) fraction = hit.fraction;
		position.copyFrom(hit.position);
		normal.init(nx, ny, nz);
		faceNormal.copyFrom(face);
	}
}

// Every overlap: one contact per triangle or shape, the deepest ones when full.
private class ContactCollector extends ConvexOverlapCallback {
	public var contacts: CharacterContacts = null;

	var group: Int = 1;
	var mask: Int = 1;
	var ignoreBody: OimoBody = null;

	public function new() {
		super();
	}

	public function reset(group: Int, mask: Int, ignoreBody: OimoBody, contacts: CharacterContacts) {
		this.group = group;
		this.mask = mask;
		this.ignoreBody = ignoreBody;
		this.contacts = contacts;
		contacts.count = 0;
	}

	override public function process(shape: Shape, hit: ConvexOverlapHit): Void {
		if (contacts == null || hit.numPoints == 0) return;
		if (skipShape(shape, ignoreBody, group, mask)) return;
		// The character is kinematic: the world pushes dynamic bodies out of it, not the reverse.
		if (shape._rigidBody != null && shape._rigidBody._type == RigidBodyType._DYNAMIC) return;

		var depth = hit.getMaxDepth();
		if (depth <= 0.0) return;

		// Points from the target towards the character: the push-out direction.
		var n = hit.normal;
		var lengthSq = n.x * n.x + n.y * n.y + n.z * n.z;
		if (lengthSq < 1e-12) return;
		var invLength = 1.0 / Math.sqrt(lengthSq);

		var index = contacts.count;
		if (index == CharacterContacts.CAPACITY) {
			var shallowest = 0;
			for (i in 1...CharacterContacts.CAPACITY) if (contacts.depths[i] < contacts.depths[shallowest]) shallowest = i;
			if (contacts.depths[shallowest] >= depth) return;
			index = shallowest;
		}
		else {
			contacts.count++;
		}
		contacts.normals[index].set(n.x * invLength, n.y * invLength, n.z * invLength, 0.0);
		contacts.depths[index] = depth;
	}
}

// Closest ray hit for the step-up probe.
private class RayCallback extends RayCastClosest {
	var group: Int = 1;
	var mask: Int = 1;
	var ignoreBody: OimoBody = null;

	public function new() {
		super();
	}

	public function reset(group: Int, mask: Int, ignoreBody: OimoBody) {
		clear();
		this.group = group;
		this.mask = mask;
		this.ignoreBody = ignoreBody;
	}

	override public function process(shape: Shape, hit: RayCastHit): Void {
		if (skipShape(shape, ignoreBody, group, mask)) return;
		super.process(shape, hit);
	}
}

// Rigid bodies whose bounds overlap a box.
private class BodiesCollector extends AabbTestCallback {
	public var ignoreBody: OimoBody = null;
	public var result: Array<RigidBody> = null;

	public function new() {
		super();
	}

	override public function process(shape: Shape): Void {
		if (result == null || shape._rigidBody == null || shape._rigidBody == ignoreBody) return;
		var ud = shape._rigidBody.userData;
		if (!Std.isOfType(ud, RigidBody)) return;
		var rb: RigidBody = cast ud;
		if (result.indexOf(rb) == -1) result.push(rb);
	}
}
#end
