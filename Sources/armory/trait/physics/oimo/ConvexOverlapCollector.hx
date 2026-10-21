package armory.trait.physics.oimo;

#if arm_oimo
import oimo.collision.narrowphase.ConvexOverlapHit;
import oimo.common.Vec3;
import oimo.dynamics.callback.ConvexOverlapCallback;
import oimo.dynamics.rigidbody.Shape;

/**
 * Internal collector for Armory's convex overlap queries.
 *
 * Keeps the deepest strict-positive overlap across the filtered targets. This
 * could become an OimoPhysics callback later if another consumer needs the same
 * selection rule.
 */
class ConvexOverlapCollector extends ConvexOverlapCallback {
	public var group: Int = 0x00000001;
	public var mask: Int = 0xFFFFFFFF;
	public var ignoreBody: oimo.dynamics.rigidbody.RigidBody = null;
	public var ignoreTriggers: Bool = false;

	public var shape: Shape = null;
	public var position: Vec3 = new Vec3();
	public var normal: Vec3 = new Vec3();
	public var depth: Float = 0;
	public var hit: Bool = false;

	public function new() {
		super();
	}

	public function clear(): Void {
		shape = null;
		position.zero();
		normal.zero();
		depth = 0;
		hit = false;
	}

	override public function process(shape: Shape, hit: ConvexOverlapHit): Void {
		if (shape == null || hit == null || hit.numPoints == 0) return;
		if (ignoreBody != null && shape._rigidBody == ignoreBody) return;
		if (ignoreTriggers && shape._isTrigger) return;
		if ((mask & shape.getCollisionGroup() == 0) || (shape.getCollisionMask() & group == 0)) return;

		var bestDepth: Float = -1;
		var bestIndex: Int = -1;
		for (i in 0...hit.numPoints) {
			var pointDepth = hit.points[i].depth;
			if (pointDepth > bestDepth) {
				bestDepth = pointDepth;
				bestIndex = i;
			}
		}

		// Overlap queries report strict positive penetration only.
		if (bestIndex < 0 || bestDepth <= 0) return;
		if (this.hit && bestDepth <= depth) return;

		var point = hit.points[bestIndex];
		this.shape = shape;
		position.copyFrom(point.position2);

		if (hit.normal.lengthSq() > 1e-12) {
			normal.copyFrom(hit.normal);
		}
		else {
			// Degenerate fallback: derive the push-out direction from the
			// witness points if the detector did not provide a normal.
			normal.copyFrom(point.position1).subEq(point.position2);
			if (normal.lengthSq() > 1e-12) normal.normalize();
			else normal.zero();
		}

		depth = bestDepth;
		this.hit = true;
	}
}
#end
