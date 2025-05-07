package armory.trait.physics.oimo;

#if arm_oimo
import iron.Trait;

class PhysicsConstraint extends Trait {
	public function new(body1:String, body2:String) { super(); }
}

@:enum abstract ConstraintAxis(Int) from Int to Int {
	var X = 0;
	var Y = 1;
	var Z = 2;
}
#end
