package armory.trait.physics.oimo;

#if arm_oimo
import armory.trait.physics.oimo.PhysicsWorld.DebugDrawMode;

import iron.math.Vec4;

import kha.Color;
import kha.System;
import kha.graphics2.Graphics;

import oimo.common.Vec3;
import oimo.dynamics.common.DebugDraw;

class DebugDrawHelper extends DebugDraw {
    final physicsWorld: PhysicsWorld;
    var debugDrawMode: DebugDrawMode = NoDebug;

    var g2: Graphics;
    var rayCasts: Array<TRayCastData> = [];

    final rayCastColor: Vec3 = new Vec3(0.0, 1.0, 0.0);
    final rayCastHitColor: Vec3 = new Vec3(1.0, 0.0, 0.0);

    public function new(physicsWorld: PhysicsWorld, debugDrawMode: DebugDrawMode) {
        super();

        style.staticShapeColor = new Vec3(0, 1.0, 1.0);

        this.physicsWorld = physicsWorld;
        setDebugMode(debugDrawMode);

        iron.App.notifyOnRender2D(onRender2D);

        if (debugDrawMode & DrawRaycast != 0) {
			iron.App.notifyOnFixedUpdate(function () {
				rayCasts.resize(0);
			});
		}
    }

    public function setDebugMode(debugDrawMode: DebugDrawMode) {
        this.debugDrawMode = debugDrawMode;

        wireframe = debugDrawMode & DrawWireframe != 0;
        drawShapes = wireframe;
        drawAabbs = debugDrawMode & DrawAABB != 0;
        drawBases = debugDrawMode & DrawBases != 0;
        drawContacts = debugDrawMode & DrawContactPoint != 0;
        drawJoints = debugDrawMode & DrawJoints != 0;
        drawJointLimits = debugDrawMode & DrawJointLimits != 0;
    }

    public function getDebugMode(): DebugDrawMode {
        return debugDrawMode;
    }

    function onRender2D(g: Graphics) {
        if (getDebugMode() == NoDebug) return;
        if (g2 == null) g2 = g;

        physicsWorld.world.debugDraw();

        if (debugDrawMode & DrawRaycast != 0) {
            for (rayCastData in rayCasts) {
                if (rayCastData.hasHit) {
                    drawRayCast(rayCastData.from, rayCastData.hitPoint, true);
                    drawHitPoint(rayCastData.hitPoint);
                } else {
                    drawRayCast(rayCastData.from, rayCastData.to, false);
                }
            }
        }
    }

	override public function point(v: Vec3, color: Vec3): Void {
        if (g2 == null) return;

        var p: Vec4 = worldToScreenFast(new Vec4(v.x, v.y, v.z));
        var c: Color = Color.fromFloats(color.x, color.y, color.z);

        if (p.w != 0) {
            g2.color = c;
            g2.fillRect(p.x - 2, p.y - 2, 4, 4);
        }
	}

    override public function triangle(v1: Vec3, v2: Vec3, v3: Vec3, n1: Vec3, n2: Vec3, n3: Vec3, color: Vec3): Void {
        if (g2 == null) return;

		var p1: Vec4 = worldToScreenFast(new Vec4(v1.x, v1.y, v1.z));
        var p2: Vec4 = worldToScreenFast(new Vec4(v2.x, v2.y, v2.z));
        var p3: Vec4 = worldToScreenFast(new Vec4(v3.x, v3.y, v3.z));
        var c: Color = Color.fromFloats(color.x, color.y, color.z);

        if (p1.w != 0 && p2.w != 0 && p3.w != 0) {
            g2.color = c;
            g2.fillTriangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
        }
	}

    override public function line(v1: Vec3, v2: Vec3, color: Vec3): Void {
        if (g2 == null) return;

		var from: Vec4 = worldToScreenFast(new Vec4(v1.x, v1.y, v1.z));
        var to: Vec4 = worldToScreenFast(new Vec4(v2.x, v2.y, v2.z));
        var c: Color = Color.fromFloats(color.x, color.y, color.z);

        if (from.w != 0 && to.w != 0) {
            g2.color = c;
            g2.drawLine(from.x, from.y, to.x, to.y);
        }
	}

    public function rayCast(rayCastData: TRayCastData) {
        rayCasts.push(rayCastData);
    }

    function drawRayCast(f: Vec4, t: Vec4, hit: Bool) {
        if (g2 == null) return;

		var from: Vec4 = worldToScreenFast(f.clone());
        var to: Vec4 = worldToScreenFast(t.clone());
        var c: Color;

        if (hit) c = Color.fromFloats(rayCastHitColor.x, rayCastHitColor.y, rayCastHitColor.z);
        else c = Color.fromFloats(rayCastColor.x, rayCastColor.y, rayCastColor.z);

        if (from.w != 0 && to.w != 0) {
            g2.color = c;
            g2.drawLine(from.x, from.y, to.x, to.y);
        }
    }

    function drawHitPoint(hp: Vec4) {
        var hitPoint: Vec4 = worldToScreenFast(hp.clone());
        final c = Color.fromFloats(rayCastHitColor.x, rayCastHitColor.y, rayCastHitColor.z);
        g2.fillRect(hitPoint.x - 2, hitPoint.y - 2, 4, 4);
    }

    inline function worldToScreenFast(loc: Vec4): Vec4 {
		final cam = iron.Scene.active.camera;
		loc.w = 1.0;
		loc.applyproj(cam.VP);

		if (loc.z < -1 || loc.z > 1) {
			loc.w = 0.0;
		}
		else {
			loc.x = (loc.x + 1) * 0.5 * System.windowWidth();
			loc.y = (1 - loc.y) * 0.5 * System.windowHeight();
			loc.w = 1.0;
		}

		return loc;
	}
}

typedef TRayCastData = {
	var from: Vec4;
	var to: Vec4;
	var hasHit: Bool;
	@:optional var hitPoint: Vec4;
    @:optional var hitNormal: Vec4;
}
#end