package mrl.motion.neural.play;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.util.Pair;

public class LocomotionController extends RuntimeController{

	public LocomotionController(String name) {
		super(name);
	}

	@Override
	public void init() {
	}

	@Override
	public Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key) {
		return null;
	}

}
