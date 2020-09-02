package mrl.motion.neural.play;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.util.Pair;
import mrl.widget.app.MainApplication;

public abstract class RuntimeController {

	public String name;
	public MainApplication app;
	public int omitTime = 0;
	
	public RuntimeController(String name) {
		this.name = name;
	}
	
	
	
	public abstract void init();

	public abstract Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key);
	
	public Point2d getTarget(){
		return null;
	}
}
