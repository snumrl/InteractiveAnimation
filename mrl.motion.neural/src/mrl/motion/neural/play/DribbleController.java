package mrl.motion.neural.play;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import mrl.motion.data.Motion;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.run.RNNPython;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.util.Pair;

public class DribbleController extends RuntimeController{
	
	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	
	private Point2d target;

	public DribbleController(String name) {
		super(name);
	}


	@Override
	public void init() {
		python = new RNNPython(name, false);
		normal = new Normalizer(name);
		double[] initialY = normal.yList.get(3);
		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		g = new RuntimeMotionGenerator();
	}

	@Override
	public Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key) {
		target = mousePoint;
		Point2d target = g.pose.globalToLocal(mousePoint);
		Vector2d v = new Vector2d(target);
		double maxLen = 250;
//		if (v.length() > maxLen){
//			v.scale(maxLen/v.length());
//		}
		double lenLimit = maxLen/2;
		Vector2d n = new Vector2d(v);
		n.normalize();
		lenLimit += lenLimit + maxLen/2*Math.max(n.x, 0);
		
		if (v.length() > maxLen){
			v.scale(maxLen/v.length());
		}
		
		
		double[] x = new double[]{ v.x, v.y };
		x = normal.normalizeX(x);
		double[] output;
		output = python.model.predict(x);
//		output = normal.yList.get(10000);
		output = normal.deNormalizeY(output);
		g.update(output);
		
		Motion motion = g.motion();
		Point3d ball = g.ballPosition();
		if (ball == null) ball = new Point3d();
		return new Pair<Motion[], Point3d>(new Motion[]{ motion }, ball);
	}
	
	public Point2d getTarget(){
		return target;
	}
	
	public static void main(String[] args) {
		DribbleController c;
		BasketDataGenerator.loadBasketData();
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setUseOrientation();
		RuntimeMotionGenerator.ALWAYS_HAS_BALL = true;
//		MotionDataConverter.setNoBall();
//		c = new DribbleController("dribble_new_nv");
//		c = new DribbleController("drb_edit");
		c = new DribbleController("drb_e128k");
//		c = new DribbleController("drb_origin");
//		c = new DribbleController("dribble_all");
//		c = new DribbleController("dribble_all_l2");
//		c = new DribbleController("drb_graph_edit");
//		c = new DribbleController("dribble_new7");
		new OgreRecorder(c).run(new double[]{ 0 });
	}
}
