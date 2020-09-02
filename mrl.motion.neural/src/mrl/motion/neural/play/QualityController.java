package mrl.motion.neural.play;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.run.RNNPython;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.util.Pair;

public class QualityController extends RuntimeController{
	
	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;

	public QualityController(String name) {
		super(name);
	}


	@Override
	public void init() {
		python = new RNNPython(name, false);
		normal = new Normalizer(name);
		double[] initialY = normal.yList.get(5089);
		python.model.setStartMotion(initialY);
		g = new RuntimeMotionGenerator();
	}

	@Override
	public Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key) {
		Pose2d pose = new Pose2d(582.9164,92.5127,-0.9779,-0.2090);
		Pose2d target = Pose2d.relativePose(g.pose, pose);
		double movement = 1;
		double[] x = new double[]{ target.position.x, target.position.y, 150 - frame, normal.xMeanAndStd[0][3] + movement*normal.xMeanAndStd[1][3] };
		x = normal.normalizeX(x);
		double[] output = python.model.predict(x);
		output = normal.deNormalizeY(output);
		g.update(output);
		Motion motion = g.motion();
		Point3d ball = new Point3d(0, -200, 0);
		return new Pair<Motion[], Point3d>(new Motion[]{ motion }, ball);
	}
	
	
	public static void main(String[] args) {
		QualityController c;
		MotionDataConverter.setNoBall();
		c = new QualityController("dribble_g4");
		new OgreRecorder(c).run(new double[]{ 0 });
	}
}
