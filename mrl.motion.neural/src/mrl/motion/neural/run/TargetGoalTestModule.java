package mrl.motion.neural.run;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.eclipse.swt.SWT;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHWriter;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class TargetGoalTestModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Pose2dEditModule poseModule;
	
	private PointIKSolver solver;
	
//	String folder = "dribble_gb"; // 120
	String folder = "dribble_new_g5"; // 120
//	String folder = "dribble_g4"; // 120
//		String folder = "dribble_g"; // 120
//		String folder = "dribble_g2"; // 60
//		String folder = "walk";
//		String folder = "walk_gru";
	
	@Override
	protected void initializeImpl() {
//		MotionDataConverter.setNoBall();
		MotionDataConverter.setAllJoints();
		python = new RNNPython(folder, false);
		normal = new Normalizer(folder);
		
		
		MotionTransform t = new MotionTransform();
		solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
		
		poseModule = getModule(Pose2dEditModule.class);
		poseModule.setPoseSize(2);
		poseModule.setPose(new Pose2d(600,150,1,0));
		poseModule.setTimeList(new int[]{ 150, 30});
//		poseModule.setTimeList(new int[]{ 150, 170});
//		poseModule.setPose(new Pose2d(582.9164,92.5127,-0.9779,-0.2090));
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				Pose2d pose = poseModule.getPose();
				int time = poseModule.getTime();
				start(pose, time);
			}
		});
//		OgreJNI.open();
		
	}
	
	public void start(Pose2d pose, int time){
		double[] initialY = normal.yList.get(5089);
//		double[] initialY = normal.yList.get(5089);
//		double[] initialY = normal.yList.get(89);
//		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		
		
		ArrayList<Motion> motionList = new ArrayList<Motion>();
		TimeBasedList<Point3d> ballTrajectory = new TimeBasedList<Point3d>();
		g = new RuntimeMotionGenerator();
		for (int i = 0; i < time+5; i++) {
			Pose2d target = Pose2d.relativePose(g.pose, pose);
			int remainTime = time - i;
			
			double max_dist = 100;
			Vector2d normalP = new Vector2d(target.position);
			if (normalP.length() > max_dist){
				normalP.scale(max_dist/normalP.length());
			}
//			double remainTime = target - index;
			double t2 = Math.min(40, remainTime);

			double movement = (poseModule.getTimeList().get(1)-100)/70d;
			int mvIdx = normal.xMeanAndStd[0].length-1;
			movement = normal.xMeanAndStd[0][mvIdx] + movement*normal.xMeanAndStd[1][mvIdx];
			double[] x = new double[]{ target.position.x, target.position.y, normalP.x, normalP.y, remainTime, t2, movement };
//			double[] x = new double[]{ target.position.x, target.position.y, remainTime, normal.xMeanAndStd[0][3] + movement*normal.xMeanAndStd[1][3] };
			x = normal.normalizeX(x);
			System.out.println("xx : " + Arrays.toString(x));
			double[] output = python.model.predict(x);
			output = normal.deNormalizeY(output);
			g.update(output);
			
			HashMap<String, Point3d> map = MotionDataConverter.dataToPointMap(output);
			motionList.add(solver.solve(map, g.pose));
			
			double[] b = new double[3];
			System.arraycopy(output, 0, b, 0, 3);
			Point3d p = new Point3d(b[0], 0, b[2]);
			p = Pose2d.to3d(g.pose.localToGlobal(Pose2d.to2d(p)));
			p.y = b[1];
			ballTrajectory.add(p);
			System.out.println("ball con : " + i + " : " + String.format("%.4f, %.4f", output[3], output[4]));
		}
		
		{
			
			String[] ballTraj = new String[ballTrajectory.size()];
			for (int i = 0; i < ballTraj.length; i++) {
				Point3d p = ballTrajectory.get(i);
				if (p == null) p = new Point3d(-100000, -100000, -100000);
				ballTraj[i] = Utils.toString(p.x, p.y, p.z);
			}
			String folder = "output\\dribble_quality";
			new File(folder).mkdirs();
			FileUtil.writeAsString(ballTraj, folder + "\\ball.txt");
			new BVHWriter().write(new File(folder + "\\person_1.bvh"), new MotionData(motionList));
		}
		
		
//		new BVHWriter().write(new File("output\\dribble_quality\\motion1.bvh"), new MotionData(motionList));
		
		getModule(ItemListModule.class).addSingleItem("Motion", g.motionSequence);
		getModule(ItemListModule.class).addSingleItem("Target", pose, new ItemDescription(new Vector3d(0, 1, 0)));
		getModule(ItemListModule.class).addSingleItem("Ball", ballTrajectory, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
//		MotionData mData = new MotionData(motionList);
//		getModule(ItemListModule.class).addSingleItem("Motion2", new MotionData(motionList));
//		OgreJNI.instance.setMotion(new MotionData[]{ mData } );
	}

	public static void main(String[] args) {
//		MotionDataConverter.setNoBall();
		MotionDataConverter.setAllJoints();
//		MotionDataConverter.ROOT_OFFSET = 10;
		MainApplication.run(new TargetGoalTestModule());
//		OgreJNI.instance.close();
	}
}

