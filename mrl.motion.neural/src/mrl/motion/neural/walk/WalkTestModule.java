package mrl.motion.neural.walk;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.run.RNNPython;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.neural.tennis.TennisDataGenerator;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.Configuration;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class WalkTestModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Normalizer jointNormal;
	
	private PositionResultMotion totalMotion = new PositionResultMotion();
	private ArrayList<Motion> totalMotion2 = new ArrayList<Motion>();
	private TimeBasedList<Point3d> totalBall = new TimeBasedList<Point3d>();
	
//	double maxLen = 140;
	double maxLen = 220;
	
	@Override
	protected void initializeImpl() {
//		OgreJNI.open(new double[]{ 0 });

		MotionDataConverter.setAllJoints();
//		MotionDataConverter.setCMUJointSet();
//		MotionDataConverter.setUseOrientation();
		
		Configuration.BASE_MOTION_FILE = "walkTest\\motion\\69_01_1.bvh";
		MotionDataConverter.setUseOrientation();
		
//		MotionDataConverter.setAllJoints();
		MotionDataConverter.setNoBall();
//		MotionDataConverter.setUseOrientation();
//		String folder = "walk";
//		String folder = "bwalk_ori";
//		String folder = "loco_walk";
//		String folder = "loco_drunk";
//		String folder = "loco_zombie";
//		String folder = "loco_indian2";
		String folder = "walk";
//		String folder = "loco_gorilla";
//		String folder = "tn_d01";
//		String folder = "walk_zero3";
		python = new RNNPython(folder, false);
		normal = new Normalizer(folder);
		
		g = new RuntimeMotionGenerator();
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
		
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				start();
			}
		});
		addMenu("&Menu", "Test &Stop\tCtrl+S", SWT.MOD1 + 'S', new Runnable() {
			@Override
			public void run() {
				isStop = true;
				System.out.println("stop--------------");
				if (totalMotion2 != null && totalMotion2.size() > 0){
					getModule(ItemListModule.class).addSingleItem("TotalMotion2", new MotionData(totalMotion2));
				} else {
					getModule(ItemListModule.class).addSingleItem("TotalMotion", totalMotion);
				}
				getModule(ItemListModule.class).addSingleItem("TotalBall", totalBall, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
			}
		});
	}
	
	boolean isStop = false;
	Point3d prevBallPos = null;
	
	double[] prevOutput;
	public void start(){
//		double[] initialY = normal.yMeanAndStd[0];
		double[] initialY = normal.yList.get(3);
		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		
		MainViewerModule mainViewer = getModule(MainViewerModule.class);
		long startTime = System.currentTimeMillis();
		
//		MotionTransform t = new MotionTransform();
//		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		prevOutput = initialY;
		dummyParent().getDisplay().timerExec(1, new Runnable() {
			int frame = 0;
			boolean isFirst = false;
			@Override
			public void run() {
				if (dummyParent().isDisposed()) return;
				if (isStop) return;
				
				while (true){
					int dt = (int)(System.currentTimeMillis() - startTime);
					int tIndex = dt/33;
					if (frame > tIndex) break;
					frame++;
					
					
					Point3d target3d;
					if (OgreJNI.isOpened()){
						OgreStatus status = OgreJNI.getStatus();
						target3d = status.mouse;
					} else {
						target3d = mainViewer.getPickPoint();
						if (target3d == null) break;
					}
					Point2d target = Pose2d.to2d(target3d);
//					System.out.println("pp : " + target3d + " : " + target);
					target = g.pose.globalToLocal(target);
					Vector2d v = new Vector2d(target);
					
					double lenLimit = maxLen/2;
					Vector2d n = new Vector2d(v);
					n.normalize();
					lenLimit += lenLimit + maxLen/2*Math.max(n.x, 0);
					
					if (v.length() > maxLen){
						v.scale(maxLen/v.length());
					}
//					v.scale(1d/20);
					
//					int time = getModule(Pose2dEditModule.class).getTime();
//					double movement = (time-100)/70d;
//					double mv = normal.xMeanAndStd[0][2] + movement*normal.xMeanAndStd[1][2];
					
					double[] x = new double[]{ v.x, v.y };
					x = normal.normalizeX(x);
					
					double[] predictX = new double[]{ prevOutput[prevOutput.length-2], prevOutput[prevOutput.length-1] };
					double ratio = 0.2;
					predictX[0] = (1-ratio)*predictX[0] + ratio*x[0];
					predictX[1] = (1-ratio)*predictX[1] + ratio*x[1];
					x = predictX;
//					double[] output = python.model.predict(predictX);
////					double[] output = python.model.predict(x);
//					prevOutput = output;
//					output = normal.deNormalizeY(output);
//					Point2d pp = new Point2d(output[prevOutput.length-2], output[prevOutput.length-1]);
//					pp = g.pose.localToGlobal(pp);
//					double[] dp = normal.deNormalizeX(predictX);
//					Point2d px = new Point2d(dp[0], dp[1]);
//					px = g.pose.localToGlobal(px);
					
					double[] output = python.model.predict(x);
					output = normal.deNormalizeY(output);
					
					PositionResultMotion motion = g.update(output);
//					mainViewer.addCameraTracking(g.pose.position3d());
					totalMotion.addAll(motion);
					getModule(ItemListModule.class).addSingleItem("Motion", motion);
					getModule(ItemListModule.class).addSingleItem("Target", target3d, new ItemDescription(new Vector3d(0, 1, 0)));
					
//					getModule(ItemListModule.class).addSingleItem("pp", Pose2d.to3d(pp), new ItemDescription(new Vector3d(1, 1, 0)));
//					getModule(ItemListModule.class).addSingleItem("px", Pose2d.to3d(px), new ItemDescription(new Vector3d(1, 0, 0)));
				}
				dummyParent().getDisplay().timerExec(1, this);
			}
		});
	}
	
	

	public static void main(String[] args) {
		MainApplication.run(new WalkTestModule());
		OgreJNI.close();
	}
}
