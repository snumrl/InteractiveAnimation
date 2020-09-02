package mrl.motion.neural.run;

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
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.data.ShootTimeControl.ActionType;
import mrl.motion.neural.data.ShootTimeControl;
import mrl.motion.position.PositionMotion;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;
import static mrl.motion.neural.data.ShootTimeControl.*;

public class AllTestModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Pose2dEditModule poseModule;
	
	private PointIKSolver solver;
	
	String folder = "all_time5"; // 120
//	String folder = "dribble_g4"; // 120
//		String folder = "dribble_g"; // 120
//		String folder = "dribble_g2"; // 60
//		String folder = "walk";
//		String folder = "walk_gru";
	
	@Override
	protected void initializeImpl() {
		MotionDataConverter.setAllJoints();
		python = new RNNPython(folder, false);
		normal = new Normalizer(folder);
		
		
		MotionTransform t = new MotionTransform();
		solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
		
		poseModule = getModule(Pose2dEditModule.class);
		poseModule.setPoseSize(3);
		ArrayList<Pose2d> pList = new ArrayList<Pose2d>();
		pList.add(new Pose2d(600,0,0.9779,-0.2090));
		pList.add(new Pose2d(Pose2d.BASE));
		pList.add(new Pose2d(Pose2d.BASE));
		ArrayList<Integer> timeList = new ArrayList<Integer>();
		timeList.add(90);
		timeList.add(150);
		timeList.add(0);
		poseModule.setPoseList(pList);
		poseModule.setTimeList(new int[]{ 90, 100, 1 });
		
//		poseModule.setPoseList(poseList);
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
		double[] initialY = normal.yList.get(100192);
//		initialY = new double[initialY.length];
		String[] actions = { "pass", "shoot", "pass'" };
		
		String action = actions[poseModule.getTimeList().get(2)-1];
		if (action.equals("pass'")){
			initialY = normal.yList.get(100);
		}
		System.out.println("Action :: " + action);
		python.model.setStartMotion(initialY);
		
		ArrayList<Motion> motionList = new ArrayList<Motion>();
		TimeBasedList<Point3d> ballTrajectory = new TimeBasedList<Point3d>();
		TimeBasedList<Pose2d> targetList = new TimeBasedList<Pose2d>();
		g = new RuntimeMotionGenerator();
		int reachTime = -1;
		for (int i = 0; i < time+15; i++) {
			Pose2d target = Pose2d.relativePose(g.pose, pose);
			double remainTime = time - i;
			
			double movement = (poseModule.getTimeList().get(1) - 100)/50d;
//			0.30933822363081215, 0.14159156995791775
			movement = 0.191 + movement*0.136;
//			movement = 0.309 + movement*0.141;
//			double movement = (poseModule.getTimeList().get(1) - 20)/160d;
//			double movement = (poseModule.getTimeList().get(1)-100)/70d;
			
			double[] control = BasketDataGenerator.getActionType(action);
			double[] goalInfo;
			double maxTime = MAX_TIME;
//			double maxTime = MAX_TIME - 30;
			boolean isOverTime = remainTime >= maxTime;
			if (isOverTime){
				isOverTime = true;
				
				double ratio = maxTime/remainTime;
				remainTime = maxTime;
				target.position.scale(ratio);
				System.out.println("scale :: " + i + " : " + ratio);
				goalInfo = BasketDataGenerator.getControlData(target);
				// position only
				goalInfo[2] = 1;
				goalInfo[3] = 0;
			} else {
				isOverTime = false;
				goalInfo = BasketDataGenerator.getControlData(target);
			}
			targetList.add(g.pose.localToGlobal(target));
			double activation;
			if (reachTime < 0){
				activation = Math.max(0, 1 - Math.max(0, remainTime)/ACTIVATION_MARGIN);
			} else {
				activation = Math.max(0, 1 - Math.abs(remainTime)/ACTIVATION_MARGIN);
			}
			
			control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0, activation, remainTime });
			control = MathUtil.concatenate(control, goalInfo);
			control = MathUtil.concatenate(control, new double[]{ movement });
			
			double[] x = control;
			x = normal.normalizeX(x);
			double[] output = python.model.predict(x);
			output = normal.deNormalizeY(output);
			g.update(output);
			double hasBall = output[output.length-1];
			System.out.println("goal :: " + Arrays.toString(goalInfo) + " : " + hasBall + " : "+ activation);
			if (reachTime < 0 && hasBall < 0.5){
				reachTime = i-1;
			}
			
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
		System.out.println("reach time :: " + reachTime);
		
		getModule(ItemListModule.class).addSingleItem("Motion", g.motionSequence);
		getModule(ItemListModule.class).addSingleItem("Target", pose, new ItemDescription(new Vector3d(0, 1, 0)));
		getModule(ItemListModule.class).addSingleItem("targetList", targetList, new ItemDescription(new Vector3d(0, 1, 1)));
		getModule(ItemListModule.class).addSingleItem("Ball", ballTrajectory, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
//		MotionData mData = new MotionData(motionList);
//		getModule(ItemListModule.class).addSingleItem("Motion2", new MotionData(motionList));
//		OgreJNI.instance.setMotion(new MotionData[]{ mData } );
	}

	public static void main(String[] args) {
//		MotionDataConverter.setNoBall();
		MotionDataConverter.setAllJoints();
		MainApplication.run(new AllTestModule());
//		OgreJNI.instance.close();
	}
}

