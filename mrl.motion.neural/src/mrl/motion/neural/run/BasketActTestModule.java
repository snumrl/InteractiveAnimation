package mrl.motion.neural.run;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.BasketActivationControl;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class BasketActTestModule extends Module{

	private PythonRuntime python;
	
	@Override
	protected void initializeImpl() {
		python = new PythonRuntime("basketAct", 1);
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
		Pose2dEditModule poseModule = getModule(Pose2dEditModule.class);
		
		
		poseModule.setPose(new Pose2d(288.3849,488.9990,0.5040,0.8637));
		poseModule.setTime(200);
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
//				start(poseModule.getPose(), poseModule.getTime());
				trainingValidation(500);
			}
		});
		
		
	}
	
	private double[][] to2d(double[] data){
		return new double[][] { data } ;
	}
	
	private void trainingValidation(int timeLimit){
		int startIdx = 3000;
		double[] initialY = python.normal.yList.get(startIdx + 1000);
		python.setStartMotion(to2d(initialY));
		RuntimeMotionGenerator g = new RuntimeMotionGenerator();
		PositionResultMotion motion = new PositionResultMotion();
		
		TimeBasedList<Pose2d> targetList = new TimeBasedList<Pose2d>();
		for (int i = 0; i < timeLimit; i++) {
			double[] x = python.normal.xList.get(startIdx + i);
			double[] goal = Utils.cut(x, 2, 10);
//			goal[4] += Utils.rand1();
			goal[8] += Utils.rand1();
//			goal[6] += Utils.rand1();
//			double[] eOutput = python.estimate(goal);
//			
//			double[] tAndA = new double[]{
//				eOutput[0] - x[11],
//				x[1]
//			};
//			System.out.println(i  + "\t" + Arrays.toString(eOutput) + " ,, " + Arrays.toString(tAndA));
			System.out.println(i + ": " + Arrays.toString(Utils.cut(goal, 0, 2)));
			
			double[] tAndA = new double[]{
					0, x[1]
				};
//			tAndA[0] = 0;
//			tAndA[1] = 0;
			double[] cInput = MathUtil.concatenate(tAndA, goal);
			double[] output = python.predict(to2d(cInput))[0];
			output = python.normal.deNormalizeY(output);
			
			x = python.normal.deNormalizeX(cInput);
//			x = python.normal.deNormalizeX(x);
			double[] p = Utils.cut(x, 5, 8);
			Pose2d rp = new Pose2d(p[0], p[1], p[2], p[3]);
			targetList.add(g.pose.localToGlobal(rp));
			
			motion.add(g.update(output).get(0));
			
			
			
		}
		getModule(ItemListModule.class).addSingleItem("target", targetList);
		getModule(ItemListModule.class).addSingleItem("motion", motion);
	}
	
	public void start(Pose2d target, int time){
		double[] initialY = python.normal.yList.get(1000);
		python.setStartMotion(to2d(initialY));
		
		RuntimeMotionGenerator g = new RuntimeMotionGenerator();
		PositionResultMotion motion = new PositionResultMotion();
		TimeBasedList<Pose2d> targetList = new TimeBasedList<Pose2d>();
		for (int i = 0; i < time; i++) {
			Pose2d t = Pose2d.relativePose(g.pose, target);
			
			int aTime = 100;
			double activation = BasketActivationControl.getActivation(Math.abs(aTime - i), BasketActivationControl.ACTIVATION_MARGIN);
			activation = 0;
			
			
			double[] timeAndActi = new double[]{ -0.5, activation };
//			double[] timeAndActi = new double[]{ 0.2 + Utils.rand1()*0.2, activation };
			double[] action = BasketDataGenerator.getActionType("pass");
			double[] goal = BasketDataGenerator.getControlData(t, Integer.MIN_VALUE);
//			# x : pre-acti, acti, action=3, goal pos=2, goal ori=2, normalized goal pos=2, remain time => 12
			goal = python.normal.normalizeX(goal, 5);
//			System.out.println(i + "\t" + Arrays.toString(python.estimate(MathUtil.concatenate(action, goal))) + " : " + activation);
			System.out.println("gg  :: " + i + " : " + Arrays.toString(goal));
			
//			# cInput : v_time, acti, action=3, goal=6, prev_y=45
			double[] gInput = MathUtil.concatenate(timeAndActi, action, goal);
			double[] output = python.predict(to2d(gInput))[0];
			output = python.normal.deNormalizeY(output);
			
			double[] x = python.normal.deNormalizeX(gInput);
			double[] p = Utils.cut(x, 5, 10);
			Pose2d rp = new Pose2d(p[0], p[1], p[2], p[3]);
//			System.out.println("rp : " + i + " : " + Arrays.toString(p));
			targetList.add(g.pose.localToGlobal(rp));
			
			motion.add(g.update(output).get(0));
		}
		getModule(ItemListModule.class).addSingleItem("targetList", targetList);
		getModule(ItemListModule.class).addSingleItem("target", target);
		getModule(ItemListModule.class).addSingleItem("motion", motion);
	}

	public static void main(String[] args) {
		MainApplication.run(new BasketActTestModule());
	}
}
