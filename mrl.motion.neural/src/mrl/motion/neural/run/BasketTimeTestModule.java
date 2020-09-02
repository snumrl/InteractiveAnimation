package mrl.motion.neural.run;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.BasketActivationControl;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class BasketTimeTestModule extends Module{

	private PythonRuntime python;
	private TimeEstimator[] teList;
	
	@Override
	protected void initializeImpl() {
		python = new PythonRuntime("basketTime", 2);
//		python = new PythonRuntime("basketTime_ti", 2);
		
		System.out.println("mean and std :: " +  python.normal.xMeanAndStd[1][10] + "," +  python.normal.xMeanAndStd[0][10]);
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
		
		teList = new TimeEstimator[2];
		for (int i = 0; i < teList.length; i++) {
			teList[i] = new TimeEstimator();
		}
		
		Pose2d[] poseList = new Pose2d[]{
				new Pose2d(-100, -100, -0.5, 1),
				new Pose2d(-100, 300, 1, 0),
				new Pose2d(100, 200, 0.5, 1),
				new Pose2d(100, 300, -1, 0),
			};
		for (Pose2d pose : poseList){
			pose.position.scale(2);
		}
		
		
		Pose2dEditModule poseModule = getModule(Pose2dEditModule.class);
		poseModule.setPoseSize(4);
		poseModule.setPoseList(Utils.toList(poseList));
		
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				generate(Utils.toArray(poseModule.getPoseList()));
			}
		});
		
		
	}
	
	private double nTime(double t){
		return t * python.normal.xMeanAndStd[1][10] + python.normal.xMeanAndStd[0][10];
	}
	private double dnTime(double t){
		return t/python.normal.xMeanAndStd[1][10];
	}
	
	private void generate(Pose2d[] poseList){
		ArrayList<Pose2d> originList = new ArrayList<Pose2d>();
		for (Pose2d p : poseList) originList.add(new Pose2d(p));
		
		RuntimeMotionGenerator[] gList = new RuntimeMotionGenerator[2];
		for (int i = 0; i < gList.length; i++) {
			gList[i] = new RuntimeMotionGenerator();
			gList[i].pose = poseList[i*2];
		}
		
		int dLen = python.normal.yList.get(3).length;
		python.setStartMotion(new double[][]{ 
//				python.normal.yList.get(3), 
//				python.normal.yList.get(1000),
				
				new double[dLen],
				new double[dLen],
		});
		
		PositionResultMotion[] pmList = new PositionResultMotion[2];
		for (int i = 0; i < pmList.length; i++) {
			pmList[i] = new PositionResultMotion();
		}
		String[] actions = new String[]{
//			"pass", "pass'"	
			"pass'", "pass"	
//			"pass'", "pass"	
		};
		
		double activateFrame = -1;
		for (int iter = 0; iter < 300; iter++) {
			double[][] goalList = new double[2][];
			for (int i = 0; i < gList.length; i++) {
				RuntimeMotionGenerator g = gList[i];
				Pose2d t = Pose2d.relativePose(g.pose, poseList[i*2 + 1]);
				double[] action = BasketDataGenerator.getActionType(actions[i]);
				double[] goal = BasketDataGenerator.getControlData(t);
//				# x : acti, action=3, goal pos=2, goal ori=2, normalized goal pos=2, remain time => 12
				goal = python.normal.normalizeX(goal, 4);
				goalList[i] = MathUtil.concatenate(action, goal);
			}
			
			double[][] estimated = python.estimate(goalList);
//			if (activateFrame < 0){
//				double eTime = nTime(Math.max(estimated[0][0], estimated[1][0]));
//				System.out.println("accc :: " + iter 
//						+ " : " + Utils.toString(nTime(estimated[0][0]), nTime(estimated[1][0])));
//				if (eTime < 30){
//					activateFrame = iter + BasketActivationControl.PRE_ACT_MARGIN;
//				}
//			}
			double activation = 0;
			if (activateFrame >= 0){
				activation = BasketActivationControl.getActivation(Math.abs(activateFrame - iter), BasketActivationControl.ACTIVATION_MARGIN);
			}
			System.out.print("activation :: " + iter + " :: " + activation);
			double[][] inputList = new double[2][];
			for (int i = 0; i < gList.length; i++) {
				double est_time1 = estimated[i][0];
				double est_time2 = estimated[(i+1)%2][0];
				double time = teList[i].update(est_time1, est_time2);
				
				System.out.print(", " + nTime(time));
				if (activateFrame < 0 && nTime(time) <= 30){
					activateFrame = iter + BasketActivationControl.PRE_ACT_MARGIN;
				}
				double[] timeAndActi = new double[]{ time, activation };
				double[] gInput = MathUtil.concatenate(timeAndActi, goalList[i]);
				inputList[i] = gInput;
			}
			System.out.println();
			double[][] output = python.predict(inputList);
			for (int i = 0; i < output.length; i++) {
				output[i] = python.normal.deNormalizeY(output[i]);
				pmList[i].add(gList[i].update(output[i]).get(0));
			}
			
			if (activateFrame >= 0 && iter > activateFrame+50) break;
		}
		
		getModule(ItemListModule.class).addSingleItem("motion1", pmList[0]);
		getModule(ItemListModule.class).addSingleItem("motion2", pmList[1]);
		getModule(ItemListModule.class).addSingleItem("Target", originList, new ItemDescription(new Vector3d(0, 1, 0)));
	}
	
	private class TimeEstimator{
		
		private boolean isFirst = true;
		private double start = -1;
		
		public double update(double my, double opposite){
			if (isFirst){
				isFirst = false;
				start = (my + opposite)/2*0.8;
			}
			start -= dnTime(1);
			return start;
//			return Math.max(my, opposite) - 1;
		}
	}

	public static void main(String[] args) {
		MainApplication.run(new BasketTimeTestModule());
	}
}
