package mrl.motion.neural.experiment;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.run.RNNPython;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;

public class ResponsivenessByData {

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Point2d target = new Point2d();
	private Point2d prevTarget;
	
	private ArrayList<Motion> motionList = new ArrayList<Motion>();
	private ArrayList<Point3d> ballList = new ArrayList<Point3d>();
	private ArrayList<Point3d> targetList = new ArrayList<Point3d>();
	
	public ResponsivenessByData(String name){
		System.out.println("ResponsivenessByData : " + name);
		python = new RNNPython(name, false);
		normal = new Normalizer(name);
		double[] initialY = normal.yList.get(3);
		initialY = new double[initialY.length];
		for (int i = 0; i < initialY.length; i++) {
			initialY[i] = (MathUtil.random.nextDouble()*0.1 - 0.05);
		}
		python.model.setStartMotion(initialY);
		g = new RuntimeMotionGenerator();
	}
	
	
	private Point2d makeTarget(){
//		double length = 4 + 4*MathUtil.random.nextDouble();
//		double length = 600;
		double length = 550 + 100*MathUtil.random.nextDouble();
		Vector2d v = new Vector2d(1, 0);
		v = MathUtil.rotate(v, Math.PI*2*MathUtil.random.nextDouble());
		v.scale(length);
		
		v.add(g.pose.position);
		return new Point2d(v);
	}
	
	int MAX_STEP = 600;
	public void experiment(int count){
		double sum = 0;
		for (int i = 0; i < count; i++) {
			int steps = oneStep();
			sum += steps;
			System.out.println("steps : " + Utils.toString(i, steps, sum/(i+1)));
			if (steps == MAX_STEP){
				showResult();
			}
		}
		System.out.println("pp : " + sum/count);
	}
	
	
	private int oneStep(){
		prevTarget = target;
		target = makeTarget();
		
		double REACH_RADIUS = 100;
		int steps = 0;
		int inBoundaryCount = 0;
		for (steps = 0; steps < MAX_STEP; steps++) {
			update(steps);
			if (g.pose.position.distance(target) < REACH_RADIUS){
				inBoundaryCount++;
				if (inBoundaryCount > 10){
					break;
				}
			}
		}
		return steps;
	}
	
	private void update(int iter){
		Point2d p = new Point2d(target);
		if (iter < 8){
//			Vector2d v1 = MathUtil.sub(prevTarget, g.pose.position);
//			Vector2d v2 = MathUtil.sub(prevTarget, g.pose.position);
			p.interpolate(prevTarget, target, Math.min(1, (iter+1)/10d));
		}
		
		Point2d tp = g.pose.globalToLocal(p);
		Vector2d v = new Vector2d(tp);
		double maxLen = 300;
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
		motionList.add(motion);
		ballList.add(ball);
		targetList.add(Pose2d.to3d(target));
	}
	
	public void showResult(){
		MainViewerModule.runWithDescription(new Object[]{
			new MotionData(motionList), new TimeBasedList<>(ballList), new TimeBasedList<>(targetList)
		}, new ItemDescription[]{
				null, BallTrajectoryGenerator.ballDescription(), null
		});
		System.exit(0);
	}
	
	public static void main(String[] args) {
		BasketDataGenerator.loadBasketData();
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setUseOrientation();
		RuntimeMotionGenerator.ALWAYS_HAS_BALL = true;
//		MotionDataConverter.setNoBall();
		
//		1k	112.74
//		2k	94.95
//		4k	88.75
//		8k	88.34, 87.90
//		16k	83.09
//		32k	81.04
//		64k	82.06 , 81.69
//		128k	80.87

		MathUtil.random = new Random();
//		ResponsivenessByData e = new ResponsivenessByData("drb_origin");
		ResponsivenessByData e = new ResponsivenessByData("drb_e8k"); // 81.3
//		ResponsivenessByData e = new ResponsivenessByData("drb_e16k");
//		ResponsivenessByData e = new ResponsivenessByData("drb_e4k"); // 89
//		ResponsivenessByData e = new ResponsivenessByData("drb_e2k"); // 89
//		ResponsivenessByData e = new ResponsivenessByData("drb_graph_edit");
		e.experiment(300);
		System.exit(0);
		e.showResult();
	}
}
