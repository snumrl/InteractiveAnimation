package mrl.motion.neural.run;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.DataExtractor;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.position.PositionResultMotion.PositionFrame;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class TrainingValidationModule2 extends Module {

	@Override
	protected void initializeImpl() {
		MotionDataConverter.setAllJoints();
//		MotionDataConverter.setNoBallVelocity();
		MotionDataConverter.setNoBall();
//		MotionDataConverter.setUseOrientation();
		String label = "loco_edin_sd";
		
		String folder = label;
		if (folder != null){
			folder = "neuralData" + "\\" + folder + "\\data";
		} else {
			folder = ".";
		}
		
		double[][] xMeanAndStd = DataExtractor.readNormalizeInfo(folder + "\\xNormal.dat");
		double[][] yMeanAndStd = DataExtractor.readNormalizeInfo(folder + "\\yNormal.dat");
		ArrayList<ArrayList<double[]>> xList = DataExtractor.readListData(folder + "\\xData.dat");
		ArrayList<ArrayList<double[]>> yList = DataExtractor.readListData(folder + "\\yData.dat");
		
		double min = Integer.MAX_VALUE;
		double max = -Integer.MAX_VALUE;
		for (ArrayList<double[]> ll : xList){
			for (double[] l : ll){
				for (double v : l){
					min = Math.min(min, v);
					max = Math.max(max, v);
					if (Double.isNaN(v)){
						System.out.println("VVVVVVVVVVVVVVVVVVV");
						System.exit(0);
					}
				}
			}
		}
		for (ArrayList<double[]> ll : yList){
			for (double[] l : ll){
				for (double v : l){
					min = Math.min(min, v);
					max = Math.max(max, v);
					if (Double.isNaN(v)){
						System.out.println("VVVVVVVVVVVVVVVVVVV");
						System.exit(0);
					}
				}
			}
		}
		System.out.println("minmax :: " + min + " , " + max);
		
		int dataIndex = 88;
		
		Normalizer normal = new Normalizer(xMeanAndStd, yMeanAndStd, xList.get(dataIndex), yList.get(dataIndex));
		System.out.println(normal.xMeanAndStd[0].length + " , " + normal.yMeanAndStd[0].length);
		System.out.println(Arrays.toString(normal.xMeanAndStd[0]));
		System.out.println(Arrays.toString(normal.xMeanAndStd[1]));
		System.out.println(Arrays.toString(normal.yMeanAndStd[0]));
		System.out.println(Arrays.toString(normal.yMeanAndStd[1]));
		
//		for (int i = 0; i < normal.yMeanAndStd[1].length; i++) {
//			if (normal.yMeanAndStd[1][i] <= 0.001){
//				int jIndex = (i - 6)/3;
//				int aIndex = (i - 6)%3;
//				String j = MotionDataConverter.OrientationJointList[jIndex];
//				System.out.println(i + " ; " + j + aIndex + " : " + normal.yMeanAndStd[0][i] + "\t" + normal.yMeanAndStd[1][i]);
//			}
//		}
		System.out.println("########");
		System.exit(0);
		
		RuntimeMotionGenerator g = new RuntimeMotionGenerator();
		PositionResultMotion pm = new PositionResultMotion();
		TimeBasedList<Pose2d> targetList = new TimeBasedList<Pose2d>();
		int poseStart = 0;
		int len = Math.min(1000, normal.yList.size());
		int offset = 0;
//		int offset = 200000;
		MotionTransform t = new MotionTransform();
		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		ArrayList<Motion> motionList = new ArrayList<Motion>();
		
//		int len = Math.min(10000, normal.yList.size());
		TimeBasedList<Point3d> ballList = new TimeBasedList<Point3d>();
		for (int i = offset; i < offset+len; i++) {
			double[] y = normal.yList.get(i);
			y = normal.deNormalizeY(y);
			System.out.println("## " + i + " : " + (i - offset) + " : " + y[y.length-1]);
//			System.out.println(Arrays.toString(y));
			
			double[] x = normal.xList.get(i);
			x = normal.deNormalizeX(x);
			System.out.println(Arrays.toString(x));
			if (poseStart < x.length){
				Pose2d p;
				int s = poseStart;
				if (poseStart + 4 <= x.length){
					p = g.pose.localToGlobal(new Pose2d(x[s+0], x[s+1], x[s+2], x[s+3]));
				} else {
					p = g.pose.localToGlobal(new Pose2d(x[s+0], x[s+1], 1, 0));
					p.direction = Pose2d.BASE.direction;
				}
				targetList.add(p);
			}
			
			
			PositionResultMotion motion = g.update(y);
			pm.add(motion.get(0));
			
			HashMap<String, Point3d> map = MotionDataConverter.dataToPointMap(y);
			Motion mm = solver.solve(map, g.pose);
			mm.ballContact = motion.get(0).ballContact;
			mm.isLeftFootContact = motion.get(0).footContact.left;
			mm.isRightFootContact = motion.get(0).footContact.right;
			motionList.add(mm);
			
			if (MotionDataConverter.includeBall){
				double[] b = new double[3];
				System.arraycopy(y, 0, b, 0, 3);
				Point3d p = new Point3d(b[0], 0, b[2]);
				p = Pose2d.to3d(g.pose.localToGlobal(Pose2d.to2d(p)));
				p.y = b[1];
				ballList.add(p);
				System.out.println("bb : " + p + " : " + y[y.length-1]);
			}
		}
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("motion", pm);
//		getModule(ItemListModule.class).addSingleItem("motion2", new MotionData(motionList));
		getModule(ItemListModule.class).addSingleItem("target", targetList);
		getModule(ItemListModule.class).addSingleItem("ball", ballList, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
		
		
//		getModule(MainViewerModule.class).addTimeListener(new Listener() {
//			@Override
//			public void handleEvent(Event event) {
//				Motion m = motionList.get(getModule(MainViewerModule.class).getTimeIndex());
//				OgreJNI.setMotion(new MotionData[]{ new MotionData(Utils.singleList(m)) });
//			}
//		});
	}
	
	public static void main(String[] args) {
		MainApplication.run(new TrainingValidationModule2());
		OgreJNI.close();
	}

}
