package mrl.motion.neural.loco;

import java.io.File;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.parser.BVHWriter;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.data.PointIKSolver_CMU;
import mrl.motion.neural.run.RNNPython;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.neural.tennis.TennisDataGenerator;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.position.PositionResultMotion.PositionFrame;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.Configuration;
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class LocomotionRecordModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Normalizer jointNormal;
	
	private PositionResultMotion totalMotion = new PositionResultMotion();
	private ArrayList<Point2d> targetList = new ArrayList<Point2d>();
	
	private boolean useDirection = false;
	private Vector2d direction = new Vector2d(1, 0);
	
	double maxLen = 150;
//	double maxLen = 250;
	
	@Override
	protected void initializeImpl() {
//		OgreJNI.open(new double[]{ 0 });
		
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setNoBall();
//		MotionDataConverter.setCMUAllJoints();
//		MotionDataConverter.setUseOrientation();
		
		
//		alwaysUseDirection = useDirection = true;
//		String folder = "loco_indian2";
//		String folder = "loco_gorilla";
//		String folder = "loco_drunk";
//		String folder = "loco_bwalk_dr_pre";
		String folder = "loco_bwalk_dr";
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
				
				PositionResultMotion mSequence = g.motionSequence;
				String folder =  "output\\" + python.name;
				new File(folder).mkdirs();
//				BVHWriter bw = new BVHWriter();
//				bw.write(new File(folder + "\\motion.bvh"), new MotionData(g.motionList));
				FileUtil.writeObject(mSequence, folder + "\\positionMotion.prm");
				FileUtil.writeObject(targetList, folder + "\\targetList.dat");
				loadSavedMotion();
			}
		});
		addMenu("&Menu", "Test &Load\tCtrl+L", SWT.MOD1 + 'L', new Runnable() {
			@SuppressWarnings("unchecked")
			@Override
			public void run() {
				String folder =  "output\\" + python.name;
				totalMotion = (PositionResultMotion)FileUtil.readObject(folder + "\\positionMotion.prm");
				targetList = (ArrayList<Point2d>)FileUtil.readObject(folder + "\\targetList.dat");
				loadSavedMotion();
			}
		});
	}
	
	private void loadSavedMotion(){
		getModule(ItemListModule.class).addSingleItem("Motion", totalMotion);
//		ArrayList<Motion> solvedList = new ArrayList<Motion>();
//		for (PositionFrame f : totalMotion){
//			solvedList.add(RuntimeMotionGenerator.ikSolver.solve(f));
//		}
		
//		getModule(ItemListModule.class).addSingleItem("IK", new MotionData(solvedList));
		TimeBasedList<Point3d> t3dList= new TimeBasedList<Point3d>();
		for (Point2d s : targetList){
			t3dList.add(Pose2d.to3d(s));
		}
		getModule(ItemListModule.class).addSingleItem("Target", t3dList, new ItemDescription(new Vector3d(0, 1, 0)));
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
					targetList.add(new Point2d(target));
//					System.out.println("pp : " + target3d + " : " + target);
					target = g.pose.globalToLocal(target);
					Vector2d v = new Vector2d(target);
					
					double lenLimit = maxLen;
					Vector2d n = new Vector2d(v);
					n.normalize();
//					lenLimit += lenLimit + maxLen/2*Math.max(n.x, 0);
					
					if (v.length() > maxLen){
						v.scale(maxLen/v.length());
					}
//					v.scale(1d/20);
					
//					int time = getModule(Pose2dEditModule.class).getTime();
//					double movement = (time-100)/70d;
//					double mv = normal.xMeanAndStd[0][2] + movement*normal.xMeanAndStd[1][2];
					double[] x = new double[]{ v.x, v.y};
					x = normal.normalizeX(x);
					
					double[] output = python.model.predict(x);
					prevOutput = output;
					output = normal.deNormalizeY(output);
					
					
					PositionResultMotion motion = g.update(output);
//					mainViewer.addCameraTracking(g.pose.position3d());
					totalMotion.addAll(motion);
					getModule(ItemListModule.class).addSingleItem("Motion", motion);
//					getModule(ItemListModule.class).addSingleItem("IK", new MotionData(Utils.singleList(Utils.last(g.motionList))));
					getModule(ItemListModule.class).addSingleItem("Target", target3d, new ItemDescription(new Vector3d(0, 1, 0)));
					
//					getModule(MainViewerModule.class).addCameraTracking(Pose2d.to3d(g.pose.position));
				}
				dummyParent().getDisplay().timerExec(1, this);
			}
		});
	}
	
	
	public static class DPStatus implements Serializable{
		private static final long serialVersionUID = 3530668356241288117L;
		
		public Point2d targetPoint;
		public Pose2d direction;
		public boolean useDirection;
		
		public DPStatus(){
		}
	}

	public static void main(String[] args) {
//		Configuration.BASE_MOTION_FILE = "cmu.bvh";
//		Configuration.BASE_MOTION_FILE = "locomotion\\edin_loco\\motion\\locomotion_jog_000_000.bvh";
		MainApplication.run(new LocomotionRecordModule());
		OgreJNI.close();
	}
}
