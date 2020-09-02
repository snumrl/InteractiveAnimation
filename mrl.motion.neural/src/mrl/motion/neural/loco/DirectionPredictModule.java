package mrl.motion.neural.loco;

import java.io.File;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

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
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class DirectionPredictModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Normalizer jointNormal;
	
	private PositionResultMotion totalMotion = new PositionResultMotion();
	private ArrayList<Motion> totalMotion2 = new ArrayList<Motion>();
	private TimeBasedList<Point3d> totalBall = new TimeBasedList<Point3d>();
	
	private ArrayList<DPStatus> statusList = new ArrayList<DPStatus>();
	
	private boolean useDirection = false;
	private Vector2d direction = new Vector2d(1, 0);
	
	double maxLen = 150;
//	double maxLen = 250;
	
	@Override
	protected void initializeImpl() {
//		OgreJNI.open(new double[]{ 0 });
		
//		MotionDataConverter.setCMUJointSet();
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setNoBall();
//		MotionDataConverter.setUseOrientation();
		
//		alwaysUseDirection = useDirection = true;
		String folder = "loco_edin_dr_pre";
//		String folder = "loco_bwalk_dr_pre";
//		String folder = "loco_bwalk_dr";
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
				loadSavedMotion();
			}
		});
		addMenu("&Menu", "Test &Load\tCtrl+L", SWT.MOD1 + 'L', new Runnable() {
			@SuppressWarnings("unchecked")
			@Override
			public void run() {
				String folder =  "output\\" + python.name;
				totalMotion = (PositionResultMotion)FileUtil.readObject(folder + "\\positionMotion.prm");
				statusList = (ArrayList<DPStatus>)FileUtil.readObject(folder + "\\statusList.dat");
				loadSavedMotion();
				
			}
		});
		
		addMenu("&Menu", "Test &Write\tCtrl+W", SWT.MOD1 + 'W', new Runnable() {
			@Override
			public void run() {
				if (!isStop) return;
				
				PositionResultMotion mSequence = g.motionSequence;
				String folder =  "output\\" + python.name;
				new File(folder).mkdirs();
				FileUtil.writeObject(mSequence, folder + "\\positionMotion.prm");
				FileUtil.writeObject(statusList, folder + "\\statusList.dat");
			}
		});
		
		
		addMenu("&Menu", "Test d\tCtrl+D", SWT.MOD1 + 'D', new Runnable() {
			@Override
			public void run() {
				useDirection = !useDirection;
				System.out.println("useDirection : " + useDirection);
			}
		});
		addMenu("&Menu", "Test 1\tCtrl+1", SWT.MOD1 + '1', new Runnable() {
			@Override
			public void run() {
				direction = MathUtil.rotate(direction, Math.toRadians(15));
				System.out.println("direction : " + direction);
			}
		});
		addMenu("&Menu", "Test 2\tCtrl+2", SWT.MOD1 + '2', new Runnable() {
			@Override
			public void run() {
				direction = MathUtil.rotate(direction, Math.toRadians(-15));
				System.out.println("direction : " + direction);
			}
		});
	}
	
	private void loadSavedMotion(){
		getModule(ItemListModule.class).addSingleItem("Motion", totalMotion);
		
		TimeBasedList<Point3d> targetList= new TimeBasedList<Point3d>();
		for (DPStatus s : statusList){
			targetList.add(Pose2d.to3d(s.targetPoint));
		}
		getModule(ItemListModule.class).addSingleItem("Target", targetList, new ItemDescription(new Vector3d(0, 1, 0)));
		
		TimeBasedList<Pose2d> dirList= new TimeBasedList<Pose2d>();
		for (int i = 0; i < statusList.size(); i++) {
			if (statusList.get(i).useDirection){
				dirList.add(statusList.get(i).direction);
			} else {
				dirList.add(null);
			}
		}
		getModule(ItemListModule.class).addSingleItem("Direction", dirList, new ItemDescription(new Vector3d(0, 1, 0)));
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
		
		MotionTransform t = new MotionTransform();
		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
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
					
					DPStatus dpStatus = new DPStatus();
					
					Point3d target3d;
					if (OgreJNI.isOpened()){
						OgreStatus status = OgreJNI.getStatus();
						target3d = status.mouse;
					} else {
						target3d = mainViewer.getPickPoint();
						if (target3d == null) break;
					}
					Point2d target = Pose2d.to2d(target3d);
					dpStatus.targetPoint = new Point2d(target);
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
					Vector2d d = g.pose.globalToLocal(direction);
					double[] x = new double[]{ v.x, v.y, d.x, d.y};
					x = normal.normalizeX(x);
					
					
					if (!useDirection){
						double[] predictX = new double[]{ prevOutput[prevOutput.length-2], prevOutput[prevOutput.length-1] };
						x[2] = predictX[0];
						x[3] = predictX[1];
					} else {
//						prevOutput[prevOutput.length-2] = x[2];
//						prevOutput[prevOutput.length-1] = x[3];
//						python.model.setStartMotionOnly(prevOutput);
					}
					
					double[] output = python.model.predict(x);
					prevOutput = output;
					output = normal.deNormalizeY(output);
					
					
					PositionResultMotion motion = g.update(output);
//					mainViewer.addCameraTracking(g.pose.position3d());
					totalMotion.addAll(motion);
					getModule(ItemListModule.class).addSingleItem("Motion", motion);
					getModule(ItemListModule.class).addSingleItem("Target", target3d, new ItemDescription(new Vector3d(0, 1, 0)));
					dpStatus.useDirection = useDirection;
					if (useDirection){
						dpStatus.direction = new Pose2d(g.pose.position, direction);
						getModule(ItemListModule.class).addSingleItem("Direction", dpStatus.direction, new ItemDescription(new Vector3d(0, 1, 0)));
					} else {
						dpStatus.direction = new Pose2d(g.pose.position, new Vector2d(output[output.length-2], output[output.length-1]));
						getModule(ItemListModule.class).addSingleItem("Direction", dpStatus.direction, new ItemDescription(new Vector3d(1, 1, 0)));
//						getModule(ItemListModule.class).addSingleItem("Direction", g.pose.position, new ItemDescription(new Vector3d(0, 1, 0)));
					}
					statusList.add(dpStatus);
					
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
		MainApplication.run(new DirectionPredictModule());
		OgreJNI.close();
	}
}
