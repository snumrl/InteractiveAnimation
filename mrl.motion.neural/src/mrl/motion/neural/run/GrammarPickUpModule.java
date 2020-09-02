package mrl.motion.neural.run;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.data.BasketPickupControl;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.Pose2dEditModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.module.MainViewerModule.MainViewer;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.MathUtil;
import mrl.util.Pair;
import mrl.util.TimeChecker;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;

public class GrammarPickUpModule extends Module{

	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	private Normalizer jointNormal;
	
	private PositionResultMotion totalMotion = new PositionResultMotion();
	private ArrayList<Motion> totalMotion2 = new ArrayList<Motion>();
	private TimeBasedList<Point3d> totalBall = new TimeBasedList<Point3d>();
	private Integer currentKey = null;
	private Controller controller = new Controller();
	private static Point3d GOAL_POS = new Point3d(0, 330, 120);
	
	@Override
	protected void initializeImpl() {
		MotionDataConverter.setAllJoints();
//		OgreJNI.open(OgreJNI.courtParam());
		
//		MotionDataConverter.setNoBall();
//		String folder = "shoot5";
//		
		String folder = "shoot_o_lc";
		BasketGraph.INCLUDE_CATCH = true;
		python = new RNNPython(folder);
		normal = new Normalizer(folder);
		
		g = new RuntimeMotionGenerator();
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("Origin", Pose2d.BASE, new ItemDescription(new Vector3d(1, 0, 0)));
//		getModule(Pose2dEditModule.class);
		
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
		getModule(ItemListModule.class).addSingleItem("goalPos", GOAL_POS, new ItemDescription(new Vector3d(0,1,1)));
		
		MainViewer viewer = getModule(MainViewerModule.class).getMainViewer();
		viewer.getCanvas().addKeyListener(new KeyListener() {
			@Override
			public void keyReleased(KeyEvent e) {
				currentKey = null;
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
				currentKey = (int)e.character;
			}
		});
	}
	
	boolean isStop = false;
	int frame = 0;
	public void start(){
		double[] initialY = normal.yList.get(100000 + 40);
		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		
		MainViewerModule mainViewer = getModule(MainViewerModule.class);
		long startTime = System.currentTimeMillis();
		
		MotionTransform t = new MotionTransform();
		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		
		frame = 0;
		TimeChecker.instance.enable = true;
		dummyParent().getDisplay().timerExec(1, new Runnable() {
			
			boolean isFirst = false;
			long  lastTime = System.currentTimeMillis();
			@Override
			public void run() {
				if (dummyParent().isDisposed()) return;
				if (isStop) return;
				
				long tt = System.currentTimeMillis() - lastTime;
				lastTime = System.currentTimeMillis();
				System.out.println(tt);
				
				MotionData mData = null;
				while (true){
					int dt = (int)(System.currentTimeMillis() - startTime);
					int tIndex = dt/33;
					if (frame > tIndex) break;
					frame++;
					
					
					Point3d target3d;
					if (OgreJNI.isOpened()){
						OgreStatus status = OgreJNI.getStatus();
						target3d = status.mouse;
						currentKey = status.key;
					} else {
						target3d = mainViewer.getPickPoint();
					}
					if (target3d == null) break;
					
					TimeChecker.instance.state("update");
					PositionResultMotion motion = controller.update(target3d);
					totalMotion.addAll(motion);
//					getModule(ItemListModule.class).addSingleItem("Motion", motion);
					getModule(ItemListModule.class).addSingleItem("Target", target3d, new ItemDescription(new Vector3d(0, 1, 0)));
					
					
					TimeChecker.instance.state("solve");
					HashMap<String, Point3d> map = MotionDataConverter.dataToPointMap(controller.output);
					Motion mm = solver.solve(map, g.pose);
					mm.ballContact = motion.get(0).ballContact;
					mm.isLeftFootContact = motion.get(0).footContact.left;
					mm.isRightFootContact = motion.get(0).footContact.right;
					mData = new MotionData(Utils.singleList(mm));
					totalMotion2.add(mm);
					getModule(ItemListModule.class).addSingleItem("Motion2", mData);
					TimeChecker.instance.state("other");
					
					if (controller.ballPosition != null){
						getModule(ItemListModule.class).addSingleItem("ball", controller.ballPosition, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
						totalBall.add(controller.ballPosition);
					}
				}
				if (mData != null){
					OgreJNI.setMotion(new MotionData[] { mData });
					if (controller.ballPosition != null){
						OgreJNI.setBall(Utils.singleList(controller.ballPosition));
					}
				}
				dummyParent().getDisplay().timerExec(1, this);
			}
		});
	}
	
	public enum ActionType { 
		Dribble, Walk, Shoot, Pickup, Shoot_Near; 
		
		public String actionName(){
			return name().toLowerCase();
		}
		
		public boolean isShoot(){
			return this == Shoot || this == Shoot_Near;
		}
	}
	
	private class Controller{
		
		
		
		private ActionType currentType = ActionType.Dribble;
		private boolean hasBall = true;
		private double[] output;
		private Point3d ballPosition = new Point3d();
		private int shootAfterCount = -1;
		private BallTrajectoryGenerator bg = new BallTrajectoryGenerator();
		private ArrayList<Point3d> ballShootTrajectory;
		private boolean isBallChanged = false;
		private Point3d pickupTarget = null;
		private Integer prevKey = null;
		
		private int actionStartFrame = -1;
		private int actionFrame = -1;
		
		Controller(){
		}
		
		public PositionResultMotion update(Point3d target3d){
			if (MotionDataConverter.includeBall && currentType == ActionType.Pickup){
//				target3d = pickupTarget;
			}
			Point2d targetP = Pose2d.to2d(target3d);
			Vector2d direction = Pose2d.BASE.direction;
			if (currentType.isShoot()){
				direction = MathUtil.sub(Pose2d.to2d(GOAL_POS), targetP);
			} else if (currentType == ActionType.Walk){
				direction = MathUtil.sub(Pose2d.to2d(pickupTarget), targetP);
			}
			Pose2d target = new Pose2d(targetP, direction);
			target = Pose2d.relativePose(g.pose, target);
			normalize(target.position, 250);
//			if (currentType != ActionType.Shoot){
//				target.direction = Pose2d.BASE.direction;
//			}
			
			String aType = null;
			double overTime = 0;
			switch (currentType){
			case Dribble:
				overTime = 1;
				target.direction = Pose2d.BASE.direction;
			case Shoot:
				aType = "shoot";
				break;
			case Shoot_Near:
				aType = "shoot_near";
				break;
			case Walk:
				overTime = 1;
				aType = "pickup";
				break;
			case Pickup:
				aType = "catch";
//				aType = "pickup";
				target.direction = Pose2d.BASE.direction;
				break;
			default:
				throw new RuntimeException();
			}
			
			double[] control = BasketPickupControl.getActionType(aType);
			double activation;
			double aMargin = BasketPickupControl.ACTION_MARGIN;
			aMargin = 45;
			if (overTime == 1){
				activation = 0;
			} else if (isBallChanged){
				activation = 1 - Math.max(0, (frame - actionFrame)/aMargin);
			} else {
//				activation = Math.min(1, (frame - actionStartFrame)/aMargin);
				activation = Math.min(1, (frame - actionStartFrame + 15)/aMargin);
			}
			control = MathUtil.concatenate(control, new double[]{ overTime, activation });
			control = MathUtil.concatenate(control, target.toArray());
			
			double[] x = control;
//			double[] x = ShootAndPickupControl.getControlData(currentType, target);
			x = normal.normalizeX(x);
			output = python.model.predict(x);
			output = normal.deNormalizeY(output);
			
			PositionResultMotion motion = g.update(output);
			double ball = output[output.length-1];
			boolean newBall = ball > 0.5;
			
			boolean useGenerated = false;
			if (MotionDataConverter.includeBall){
				if (currentType.isShoot() && hasBall && !newBall){
					// shoot
					shootAfterCount = 0;
					ArrayList<Point3d> shootTrajectory = bg.getShootTrajectory(ballPosition, GOAL_POS, 80);
					ArrayList<Point3d> fallTrajectory = bg.getFreeFallTrajectory(ballPosition, GOAL_POS);
					ballShootTrajectory = new ArrayList<Point3d>();
					ballShootTrajectory.addAll(shootTrajectory);
					ballShootTrajectory.addAll(fallTrajectory);
					hasBall = newBall;
					isBallChanged = true;
					actionFrame = frame;
				} else if (currentType == ActionType.Pickup && !hasBall && newBall){
//					pickupTarget = new Point3d(ballPosition);
					hasBall = newBall;
					isBallChanged = true;
					actionFrame = frame;
				}
				
				if (currentType.isShoot()){
					useGenerated = !isBallChanged;
				} else if (currentType == ActionType.Pickup){
					useGenerated = isBallChanged;
				} else if (currentType == ActionType.Dribble){
					useGenerated = true;
				} else {
					useGenerated = false;
				}
				
				if (useGenerated || ballShootTrajectory == null){
					double[] b = new double[3];
					System.arraycopy(output, 0, b, 0, 3);
					Point3d p = new Point3d(b[0], 0, b[2]);
					p = Pose2d.to3d(g.pose.localToGlobal(Pose2d.to2d(p)));
					p.y = b[1];
					ballPosition = p;
				} else {
					ballPosition = ballShootTrajectory.get(Math.min(shootAfterCount, ballShootTrajectory.size()-1));
					pickupTarget = new Point3d(ballPosition);
					shootAfterCount++;
				}
			}
			
			System.out.println("has ball :: " + totalBall.size() + " : " + currentType + " : " + hasBall + " :: " + ball + " : " + ballPosition + " : " + useGenerated + " : " + prevKey);
			
			ActionType nextType = null;
			if (currentKey != null && currentKey > 0){
				if (prevKey != currentKey){
					if (currentKey == (int)'3'){
						nextType = ActionType.Shoot;
						isBallChanged = false;
						actionStartFrame = frame;
					} else if (currentKey == (int)'4'){
						nextType = ActionType.Pickup;
						isBallChanged = false;
						actionStartFrame = frame;
					} else if (currentKey == (int)'5'){
						nextType = ActionType.Shoot_Near;
						isBallChanged = false;
						actionStartFrame = frame;
					}
				}
			} else {
				if (currentType.isShoot()){
					nextType = ActionType.Walk;
				} else if (currentType == ActionType.Pickup){
					nextType = ActionType.Dribble;
				}
			}
			prevKey = currentKey;
			if (nextType != null) currentType = nextType;
			return motion;
		}
		
		private void normalize(Tuple2d target, double maxLen){
			double len = MathUtil.length(target);
			if (len > maxLen){
				target.scale(maxLen/len);
			}
		}
	}

	public static void main(String[] args) {
		MainApplication.run(new GrammarPickUpModule());
		OgreJNI.close();
	}
}
