package mrl.motion.neural.run;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;

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
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.data.BasketGameControl;
import mrl.motion.neural.data.BasketPickupControl;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.data.ShootAndPickupControl;
import mrl.motion.neural.data.ShootAndPickupControl.ActionType;
import mrl.motion.neural.run.GameAIPlayer.ActionTarget;
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

import static mrl.motion.neural.basket.BallTrajectoryGenerator.GOAL_POS;

public class GameTestModule extends Module{

	private MultiPersonRNNPython python;
	private Normalizer normal;
	private BallTrajectoryGenerator g;
	private Normalizer jointNormal;
	
	private PositionResultMotion totalMotion = new PositionResultMotion();
	private ArrayList<Motion> totalMotion2 = new ArrayList<Motion>();
	private TimeBasedList<Point3d> totalBall = new TimeBasedList<Point3d>();
	private Integer currentKey = null;
	private Integer prevKey = null;
	private Controller controller = new Controller();
	
	@Override
	protected void initializeImpl() {
		MotionDataConverter.setAllJoints();
//		OgreJNI.open(new double[]{ 1 });
		
//		MotionDataConverter.setNoBall();
//		String folder = "shoot5";
//		
		BasketGraph.INCLUDE_CATCH = true;
		String folder = "game_lc";
//		String folder = "game3";
		python = new MultiPersonRNNPython(folder, 2);
		normal = python.normal;
		
		g = new BallTrajectoryGenerator();
		
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
	
	public void start(){
		double[] initialY = normal.yList.get(100000 + 40);
		initialY = new double[initialY.length];
		python.model.setStartMotion(new double[][]{ initialY, initialY });
		
		MainViewerModule mainViewer = getModule(MainViewerModule.class);
		long startTime = System.currentTimeMillis();
		
		MotionTransform t = new MotionTransform();
		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
		
		TimeChecker.instance.enable = true;
		dummyParent().getDisplay().timerExec(1, new Runnable() {
			int frame = -1;
			long  lastTime = System.currentTimeMillis();
			@Override
			public void run() {
				if (dummyParent().isDisposed()) return;
				if (isStop) return;
				
//				long tt = System.currentTimeMillis() - lastTime;
//				lastTime = System.currentTimeMillis();
//				System.out.println(tt);
				
				ArrayList<MotionData> mData = null;
				while (true){
					int dt = (int)(System.currentTimeMillis() - startTime);
					int tIndex = dt/33;
					if (frame > tIndex) break;
					if (frame < 0) frame = tIndex;
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
					long t = System.nanoTime();
					mData = controller.update(target3d, frame);
					System.out.println("dt :: " + (System.nanoTime() - t)/1000);
					getModule(ItemListModule.class).addSingleItem("Target", target3d, new ItemDescription(new Vector3d(0, 1, 0)));
					getModule(ItemListModule.class).addSingleItem("Motion2", mData);
					
					if (controller.ballPosition != null){
						getModule(ItemListModule.class).addSingleItem("ball", controller.ballPosition, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS));
					}
					totalBall.add(controller.ballPosition);
				}
				if (mData != null){
					OgreJNI.setMotion(Utils.toArray(mData));
					if (controller.ballPosition != null){
						OgreJNI.setBall(Utils.singleList(controller.ballPosition));
					}
				}
				dummyParent().getDisplay().timerExec(1, this);
			}
		});
	}
	
	private class Controller{
		
		GameAIPlayer[] players;
		int ballPlayer;
		Point3d ballPosition = null;
		
		int btFrame = -1;
		ArrayList<Point3d> ballTrajectory = null;
		
		int processedFrame = -1;
		LinkedList<ArrayList<MotionData>> motionCache = new LinkedList<ArrayList<MotionData>>();
		LinkedList<Point3d> ballCache = new LinkedList<Point3d>();
		boolean ballFixed = false;
		
		Controller(){
			double[][] basePoints = new double[][]{
					{ -300, -200 },
					{ 300, -200 },
			};
			ballPlayer = 0;
			players = new GameAIPlayer[basePoints.length];
			for (int i = 0; i < basePoints.length; i++) {
				players[i] = new GameAIPlayer(new Point2d(basePoints[i][0], basePoints[i][1]), 150);
				if (i == ballPlayer){
					players[i].currentType = ActionType.Dribble;
				}
			}
		}
		
		private GameAIPlayer getPassivePlayer(){
			return players[(ballPlayer + 1)%players.length];
		}
		
		public ArrayList<MotionData> update(Point3d target3d, int frame){
			GameAIPlayer pp = getPassivePlayer();
			GameAIPlayer bp = players[ballPlayer];
			if ((bp.actionTarget != null && frame <= bp.actionTarget.frame + 1) 
					&& (pp.actionTarget == null || frame >= pp.actionTarget.frame)){
				if (!ballFixed && ballTrajectory == null){
					// last ball position
					Point3d start = ballPosition;
					Point3d end = Pose2d.to3d(bp.actionTarget.pose.position);
					end.y = 80;
					ballTrajectory = g.getBounceTrajectory(start, end, pp.actionTarget.frame, bp.actionTarget.frame);
					ballCache.addAll(ballTrajectory);
					ballFixed = false;
					btFrame = frame;
					processedFrame = frame - 1;
				}
				int lastFrame = bp.actionTarget.frame;
				if (processedFrame <= lastFrame){
					Point3d lastBall = ballPosition;
					for (int i = 0; i < 2; i++) {
						if (processedFrame == lastFrame + 1) break;
						processedFrame++;
						ArrayList<MotionData> mData = updateImpl(target3d, processedFrame);
						motionCache.add(mData);
					}
					
					if (processedFrame == lastFrame + 1){
						Point3d start = lastBall;
						Point3d end = bp.g.ballPosition();
						ArrayList<Point3d> bt = g.getBounceTrajectory(start, end, frame, lastFrame);
						ballCache.clear();
						ballCache.addAll(bt);
						ballCache.add(ballPosition);
						ballFixed = true;
					}
				}
				if (ballCache.size() == 0){
					System.out.println("non :: " + ballCache.size() + " : " + motionCache.size() +" : " + bp.actionTarget.frame + " : " + frame);
				}
				ballPosition = ballCache.removeFirst();
				return motionCache.removeFirst();
			}
			if (ballCache.size() > 0 || motionCache.size() > 0){
				System.out.println("awfawfa :: " + ballCache.size() + " : " + motionCache.size() +" : " + bp.actionTarget.frame + " : " + frame);
			}
			return updateImpl(target3d, frame);
		}
		
		public ArrayList<MotionData> updateImpl(Point3d target3d, int frame){
			Point2d targetP = Pose2d.to2d(target3d);
			
			if (currentKey != null){
				if (prevKey != currentKey){
					GameAIPlayer bp = players[ballPlayer];
					GameAIPlayer pp = getPassivePlayer();
					
					
					if (currentKey == (int)'3'){
						// pass
						Point2d passPoint = targetP;
						Point2d receivePoint = pp.getReceivePoint(passPoint);
						
						Pose2d passPose = new Pose2d(passPoint, MathUtil.sub(receivePoint, passPoint));
						Pose2d receivePose = new Pose2d(receivePoint, MathUtil.sub(passPoint, receivePoint));
						int passTime = frame + 45;
						int receiveTime = passTime + 20;
						
						bp.setActionTarget(new ActionTarget(ActionType.Pass, passTime, passPose));
						pp.setActionTarget(new ActionTarget(ActionType.Pass_, receiveTime, receivePose));
						
						ballPlayer = (ballPlayer + 1)%players.length;
						getModule(ItemListModule.class).addSingleItem("receivePose", receivePose);
					} else if (currentKey == (int)'4'){
						// shoot
					}
				}
			}
			prevKey = currentKey;
			
			
			
			double[][] inputList = new double[players.length][];
			for (int i = 0; i < inputList.length; i++) {
				if (i == ballPlayer){
					inputList[i] = players[i].getActiveInput(targetP, frame);
				} else {
					inputList[i] = players[i].getPassiveInput(players[ballPlayer].currentPosition(), frame);
				}
				System.out.println("xx len :: " + inputList[i].length + " // " + normal.xMeanAndStd[0].length);
				inputList[i] = normal.normalizeX(inputList[i]);
//				System.out.println("xx : " + i + " : " + Arrays.toString(inputList[i]));
			}
			long t = System.nanoTime();
			double[][] outputList = python.model.predict(inputList);
			System.out.println("pt :: " + (System.nanoTime() - t)/1000);
			ArrayList<MotionData> motionList = new ArrayList<MotionData>();
			for (int i = 0; i < outputList.length; i++) {
//				System.out.println("yy : " + i + " : " + Arrays.toString(normal.deNormalizeY(outputList[i])));
				Motion motion = players[i].update(normal.deNormalizeY(outputList[i]), frame);
				motionList.add(new MotionData(Utils.singleList(motion)));
			}
			
			GameAIPlayer pp = getPassivePlayer();
			GameAIPlayer bp = players[ballPlayer];
			if (bp.actionTarget == null || frame > bp.actionTarget.frame){
				ballPosition = bp.g.ballPosition();
				ballTrajectory = null;
				ballFixed = false;
			} else if (pp.actionTarget != null && frame < pp.actionTarget.frame){
				ballPosition = pp.g.ballPosition();
			} else {
//				if (ballTrajectory == null){
//					// last ball position
//					Point3d start = ballPosition;
//					Point3d end = Pose2d.to3d(bp.actionTarget.pose.position);
//					end.y = 80;
//					ballTrajectory = g.getBounceTrajectory(start, end, pp.actionTarget.frame, bp.actionTarget.frame);
//					btFrame = frame;
//				}
//				ballPosition = ballTrajectory.get(frame - btFrame);
			}
			return motionList;
		}
	}

	public static void main(String[] args) {
		MainApplication.run(new GameTestModule());
		OgreJNI.close();
	}
}
