package mrl.motion.neural.play;

import static mrl.motion.neural.basket.BallTrajectoryGenerator.GOAL_POS;

import java.util.ArrayList;
import java.util.LinkedList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.data.BasketGameControl;
import mrl.motion.neural.data.BasketTimeControl;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.ShootAndPickupControl.ActionType;
import mrl.motion.neural.run.GameAIPlayer;
import mrl.motion.neural.run.GameAIPlayer.ActionTarget;
import mrl.motion.neural.run.MultiPersonRNNPython;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.MathUtil;
import mrl.util.Pair;
import mrl.util.Utils;
import mrl.widget.app.ItemListModule;
import static mrl.motion.neural.play.PickupController.*;

public class GameController extends RuntimeController{
	
	private MultiPersonRNNPython python;
	private Normalizer normal;
	private BallTrajectoryGenerator g;
	
	
	private int prevKey = -1;
	
	GameAIPlayer[] players;
	int ballPlayer;
	Point3d lastBallPosition = null;
	Point3d ballPosition = null;
	Point2d globalTarget = null;
	
	int btFrame = -1;
	int goalReachTime = -1;
	
	private ArrayList<Point3d> ballTrajectory;
	
	int processedFrame = -1;
	LinkedList<ArrayList<Motion>> motionCache = new LinkedList<ArrayList<Motion>>();
	LinkedList<Point3d> ballCache = new LinkedList<Point3d>();
	LinkedList<Point2d> targetCache = new LinkedList<Point2d>();
	boolean isShoot = false;
	boolean noBounce;

	Point3d catchBallPos;
	int catchFrame = -1;
	
	public GameController(String name) {
		super(name);
	}

	@Override
	public void init() {
		python = new MultiPersonRNNPython(name, 2);
		normal = python.normal;
		g = new BallTrajectoryGenerator();
		
		double[] initialY = normal.yList.get(100000 + 40);
		initialY = new double[initialY.length];
		python.model.setStartMotion(new double[][]{ initialY, initialY });
		
		
		double[][] basePoints = new double[][]{
//				{ -400, -300 },
//				{ 400, -300 },
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

	@Override
	public Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key) {
		GameAIPlayer pp = getPassivePlayer();
		GameAIPlayer bp = players[ballPlayer];
		if (bp.actionTarget != null && bp.actionTarget.type == ActionType.Catch){
			if (processedFrame < 0) processedFrame = frame - 1;
			
			for (int i = 0; i < 3; i++) {
				if (bp.actionTarget == null) break;
				processedFrame++;
				ArrayList<Motion> mData = updateImpl(mousePoint, processedFrame, key);
				motionCache.add(mData);
				targetCache.add(globalTarget);
				ballCache.add(ballPosition);
			}
			
			if (bp.actionTarget == null){
				catchFrame = processedFrame;
				catchBallPos = bp.g.ballPosition();
//				catchBallPos = ballPosition;
				System.out.println("catch ball frame :: " + catchFrame + " :: " + catchBallPos + " : " + ballPosition);
			}
		}
		if (bp.actionTarget != null && bp.actionTarget.type == ActionType.Pass_){
			if (processedFrame < 0) processedFrame = frame - 1;
			
			for (int i = 0; i < 3; i++) {
				if (bp.actionTarget == null) break;
				processedFrame++;
				ArrayList<Motion> mData = updateImpl(mousePoint, processedFrame, key);
				motionCache.add(mData);
				targetCache.add(globalTarget);
				ballCache.add(ballPosition);
			}
			
			if (bp.actionTarget == null){
				Point3d start = ballCache.getFirst();
				Point3d end = bp.g.ballPosition();
				ArrayList<Point3d> bt = g.getBounceTrajectory(start, end, frame, processedFrame);
				ballCache.clear();
				ballCache.addAll(bt);
//				ballCache.add(ballPosition);
				catchFrame = processedFrame;
			}
		}
		
		
		if (goalReachTime == frame){
			if (bp.actionTarget != null){
				System.out.println("Action :: " + bp.actionTarget.frame);
			}
			if (catchBallPos == null){
				System.out.println("ff :: " + frame + " ; " +  btFrame + " : " + processedFrame);
				catchFrame = processedFrame;
				catchBallPos = bp.g.ballPosition();
				goalReachTime++;
//				throw new RuntimeException();
			}
			
			Point3d start = ballCache.getFirst();
			Point3d end = catchBallPos;
			System.out.println("make air :: " + start + " , " + end);
			double maxHeight = GOAL_POS.y*1.4;
			ballTrajectory = g.getAirTrajectory(start, end, frame, catchFrame);
			if (!noBounce && maxHeight(ballTrajectory) > maxHeight){
				ballTrajectory = g.getOneBounceTrajectory(start, end, maxHeight, frame, catchFrame);
			}
			
			btFrame = frame;
			int idx = 0;
			System.out.println("ppp : " + Utils.toString(start, end, frame, catchFrame, processedFrame));
			for (Point3d p : ballCache){
				p.set(ballTrajectory.get(idx));
				idx++;
			}
		}
		
		
//		else if ((bp.actionTarget != null && frame <= bp.actionTarget.frame + 1) 
//				&& (pp.actionTarget == null || frame >= pp.actionTarget.frame)){
//			if (!ballFixed && ballTrajectory == null){
//				// last ball position
//				Point3d start = ballPosition;
//				Point3d end = Pose2d.to3d(bp.actionTarget.pose.position);
//				end.y = 80;
//				ballTrajectory = g.getBounceTrajectory(start, end, pp.actionTarget.frame, bp.actionTarget.frame);
//				ballCache.addAll(ballTrajectory);
//				ballFixed = false;
//				btFrame = frame;
//				processedFrame = frame - 1;
//			}
//			int lastFrame = bp.actionTarget.frame;
//			if (processedFrame <= lastFrame){
//				Point3d lastBall = ballPosition;
//				for (int i = 0; i < 2; i++) {
//					if (processedFrame == lastFrame + 1) break;
//					processedFrame++;
//					ArrayList<Motion> mData = updateImpl(mousePoint, processedFrame, key);
//					motionCache.add(mData);
//				}
//				
//				if (processedFrame == lastFrame + 1){
//					Point3d start = lastBall;
//					Point3d end = bp.g.ballPosition();
//					ArrayList<Point3d> bt = g.getBounceTrajectory(start, end, frame, lastFrame);
//					ballCache.clear();
//					ballCache.addAll(bt);
//					ballCache.add(ballPosition);
//					ballFixed = true;
//				}
//			}
//			if (ballCache.size() == 0){
//				System.out.println("non :: " + ballCache.size() + " : " + motionCache.size() +" : " + bp.actionTarget.frame + " : " + frame);
//			}
//			ballPosition = ballCache.removeFirst();
//			ArrayList<Motion> motions = motionCache.removeFirst();
//			return new Pair<Motion[], Point3d>(Utils.toArray(motions), ballPosition);
//		}
		
		
		if (frame == catchFrame){
			btFrame = -1;
			goalReachTime = -1;
			catchFrame = -1;
			catchBallPos = null;
			ballTrajectory = null;
		}
		
		if (motionCache.size() != ballCache.size()){
			System.out.println("awfawfaw :: " + motionCache.size() + " / " + ballCache.size());
			
		}
		if (motionCache.size() > 0){
			ballPosition = ballCache.removeFirst();
			globalTarget = targetCache.removeFirst();
			return new Pair<Motion[], Point3d>(Utils.toArray(motionCache.removeFirst()), ballPosition);
		}
		
		processedFrame = -1;
		ArrayList<Motion> motions = updateImpl(mousePoint, frame, key);
		return new Pair<Motion[], Point3d>(Utils.toArray(motions), ballPosition);
	}
	
	public Point2d getTarget(){
		return globalTarget;
	}
	
	public ArrayList<Motion> updateImpl(Point2d targetP, int frame, int key){
		if (key >= 0){
			if (prevKey != key){
				GameAIPlayer bp = players[ballPlayer];
				GameAIPlayer pp = getPassivePlayer();
				
				int actionOffset = BasketGameControl.ACTION_MARGIN;
//				actionOffset = 45;
				if (key == (int)'3'){
					// pass
					Point2d passPoint = targetP;
					Point2d receivePoint = pp.getReceivePoint(passPoint);
					
					Pose2d passPose = new Pose2d(passPoint, MathUtil.sub(receivePoint, passPoint));
					Pose2d receivePose = new Pose2d(receivePoint, MathUtil.sub(passPoint, receivePoint));
					int passTime = frame + actionOffset;
					int receiveTime = passTime + 20;
					
					bp.setActionTarget(new ActionTarget(ActionType.Pass, passTime, passPose));
					pp.setActionTarget(new ActionTarget(ActionType.Pass_, receiveTime, receivePose));
					
//					ballPlayer = (ballPlayer + 1)%players.length;
//					getModule(ItemListModule.class).addSingleItem("receivePose", receivePose);
				} else if (key == (int)'4'){
					// shoot
					int shootFrame = frame + actionOffset;
					Point2d goalPos = Pose2d.to2d(BallTrajectoryGenerator.GOAL_POS);
					Pose2d shootPose = new Pose2d(targetP, MathUtil.sub(goalPos, targetP));
					ActionTarget shoot = new ActionTarget(ActionType.Shoot, shootFrame, shootPose);
					bp.setActionTarget(shoot);
					isShoot = true;
				} else if (key == (int)'5'){
					// shoot
					int shootFrame = frame + actionOffset;
					Point2d goalPos = Pose2d.to2d(BallTrajectoryGenerator.GOAL_POS);
					Pose2d shootPose = new Pose2d(targetP, MathUtil.sub(goalPos, targetP));
					ActionTarget shoot = new ActionTarget(ActionType.Shoot_Near, shootFrame, shootPose);
					bp.setActionTarget(shoot);
					isShoot = true;
				}
			}
		}
		prevKey = key;
		
		
		GameAIPlayer bp = players[ballPlayer];
		GameAIPlayer pp = getPassivePlayer();
		boolean isActioned = bp.actionTarget != null;
		
		double[][] inputList = new double[players.length][];
		for (int i = 0; i < inputList.length; i++) {
			if (i == ballPlayer){
				inputList[i] = players[i].getActiveInput(targetP, frame);
			} else {
				inputList[i] = players[i].getPassiveInput(Pose2d.to2d(BallTrajectoryGenerator.GOAL_POS), frame);
//				inputList[i] = players[i].getPassiveInput(players[ballPlayer].currentPosition(), frame);
			}
			inputList[i] = normal.normalizeX(inputList[i]);
		}
//		long t = System.nanoTime();
		double[][] outputList = python.model.predict(inputList);
//		System.out.println("pt :: " + (System.nanoTime() - t)/1000);
		ArrayList<Motion> motionList = new ArrayList<Motion>();
		for (int i = 0; i < outputList.length; i++) {
			Motion motion = players[i].update(normal.deNormalizeY(outputList[i]), frame);
			motionList.add(motion);
		}
		
		
//		System.out.println("Actionos : " + frame + " : " + bp.actionTarget + " , " + bp.currentType + " :: " + pp.actionTarget + " , " + pp.currentType);
		
		if (bp.actionTarget != null && bp.actionTarget.type.isShoot() && bp.actionTarget.isFinished){
			double gLen = Pose2d.to2d(MathUtil.sub(lastBallPosition, GOAL_POS)).length();
//			double gLen = Pose2d.to2d(MathUtil.sub(ballPosition, GOAL_POS)).length();
			double hOffset = 40;
			hOffset += 40*Math.min(gLen, 300)/300d;
			
			ActionTarget actionTarget = bp.actionTarget;
			
			noBounce = actionTarget.type == ActionType.Shoot_Near;
			if (noBounce) hOffset = 40;
			ArrayList<Point3d> shootTrajectory = g.getShootTrajectory(lastBallPosition, GOAL_POS, hOffset);
			ballTrajectory = new ArrayList<Point3d>();
			ballTrajectory.addAll(shootTrajectory);
			btFrame = frame;
			
			goalReachTime = frame + shootTrajectory.size();
			
			Pair<Point2d, Integer> pair = null;
			Point2d cPoint = null;
			
			while (true){
				pair = pickCatch(actionTarget.pose.position, bp.g.pose, shootTrajectory.size());
				cPoint = pair.first;
				bp.actionTarget = null;
				double d0 = players[0].g.pose.position.distance(cPoint);
				double d1 = players[1].g.pose.position.distance(cPoint);
				if (d0 < d1){
					ballPlayer = 0;
				} else {
					ballPlayer = 1;
				}
				Point2d position = players[ballPlayer].g.pose.position;
				Point2d gp = Pose2d.to2d(GOAL_POS);
				if (cPoint.distance(gp) > position.distance(gp)*0.9 ) continue;
				break;
			}
			
			
			bp = players[ballPlayer];
			
			
			int eTime = pair.second + 5;
			Point3d cpi = Pose2d.to3d(cPoint);
			cpi.x *= -1;
			ArrayList<Point3d> fallTrajectory = g.getFreeFallTrajectory(cpi, GOAL_POS);
			ballTrajectory.addAll(fallTrajectory);
			Pose2d catP = new Pose2d(cPoint, Pose2d.BASE.direction);
			if (app != null){
				app.getModule(ItemListModule.class).addSingleItem("catch", catP);
			}
			System.out.println("estimation time :: " + eTime);
			bp.actionTarget = new ActionTarget(ActionType.Catch, frame + eTime,  catP);
		}
		if (bp.actionTarget != null && bp.actionTarget.type == ActionType.Pass && bp.actionTarget.isFinished){
			
			ballPlayer = (ballPlayer + 1)%players.length;
			bp = players[ballPlayer];
			pp = getPassivePlayer();
			
			// last ball position
			Point3d start = ballPosition;
			Point3d end = Pose2d.to3d(bp.actionTarget.pose.position);
			end.y = 80;
			ballTrajectory = g.getBounceTrajectory(start, end, pp.actionTarget.frame, bp.actionTarget.frame);
			ballTrajectory.addAll(ballTrajectory);
			btFrame = frame;
			pp.actionTarget = null;
		}
		
		if (btFrame >= 0 && frame - btFrame >= 0){
			ballPosition = ballTrajectory.get(Math.min(frame - btFrame, ballTrajectory.size()-1));
		} else {
			ballPosition = bp.g.ballPosition();
			if (isActioned && bp.actionTarget == null){
				System.out.println("bppp :: " + frame + " : " + bp.g.ballPosition());
			}
		}
//		System.out.println("## : " + frame + " : " + ballPosition);
		
		if (ballPosition != null){
			lastBallPosition = ballPosition;
		}
		
		if (bp.actionTarget != null){
			globalTarget = bp.actionTarget.pose.position;
		} else {
			globalTarget = targetP;
		}
		
//		GameAIPlayer pp = getPassivePlayer();
//		GameAIPlayer bp = players[ballPlayer];
//		if (bp.actionTarget != null && frame < bp.actionTarget.frame && isShoot){
//			ballPosition = bp.g.ballPosition();
//		} else if (bp.actionTarget == null || frame > bp.actionTarget.frame){
//			if (isShoot){
//				ballPosition = ballTrajectory.get(Math.min(frame - btFrame, ballTrajectory.size()-1));
//			} else {
//				ballPosition = bp.g.ballPosition();
//				ballTrajectory = null;
//				ballFixed = false;
//			}
//		} else if (pp.actionTarget != null && frame < pp.actionTarget.frame){
//			ballPosition = pp.g.ballPosition();
//		} else {
//		}
		return motionList;
	}
	
	private GameAIPlayer getPassivePlayer(){
		return players[(ballPlayer + 1)%players.length];
	}

	private Pair<Point2d, Integer> pickCatch(Point2d ball, Pose2d current, int shootTime){
		while (true){
			Point2d goal = Pose2d.to2d(GOAL_POS);
			Point2d p = PickupController.getCatchPoint(ball);
			
			double len1 = ball.distance(goal);
			double len2 = ball.distance(p);
			double vel1 = len1/shootTime;
			vel1 = Math.min(vel1, 500/30d);
			vel1 = Math.max(vel1, 50/30d);
			double vel2 = vel1*0.9;
			
			double totalTime = len1/vel1 + len2/vel2;
			
			double len3 = current.position.distance(p);
//			double cTime = len3/PickupController.characterVel;
//			
//			System.out.println("ttt :: " + Utils.toString(cTime - totalTime, cTime, totalTime));
//			if (totalTime < cTime) continue;
//			if (cTime > 70) continue;
			if (len2/vel2 > 50) continue;
			if (!noBounce && len3 < 100) continue;
			
			if (noBounce){
				totalTime = len1/vel1 + 20;
			}
			return new Pair<Point2d, Integer>(p, (int)totalTime);
		}
	}
	
	public static void main(String[] args) {
		BasketGraph.INCLUDE_CATCH = true;
		MotionDataConverter.setAllJoints();
		GameController c;
//			MotionDataConverter.setNoBallVelocity();
//			c = new DribbleController("dribble_new_nv");
//		GameAIPlayer.USE_INTERVAL_BAR = true;
//		c = new GameController("game_vel");
		
		BasketGameControl.useDoubleAction = true;
		MotionDataConverter.setUseOrientation();
//		c = new GameController("game_ori");
//		c = new GameController("game_da_tf2");
		c = new GameController("game_da_ori2");
		
		
//		GameAIPlayer.USE_ACTION_DIVISION = true;
//		c = new GameController("game_ad_ori_pr");
//		c = new GameController("game_ad");
		
//		c = new GameController("game_lc");
//		c = new GameController("game_pr");
//		new OgreRecorder(c).run(OgreJNI.courtParam());
		OpenGLRecorder.run(c);
	}
}
