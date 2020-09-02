package mrl.motion.neural.play;

import static mrl.motion.neural.basket.BallTrajectoryGenerator.GOAL_POS;

import java.util.ArrayList;
import java.util.LinkedList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

import mrl.motion.data.Motion;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.data.BasketPickupControl;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.ShootAndPickupControl.ActionType;
import mrl.motion.neural.run.RNNPython;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.MathUtil;
import mrl.util.Pair;
import mrl.util.Utils;
import mrl.widget.app.ItemListModule;

public class PickupController extends RuntimeController{
	
	private RNNPython python;
	private Normalizer normal;
	private RuntimeMotionGenerator g;
	
	
	private ActionType currentType = ActionType.Dribble;
	private BallTrajectoryGenerator bg = new BallTrajectoryGenerator();
	private ArrayList<Point3d> ballShootTrajectory;
	private int prevKey = -1;
	private Point3d ballPosition = new Point3d();
	private Point2d globalTarget = null;
	
	private int btStartFrame = -1;
	private int goalReachTime = -1;

	private ActionTarget actionTarget;
//	private ActionTarget nextTarget;
	
	public static double characterVel = 300/30d;
	
	LinkedList<Motion> motionCache = new LinkedList<Motion>();
	LinkedList<Point3d> ballCache = new LinkedList<Point3d>();
	LinkedList<Point2d> targetCache = new LinkedList<Point2d>();
	int processedFrame = -1;
	
	Point3d catchBallPos;
	int catchFrame = -1;
	
	boolean noBounce = false;
	
	public PickupController(String name) {
		super(name);
	}


	@Override
	public void init() {
		python = new RNNPython(name, false);
		normal = new Normalizer(name);
		double[] initialY = normal.yList.get(3);
		initialY = new double[initialY.length];
		python.model.setStartMotion(initialY);
		g = new RuntimeMotionGenerator();
	}
	
	private void normalize(Tuple2d target, double maxLen){
		double len = MathUtil.length(target);
		if (len > maxLen){
			target.scale(maxLen/len);
		}
	}
	
	public static double maxHeight(ArrayList<Point3d> points){
		double max = Integer.MIN_VALUE;
		for (Point3d p : points){
			max = Math.max(max, p.y);
		}
		return max;
	}

	@Override
	public Pair<Motion[], Point3d> process(int frame, Point2d mousePoint, int key) {
		
		if (actionTarget != null && actionTarget.type == ActionType.Catch){
			if (processedFrame < 0) processedFrame = frame - 1;
			for (int i = 0; i < 4; i++) {
				if (actionTarget == null) break;
				processedFrame++;
				Motion motion = updateImpl(processedFrame, mousePoint, key);
				motionCache.add(motion);
				ballCache.add(ballPosition);
				targetCache.add(globalTarget);
			}
			
			if (actionTarget == null){
				catchFrame = processedFrame;
				catchBallPos = ballPosition;
				System.out.println("catch ball frame :: " + catchFrame);
			}
		}
		if (goalReachTime == frame){
			if (actionTarget != null){
				System.out.println("Action :: " + actionTarget.frame);
			}
			if (catchBallPos == null){
				System.out.println("ff :: " + frame + " ; " +  btStartFrame + " : " + processedFrame);
				throw new RuntimeException();
			}
			
			Point3d start = ballCache.getFirst();
			Point3d end = catchBallPos;
			
			double maxHeight = GOAL_POS.y*1.4;
			ballShootTrajectory = bg.getAirTrajectory(start, end, frame, catchFrame);
			if (!noBounce && maxHeight(ballShootTrajectory) > maxHeight){
				ballShootTrajectory = bg.getOneBounceTrajectory(start, end, maxHeight, frame, catchFrame);
			}
			
			btStartFrame = frame;
			int idx = 0;
			System.out.println("ppp : " + Utils.toString(start, end, frame, catchFrame, processedFrame));
			for (Point3d p : ballCache){
				p.set(ballShootTrajectory.get(idx));
				idx++;
			}
		}
		
		if (frame == catchFrame){
			btStartFrame = -1;
			goalReachTime = -1;
			catchFrame = -1;
			catchBallPos = null;
			ballShootTrajectory = null;
		}
		
		if (motionCache.size() > 0){
			ballPosition = ballCache.removeFirst();
			globalTarget = targetCache.removeFirst();
			return new Pair<Motion[], Point3d>(new Motion[]{ motionCache.removeFirst() }, ballPosition);
		}
		
		processedFrame = -1;
		Motion m = updateImpl(frame, mousePoint, key);
		return new Pair<Motion[], Point3d>(new Motion[]{ m }, ballPosition);
	}
	
	
	
	public Motion updateImpl(int frame, Point2d mousePoint, int key) {
		int currentKey = key;
		
		Pose2d target = new Pose2d(mousePoint, Pose2d.BASE.direction);
		if (actionTarget != null){
			target = actionTarget.pose;
			currentType = actionTarget.type;
		}
		
		globalTarget = new Point2d(target.position);
		
		
		target = Pose2d.relativePose(g.pose, target);
		normalize(target.position, 250);
		
		String aType = currentType.actionName();
		double overTime = 0;
		switch (currentType){
		case Dribble:
			overTime = 1;
			target.direction = Pose2d.BASE.direction;
			aType = "shoot";
			break;
		case Walk:
			overTime = 1;
			aType = "pickup";
			break;
		case Pickup:
		case Catch:
			target.direction = Pose2d.BASE.direction;
			break;
		default:
			break;
		}
		
		
		double activation;
		double aMargin = BasketPickupControl.ACTION_MARGIN;
		if (overTime == 1){
			activation = 0;
		} else {
			int remainTime = actionTarget.frame - frame;
			if (remainTime > aMargin){
				overTime = 1;
			}
			activation = 1 - Math.max(0, Math.abs(remainTime/aMargin));
		}
		if (overTime == 1 && actionTarget != null && actionTarget.type == ActionType.Catch){
			aType = "pickup";
		}
		
		double[] control = BasketPickupControl.getActionType(aType);
		control = MathUtil.concatenate(control, new double[]{ overTime, activation });
		control = MathUtil.concatenate(control, target.toArray());
		double[] x = control;
//		System.out.println(frame + " : " + actionTarget + " : " + Arrays.toString(x));
		x = normal.normalizeX(x);
		double[] output = python.model.predict(x);
		output = normal.deNormalizeY(output);
		double hasBall = output[output.length - 1];
//		System.out.println("hasBall :: " + hasBall);
		g.update(output);
		
		if (actionTarget != null && frame == actionTarget.frame){
			if (actionTarget.type.isShoot()){
				if (hasBall > 0.5){
					actionTarget.frame++;
				} else {
					double gLen = Pose2d.to2d(MathUtil.sub(ballPosition, GOAL_POS)).length();
					double hOffset = 40;
					hOffset += 40*Math.min(gLen, 300)/300d;
					
					noBounce = actionTarget.type == ActionType.Shoot_Near;
					if (noBounce) hOffset = 40;
					ArrayList<Point3d> shootTrajectory = bg.getShootTrajectory(ballPosition, GOAL_POS, hOffset);
					ballShootTrajectory = new ArrayList<Point3d>();
					ballShootTrajectory.addAll(shootTrajectory);
					btStartFrame = frame;
					
					goalReachTime = frame + shootTrajectory.size();
					
					Pair<Point2d, Integer> pair = pickCatch(actionTarget.pose.position, g.pose, shootTrajectory.size());
					Point2d cp = pair.first;
//					Point2d cp = getCatchPoint(actionTarget.pose.position);
					
					int eTime = pair.second + 5;
					Point3d cpi = Pose2d.to3d(cp);
					cpi.x *= -1;
					ArrayList<Point3d> fallTrajectory = bg.getFreeFallTrajectory(cpi, GOAL_POS);
					ballShootTrajectory.addAll(fallTrajectory);
					Pose2d catP = new Pose2d(cp, Pose2d.BASE.direction);
					if (app != null){
						app.getModule(ItemListModule.class).addSingleItem("catch", catP);
					}
					System.out.println("estimation time :: " + eTime);
					actionTarget = new ActionTarget(ActionType.Catch, frame + eTime,  catP);
				}
			} else if (actionTarget.type == ActionType.Catch){
				if (hasBall < 0.5 || g.ballPosition() == null){
					actionTarget.frame++;
				} else {
					currentType = ActionType.Dribble;
					btStartFrame = -1;
					actionTarget = null;
				}
			}
		} else if (actionTarget != null && frame >= actionTarget.frame + 15){
			
		}
		
		if (btStartFrame >= 0 && frame - btStartFrame >= 0){
			ballPosition = ballShootTrajectory.get(Math.min(frame - btStartFrame, ballShootTrajectory.size()-1));
		} else {
			ballPosition = g.ballPosition();
		}
		
		if (currentKey >= 0){
			if (prevKey != currentKey){
				if (currentKey == (int)'3'){
					Pose2d actionPose = new Pose2d(mousePoint,  MathUtil.sub(Pose2d.to2d(GOAL_POS), mousePoint));
					actionTarget = new ActionTarget(ActionType.Shoot, frame + 40, actionPose);
				} else if (currentKey == (int)'4'){
					Pose2d actionPose = new Pose2d(mousePoint,  MathUtil.sub(Pose2d.to2d(GOAL_POS), mousePoint));
					actionTarget = new ActionTarget(ActionType.Shoot_Near, frame + 40, actionPose);
					
//					nextType = ActionType.Pickup;
//					isBallChanged = false;
//					actionStartFrame = frame;
				}
			}
		}
		prevKey = currentKey;
		
		return g.motion();
	}
	
	private Pair<Point2d, Integer> pickCatch(Point2d ball, Pose2d current, int shootTime){
		while (true){
			Point2d goal = Pose2d.to2d(GOAL_POS);
			Point2d p = getCatchPoint(ball);
			
			double len1 = ball.distance(goal);
			double len2 = ball.distance(p);
			double vel1 = len1/shootTime;
			vel1 = Math.min(vel1, 500/30d);
			vel1 = Math.max(vel1, 100/30d);
			double vel2 = vel1*0.9;
			
			double totalTime = len1/vel1 + len2/vel2;
			
			double len3 = current.position.distance(p);
			double cTime = len3/characterVel;
			
			System.out.println("ttt :: " + Utils.toString(cTime - totalTime, cTime, totalTime));
			if (totalTime < cTime) continue;
			if (cTime > 70) continue;
			if (len2/vel2 > 50) continue;
			if (!noBounce && len3 < 100) continue;
			
			if (noBounce){
				totalTime = len1/vel1 + 20;
			}
			return new Pair<Point2d, Integer>(p, (int)totalTime);
		}
	}
	
	public static Point2d getCatchPoint(Point2d base){
		Vector2d v = MathUtil.sub(base, Pose2d.to2d(GOAL_POS));
		double len = v.length()*0.1;
		if (MathUtil.random.nextDouble() > 0.3){
			System.out.println("Reversed");
			v.x = -v.x;
		}
		double sign = MathUtil.random.nextBoolean() ? 1 : -1;
		MathUtil.rotate(v, sign*Math.toRadians((30 + MathUtil.random.nextDouble()*25)));
		v.scale(0.65 + Utils.rand1()*0.1);
		
		
		Point2d p = new Point2d();
		p.add(Pose2d.to2d(GOAL_POS), v);
		p.x += Utils.rand1()*len;
		p.y += Utils.rand1()*len;
		return p;
	}
	
	public Point2d getTarget(){
		return globalTarget;
	}
	
	private static class ActionTarget{
		public ActionType type;
		public int frame;
		public Pose2d pose;
		public ActionTarget(ActionType type, int frame, Pose2d pose) {
			this.type = type;
			this.frame = frame;
			this.pose = pose;
		}
		
		public String toString(){
			return Utils.toString("ActionTarget", type, frame, pose);
		}
	}
	
	
	public static void main(String[] args) {
		PickupController c;
//		MotionDataConverter.setNoBallVelocity();
//		c = new DribbleController("dribble_new_nv");
		BasketPickupControl.ACTION_MARGIN = 30;
		BasketGraph.INCLUDE_CATCH = true;
		c = new PickupController("shoot_o_lc");
		new OgreRecorder(c).run(OgreJNI.courtParam());
//		OpenGLRecorder.run(c);
	}
}
