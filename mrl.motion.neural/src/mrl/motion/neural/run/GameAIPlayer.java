package mrl.motion.neural.run;

import java.util.HashMap;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

import mrl.motion.data.Motion;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.BasketGameControl;
import mrl.motion.neural.data.BasketGameControl2;
import mrl.motion.neural.data.BasketGameControl3;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.SpatioTemporalBar;
import mrl.motion.neural.data.ShootAndPickupControl.ActionType;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class GameAIPlayer {
	
	public static boolean USE_INTERVAL_BAR = false;
	public static boolean USE_ACTION_DIVISION = false;

	private Point2d base;
	private double radius;
	
	private Point2d targetPoint;
	private Vector2d targetMoveVel;
	
	public RuntimeMotionGenerator g;
	
	public ActionType currentType = ActionType.Walk;
	public int targetTime = -1;
	public ActionTarget actionTarget = null;
	
	public GameAIPlayer(Point2d base, double radius) {
		this.base = base;
		this.radius = radius;
		
		g = new RuntimeMotionGenerator();
		g.pose = new Pose2d(base, MathUtil.sub(Pose2d.to2d(BallTrajectoryGenerator.GOAL_POS), base));
		
		targetPoint = new Point2d(base);
		targetMoveVel = new Vector2d(30, -5);
	}
	
	public Point2d currentPosition(){
		return g.pose.position;
	}
	
	public void setActionTarget(ActionTarget actionTarget){
		this.actionTarget = actionTarget;
	}
	
	public double[] getActiveInput(Point2d targetPoint, int frame){
		Pose2d target = new Pose2d(targetPoint, Pose2d.BASE.direction);
		return getInputData(target, frame);
	}
	public double[] getPassiveInput(Point2d ballPos, int frame){
		updateTargetPoint();
		Pose2d target = new Pose2d(targetPoint, MathUtil.sub(ballPos, targetPoint));
		return getInputData(target, frame);
	}
	
	private void updateTargetPoint(){
		Vector2d randV = new Vector2d(Utils.rand1(), Utils.rand1());
		randV.scale(4);
		Vector2d error = MathUtil.sub(base, targetPoint);
		if (error.length() > 0.00001){
			error.normalize();
		}
		error.scale(0.1);
		
		Vector2d change = new Vector2d();
		change.add(randV, error);
		
		targetMoveVel.add(change);
		normalize(targetMoveVel, 30);
		Point2d moved = new Point2d();
		moved.add(targetPoint, targetMoveVel);
		if (moved.distance(base) > radius){
			targetMoveVel.set(change);
			moved.add(targetPoint, targetMoveVel);
		}
		targetPoint.set(moved);
		
		if (targetPoint.distance(currentPosition()) > 130){
			Vector2d diff = MathUtil.sub(targetPoint, currentPosition());
			normalize(diff, 130);
			targetPoint.add(currentPosition(), diff);
		}
	}
	
	int actionContinueCount = 0;
	public Motion update(double[] output, int frame){
		Motion motion;
		if (MotionDataConverter.useOrientation){
			motion = g.updateByOri(output);
			HashMap<String, Point3d> map = MotionDataConverter.dataToPointMapByPosition(output);
			motion = RuntimeMotionGenerator.ikSolver.solveFoot(motion, map);
		} else {
			g.update(output);
			motion = Utils.last(g.motionList);
		}
		
		if (actionTarget != null && frame == actionTarget.frame){
			double hasBall = output[output.length - 1];
			ActionType type = actionTarget.type;
			if (type.isShoot()){
				if (hasBall > 0.5 && g.isBallInHand() && actionContinueCount < 10){
					actionTarget.frame++;
					actionContinueCount++;
				} else {
					actionTarget.isFinished = true;
					currentType = ActionType.Walk; 
					actionContinueCount = 0;
				}
			} else if (type == ActionType.Catch || type == ActionType.Pass_){
				if (actionContinueCount < 10 && (hasBall < 0.5 || g.ballPosition() == null)){
					actionTarget.frame++;
					actionContinueCount++;
				} else {
					actionTarget = null;
					currentType = ActionType.Dribble;
					actionContinueCount = 0;
				}
			} else if (type == ActionType.Pass){
				actionTarget.isFinished = true;
				currentType = ActionType.Walk; 
			}
		}
		
		return motion;
	}
	
	
	public static void normalize(Tuple2d target, double maxLen){
		double len = MathUtil.length(target);
		if (len > maxLen){
			target.scale(maxLen/len);
		}
	}
	
	public Point2d getReceivePoint(Point2d passPos){
		Vector2d randV = new Vector2d(Utils.rand1(), Utils.rand1());
		randV.scale(100);
		Point2d p = new Point2d();
		p.add(currentPosition(), randV);
		return p;
	}
	
	private SpatioTemporalBar targetBar = new SpatioTemporalBar(4, -BasketGameControl2.POST_MARGIN, BasketGameControl2.ACTION_MARGIN+15, 400);
	
	private double[] getInputData(Pose2d target, int frame){
		double overTime = 0;
		int remainTime = -1000000;
		ActionType type;
		if (actionTarget != null){
			target = actionTarget.pose;
			type = actionTarget.type;
			remainTime = actionTarget.frame - frame;
		} else {
			type = currentType;
		}
		target = Pose2d.relativePose(g.pose, target);
		
//		double maxLen = 250;
//		double lenLimit = maxLen/2;
//		Vector2d n = new Vector2d(target.position);
//		n.normalize();
//		lenLimit += lenLimit + maxLen/2*Math.max(n.x, 0);
//		double vLen = MathUtil.length(target.position);
//		if (vLen > maxLen){
//			target.position.scale(maxLen/vLen);
//		}
		
		
		String aType = type.actionName();
		switch (type){
		case Dribble:
			overTime = 1;
			target.direction = new Vector2d(Double.NaN, Double.NaN);
//			target.direction = Pose2d.BASE.direction;
//			aType = "pass";
			aType = "shoot";
			break;
		case Walk:
			overTime = 1;
			aType = "pass'";
			break;
		case Catch:
			target.direction = new Vector2d(Double.NaN, Double.NaN);
//			target.direction = Pose2d.BASE.direction;
			break;
		default:
			break;
		}
		
		double[] control = BasketGameControl.getActionType(aType);
		double activation;
		double aMargin = BasketGameControl.ACTION_MARGIN;
//		aMargin = 45;
		if (remainTime > aMargin){
			overTime = 1;
		}
		if (overTime == 1){
			normalize(target.position, 250);
			activation = 0;
		} else {
			activation = 1 - Math.max(0, Math.abs(remainTime/aMargin));
		}
		control = MathUtil.concatenate(control, new double[]{ overTime, activation });
		if (USE_INTERVAL_BAR){
			double posAngle = Math.atan2(target.position.y, target.position.x);
			double posLen = MathUtil.length(target.position);
			double dirAngle = Math.atan2(target.direction.y, target.direction.x);
			control = MathUtil.concatenate(control, 
					targetBar.timeBar.getBar(remainTime), targetBar.posAngleBar.getBar(posAngle), 
					targetBar.posLengthBar.getBar(posLen), targetBar.dirAngleBar.getBar(dirAngle));
		} else if (USE_ACTION_DIVISION){
			int actionSize = BasketGameControl3.getActionSize();
			int actionIndex = BasketGameControl3.getActionIndex(aType, overTime == 1);
			boolean[] includeDirection = BasketGameControl3.getIncludeDirection();
			double[] aControl = new double[actionSize];
			aControl[actionIndex] = 1;
			for (int i = 0; i < actionSize; i++) {
				if (i == actionIndex){
					aControl = MathUtil.concatenate(aControl, new double[]{ activation });
					if (includeDirection[i]){
						aControl = MathUtil.concatenate(aControl, target.toArray());
					} else {
						aControl = MathUtil.concatenate(aControl, new double[]{ target.position.x, target.position.y });
					}
				} else {
					double[] append = new double[includeDirection[i] ? 5 : 3];
					for (int j = 0; j < append.length; j++) {
						append[j] = Double.NaN;
					}
					aControl = MathUtil.concatenate(aControl, append);
				}
			}
			control = aControl;
		} else {
			control = MathUtil.concatenate(control, target.toArray());
		}
		return control;
	}
	
	public static class ActionTarget{
		public ActionType type;
		public int frame;
		public Pose2d pose;
		public boolean isFinished = false;
		public ActionTarget(ActionType type, int frame, Pose2d pose) {
			this.type = type;
			this.frame = frame;
			this.pose = pose;
		}
		
		public String toString(){
			return "ActionTarget[ " + Utils.toString(type, frame, pose, isFinished) + "]";
		}
	}
}
