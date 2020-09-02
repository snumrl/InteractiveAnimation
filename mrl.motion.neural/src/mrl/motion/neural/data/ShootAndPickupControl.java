package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map.Entry;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Contact;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.position.PositionMotion;
import mrl.util.Logger;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class ShootAndPickupControl  extends ControlDataGenerator{
	
	public static int ACTION_MARGIN = 45;
	
	private int interval = 30;
	private MDatabase database;
	private HashMap<String, HashSet<Integer>> actionFrameMap;
	private HashMap<MotionAnnotation, Integer> interactionFrameMap;
	
	private ArrayList<ShootAction> targetList;
	private ShootAction target;
	private int tIndex;
	private boolean havingBall;
	
	public int dribbleStart;
	public int graphStart;
	
	private int dribbleStartIndex;
	private int graphStartIndex;
	
	public static int ACTION_SIZE = 4;
	
	public enum ActionType { 
		Dribble, Walk, Shoot, Pickup, Pass, Pass_, Shoot_Near, Catch; 
		
		public String actionName(){
			if (this == Pass_) return "pass'";
			return name().toLowerCase();
		}
		
		public boolean isShoot(){
			return this == Shoot || this == Shoot_Near;
		}
	}
	
	public ShootAndPickupControl(MDatabase database, HashMap<String, HashSet<Integer>> actionFrameMap, HashMap<MotionAnnotation, Integer> interactionFrameMap) {
		this.database = database;
		this.actionFrameMap = actionFrameMap;
		this.interactionFrameMap = interactionFrameMap;
	}

	public void initImpl(){
		targetList = new ArrayList<ShootAction>();
		Logger.startLogging("ShootAndPickupControl.log");
		for (int i = 1; i < mList.size()-1; i++) {
			int mStart = mList.get(i).motionIndex;
			int mEnd = mList.get(i+1).motionIndex;
			if (Math.abs(mEnd - mStart) > 2) continue;
			for (int mIdx = mStart; mIdx < mEnd; mIdx++) {
				if (actionFrameMap.get("shoot").contains(mIdx)){
					targetList.add(new ShootAction(i, ActionType.Shoot, getShootPose(i)));
					i+=2;
				} else if (actionFrameMap.get("pickup").contains(mIdx)){
					mList.get(i).ballContact = new Contact(true, true);
					Point3d ball = BallTrajectoryGenerator.getBallPositionByHand(mList.get(i));
					Pose2d pose = new Pose2d(Pose2d.BASE);
					pose.position = Pose2d.to2d(ball);
					targetList.add(new ShootAction(i, ActionType.Pickup, pose));
					i+=2;
				} else if (actionFrameMap.get("pass").contains(mIdx)){
					double angleOffset =  getPassAngleOffset(mIdx, 10);
					Pose2d pose = getInteractionPose(i, angleOffset);
					targetList.add(new ShootAction(i, ActionType.Pass, pose));
					i+=2;
				} if (actionFrameMap.get("pass_").contains(mIdx)){
					double angleOffset =  getPassAngleOffset(mIdx, -5);
					Pose2d pose = getInteractionPose(i, angleOffset);
					targetList.add(new ShootAction(i, ActionType.Pass_, pose));
					i+=2;
				}
			}
		}
		Logger.line("target size :: " + targetList.size());
		for (ShootAction target : targetList){
			Logger.line(target.toString() + " : " + getAnn(mList.get(target.fIndex).motionIndex));
		}
		double dtSum = 0;
		for (int i = 0; i < targetList.size() - 1; i++) {
			dtSum += targetList.get(i+1).fIndex - targetList.get(i).fIndex;
		}
		System.out.println("mean interval :: " + dtSum/(targetList.size()-1));
		tIndex = 0;
		target = targetList.get(tIndex);
		
		dribbleStartIndex = findMotionIndex(dribbleStart);
		graphStartIndex = findMotionIndex(graphStart);
		System.out.println("start index :: " + dribbleStartIndex + " , " + graphStartIndex);
		
		
		for (int i = 0; i < targetList.size(); i++) {
			ShootAction t1 = targetList.get(i);
			if (i <= 0) {
				t1.prevMargin = ACTION_MARGIN;
			} else {
				ShootAction t0 = targetList.get(i-1);
				t1.prevMargin = Math.min((t1.fIndex - t0.fIndex)/2, ACTION_MARGIN);
			}
			if (i >= targetList.size()-1) {
				t1.postMargin = ACTION_MARGIN;
			} else {
				ShootAction t2 = targetList.get(i+1);
				t1.postMargin = Math.min((t2.fIndex - t1.fIndex)/2, ACTION_MARGIN);
			}
		}
	}
	
	private double getPassAngleOffset(int motionIndex, int offset){
		MotionAnnotation ann = getAnn(motionIndex);
		String oppFile = ann.file.substring(0, ann.file.length()-5) + ann.oppositePerson + ".bvh";
		Motion m1 = database.getMotionList()[motionIndex];
		Motion m2 = database.findMotion(oppFile, ann.interactionFrame + offset);
		Matrix4d t1 = m1.root();
		Matrix4d t2 = m2.root();
		
		Vector3d p1 = MathUtil.getTranslation(t1);
		Vector3d p2 = MathUtil.getTranslation(t2);
		
		Vector2d v = new Vector2d(p2.x - p1.x, p2.z - p1.z);
		double angleOffset = Math.atan2(-v.y, v.x);
		return angleOffset;
	}
	
	private MotionAnnotation getAnn(int mIndex){
		for (Entry<MotionAnnotation, Integer> entry : interactionFrameMap.entrySet()){
			if (entry.getValue() == mIndex) return entry.getKey();
		}
		return null;
	}
	
	private int findMotionIndex(int mIndex){
		for (int i = 0; i < mList.size(); i++) {
			if (mList.get(i).motionIndex == mIndex) return i;
		}
		return -1;
	}
	
	@Override
	public double[] getControl(int index) {
		if (index >= target.fIndex + target.postMargin){
			tIndex++;
			if (tIndex >= targetList.size()) return null;
			target = targetList.get(tIndex);
		}
		
		int remainTime = target.fIndex - index;
		
		Point2d targetPosition = null;
		Vector2d targetDirection = Pose2d.BASE.direction;
		Pose2d p = PositionMotion.getPose(mList.get(index));
		ActionType actionType = null;
		boolean havingBall;
		if (remainTime < target.prevMargin){
			Pose2d targetPose = Pose2d.relativePose(p, target.pose);
			targetPosition = targetPose.position;
			actionType = target.type;
			if (target.type ==ActionType.Shoot || target.type == ActionType.Pass){
				targetDirection = targetPose.direction;
				havingBall = remainTime >= 0;
			} else if (target.type == ActionType.Pass_){
				targetDirection = targetPose.direction;
				havingBall = remainTime <= 0;
			} else {
				havingBall = remainTime <= 0;
			}
		} else {
			int controlIndex = index + interval;
			if (controlIndex >= mList.size()) return null;
			
			Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
			Pose2d targetPose = Pose2d.relativePose(p, cp);
			targetPosition = targetPose.position;
			if (index < dribbleStartIndex){
				actionType = ActionType.Walk;
				havingBall = false;
			} else if (index < graphStartIndex){
				actionType = ActionType.Dribble;
				havingBall = true;
			} else if (target.type == ActionType.Shoot || target.type == ActionType.Pass){
				actionType = ActionType.Dribble;
				havingBall = true;
			} else {
				actionType = ActionType.Walk;
				havingBall = false;
			}
		}
		
		double[] control = getControlData(actionType, new Pose2d(targetPosition, targetDirection));
		
		this.havingBall = havingBall;
		
		return control;
	}
	
	public static double[] getControlData(ActionType actionType, Pose2d target){
		if (actionType == null) throw new RuntimeException();
		double[] action = getActionData(actionType);
		double[] goal = target.toArray();
		return MathUtil.concatenate(action, goal);
	}
	
	public double[] getHasBall(int index){
		return new double[]{ havingBall ? 1 : 0 };
	}
	
	public static double[] getActionData(ActionType type){
		ActionType[] typeList = ActionType.values();
		double[] data = new double[ACTION_SIZE];
//		double[] data = new double[typeList.length];
		for (int i = 0; i < data.length; i++) {
			if (typeList[i] == type){
				data[i] = 1;
			}
		}
		return data;
	}
	
	public Pose2d getInteractionPose(int mIndex, double angleOffset){
		Motion originMotion = database.getMotionList()[mList.get(mIndex).motionIndex];
		Matrix4d m = new Matrix4d();
		m.rotY(angleOffset);
		Vector3d vX = new Vector3d(1, 0, 0);
		m.transform(vX);
		Vector3d interactionDirection = new Vector3d(vX);
		Point3d interactionPosition = new Point3d(MathUtil.getTranslation(originMotion.root()));
		interactionPosition.y = 0;
		if (Double.isNaN(interactionDirection.x)){
			throw new RuntimeException();
		}
		Pose2d interactionPose = new Pose2d(interactionPosition, interactionDirection);
		Pose2d iMotionPose = PositionMotion.getPose(originMotion);
		Pose2d relativePose = Pose2d.relativePose(iMotionPose, interactionPose);
		
		Motion gMotion = mList.get(mIndex);
		return PositionMotion.getPose(gMotion).localToGlobal(relativePose);
	}
	
	public Pose2d getShootPose(int mIndex){
		Motion originMotion = database.getMotionList()[mList.get(mIndex).motionIndex];
		Vector3d p1 = MathUtil.getTranslation(originMotion.root());
		// basket position( when motion captured )
		Vector3d p2 = new Vector3d();
		
		Vector2d v = new Vector2d(p2.x - p1.x, p2.z - p1.z);
		double angleOffset = Math.atan2(-v.y, v.x);
		return getInteractionPose(mIndex, angleOffset);
	}
	
	
	public boolean[] getNormalMarking(){
		boolean[] mark = new boolean[ActionType.values().length];
		for (int i = 0; i < mark.length; i++) {
			mark[i] = true;
		}
		return mark;
	}
	
	private static class ShootAction{
		public int fIndex;
		public ActionType type;
		public Pose2d pose;
		
		public int prevMargin = -1;
		public int postMargin = -1;
		
		public ShootAction(int fIndex, ActionType type, Pose2d pose) {
			this.fIndex = fIndex;
			this.type = type;
			this.pose = pose;
		}
		
		public String toString(){
			 return ("action[" + Utils.toString(fIndex, type, pose) + "]");
		}
	}
}