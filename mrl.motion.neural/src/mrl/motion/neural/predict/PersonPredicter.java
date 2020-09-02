package mrl.motion.neural.predict;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.sketch.SketchCanvas;
import mrl.motion.neural.basket.sketch.SketchLine;
import mrl.motion.neural.basket.sketch.SketchNode;
import mrl.motion.neural.basket.sketch.SketchTexture;
import mrl.motion.neural.data.BasketTimeControl;
import mrl.motion.neural.data.BasketTimeControl2;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.data.ParameterBar;
import mrl.motion.neural.data.SpatioTemporalBar;
import mrl.motion.neural.data.ParameterBar.AngleBar;
import mrl.motion.neural.data.ParameterBar.LengthBar;
import mrl.motion.neural.run.GameAIPlayer;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.position.PositionResultMotion;
import mrl.util.MathUtil;


public class PersonPredicter{
	
	public static boolean USE_INTERVAL_BAR = false;
	
	public int[] fixedTime = null;
	
	private SketchToPredicter sToP;
	public Normalizer normal;
	private String label;
	
	
	private SketchNode currentNode;
	private SketchNode targetNode;
	private GoalInfo targetGoal;
	
	private GoalInfo nextGoal;
	private int nextTimeInterval;
	
	private double currentRemainTime;
	private boolean waitOpposite;
	private RuntimeMotionGenerator g;
	
	public boolean isFinished = false;
	public ArrayList<Pose2d> poseList = new ArrayList<Pose2d>();
	
	public static boolean enableRootMove = true;
	public static double rootMovement = -2;
	private int totalTime = 0;
	
	public PersonPredicter(SketchToPredicter sToP, String label) {
		this.sToP = sToP;
		this.label = label;
		g = new RuntimeMotionGenerator();
	}
	
	public PositionResultMotion getMotion(){
		return g.motionSequence;
	}
	
	public RuntimeMotionGenerator getMotionGenerator(){
		return g;
	}
	
	public void setStartNode(SketchNode node){
		g.pose = getGoalInfo(node).pose;
		poseList.add(new Pose2d(g.pose));
		targetNode = node;
		jumpToNext();
	}
	
	public void printAll(){
		System.out.println("print all :: " + label);
		while (true){
			System.out.println(targetGoal.type + " : " + targetGoal.pose + " : " + targetGoal.isLast);
			jumpToNext();
			if (isFinished) break;
		}
	}

	public double[] getEstimationGoal(){
		Pose2d gPose = Pose2d.relativePose(g.pose, targetGoal.pose);
		double[] goalPose = BasketDataGenerator.getControlData(gPose);
		goalPose = normal.normalizeX(goalPose, 4);
		if (enableRootMove){
			goalPose = MathUtil.concatenate(goalPose, new double[]{ rootMovement });
		}
		double[] goalAction = BasketDataGenerator.getActionType(targetGoal.type);
		return MathUtil.concatenate(goalAction, goalPose);
	}
	
//	private ParameterBar posAngleBar = new AngleBar(8, 2);
//	private ParameterBar posLengthBar = new LengthBar(0, 600, 12, 2);
//	private ParameterBar dirAngleBar = new AngleBar(8, 2);
//	private ParameterBar timeBar = new LengthBar(-BasketTimeControl2.POST_MARGIN, BasketTimeControl2.ACTION_MARGIN, 8, 2);
	
	private SpatioTemporalBar targetBar = new SpatioTemporalBar(4, -BasketTimeControl2.POST_MARGIN, BasketTimeControl2.ACTION_MARGIN, 500);
	private SpatioTemporalBar nextBar = new SpatioTemporalBar(2, 30, 210, 1000);

	int index = 0;
	public double[] getPredictGoal(){
//		double activation = BasketActivationControl.getActivation(Math.min(currentRemainTime, currentElapsedTime), BasketActivationControl.ACTIVATION_MARGIN);
//		double nTime =  normal.normalizeX(new double[]{ currentRemainTime }, normal.xMeanAndStd[0].length-1)[0];
//		double[] timeAndActi = new double[]{ nTime, activation };
//		double[] eGoal = getEstimationGoal();
		
//		System.out.println("p :: " + Utils.toString(label, index, currentRemainTime, activation, 
//				targetGoal.type, Pose2d.relativePose(g.pose, targetGoal.pose).position));
//		index++;
//		return MathUtil.concatenate(timeAndActi, eGoal);
		
		
		// 0.30933822363081215, 0.14159156995791775
//		double movement = 0.309338;
		
//		0.1916998935597095, 0.13657315555771873
		double movement = 0.25;
//		double movement = (poseModule.getTimeList().get(1)-100)/70d;
		
		double[] control = BasketTimeControl.getActionType(targetGoal.type);
//		double[] control = BasketDataGenerator.getActionType(targetGoal.type);
		double[] goalInfo;
		double maxTime = BasketTimeControl.ACTION_MARGIN;
//		double maxTime = MAX_TIME;
//		double maxTime = MAX_TIME - 30;
		double remainTime = currentRemainTime;
		boolean isOverTime = remainTime >= maxTime;
		Pose2d target = Pose2d.relativePose(g.pose, targetGoal.pose);
		if (isOverTime){
			isOverTime = true;
			
			double ratio = maxTime/remainTime;
			remainTime = maxTime;
//			GameAIPlayer.normalize(target.position, 550);
//			target.position.scale(ratio);
			goalInfo = BasketDataGenerator.getControlData(target);
			// position only
			goalInfo[2] = 1;
			goalInfo[3] = 0;
		} else {
			isOverTime = false;
			goalInfo = BasketDataGenerator.getControlData(target);
		}
		double activation;
		activation = Math.max(0, 1 - Math.abs(remainTime)/BasketTimeControl.ACTIVATION_MARGIN);
		
		if (USE_INTERVAL_BAR){
			control = BasketTimeControl2.getActionType(targetGoal.type);
			control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0, activation });
			
			Pose2d targetPose = Pose2d.relativePose(g.pose, targetGoal.pose);
			if (isOverTime){
				targetPose.direction = new Vector2d(1, 0);
			}
			double posAngle = Math.atan2(targetPose.position.y, targetPose.position.x);
			double posLen = MathUtil.length(targetPose.position);
			double dirAngle = Math.atan2(targetPose.direction.y, targetPose.direction.x);
			control = MathUtil.concatenate(control, targetBar.timeBar.getBar(remainTime), targetBar.posAngleBar.getBar(posAngle), 
					targetBar.posLengthBar.getBar(posLen), targetBar.dirAngleBar.getBar(dirAngle));
			
			if (BasketTimeControl2.includeNextTarget){
				targetPose = Pose2d.relativePose(g.pose, nextGoal.pose);
				remainTime = remainTime + nextTimeInterval;
				
				posAngle = Math.atan2(targetPose.position.y, targetPose.position.x);
				posLen = MathUtil.length(targetPose.position);
				dirAngle = Math.atan2(targetPose.direction.y, targetPose.direction.x);
				
				control = MathUtil.concatenate(control, 
						nextBar.timeBar.getBar(remainTime), nextBar.posAngleBar.getBar(posAngle), 
						nextBar.posLengthBar.getBar(posLen), nextBar.dirAngleBar.getBar(dirAngle));
			}
		} else {
	//		control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0, remainTime });
			control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0, activation, remainTime });
			control = MathUtil.concatenate(control, goalInfo);
			
//			control = MathUtil.concatenate(control, new double[]{ movement });
			
			if (BasketTimeControl2.includeNextTarget){
				Pose2d nextInteraction = nextGoal.pose;
				Pose2d targetPose = Pose2d.relativePose(g.pose, nextGoal.pose);
				Vector2d v = new Vector2d(targetPose.position);
				v.normalize();
				
				double length = targetGoal.pose.position.distance(nextInteraction.position);
				int timeInterval = nextTimeInterval;
				control = MathUtil.concatenate(control, new double[]{ v.x, v.y, length, timeInterval });
			}
		}
		
		index++;
		return normal.normalizeX(control);
	}
	
	public void updateMotion(double[] output){
		g.update(output);
		totalTime++;
		if (!waitOpposite) currentRemainTime--;
//		if (currentRemainTime < -ShootTimeControl.ACTION_MARGIN/4){
		if (currentRemainTime < -5){
			jumpToNext();
		}
	}
	
	public boolean isTimeUpdateRequired(){
		return currentRemainTime < -1000;
	}
	
	public void updateRemainTime(ArrayList<PersonPredicter> personList, double[] estimated){
		if (fixedTime != null) return;
//		if (!(currentRemainTime < -1000)) return;
		int myIdx = personList.indexOf(this);
		int opIdx = getOppositeIndex(personList);
		
		if (opIdx >= 0){
			PersonPredicter opposite = personList.get(opIdx);
			if (opposite.targetGoal.opposite.label.equals(this.label)){
				double maxTime = Math.max(estimated[myIdx], estimated[opIdx]);
				double minTime = Math.min(estimated[myIdx], estimated[opIdx]);
				double time = maxTime*0.75 + minTime*0.25;
				if (checkUpdateTime(time)){
					currentRemainTime = time;
					opposite.currentRemainTime = time;
					opposite.waitOpposite = false;
					waitOpposite = false;
				}
			} else {
				waitOpposite = true;
				currentRemainTime = BasketDataGenerator.MAX_TIME;
			}
		} else {
			double time = estimated[myIdx];
			if (targetGoal.isLast){
				time += 75;
			}
			if (checkUpdateTime(time)){
				currentRemainTime = time;
				waitOpposite = false;
			}
		}
	}
	
	private boolean checkUpdateTime(double newTime){
		if (Math.abs(newTime - currentRemainTime) > 30){
			return true;
		} 
		return false;
	}
	
	
	private int getOppositeIndex(ArrayList<PersonPredicter> personList){
		if (targetGoal.opposite == null) return -1;
		String label = targetGoal.opposite.label;
		for (int i = 0; i < personList.size(); i++) {
			if (personList.get(i).label.equals(label)) return i;
		}
		return -1;
	}
	
	
	private int targetIndex = 0;
	private void jumpToNext(){
		currentNode = targetNode;
		SketchLine outGoing = sToP.getOutgoingLine(currentNode);
		if (outGoing == null){
			if (!isFinished){
				System.out.println(label + " : finished at : " + index);
				isFinished = true;
			}
			return;
		}
		targetNode = outGoing.target;
		targetGoal = getGoalInfo(targetNode);
		poseList.add(targetGoal.pose);
		
		if (BasketTimeControl2.includeNextTarget){
			outGoing = sToP.getOutgoingLine(targetNode);
			if (outGoing == null){
				nextGoal = targetGoal;
				nextTimeInterval = 60;
			} else {
				nextGoal = getGoalInfo(outGoing.target);
				nextTimeInterval = fixedTime[targetIndex + 1] - fixedTime[targetIndex];
			}
		}
		
		// do estimation
		if (fixedTime == null){
			currentRemainTime = -100000;
		} else {
			currentRemainTime = fixedTime[targetIndex] - totalTime;
			System.out.println("jump :: " + label + " : " + targetGoal.type + " : " + currentRemainTime + " : " + fixedTime[targetIndex] + " : " + totalTime);
//			if (targetGoal.type.equals("pass'")){
//				currentRemainTime += 22;
//			}
			targetIndex++;
			if (targetGoal.isLast){
				currentRemainTime += 75;
			}
		}
		
//		System.out.println(label + " : jump at : " + index);
	}
	
	private GoalInfo getGoalInfo(SketchNode node){
		GoalInfo info = new GoalInfo();
		if (node.isShoot){
			info.oppositePosition = SketchCanvas.BASKET_POS;
			info.type = "shoot_near";
		} else {
			SketchLine passLine = sToP.getPassLine(node);
			if (passLine != null){
				if (passLine.source == node){
					info.oppositePosition = passLine.target.position;
					info.opposite = passLine.target;
					info.type = "pass";
				} else {
					info.oppositePosition = passLine.source.position;
					info.opposite = passLine.source;
					info.type = "pass'";
				}
			} else {
				SketchLine outGoing = sToP.getOutgoingLine(node);
				if (outGoing != null){
					info.oppositePosition = outGoing.target.position;
					// TODO:: start node
					info.type = null;
				} else {
					info.oppositePosition = SketchCanvas.BASKET_POS;
					// TODO:: end node
					info.type = "pass'";
					info.isLast = true;
				}
			}
		}
//		System.out.println("pose :: " + node.label + "-> " + (info.opposite == null ? "null " : info.opposite.label));
		info.pose = getPose(node.position, info.oppositePosition);
//		System.out.println(info.pose);
		return info;
	}
	
	private Pose2d getPose(Point2d position, Point2d target){
//		System.out.println(position + " : " + target);
		position = new Point2d(position.x, -position.y);
		target = new Point2d(target.x, -target.y);
		Vector2d v = MathUtil.sub(target, position);
//		System.out.println(position + " : " + target + " : " + v);
		position.scale(SketchTexture.COURT_SCALE);
		return new Pose2d(position, v);
	}
	
	private static class GoalInfo{
		String type;
		Point2d oppositePosition;
		SketchNode opposite;
		Pose2d pose;
		boolean isLast;
	}
}
