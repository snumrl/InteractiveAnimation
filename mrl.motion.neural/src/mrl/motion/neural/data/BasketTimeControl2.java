package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.basket.BasketDataGenerator.ControlTarget;
import mrl.motion.neural.data.ParameterBar.AngleBar;
import mrl.motion.neural.data.ParameterBar.LengthBar;
import mrl.motion.position.PositionMotion;
import mrl.util.MathUtil;

public class BasketTimeControl2 extends ControlDataGenerator{
	
	public static int ACTION_MARGIN = 90;
	public static int ACTIVATION_MARGIN = 45;
	public static int POST_MARGIN = 15;
//	public static int POST_MARGIN = 8;
	
	public static boolean includeActivation = true;
	public static boolean includeNextTarget = false;
	
	private MDatabase database;
	private ArrayList<ControlTarget> targetList;
	private ControlTarget target;
	private ControlTarget nextTarget;
	private int tIndex;
	private boolean havingBall;
	
	public BasketTimeControl2(MDatabase database, ArrayList<ControlTarget> targetList) {
		this.database = database;
		this.targetList = targetList;
		tIndex = 0;
		target = targetList.get(tIndex);
		nextTarget = targetList.get(tIndex+1);
		
		for (int i = 0; i < targetList.size(); i++) {
			ControlTarget t1 = targetList.get(i);
			if (i <= 0) {
				t1.prevMargin = ACTION_MARGIN;
			} else {
				ControlTarget t0 = targetList.get(i-1);
				t1.prevMargin = Math.min((t1.mIndex - t0.mIndex)/2, ACTION_MARGIN);
			}
			if (i >= targetList.size()-1) {
				t1.postMargin = POST_MARGIN;
			} else {
				ControlTarget t2 = targetList.get(i+1);
				t1.postMargin = Math.min((t2.mIndex - t1.mIndex)/2, POST_MARGIN);
			}
		}
		
		double dtSum = 0;
		for (int i = 0; i < targetList.size() - 1; i++) {
			ControlTarget t = targetList.get(i);
			System.out.println("tt : " + i + " : " + t.clip + " : " + t.prevMargin + " , " + t.postMargin + " : " + t.mIndex);
			if (i > 4){
				dtSum += targetList.get(i+1).mIndex - targetList.get(i).mIndex;
			}
		}
		System.out.println("mean interval :: " + dtSum/(targetList.size()-1));
	}
	
	private SpatioTemporalBar targetBar = new SpatioTemporalBar(4, -POST_MARGIN, ACTION_MARGIN, 500);
	private SpatioTemporalBar nextBar = new SpatioTemporalBar(2, 30, 210, 1000);
	
	@Override
	public double[] getControl(int index) {
		if (index >= target.mIndex + target.postMargin){
			tIndex++;
			if (tIndex >= targetList.size() - 3){
				targetBar.posAngleBar.printStatistics();
				targetBar.posLengthBar.printStatistics();
				targetBar.dirAngleBar.printStatistics();
				targetBar.timeBar.printStatistics();
				System.out.println("#################### next bar");
				nextBar.posAngleBar.printStatistics();
				nextBar.posLengthBar.printStatistics();
				nextBar.dirAngleBar.printStatistics();
				nextBar.timeBar.printStatistics();
				return null;
			}
			target = targetList.get(tIndex);
			nextTarget = targetList.get(tIndex+1);
		}
		
		int remainTime = target.mIndex - index;
		
		
		boolean havingBall;
		if (remainTime < target.prevMargin){
			if (target.clip.type.equals("pass'")){
				havingBall = remainTime <= 0;
			} else {
				havingBall = remainTime >= 0;
			}
		} else {
			if (target.clip.type.equals("pass'")){
				havingBall = false;
			} else{
				havingBall = true;
			}
		}
		this.havingBall = havingBall;
		
		
		
		double[] action = getActionType(target.clip.type);
		double activation;
		
		
		Pose2d p = PositionMotion.getPose(mList.get(index));
		double[] goalInfo;
		boolean isOverTime = remainTime >= ACTION_MARGIN;
		Pose2d targetPose;
		if (isOverTime){
			isOverTime = true;
			remainTime = ACTION_MARGIN;
			activation = 0;
			targetPose = PositionMotion.getPose(mList.get(index + ACTION_MARGIN));
			targetPose = Pose2d.relativePose(p, targetPose);
			targetPose.direction = new Vector2d(1, 0);
		} else {
			activation = Math.max(0, 1 - Math.abs(remainTime)/(double)ACTIVATION_MARGIN);
			isOverTime = false;
			targetPose = Pose2d.relativePose(p, target.getInteractionPose(mList, database));
		}
		goalInfo = BasketDataGenerator.getControlData(targetPose);
		
		double[] control = action;
		if (includeActivation){
			control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0, activation });
		} else {
			control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0 });
		}
		
		double posAngle = Math.atan2(targetPose.position.y, targetPose.position.x);
		double posLen = MathUtil.length(targetPose.position);
		double dirAngle = Math.atan2(targetPose.direction.y, targetPose.direction.x);
		control = MathUtil.concatenate(control, 
				targetBar.timeBar.getBar(remainTime), targetBar.posAngleBar.getBar(posAngle), 
				targetBar.posLengthBar.getBar(posLen), targetBar.dirAngleBar.getBar(dirAngle));
		
//		control = MathUtil.concatenate(control, goalInfo);
		
		if (includeRootMove){
//			Vector3d base;
//			if (remainTime < ACTION_MARGIN){
//				base = MathUtil.getTranslation(mList.get(target.mIndex - ACTION_MARGIN).root());
//			} else {
//				base = MathUtil.getTranslation(mList.get(index).root());
//			}
//			Vector3d tPos = MathUtil.getTranslation(mList.get(Math.min(target.mIndex, index + BasketDataGenerator.MAX_TIME)).root());
//			Vector3d direction = MathUtil.sub(tPos, base);
//			double movement = getDirectionalMovement(direction, index, 45);
			double movement = getMovement(rootMove, index);
			control = MathUtil.concatenate(control, new double[]{ movement });
		}
		
		if (includeNextTarget){
			targetPose = Pose2d.relativePose(p, nextTarget.getInteractionPose(mList, database));
			remainTime = nextTarget.mIndex - index;
			
			posAngle = Math.atan2(targetPose.position.y, targetPose.position.x);
			posLen = MathUtil.length(targetPose.position);
			dirAngle = Math.atan2(targetPose.direction.y, targetPose.direction.x);
			
			control = MathUtil.concatenate(control, 
					nextBar.timeBar.getBar(remainTime), nextBar.posAngleBar.getBar(posAngle), 
					nextBar.posLengthBar.getBar(posLen), nextBar.dirAngleBar.getBar(dirAngle));
		}
		
		return control;
	}
	
	public double[] getHasBall(int index){
		return new double[]{ havingBall ? 1 : 0 };
	}
	
	public static double[] getActionType(String action){
		String[] types;
		types = new String[]{"shoot", "shoot_near", "pass", "pass'"};
		double[] data = new double[types.length];
		boolean isExist = false;
		for (int i = 0; i < data.length; i++) {
			if (action.equals(types[i])){
				data[i] = 1;
				isExist = true;
			}
		}
		if (!isExist){
			System.out.println("no action :: " + action + " : " + Arrays.toString(types));
			throw new RuntimeException();
		}
		return data;
	}
	
	

	public boolean[] getNormalMarking(){
		// action=4, overTime, activation
		int size = includeActivation ? 6 : 5;
		size += 32 + 4;
		if (includeNextTarget){
			size += 16 + 2; 
		}
		return getTrueList(size);
	}
}
