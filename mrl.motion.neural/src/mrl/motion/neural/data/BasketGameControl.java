package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Contact;
import mrl.motion.data.MDatabase;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.BasketGraph;
import mrl.motion.neural.basket.BasketDataGenerator.ControlTarget;
import mrl.motion.position.PositionMotion;
import mrl.util.MathUtil;

public class BasketGameControl extends ControlDataGenerator{
	
	public static int ACTION_MARGIN = 60;
	public static int CONTROL_MARGIN = 30;
	public static int POST_MARGIN = 20;
	
	public static boolean useDoubleAction = false;
	
	private MDatabase database;
	private ArrayList<ControlTarget> targetList;
	private ControlTarget target;
	private int tIndex;
	private boolean havingBall;
	
	public BasketGameControl(MDatabase database, ArrayList<ControlTarget> targetList) {
		this.database = database;
		this.targetList = targetList;
		tIndex = 0;
		target = targetList.get(tIndex);
		
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
	
	private boolean isReceive(){
		String type = target.clip.type;
		return type.equals("pass'") || type.equals("catch");
	}
	
	@Override
	public double[] getControl(int index) {
		if (index >= target.mIndex + target.postMargin){
			tIndex++;
			if (tIndex >= targetList.size() - 2) return null;
			target = targetList.get(tIndex);
		}
		
		int remainTime = target.mIndex - index;
		
		
		boolean havingBall;
		if (remainTime < target.prevMargin){
			if (isReceive()){
				havingBall = remainTime <= 0;
			} else {
				havingBall = remainTime >= 0;
			}
		} else {
			if (isReceive()){
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
		if (isOverTime){
			isOverTime = true;
			activation = 0;
			Pose2d targetPose = PositionMotion.getPose(mList.get(index + CONTROL_MARGIN));
			targetPose = Pose2d.relativePose(p, targetPose);
			goalInfo = targetPose.toArray();
			
			if (!isReceive()){
				// position only
				goalInfo[2] = Double.NaN;
				goalInfo[3] = Double.NaN;
			}
		} else {
			activation = Math.max(0, 1 - Math.abs(remainTime)/(double)ACTION_MARGIN);
			isOverTime = false;
			Pose2d targetPose;
			if (target.clip.type.equals("catch")){
				mList.get(target.mIndex).ballContact = new Contact(true, true);
				Point3d ball = BallTrajectoryGenerator.getBallPositionByHand(mList.get(target.mIndex));
				targetPose = new Pose2d(Pose2d.BASE);
				targetPose.position = Pose2d.to2d(ball);
				targetPose = Pose2d.relativePose(p, targetPose);
				targetPose.direction = new Vector2d(Double.NaN, Double.NaN);
			} else {
				targetPose = Pose2d.relativePose(p, target.getInteractionPose(mList, database));
			}
			goalInfo = targetPose.toArray();
		}
		double[] control = action;
		control = MathUtil.concatenate(control, new double[]{ isOverTime ? 1 : 0, activation });
		control = MathUtil.concatenate(control, goalInfo);
		
		if (includeRootMove){
			Vector3d base;
			if (remainTime < ACTION_MARGIN){
				base = MathUtil.getTranslation(mList.get(target.mIndex - ACTION_MARGIN).root());
			} else {
				base = MathUtil.getTranslation(mList.get(index).root());
			}
			Vector3d tPos = MathUtil.getTranslation(mList.get(Math.min(target.mIndex, index + BasketDataGenerator.MAX_TIME)).root());
			Vector3d direction = MathUtil.sub(tPos, base);
			double movement = getDirectionalMovement(direction, index, 45);
			control = MathUtil.concatenate(control, new double[]{ movement });
		}
		return control;
	}
	
	public double[] getHasBall(int index){
		return new double[]{ havingBall ? 1 : 0 };
	}

	public static double[] getActionType(String action){
		String[] types;
		if (BasketGraph.INCLUDE_CATCH){
			types = new String[]{"shoot", "shoot_near", "pass", "pass'", "catch"};
		} else {
			types = new String[]{"shoot", "shoot_near", "pass", "pass'"};
		}
		
		if (useDoubleAction){
			double[] data = new double[types.length*2];
			boolean isExist = false;
			for (int i = 0; i < types.length; i++) {
				if (action.equals(types[i])){
					data[i*2] = 1;
					data[i*2+1] = 2;
					isExist = true;
				} else {
					data[i*2] = -1;
					data[i*2+1] = -2;
				}
			}
			if (!isExist){
				System.out.println("no action :: " + action + " : " + Arrays.toString(types));
				throw new RuntimeException();
			}
			return data;
		} else {
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
	}
	
	
	public boolean[] getNormalMarking(){
		// action=4,5, overTime, activation
		int size = BasketGraph.INCLUDE_CATCH ? 5 : 4;
		if (useDoubleAction) size *= 2;
		return getTrueList(size + 2);
	}
}
