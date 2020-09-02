package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;
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

public class BasketGameControl3 extends ControlDataGenerator{
	
	public static int ACTION_MARGIN = 60;
	public static int CONTROL_MARGIN = 30;
	public static int POST_MARGIN = 20;
	
	private MDatabase database;
	private ArrayList<ControlTarget> targetList;
	private ControlTarget target;
	private int tIndex;
	private boolean havingBall;
	
	public BasketGameControl3(MDatabase database, ArrayList<ControlTarget> targetList) {
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
		
		
		
		double activation;
		
		
		Pose2d p = PositionMotion.getPose(mList.get(index));
		Pose2d targetPose;
		boolean isOverTime = remainTime >= ACTION_MARGIN;
		if (isOverTime){
			isOverTime = true;
			activation = 0;
			targetPose = PositionMotion.getPose(mList.get(index + CONTROL_MARGIN));
			targetPose = Pose2d.relativePose(p, targetPose);
		} else {
			activation = Math.max(0, 1 - Math.abs(remainTime)/(double)ACTION_MARGIN);
			isOverTime = false;
			if (target.clip.type.equals("catch")){
				mList.get(target.mIndex).ballContact = new Contact(true, true);
				Point3d ball = BallTrajectoryGenerator.getBallPositionByHand(mList.get(target.mIndex));
				targetPose = new Pose2d(Pose2d.BASE);
				targetPose.position = Pose2d.to2d(ball);
				targetPose = Pose2d.relativePose(p, targetPose);
				targetPose.direction = Pose2d.BASE.direction;
			} else {
				targetPose = Pose2d.relativePose(p, target.getInteractionPose(mList, database));
			}
		}
		
		int actionSize = getActionSize();
		int actionIndex = getActionIndex(target.clip.type, isOverTime);
		boolean[] includeDirection = getIncludeDirection();
		double[] control = new double[actionSize];
		control[actionIndex] = 1;
		for (int i = 0; i < actionSize; i++) {
			if (i == actionIndex){
				control = MathUtil.concatenate(control, new double[]{ activation });
				if (includeDirection[i]){
					control = MathUtil.concatenate(control, targetPose.toArray());
				} else {
					control = MathUtil.concatenate(control, new double[]{ targetPose.position.x, targetPose.position.y });
				}
			} else {
				double[] append = new double[includeDirection[i] ? 5 : 3];
				for (int j = 0; j < append.length; j++) {
					append[j] = Double.NaN;
				}
				control = MathUtil.concatenate(control, append);
			}
		}
		
		if (includeRootMove){
			throw new RuntimeException();
		}
		return control;
	}
	
	public double[] getHasBall(int index){
		return new double[]{ havingBall ? 1 : 0 };
	}
	
	public static int getActionSize(){
		if (BasketGraph.INCLUDE_CATCH){
			return 7;
		} else {
			return 6;
		}
	}

	public static int getActionIndex(String action, boolean isOverTime){
		String[] types;
		if (BasketGraph.INCLUDE_CATCH){
			types = new String[]{"shoot", "shoot_near", "pass", "pass'", "catch"};
		} else {
			types = new String[]{"shoot", "shoot_near", "pass", "pass'"};
		}
		
		if (isOverTime){
			boolean noBall = action.equals("pass'") || action.equals("catch");
			if (noBall){
				return types.length + 1;
			} else {
				return types.length;
			}
		} else {
			for (int i = 0; i < types.length; i++) {
				if (action.equals(types[i])){
					return i;
				}
			}
			System.out.println("no action :: " + action + " : " + Arrays.toString(types));
			throw new RuntimeException();
		}
	}
	
	
	public static boolean[] getIncludeDirection(){
		if (BasketGraph.INCLUDE_CATCH){
//			types = new String[]{"shoot", "shoot_near", "pass", "pass'", "catch"};
			return new boolean[]{ true, true, true, true, false, false, false }; 
		} else {
			return new boolean[]{ true, true, true, true, false, false}; 
//			types = new String[]{"shoot", "shoot_near", "pass", "pass'"};
		}
	}
	
	public boolean[] getNormalMarking(){
		return getTrueList(getActionSize());
	}
}
