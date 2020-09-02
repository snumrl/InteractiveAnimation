package mrl.motion.neural.data;

import java.util.ArrayList;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.BasketDataGenerator.ControlTarget;
import mrl.motion.position.PositionMotion;
import mrl.util.MathUtil;

public class BasketActivationControl extends ControlDataGenerator{
	
	public static double ACTIVATION_MARGIN = 30;
	public static double PRE_ACT_MARGIN = 30;
	
	private MDatabase database;
	private ArrayList<ControlTarget> targetList;
	private ControlTarget target;
	private int tIndex;
	private int prevInterIdx;
	
	public BasketActivationControl(MDatabase database, ArrayList<ControlTarget> targetList) {
		this.database = database;
		this.targetList = targetList;
		tIndex = 0;
		prevInterIdx = 0;
		target = targetList.get(tIndex);
	}

	@Override
	public double[] getControl(int index) {
		if (index >= target.mIndex){
			prevInterIdx = target.mIndex;
			tIndex++;
			if (tIndex >= targetList.size()) return null;
			target = targetList.get(tIndex);
		}
		
		double[] action = BasketDataGenerator.getActionType(target.clip.type);
		int remainTime = target.mIndex - index;
		Pose2d p = PositionMotion.getPose(mList.get(index));
		Pose2d targetPose = Pose2d.relativePose(p, target.getInteractionPose(mList, database));
		int elapsedTime = index - prevInterIdx;
		
		double activation = getActivation(Math.min(remainTime, elapsedTime), ACTIVATION_MARGIN);
		double prevActivation = getActivation(Math.abs(remainTime - ACTIVATION_MARGIN), PRE_ACT_MARGIN);
		
		double[] control = BasketDataGenerator.getControlData(targetPose, remainTime);
		
		control = MathUtil.concatenate(action, control);
		control = MathUtil.concatenate(new double[]{ prevActivation, activation }, control);
		
		return control;
	}
	
	public static double getActivation(double time, double margin){
		return Math.max((margin - time)/margin, 0);
	}

	
	public boolean[] getNormalMarking(){
		// pre act, act, action=3
		return getTrueList(5);
	}
}
