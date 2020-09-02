package mrl.motion.neural.loco;

import javax.vecmath.Vector2d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.ControlDataGenerator;
import mrl.motion.position.PositionMotion;
import mrl.util.MathUtil;

public abstract class SelectiveControl extends ControlDataGenerator{
	
	
	public static int MIN_INTERVAL = 150;
	public static int MAX_INTERVAL = 400;

	private int nextChangeIndex = 0;

	@Override
	public double[] getControl(int index) {
		boolean doChange = false;
		if (index >= nextChangeIndex){
			doChange = true;
			int offset = MIN_INTERVAL + MathUtil.random.nextInt(MAX_INTERVAL - MIN_INTERVAL);
			nextChangeIndex += offset;
		}
		return getControl(index, doChange);
	}
	
	protected void initImpl(){
		nextChangeIndex = 0;
	}
	
	protected abstract double[] getControl(int index, boolean changeSelection);

	
	public static class SelectiveDirectionControl extends SelectiveControl{
		
		private boolean useDirection = true;

		@Override
		protected double[] getControl(int index, boolean changeSelection) {
			if (changeSelection){
				useDirection = MathUtil.random.nextBoolean();
			}
			
			
			int interval = 30; // 30frame = 1s
			
			int controlIndex = index + interval;
			if (controlIndex >= mList.size()) return null;
			
			Pose2d p = PositionMotion.getPose(mList.get(index));
			Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
			Pose2d targetPose = Pose2d.relativePose(p, cp);
			
			Vector2d direction;
			if (useDirection){
				direction = new Vector2d(targetPose.direction);
			} else {
				direction = new Vector2d(Double.NaN, Double.NaN);
			}
			
			return new double[]{ targetPose.position.x, targetPose.position.y, direction.x, direction.y, useDirection ? 1 : 0 };
		}
		
	}
	
	public static class SelectiveTimingControl extends SelectiveControl{
		
		private int timeInterval = 30;
		
		@Override
		protected double[] getControl(int index, boolean changeSelection) {
			if (changeSelection){
				timeInterval = 15 + MathUtil.random.nextInt(60);
			}
			
			int interval = timeInterval; // 30frame = 1s
			
			int controlIndex = index + interval;
			if (controlIndex >= mList.size()) return null;
			
			Pose2d p = PositionMotion.getPose(mList.get(index));
			Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
			Pose2d targetPose = Pose2d.relativePose(p, cp);
			
			return new double[]{ targetPose.position.x, targetPose.position.y, interval};
		}
		
	}
	
}
