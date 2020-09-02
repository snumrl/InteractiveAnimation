package mrl.motion.neural.loco;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.ControlDataGenerator;
import mrl.motion.position.PositionMotion;

public class PredictiveParameterControl extends ControlDataGenerator{

	private double[] prediction;
	private boolean useMean = false;
	
	public PredictiveParameterControl(boolean useMean) {
		this.useMean = useMean;
	}

	@Override
	public double[] getControl(int index) {
		int interval = 30; // 30frame = 1s
		
		int controlIndex = index + interval;
		if (controlIndex >= mList.size()) return null;
		
		if (useMean){
			prediction = getMeanControl(controlIndex);
		} else {
			Pose2d p = PositionMotion.getPose(mList.get(index));
			Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
			Pose2d targetPose = Pose2d.relativePose(p, cp);
			prediction = new double[]{ targetPose.position.x, targetPose.position.y };
		}
		return prediction;
	}
	
	private double[] getMeanControl(int index){
		int intervalStart = 20; // 30frame = 1s
		int intervalEnd = 50; // 30frame = 1s
		
		if (index + intervalEnd >= mList.size()) return null;
		
		Point2d pSum = new Point2d();
		for (int i = intervalStart; i < intervalEnd; i++) {
			int idx = index + i;
			pSum.add(PositionMotion.getPose(mList.get(idx)).position);
		}
		pSum.scale(1d/(intervalEnd - intervalStart));
		
		Pose2d p = PositionMotion.getPose(mList.get(index));
		Point2d target = p.globalToLocal(pSum);
		return new double[]{ target.x, target.y };
	}

	public double[] getPrediction(int index){
		return prediction;
	}
	
	
	public static class DirectionControl extends ControlDataGenerator{
		
		private boolean usePrediction = false;
		private double[] prediction;
		
		
		public DirectionControl(boolean usePrediction) {
			this.usePrediction = usePrediction;
		}

		@Override
		public double[] getControl(int index) {
			int interval = 30; // 30frame = 1s
			
			int controlIndex = index + interval;
			if (controlIndex >= mList.size()) return null;
			
			Pose2d p = PositionMotion.getPose(mList.get(index));
			Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
			Pose2d targetPose = Pose2d.relativePose(p, cp);
			
			Vector2d direction = new Vector2d(targetPose.direction);
			if (usePrediction){
				prediction = new double[]{ direction.x, direction.y };
			}
			return new double[]{ targetPose.position.x, targetPose.position.y, direction.x, direction.y};
		}
		
		
		public double[] getPrediction(int index){
			return prediction;
		}
	}
}
