package mrl.motion.neural.walk;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.ControlDataGenerator;
import mrl.motion.position.PositionMotion;

public class WalkControl extends ControlDataGenerator{

	@Override
	public double[] getControl(int index) {
		int interval = 30; // 30frame = 1s
		
		int controlIndex = index + interval;
		if (controlIndex >= mList.size()) return null;
		
		Pose2d p = PositionMotion.getPose(mList.get(index));
		Pose2d cp = PositionMotion.getPose(mList.get(controlIndex));
		Pose2d targetPose = Pose2d.relativePose(p, cp);
		
		return new double[]{ targetPose.position.x, targetPose.position.y };
	}

}
