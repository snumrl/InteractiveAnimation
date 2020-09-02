package mrl.motion.neural.loco;

import javax.vecmath.Point2d;

import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.ControlDataGenerator;
import mrl.motion.position.PositionMotion;

public class IntervalMeanControl extends ControlDataGenerator{

	@Override
	public double[] getControl(int index) {
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
}
