package mrl.motion.neural.walk;

import java.util.Arrays;

import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.neural.tennis.TennisDataGenerator;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class WalkTrainingDataViewer extends Module{

	@Override
	protected void initializeImpl() {
//		MotionDataConverter.setCMUJointSet();
		MotionDataConverter.setAllJoints();
//		TennisDataGenerator.setTennisJointSet();
		MotionDataConverter.setNoBall();
		MotionDataConverter.setUseOrientation();
		
		
		Normalizer normal = new Normalizer(null);
		RuntimeMotionGenerator g = new RuntimeMotionGenerator();
		PositionResultMotion pm = new PositionResultMotion();
		TimeBasedList<Pose2d> targetList = new TimeBasedList<Pose2d>();
		
		
		
		int offset = 0;
		int len = 10000;
		for (int i = offset; i < offset+len; i++) {
			double[] y = normal.yList.get(i);
			y = normal.deNormalizeY(y);
			
			double[] x = normal.xList.get(i);
			x = normal.deNormalizeX(x);
			
			Pose2d p = g.pose.localToGlobal(new Pose2d(x[0], x[1], 1, 0));
			targetList.add(p);
			
			PositionResultMotion motion = g.update(y);
			pm.add(motion.get(0));
		}
		
		
		getModule(MainViewerModule.class);
		getModule(ItemListModule.class).addSingleItem("motion", pm);
		getModule(ItemListModule.class).addSingleItem("target", targetList);
	}

	public static void main(String[] args) {
		MainApplication.run(new WalkTrainingDataViewer());
	}
}
