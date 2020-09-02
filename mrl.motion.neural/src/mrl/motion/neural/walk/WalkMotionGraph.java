package mrl.motion.neural.walk;

import java.util.ArrayList;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MGraph;
import mrl.motion.graph.MGraphExplorer;
import mrl.motion.graph.MGraphGenerator;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.Configuration;
import mrl.util.Utils;

public class WalkMotionGraph {
	
	static void printJointList(MDatabase database){
		System.out.println("--");
		Motion m = database.getMotionList()[0];
		for (String key : SkeletonData.keyList){
			if (m.containsKey(key)){
				System.out.println(key);
			}
		}
		System.exit(0);
	}

	public static void main(String[] args) {
		Configuration.setDataFolder("walkTest");
//		MotionDataConverter.setCMUJointSet();
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setNoBall();
		MDatabase database = MDatabase.load();
//		printJointList(database);
//		Configuration.BASE_MOTION_FILE = "walkTest\\motion\\69_01_1.bvh";
		MotionDataConverter.setUseOrientation();
		
		// generate new motion graph and save to files.
		MGraphGenerator g = new MGraphGenerator(database);
		g.saveResult();
		
		// load motion graph from saved files.
		MGraph graph = new MGraph(database);
		MGraphExplorer exp = new MGraphExplorer(graph);
		ArrayList<int[]> segments = exp.explore(1000);
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		
		// show generated motion
		MotionData mData = new MotionData(segment.getMotionList());
		MainViewerModule.run(mData);
		
		
		RNNDataGenerator.generate(segment, new WalkControl());
	}
}
