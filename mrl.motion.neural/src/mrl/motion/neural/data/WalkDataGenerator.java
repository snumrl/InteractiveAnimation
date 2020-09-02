package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.ControlDataGenerator.TimeIntervalControl;
import mrl.motion.neural.param.MotionParamGraph;
import mrl.motion.neural.param.ParamGraphExplorer;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class WalkDataGenerator {

	public MotionSegment generateMotion(){
		int size = 200000;
		BVHParser.MAX_FRAME = 300;
		MDatabase database = RNNDataGenerator.loadCMUDatabase("walk");
		MotionParamGraph param = new MotionParamGraph(database);
		ParamGraphExplorer exp = new ParamGraphExplorer(param);
		ArrayList<int[]> segments = exp.explore(size);
		System.out.println(Arrays.toString(MathUtil.getStatistics(MathUtil.toDouble(exp.visitCounts))));
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam(); 
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		return segment;
	}
	
	public MotionSegment generateMotion2(int size){
		MDatabase database = RNNDataGenerator.loadCMUDatabase("walkData\\motion");
		MotionParamGraph.DIST_LIMIT = 220;
		MotionParamGraph param = new MotionParamGraph(database){
			protected boolean isInBoundary(Motion m, int margin){
				margin = 5;
				if (m.frameIndex < margin || m.frameIndex + margin >= m.motionData.motionList.size()) return false;
				return true;
			}
		};
		param.makeGraph();
		param.saveGraph("walkGraph_test.graph");
//		param.loadGraph("walkGraph_test.graph");
		
		ParamGraphExplorer exp = new ParamGraphExplorer(param);
		ArrayList<int[]> segments = exp.explore(size);
		System.out.println(Arrays.toString(MathUtil.getStatistics(MathUtil.toDouble(exp.visitCounts))));
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 60;
		TrajectoryEditParam.CUT_MAX = 120;
		TrajectoryEditParam tEdit = new TrajectoryEditParam(); 
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		return segment;
	}
	
	
	public static void main(String[] args) {
		{
			int sum = 0;
			MDatabase database = RNNDataGenerator.loadCMUDatabase("walkData\\motion");
			Motion m1 = database.findMotion("16_36_1.bvh", 46);
			Motion m2 = database.findMotion("16_37_1.bvh", 13);
			MotionSegment s1 = new MotionSegment(database.getMotionList(), m1.motionIndex-20, m1.motionIndex);
			MotionSegment s2 = new MotionSegment(database.getMotionList(), m2.motionIndex, m2.motionIndex + 20);
			
			MotionSegment s = MotionSegment.stitch(s1, s2, true);
			MainViewerModule.run(new MotionData(s.getMotionList()));
			System.exit(0);
		}
		
		
		MotionSegment segment = new WalkDataGenerator().generateMotion2(10000);
//		MotionSegment segment = new WalkDataGenerator().generateMotion();
		TimeIntervalControl c = new TimeIntervalControl(30, false);
		RNNDataGenerator.generate(segment, c);
	}
}
