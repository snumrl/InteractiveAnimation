package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.MotionAnnotationManager;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.data.trasf.MotionDistByPoints;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.ControlDataGenerator.TimeIntervalControl;
import mrl.motion.neural.param.MotionParamGraph;
import mrl.motion.neural.param.ParamGraphExplorer;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class WalkGraphDataGenerator {
	
	private MDatabase database;
	
	public MotionSegment generateMotion(int size){
		database = BasketDataGenerator.loadBasketData();
		
		MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true){
			protected boolean isValid(MotionAnnotation ann){
				return ann.type.equals("walk");
			}
		};
		MotionParamGraph.DIST_LIMIT = 180;
		MotionParamGraph param = new MotionParamGraph(database){
			protected boolean isInBoundary(Motion m, int margin){
				if (eventAnn.isAnnotated(m.motionData.file.getName(), m.frameIndex)){
					return true;
				}
				return false;
			}
		};
//		param.makeGraph();
//		param.saveGraph("walkGraph.graph");
		param.loadGraph("walkGraph.graph");
		ParamGraphExplorer exp = new ParamGraphExplorer(param);
		
		
		ArrayList<int[]> segments = new ArrayList<int[]>();
		int[] walkCycle = DribbleGraphDataGenerator.findCycle(database, "s_008_4_1.bvh", 1487);
		int[] runCycle = DribbleGraphDataGenerator.findCycle(database, "s_006_7_1.bvh", 1020);
		
		int walkSize = size/5;
		int wCount = walkSize/(walkCycle[1] - walkCycle[0] + 1);
		for (int i = 0; i < wCount; i++) {
			segments.add(walkCycle);
		}
		int rCount = walkSize/(runCycle[1] - runCycle[0] + 1);
		for (int i = 0; i < rCount; i++) {
			segments.add(runCycle);
		}
		segments.addAll(exp.explore(size - walkSize*2));
//		FileUtil.writeObject(segments, "dribble.segment");
//		System.exit(0);
		System.out.println(Arrays.toString(MathUtil.getStatistics(MathUtil.toDouble(exp.visitCounts))));
		
		
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
//		tEdit.timeOffset = 0.35;
		tEdit.timeOffset = 0.15;
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		return segment;
	}
	
	
	public static void main(String[] args) {
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setNoBall();
		MotionSegment segment = new WalkGraphDataGenerator().generateMotion(200000);
//		MainViewerModule.run(new MotionData(segment.getMotionList()));
//		System.exit(0);
//		RNNDataGenerator.generate(segment, new TimeGoalControl(120, true));
//		RNNDataGenerator.generate(segment, new TimeGoalControl(60));
		
		RNNDataGenerator.generate(segment, new TimeIntervalControl(30, false));
	}
}
