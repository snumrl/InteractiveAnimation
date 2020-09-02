package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

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
import mrl.util.MathUtil;
import mrl.util.Utils;

public class DribbleGraphDataGenerator {
	
	public static boolean isWalk = false;
	private static boolean makeGraph = false;
	
	private MDatabase database;
	
	public static  int[] findCycle(MDatabase database, String file, int frame){
		int m1 = database.findMotion(file, frame).motionIndex;
		MotionDistByPoints dist = database.getDist();
		double minDist = Integer.MAX_VALUE;
		int minIndex = -1;
		for (int i = 15; i < 50; i++) {
			double d = dist.getDistance(m1, m1 + i);
			if (d < minDist){
				minDist = d;
				minIndex = i;
			}
		}
		int[] cycle = new int[]{ m1, m1 + minIndex - 1 };
		System.out.println("cycle :: " + database.getMotionList()[cycle[0]] + " , " + database.getMotionList()[cycle[1]]);
		return cycle;
	}
	
	public DribbleGraphDataGenerator(){
		this(BasketDataGenerator.loadBasketData());
	}
	public DribbleGraphDataGenerator(MDatabase database){
		this.database = database;
	}

	public MotionSegment generateMotion(int size){
		return generateMotion(size, false);
	}
	public MotionSegment generateMotion(int size, boolean noEdit){
//		MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true){
		MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation_long", true){
			protected boolean isValid(MotionAnnotation ann){
				if (isWalk){
					return ann.type.equals("walk");
				} else {
					return ann.type.equals("dribble");
				}
//				return ann.type.length() == 0;
			}
		};
		MotionParamGraph.DIST_LIMIT = 400;
//		MotionParamGraph.DIST_LIMIT = 250;
		MotionParamGraph param = new MotionParamGraph(database){
			protected boolean isInBoundary(Motion m, int margin){
				if (eventAnn.isAnnotated(m.motionData.file.getName(), m.frameIndex)){
					return true;
				}
				return false;
			}
		};
		String graphFile = isWalk ? "walkGraph.graph" :  "dribbleGraph.graph";
		if (makeGraph){
			param.makeGraph();
//			param.saveGraph(graphFile);
		} else {
			param.loadGraph(graphFile);
		}
		System.out.println("nodes :: " + param.nodeList().size());
//		System.exit(0);
		ParamGraphExplorer exp = new ParamGraphExplorer(param);
		ArrayList<int[]> segments = new ArrayList<int[]>();
		int[] cycle1 = findCycle(database, "s_003_1_1.bvh", 281);
		int walkSize = size/4;
		walkSize = 0;
//		int walkSize = 50000;
		int count = walkSize/(cycle1[1] - cycle1[0] + 1);
		for (int i = 0; i < count; i++) {
			segments.add(cycle1);
		}
		
		segments.addAll(exp.explore(size - walkSize));
//		FileUtil.writeObject(segments, "dribble.segment");
//		System.exit(0);
		System.out.println(Arrays.toString(MathUtil.getStatistics(MathUtil.toDouble(exp.visitCounts))));
		
		
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		if (noEdit) return segment;
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
//		tEdit.timeOffset = 0.35;
		tEdit.timeOffset = 0.1;
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		return segment;
	}
	
	
	public static void main(String[] args) {
//		{
////			MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true);
//			MotionAnnotationManager eventAnn = new MotionAnnotationManager("locomotion\\indianDance\\transition", true);
//			int sum = 0;
//			for (MotionAnnotation ann : eventAnn.getTotalAnnotations()){
//					sum += ann.endFrame - ann.startFrame + 1;
//			}
//			System.out.println("Sum : " + sum);
//			System.out.println("Sum : " + sum/30/60d);
//			System.exit(0);
//		}
		
		MDatabase database = BasketDataGenerator.loadBasketData();
		boolean[] iscontain = new boolean[database.getMotionList().length];
		{
//			MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true);
			MotionAnnotationManager eventAnn = new MotionAnnotationManager("basketData\\annotation", true);
			int sum = 0;
			for (MotionAnnotation ann : eventAnn.getTotalAnnotations()){
				int m1 = database.findMotion(ann.file, ann.startFrame).motionIndex;
				int length = ann.endFrame - ann.startFrame+1;
				for (int i = 0; i < length; i++) {
					iscontain[m1 + i] = true;
				}
//				if (ann.type.equals("dribble")){
//					System.out.println(ann);
					sum += ann.endFrame - ann.startFrame + 1;
//				}
			}
			System.out.println("Sum : " + sum);
			System.out.println("Sum : " + sum/30/60d);
		}
		{
			MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true);
//			MotionAnnotationManager eventAnn = new MotionAnnotationManager("basketData\\annotation", true);
			int sum = 0;
			for (MotionAnnotation ann : eventAnn.getTotalAnnotations()){
				int m1 = database.findMotion(ann.file, ann.startFrame).motionIndex;
				int length = ann.endFrame - ann.startFrame+1;
				for (int i = 0; i < length; i++) {
					iscontain[m1 + i] = true;
				}
//				if (ann.type.equals("dribble")){
//					System.out.println(ann);
				sum += ann.endFrame - ann.startFrame + 1;
//				}
			}
			System.out.println("Sum : " + sum);
			System.out.println("Sum : " + sum/30/60d);
		}
		int cc = 0;
		for (int i = 0; i < iscontain.length; i++) {
			if (iscontain[i]) cc++;
		}
		System.out.println("ccc : " + cc);
//		System.exit(0);
		
//		{
//			MDatabase db = BasketDataGenerator.loadBasketData();
//			ArrayList<Motion> mList = db.getMotionDataList()[0].motionList;
//			for (int i = 0; i < 3; i++) {
//				System.out.println("######################## " + i);
//				Motion m = mList.get(i);
//				PositionMotion pm = new PositionMotion(m);
//				HashMap<String, Point3d> pd = Motion.getPointData(SkeletonData.instance, PositionMotion.getAlignedMotion(m));
//				String[] keys = pd.keySet().toArray(new String[0]);
//				Arrays.sort(keys);
//				for (String key : keys){
//					System.out.println(key + " : " + pd.get(key) + " : " + pm.pointData.get(key));
//				}
//			}
//			System.exit(0);
//		}
		MathUtil.random = new Random();
		isWalk = false;
//		isWalk = true;
//		makeGraph = true;
		makeGraph = false;
		ParamGraphExplorer.MAX_NON_INTERACTION_LIMIT = Integer.MAX_VALUE;
		
		MotionDataConverter.setAllJoints();
		MotionDataConverter.setUseOrientation();
		if (isWalk) MotionDataConverter.setNoBall();
//		RNNDataGenerator.USE_VELOCITY = true;
		
		
		MathUtil.random = new Random(14623);
//		MotionSegment segment = new DribbleGraphDataGenerator().generateMotion(1000);
//		MotionSegment segment = new DribbleGraphDataGenerator().generateMotion(165000, true);
		MotionSegment segment = new DribbleGraphDataGenerator().generateMotion(1000);
//		MainViewerModule.run(new MotionData(segment.getMotionList()));
//		System.exit(0);
//		RNNDataGenerator.setResidualWithInput();
//		ParameterBar.ZERO_BASE = true;
//		RNNDataGenerator.generate(segment, new TimeIntervalBarControl(30, false));
		
//		ControlDataGenerator.includePrediction = true;
//		RNNDataGenerator.generate(segment, new DirectionControl(true));
//		RNNDataGenerator.generate(segment, new SelectiveDirectionControl());
		RNNDataGenerator.generate(segment, new TimeIntervalControl(30, false));
//		RNNDataGenerator.generate(segment, new TimeGoalControl(120, true));
		
//		"rm", "rh", "ho", "ha", "hc"
//		RNNDataGenerator.generate(segment, new DribbleQualityControl("hc"));
		
//		RNNDataGenerator.generate(segment, new TimeIntervalControl(30, false), true);
//		MotionDataConverter.setNoBallVelocity();
//		RNNDataGenerator.generate(segment, new TimeIntervalControl(30, false));
		
		
	}
}
