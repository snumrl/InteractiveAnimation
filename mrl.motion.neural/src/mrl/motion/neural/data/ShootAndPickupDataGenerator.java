package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;
import java.util.Map.Entry;

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
import mrl.motion.neural.data.ShootAndPickupControl.ActionType;
import mrl.motion.neural.param.MotionParamGraph;
import mrl.motion.neural.param.ParamGraphExplorer;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class ShootAndPickupDataGenerator {
	
	private MDatabase database;
	private MotionAnnotation[] annByFrames;
	private HashMap<MotionAnnotation, Integer> interactionFrameMap = new HashMap<MotionAnnotation, Integer>();
	
	private HashMap<String, HashSet<Integer>> actionFrameMap = new HashMap<String, HashSet<Integer>>(); 
	
	private int[] dribbleCycle;
	private ArrayList<int[]> graphList;
	
	public static int catchBall(String type){
		if (type.equals("pickup") || type.equals("pass_")){
			return 1;
		} else if (type.equals("shoot") || type.equals("pass")){
			return -1;
		}
		return 0;
	}
	
	private int interactionOffset(int mIndex){
		MotionAnnotation ann = annByFrames[mIndex];
		int interFrame = interactionFrameMap.get(ann);
		return mIndex - interFrame;
	}
	
	public MotionSegment generateMotion(int size){
		database = BasketDataGenerator.loadBasketData();
		annByFrames = new MotionAnnotation[database.getMotionList().length];
		MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true){
			protected boolean isValid(MotionAnnotation ann){
				return ann.type.length() > 0;
			}
		};
		
		for (ActionType action : ActionType.values()){
			actionFrameMap.put(action.actionName(), new HashSet<Integer>());
		}
		
		for (MotionAnnotation ann : eventAnn.getTotalAnnotations()){
			int start = database.findMotion(ann.file, ann.startFrame).motionIndex;
			int end = database.findMotion(ann.file, ann.endFrame).motionIndex;
			for (int i = start; i <= end; i++) {
				annByFrames[i] = ann;
			}
			if (ann.interactionFrame > 0){
				int interIdx = start + ann.interactionFrame - ann.startFrame;
				interactionFrameMap.put(ann, interIdx);
				actionFrameMap.get(ann.type).add(interIdx);
			}
		}
		MotionParamGraph.DIST_LIMIT = 250;
		int interactionMargin = 15;
		MotionParamGraph param = new MotionParamGraph(database){
			protected boolean isInBoundary(Motion m, int msargin){
				if (eventAnn.isAnnotated(m.motionData.file.getName(), m.frameIndex)){
					return true;
				}
				return false;
			}
			
			public double dist(int i, int j){
				MotionAnnotation a1 = annByFrames[i];
				MotionAnnotation a2 = annByFrames[j];
				if (a1 == null || a2 == null) return super.dist(i,  j);
				
				int b1 = catchBall(a1.type);
				int b2 = catchBall(a2.type);
				if (b1 == 0){
					if (a1.type.equals("dribble")){
						if (b2 == -1){
							if (interactionOffset(j) > -interactionMargin)  return Integer.MAX_VALUE;
						} else if (!a2.type.equals("dribble")){
							return Integer.MAX_VALUE;
						}
					}
					if (a1.type.equals("walk")){
						if (b2 == 1){
							if (interactionOffset(j) > -interactionMargin)  return Integer.MAX_VALUE;
						} else if (!a2.type.equals("walk")){
							return Integer.MAX_VALUE;
						}
					}
				} else if (b1 == -1){
					if (!a2.type.equals("walk")) return Integer.MAX_VALUE;
					if (interactionOffset(i) < interactionMargin)  return Integer.MAX_VALUE;
				} else if (b1 == 1){
					if (!a2.type.equals("dribble")) return Integer.MAX_VALUE;
					if (interactionOffset(i) < interactionMargin)  return Integer.MAX_VALUE;
				}
				
//				if (a1.type.equals("shoot")){
//					if (!a2.type.equals("walk")) return Integer.MAX_VALUE;
//					if (interactionOffset(i) < interactionMargin)  return Integer.MAX_VALUE;
//				}
//				if (a1.type.equals("pickup")){
//					if (!a2.type.equals("dribble")) return Integer.MAX_VALUE;
//					if (interactionOffset(i) < interactionMargin)  return Integer.MAX_VALUE;
//				}
//				if (a1.type.equals("dribble")){
//					if (a2.type.equals("shoot")){
//						if (interactionOffset(j) > -interactionMargin)  return Integer.MAX_VALUE;
//					} else if (!a2.type.equals("dribble")){
//						return Integer.MAX_VALUE;
//					}
//				}
//				if (a1.type.equals("walk")){
//					if (a2.type.equals("pickup")){
//						if (interactionOffset(j) > -interactionMargin)  return Integer.MAX_VALUE;
//					} else if (!a2.type.equals("walk")){
//						return Integer.MAX_VALUE;
//					}
//				}
				return super.dist(i, j);
			}
		};
//		param.makeGraph();
//		param.saveGraph("shootGraph_pass.graph");
////		param.saveGraph("shootGraph.graph");
//		System.out.println("#############");
//		for (int idx : param.removedSet){
//			if (isInteraction(idx)){
//				System.out.println("removed :: " + getAnn(idx));
//			}
//		}
//		System.out.println("#############");
//		System.exit(0);
		
		param.loadGraph("shootGraph_pass.graph");
//		param.loadGraph("shootGraph.graph");
		ParamGraphExplorer exp = new ParamGraphExplorer(param);
		exp.interactionFrames = new ArrayList<Integer>();
		exp.interactionFrames.addAll(interactionFrameMap.values());
		
		ArrayList<int[]> segments = new ArrayList<int[]>();
		int[] walkCycle = DribbleGraphDataGenerator.findCycle(database, "s_008_4_1.bvh", 1487);
		int[] runCycle = DribbleGraphDataGenerator.findCycle(database, "s_006_7_1.bvh", 1020);
		dribbleCycle = DribbleGraphDataGenerator.findCycle(database, "s_003_1_1.bvh", 281);
		int walkSize = size/18;
//		int walkSize = size/9;
		int wCount = walkSize/(walkCycle[1] - walkCycle[0] + 1);
		for (int i = 0; i < wCount; i++) {
			segments.add(walkCycle);
		}
		int rCount = walkSize/(runCycle[1] - runCycle[0] + 1);
		for (int i = 0; i < rCount; i++) {
			segments.add(runCycle);
		}
		int dCount = walkSize/(dribbleCycle[1] - dribbleCycle[0] + 1);
		for (int i = 0; i < dCount; i++) {
			segments.add(dribbleCycle);
		}
		graphList = exp.explore(size - walkSize*3);
		segments.addAll(graphList);
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
	
	private boolean isInteraction(int mIndex){
		for (HashSet<Integer> set : actionFrameMap.values()){
			if (set.contains(mIndex)) return true;
		}
		return false;
	}
	
	private MotionAnnotation getAnn(int mIndex){
		for (Entry<MotionAnnotation, Integer> entry : interactionFrameMap.entrySet()){
			if (entry.getValue() == mIndex) return entry.getKey();
		}
		return null;
	}
	
	
	public static void main(String[] args) {
//		MathUtil.random = new Random(228183);
		MathUtil.random = new Random(228182);
//		MathUtil.random = new Random();
		MotionDataConverter.setAllJoints();
//		MotionDataConverter.setNoBall();
		ShootAndPickupDataGenerator g = new ShootAndPickupDataGenerator();
		MotionSegment segment = g.generateMotion(350000);
		ShootTimeControl c = new ShootTimeControl(g.database, g.actionFrameMap, g.interactionFrameMap);
//		ShootAndPickupControl c = new ShootAndPickupControl(g.database, g.actionFrameMap, g.interactionFrameMap);
		c.dribbleStart = g.dribbleCycle[0];
		c.graphStart = g.graphList.get(0)[0];
		
		RNNDataGenerator.generate(segment, c);
	}
}
