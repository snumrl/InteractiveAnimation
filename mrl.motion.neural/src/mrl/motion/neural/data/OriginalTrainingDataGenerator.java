package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Random;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotationManager;
import mrl.motion.data.MotionData;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.loco.PredictiveParameterControl.DirectionControl;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.util.Configuration;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class OriginalTrainingDataGenerator {
	
	public static boolean isWalk = false;
	
	private MDatabase database;
	
	public OriginalTrainingDataGenerator(){
		this(BasketDataGenerator.loadBasketData());
	}
	public OriginalTrainingDataGenerator(MDatabase database){
		this.database = database;
	}
	
	public ArrayList<MotionSegment> generateMotion(int size, boolean noEdit){
//		MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation", true){
//		MotionAnnotationManager eventAnn = new MotionAnnotationManager("dribbleAnnotation_long", true){
//			protected boolean isValid(MotionAnnotation ann){
//				if (isWalk){
//					return ann.type.equals("walk");
//				} else {
//					return ann.type.equals("dribble");
//				}
//			}
//		};
		MotionAnnotationManager eventAnn = new MotionAnnotationManager(Configuration.TRANSITION_FOLDER, true);
		
		int predictMargin = 30;
		Motion[] mList = database.getMotionList();
		ArrayList<MotionSegment> segmentList = new ArrayList<MotionSegment>();
		int current = 0;
		while (current < mList.length){
			Motion m = mList[current];
			boolean isValid = eventAnn.isAnnotated(m.motionData.file.getName(), m.frameIndex);
			
			if (!isValid){
				current++;
				continue;
			} else {
				int start = current;
				while (current < mList.length){
					m = mList[current];
					isValid = eventAnn.isAnnotated(m.motionData.file.getName(), m.frameIndex);
					if (!isValid) break;
					current++;
				}
				int end = current;
//				int end = current + predictMargin;
				MotionSegment s = new MotionSegment(mList, start, end);
				segmentList.add(s);
			}
		}
		
		if (noEdit){
			for (MotionSegment s : segmentList){
				System.out.println("len :: " + s.length() + " : " + (s.length()-predictMargin));
			}
			return segmentList;
		}
		
		int totalSize = 0;
		for (MotionSegment s : segmentList){
			totalSize += (s.length() - predictMargin);
		}
		int count = size/totalSize;
		count = Math.max(count, 1);
		System.out.println("count ::: " + count + " : " + size + " / " + totalSize);
		System.out.flush();
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
		tEdit.timeOffset = 0.1;
		
		ArrayList<MotionSegment> editedList = new ArrayList<MotionSegment>();
		int totalSum = 0;
		for (int i = 0; i < count; i++) {
			for (MotionSegment s : segmentList){
				int tryCount = 0;
				while(true){
					try{ 
						MotionSegment segment = tEdit.edit(new MotionSegment(s));
//						FootSlipCleanup.clean(segment);
						ArrayList<Motion> motionList = MotionData.divideByKnot(segment.getEntireMotion());
						segment = new MotionSegment(Utils.toArray(motionList), MotionSegment.BLEND_MARGIN(), motionList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
						totalSum += segment.length();
						System.out.println("Edited :: " + segment.length() + " / " + s.length() + " :: " + totalSum);
						editedList.add(segment);
						break;
					} catch (Exception e){
						e.printStackTrace();
						System.out.println("edit fail!! : " + tryCount);
						tryCount++;
						continue;
					}
				}
			}
		}
		return editedList;
	}
	
	
	public static void main(String[] args) {
		MathUtil.random = new Random();
		isWalk = true;
		
//		MotionDataConverter.setAllJoints();
//		MotionDataConverter.setUseOrientation();
		if (isWalk) MotionDataConverter.setNoBall();
		
//		OriginalTrainingDataGenerator g = new OriginalTrainingDataGenerator();
//		TimeIntervalControl c = new TimeIntervalControl(30, false);
//		
//		MathUtil.random = new Random(1462);
//		ArrayList<MotionSegment> segmentList = g.generateMotion(165000, false);
//		RNNDataGenerator.generate(segmentList, c);
		
		Configuration.setDataFolder("locomotion\\" + "edin_loco");
		MotionDataConverter.setCMUJointSet();
		MDatabase database = MDatabase.load();
		
		
		OriginalTrainingDataGenerator g = new OriginalTrainingDataGenerator(database);
		
		MathUtil.random = new Random(1462);
		ArrayList<MotionSegment> segmentList = g.generateMotion(600000, false);
		RNNDataGenerator.generate(segmentList, new DirectionControl(true));
//		RNNDataGenerator.generate(segmentList, new SelectiveDirectionControl());
//		RNNDataGenerator.generate(segmentList, new TimeIntervalControl(30, false));
	}
}
