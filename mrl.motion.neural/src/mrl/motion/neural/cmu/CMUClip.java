package mrl.motion.neural.cmu;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.data.trasf.MotionDistByPoints;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.ControlDataGenerator.TimeIntervalControl;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.util.Utils;

public class CMUClip {

	private HashMap<Motion, Integer> indexMap = new HashMap<Motion, Integer>();
	private ArrayList<Motion> totalMotions = new ArrayList<Motion>();
	private MotionDistByPoints dist;
	private double[][] distMap;
	public MotionSegment walk;
	public MotionSegment walk2;
	private ArrayList<Motion> walkMotions;
	
	private int iter = 0;
	public MotionSegment edited;
	
	public ArrayList<MotionSegment> segments;
	
	public CMUClip(){
		
		walk = walk();
		walkMotions = walk.getMotionList();
		walk2 = new MotionSegment(walk);
		walk2 = MotionSegment.stitch(walk2, new MotionSegment(walk), true);
		
		int[][] clipInfo = new int[][]{
//			{4, 62, 104,},
			{6, 90, 185},
			{22, 29,  86},
			{25, 23, 79}
		};
		segments = new ArrayList<MotionSegment>();
		for (int i = 0; i < clipInfo.length; i++) {
			int[] info = clipInfo[i];
			String file = String.format("walkMotion\\69_%02d_1.bvh", info[0]);
			{
				Motion[] mList = Utils.toArray(CMUToBasket.convert(file, false));
				MotionSegment s = new MotionSegment(mList, info[1], info[2]);
				segments.add(s);
			}
			{
				Motion[] mList = Utils.toArray(CMUToBasket.convert(file, true));
				MotionSegment s = new MotionSegment(mList, info[1], info[2]);
				segments.add(s);
			}
		}
		
		update(walk);
		for (MotionSegment s : segments){
			update(s);
		}
		
	}
	
	public void generate(int size){
		MotionDistByPoints.USE_VELOCITY = false;
		dist = new MotionDistByPoints(SkeletonData.instance, Utils.toArray(totalMotions));
		distMap = dist.calcDistAll(0);
		
		edited = new MotionSegment(walk);
		for (iter = 0; iter < 10000; iter++) {
			for (int i = 0; i < segments.size(); i++) {
				MotionSegment s = segments.get(i);
				MotionSegment prefix = getPrefix(s.firstMotion());
				MotionSegment postfix = getPostfix(s.lastMotion());
				s = MotionSegment.stitch(prefix, new MotionSegment(s), true);
				s = MotionSegment.stitch(s, postfix, true);
				edited = MotionSegment.stitch(edited, s, true);
				if (edited.length() > size) break;
			}
			if (edited.length() > size) break;
		}
	}
	
	private MotionSegment getPrefix(Motion start){
		double min = Integer.MAX_VALUE;
		int minIdx = -1;
		for (int i = 0; i < 36; i++) {
			int wIdx = walkMotions.size() - 1 - i;
			double d = distMap[indexMap.get(start)][indexMap.get(walkMotions.get(wIdx))];
			if (d < min){
				min = d;
				minIdx = wIdx;
			}
		}
		return new MotionSegment(walk, 0, minIdx-1);
	}
	private MotionSegment getPostfix(Motion end){
		double min = Integer.MAX_VALUE;
		int minIdx = -1;
		for (int i = 0; i < 36; i++) {
			int wIdx = i;
			double d = distMap[indexMap.get(end)][indexMap.get(walkMotions.get(wIdx))];
			if (d < min){
				min = d;
				minIdx = wIdx;
			}
		}
		return new MotionSegment(walk2, minIdx+1, walk2.length()-1);
	}
	
	
	
	private void update(MotionSegment s){
		ArrayList<Motion> mList = s.getMotionList();
		for (Motion m : mList){
			indexMap.put(m, totalMotions.size());
			totalMotions.add(m);
		}
	}
	
	private MotionSegment walk(){
//		[50, 85]
		ArrayList<Motion> mList = CMUToBasket.convert("walkMotion\\69_04_1.bvh", false);
		return new MotionSegment(Utils.toArray(mList), 50, 85);
	}
	
	
	void findCycle(ArrayList<Motion> mList){
		MotionDistByPoints dist = new MotionDistByPoints(SkeletonData.instance, Utils.toArray(mList));
		double minDist = Integer.MAX_VALUE;
		int m1 = 50;
		int minIndex = -1;
		for (int i = 15; i < 50; i++) {
			double d = dist.getDistance(m1, m1 + i);
			if (d < minDist){
				minDist = d;
				minIndex = i;
			}
		}
		int[] cycle = new int[]{ m1, m1 + minIndex - 1 };
		System.out.println("cycle :: " + Arrays.toString(cycle));
	}
	
	
	public static void main(String[] args) {
		int size = 200000;
		CMUClip clip = new CMUClip();
		clip.generate(size);
		MotionSegment segment = clip.edited;
		
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
//		tEdit.timeOffset = 0.35;
		tEdit.timeOffset = 0.3;
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		MotionDataConverter.setNoBall();
		RNNDataGenerator.generate(segment, new TimeIntervalControl(45, false));
//		MainViewerModule.run(new MotionData(edited.getMotionList()));
		
	}
}
