package mrl.motion.neural.play;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.Utils;

public class OgreSceneViewer {
	
	
	private static int minIndex(ArrayList<Point3d> ballTrajectory, int start, int end){
		int minIdx = -1;
		double minHeight = Integer.MAX_VALUE;
		int mid = (start + end)/2;
		int margin = (end - start)/4;
		for (int i = mid-margin; i <= mid+margin; i++) {
			if (ballTrajectory.get(i).y < minHeight){
				minHeight = ballTrajectory.get(i).y;
				minIdx = i;
			}
		}
		return minIdx;
	}

	public static void main(String[] args) {
		OgreReplay replay = new OgreReplay("dribble_new5");
		ArrayList<Motion> mList = new ArrayList<Motion>();
		ArrayList<Point3d> ballList = new ArrayList<Point3d>();
		ArrayList<Point3d> ballList2 = new ArrayList<Point3d>();
		int[] frames = { 465, 485, 501, 511 };
//		int[] frames = { 445, 465, 485, 501, 511, 525, 545 };
//		int[] frames = { 445, 465, 485, 499,501,503, 510 };
		
		for (int i = 0; i < frames.length; i++) {
			int idx = frames[i];
			mList.add(replay.mDataList.get(0).motionList.get(idx));
			ballList.add(replay.ballTrajectory.get(idx));
			if (i > 0){
				int prev = frames[i-1];
				int minIdx = minIndex(replay.ballTrajectory, prev, idx);
				
				Point3d mp = new Point3d(replay.ballTrajectory.get(minIdx));
				if (idx == 501){
					mp.add(new Point3d(50, 0, -40));
				}
				Point3d p0 = new Point3d(replay.ballTrajectory.get(prev));
				Point3d p1 = new Point3d(replay.ballTrajectory.get(idx));
				p0.interpolate(p0, mp, 0.5);
				p1.interpolate(p1, mp, 0.5);
				
				ballList2.add(mp);
				ballList2.add(p0);
				ballList2.add(p1);
//				ballList.add(replay.ballTrajectory.get((minIdx + prev)/2));
//				ballList.add(replay.ballTrajectory.get((minIdx + idx)/2));
			}
		}
		
//		for (int i = frames[0]; i < frames[frames.length-1]; i+=3) {
//			ballList2.add(replay.ballTrajectory.get(i));
//		}
//		int interval = 20;
//		int start = frames[frames.length-1] + interval;
//		int end = 686;
//		for (int i = start; i <= end; i+=interval) {
//			mList.add(replay.mDataList.get(0).motionList.get(i));
//			ballList.add(replay.ballTrajectory.get(i));
//		}
		
		OgreJNI.open(new double[]{ 0, 1 });
		OgreJNI.setMotion(Utils.toArray(mList));
		OgreJNI.setBall(ballList);
		OgreJNI.setBall2(ballList2);
		
		OgreJNI.waitForOgreClose();
	}
	
}
