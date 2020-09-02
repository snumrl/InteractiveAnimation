package mrl.motion.data.trasf;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.util.IterativeRunnable;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class MotionDistByPoints {
	
	public static boolean USE_VELOCITY = true;

	private SkeletonData skeletonData;
	private Motion[] mList;
	private MotionPoints[] mPoints;
	public double unitLen;

	public MotionDistByPoints(SkeletonData skeletonData, final Motion[] mList) {
		this.skeletonData = skeletonData;
		this.mList = mList;
		
		double lenSum = 0;
		for (Joint j : skeletonData.values()){
			if (j.parent == null) continue;
			lenSum += j.length;
		}
		unitLen = lenSum/skeletonData.size();
		unitLen = unitLen/4;
		
		long t = System.currentTimeMillis();
		mPoints = new MotionPoints[mList.length];
		
		ArrayList<ArrayList<Point3d>> posePointList = new ArrayList<ArrayList<Point3d>>();
		for (int i = 0; i < mList.length; i++) {
			posePointList.add(getPosePoints(skeletonData, mList[i], unitLen));
		}
		for (int i = 0; i < mList.length; i++) {
//			int prev = Math.max(0, i - 2);
			int post = Math.min(mList.length-1, i + 2);
			ArrayList<Point3d> current = posePointList.get(i);
			ArrayList<Point3d> next;
			if (mList[i].next == null || mList[i].next.next == null){
				next = posePointList.get(i);
			} else if (mList[i].next.next != mList[post]){
				next = posePointList.get(post);
			} else {
				next = getPosePoints(skeletonData, mList[i].next.next, unitLen);
			}
			Point3d mean = new Point3d();
			ArrayList<Vector3d> velList = new ArrayList<Vector3d>();
			for (int j = 0; j < current.size(); j++) {
				Vector3d v = new Vector3d();
				v.sub(next.get(j), current.get(j));
				velList.add(v);
			}
			for (Point3d p : current){
				mean.add(p);
			}
			mean.scale(1d/current.size());
			mPoints[i] = new MotionPoints(current, velList, mean);
			
//			mPoints[i] = getPoints(skeletonData, mList[i], unitLen);
		}
		
		if (!USE_VELOCITY){
			for (int i = 0; i < mList.length; i++) {
				mPoints[i] = getPoints(skeletonData, mList[i], unitLen);
			}
		}
		
		System.out.println("unit len : " + unitLen + " ,  sample point size " + mPoints[0].points.size()
				 + ",  time : " + (System.currentTimeMillis() - t));
	}
	
	public int size(){
		return mList.length;
	}
	
	public double[][] calcDistAll(final int margin){
		long t = System.currentTimeMillis();
		final double[][] distMap = new double[mList.length][mList.length];
		Utils.runMultiThread(new IterativeRunnable(){
			@Override
			public void run(int i) {
				MotionPoints source = mPoints[i];
				for (int j = i+1; j < mList.length; j++) {
					MotionPoints target = mPoints[j];
					double d = getDistance(source, target);
					distMap[i][j] = distMap[j][i] = d;
				}
			}
		}, mList.length);
		System.out.println("dist time : " + (System.currentTimeMillis() - t));
		
		if (margin == 0) return distMap;
		
		final double[][] seqDistMap = new double[mList.length][mList.length];
		Utils.runMultiThread(new IterativeRunnable(){
			@Override
			public void run(int i) {
				for (int j = 0; j < mList.length; j++) {
					seqDistMap[i][j] = calcSeqDist(distMap, i, j, margin, 0.125);
				}
			}
		}, mList.length);
		return seqDistMap;
	}
	
	public static double calcSeqDist(double[][] distMap, int x, int y, int margin, double damp){
//		if (x == y) return 0;
		double dSum = distMap[x][y];
		double weight = 1;
		
		int p1 = x;
		int p2 = y;
		for (int i = 1; i <= margin; i++) {
			if (p1 > 0) p1--;
			if (p2 > 0) p2--;
			weight += (1-damp*i);
			dSum += distMap[p1][p2]*(1-damp*i);
		}
		
		int n1 = x;
		int n2 = y;
		for (int i = 1; i <= margin; i++) {
			if (n1 < distMap.length-1) n1++;
			if (n2 < distMap.length-1) n2++;
			weight += (1-damp*i);
			dSum += distMap[n1][n2]*(1-damp*i);
		}
		double poseDiff = dSum / weight;
		return poseDiff;
	}
	
	public double getDistance(int i, int j){
		return getDistance(mPoints[i], mPoints[j]);
	}
	
	public static double getDistance(MotionPoints source, MotionPoints target){
		Matrix4d t = getAlignTransform(source, target);
		double sum = 0;
		for (int i = 0; i < source.points.size(); i++) {
			Point3d p1 = source.points.get(i);
			Point3d p2 = new Point3d(target.points.get(i));
			t.transform(p2);
			double d = p1.distance(p2);
			sum += d*d;
//			sum += p1.distance(p2);
			
			if (source.velocitys != null){
				Vector3d v1 = source.velocitys.get(i);
				Vector3d v2 = new Vector3d(target.velocitys.get(i));
				t.transform(v2);
				d = MathUtil.distance(v1, v2);
				sum += d*d*2;
			}
		}
		return sum/(source.points.size()*3);
	}
	
	
	public static Matrix4d getAlignTransform(MotionPoints source, MotionPoints target){
		double w = 1d/source.points.size();
		double wSum = 1;
		
		double v1, v2, v3, v4;
		v1 = v2 = v3 = v4 = 0;
		for (int i = 0; i < source.points.size(); i++) {
			Point3d s = source.points.get(i);
			Point3d t = target.points.get(i);
			v1 += w*(s.x*t.z - t.x*s.z);
			v2 += w*(s.x*t.x + s.z*t.z);
		}
		v3 = (1/wSum)*(source.mean.x*target.mean.z - target.mean.x*source.mean.z);
		v4 = (1/wSum)*(source.mean.x*target.mean.x + source.mean.z*target.mean.z);
		double theta = Math.atan2(v1 - v3, v2 - v4);
		double sin = Math.sin(theta);
		double cos = Math.cos(theta);
		
		double x0 = (1/wSum)*(source.mean.x - target.mean.x*cos - target.mean.z*sin);
		double z0 = (1/wSum)*(source.mean.z + target.mean.x*sin - target.mean.z*cos);
		
		Matrix4d m = new Matrix4d();
		m.rotY(theta);
		m.setTranslation(new Vector3d(x0, 0, z0));
		return m;
	}
	
	
	public static MotionPoints getPoints(SkeletonData skeletonData, Motion m, double unitLen){
		HashMap<String, Point3d> jointPoints = Motion.getPointData(skeletonData, m);
		
		ArrayList<Point3d> points = new ArrayList<Point3d>();
		for (Joint j : skeletonData.values()){
			Point3d p = jointPoints.get(j.name);
			points.add(p);
			
			if (j.parent == null) continue;
			
			int count = (int)Math.round(j.length/unitLen);
			if (count > 1){
				Point3d parentP = jointPoints.get(j.parent.name);
				for (int i = 1; i < count; i++) {
					Point3d sample = new Point3d();
					sample.interpolate(p, parentP, i/(double)count);
					points.add(sample);
				}
			}
		}
		
		Point3d mean = new Point3d();
		for (Point3d p : points){
			mean.add(p);
		}
		mean.scale(1d/points.size());
		
		return new MotionPoints(points, null, mean);
	}
	
	public static ArrayList<Point3d> getPosePoints(SkeletonData skeletonData, Motion m, double unitLen){
		HashMap<String, Point3d> jointPoints = Motion.getPointData(skeletonData, m);
		
		ArrayList<Point3d> points = new ArrayList<Point3d>();
		for (Joint j : skeletonData.values()){
			Point3d p = jointPoints.get(j.name);
			points.add(p);
			
			if (j.parent == null) continue;
			
			int count = (int)Math.round(j.length/unitLen);
			if (count > 1){
				Point3d parentP = jointPoints.get(j.parent.name);
				for (int i = 1; i < count; i++) {
					Point3d sample = new Point3d();
					sample.interpolate(p, parentP, i/(double)count);
					points.add(sample);
				}
			}
		}
		
		return points;
	}
	
	public static class MotionPoints{
		public ArrayList<Point3d> points;
		public ArrayList<Vector3d> velocitys;
		public Point3d mean;
		
		public MotionPoints(ArrayList<Point3d> points,
				ArrayList<Vector3d> velocitys, Point3d mean) {
			this.points = points;
			this.velocitys = velocitys;
			this.mean = mean;
		}
	}
}
