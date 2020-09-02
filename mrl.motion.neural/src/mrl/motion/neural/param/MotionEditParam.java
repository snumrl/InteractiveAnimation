package mrl.motion.neural.param;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.jni.motion.Constraint;
import mrl.jni.motion.MotionEditJNI;
import mrl.jni.motion.RootTrajectory;
import mrl.jni.motion.RootTrajectory.RootInfo;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.position.PositionMotion;
import mrl.util.Configuration;
import mrl.util.MathUtil;
import mrl.util.Matrix2d;
import mrl.util.Utils;

public class MotionEditParam {
	
	public static int TIME_WINDOW = 48;
	public static boolean DO_TIME_EDIT = true;
	
	private MDatabase database;
	private Motion[] mList;
	private Random rand = new Random(1324);
	
	private int _mIdx = 5;
	
	public MotionEditParam() {
		database = RNNDataGenerator.loadCMUDatabase("walk");
		mList = database.getMotionList();
	}
	
	public MotionSegment getDoubleWalkMotion(MotionSegment walk){
		double[] target = getRandomTarget(160/48d*55 - 10, 40);
		int f1 = _mIdx;
		int f2 = _mIdx + 55 - 1;
		MotionSegment segment = getEditedSegment(walk, f1, f2, target);
		segment = new MotionSegment(segment, f1, f2);
		_mIdx = (f2 + 1)%35;
		if (_mIdx < 5) _mIdx += 35; 
		
		return segment;
	}
	
	private double[] getRandomTarget(double len, double lenMargin){
		double length = len + lenMargin*(rand.nextDouble()-0.5)*2;
		double angle = Math.toRadians(60)*(rand.nextDouble()-0.5)*2;
		Vector2d v = new Vector2d(length, 0);
		v = MathUtil.rotate(v, angle);
		angle = angle*1.5;
		return new double[]{ angle, v.x, v.y };
	}
	
	public MotionSegment getDoubleWalkMotions(int len){
		MotionSegment segment = getWalkMotion();
		MotionSegment current = getDoubleWalkMotion(segment);
		for (int j = 0; j < len; j++) {
			current = MotionSegment.stitch(current, getDoubleWalkMotion(segment), true);
		}
		FootSlipCleanup.clean(current);
		System.out.println("frames :: " + current.getMotionList().size());
		return current;
	}
//	public ArrayList<MotionSegment> getDoubleWalkMotions(int sizeLimit){
//		MotionSegment segment = getWalkMotion();
//		ArrayList<double[]> paramList = getWalkParameters();
//		ArrayList<MotionSegment> list = new ArrayList<MotionSegment>();
//		double rotMax = 0;
//		for (int mIdx = 5; mIdx < 40; mIdx+=5) {
//			for (int pIdx1 = 0; pIdx1 < paramList.size(); pIdx1++) {
//				double[] param1 = MathUtil.copy(paramList.get(pIdx1));
//				param1[0] += (rand.nextDouble()-0.5)*2 * Math.toRadians(15);
//				param1[1] += (rand.nextDouble()-0.5)*2 * 10;
//				param1[2] += (rand.nextDouble()-0.5)*2 * 15;
//				rotMax = Math.max(rotMax, param1[0]);
//				
//				int f1 = mIdx;
//				int f2 = mIdx + 70 - 1;
//				MotionSegment data1 = getEditedSegment(segment, f1, f2, param1[0], param1[1], param1[2]);
//				data1 = new MotionSegment(data1, f1, f2);
//				
//				for (int pIdx2 = 0; pIdx2 < paramList.size(); pIdx2++) {
//					double[] param2 = MathUtil.copy(paramList.get(pIdx2));
//					if (param2[1] != 160) continue;
//					param2[0] += (rand.nextDouble()-0.5)*2 * Math.toRadians(15);
//					param2[1] += (rand.nextDouble()-0.5)*2 * 10;
//					param2[2] += (rand.nextDouble()-0.5)*2 * 15;
//					
//					double rotDiff = param1[0] - param2[0];
//					if (Math.abs(rotDiff) < Math.toRadians(70)){
//						int f21 = f2 - 35;
//						int f22 = f21 + GANDataGenerator.TIME_WINDOW - 1;
//						MotionSegment data2 = getEditedSegment(segment, f21, f22, param2[0], param2[1], param2[2]);
//						data2 = new MotionSegment(data2, f21, f22);
//						
//						MotionSegment copy = new MotionSegment(data1);
//						copy = MotionSegment.stitch(copy, data2, true);
//						FootSlipCleanup.clean(copy);
//						list.add(copy);
//						if (sizeLimit > 0 && list.size() >= sizeLimit){
//							return list;
//						}
//					}
//				}
//			}
//		}
//		System.out.println("rot max :: " + rotMax + " : " + Math.toDegrees(rotMax));
//		return list;
//	}
	
	public ArrayList<MotionSegment> getWalkMotions(){
		MotionSegment segment = getWalkMotion();
		ArrayList<double[]> paramList = getWalkParameters();
		ArrayList<MotionSegment> list = new ArrayList<MotionSegment>();
		for (int mIdx = 5; mIdx < 40; mIdx++) {
			for (int i = 0; i < paramList.size(); i++) {
				double[] param = MathUtil.copy(paramList.get(i));
				param[0] += (rand.nextDouble()-0.5)*2 * Math.toRadians(15);
				param[1] += (rand.nextDouble()-0.5)*2 * 10;
				param[2] += (rand.nextDouble()-0.5)*2 * 15;
				
				int f1 = mIdx;
				int f2 = mIdx + TIME_WINDOW - 1;
				MotionSegment data = getEditedSegment(segment, f1, f2, param[0], param[1], param[2]);
				data = new MotionSegment(data, f1, f2);
				FootSlipCleanup.clean(data);
				list.add(data);
			}
		}
		return list;
	}
	
	private ArrayList<double[]> getWalkParameters(){
		ArrayList<double[]> list = new ArrayList<double[]>();
		// 160
		int xSize = 2;
		int ySize = 5;
		double xMargin = 20;
		double yMargin = 30;
		for (int xIdx = -xSize; xIdx <= xSize; xIdx++) {
			if (xIdx == xSize) continue;
			for (int yIdx = -ySize; yIdx <= ySize; yIdx++) {
				double tx = 160 + xMargin*xIdx;
				double ty = yMargin*yIdx;
				double angle = Math.atan2(ty, tx);
				angle = angle*2;
				list.add(new double[]{ angle, tx, ty });
			}
		}
		return list;
	}
	
	public MotionSegment getRandomTurnMotion(){
		Point2d base = new Point2d(-14.2, 0);
		MotionSegment motion = getTurnInPlace();
		double x = 7 * (rand.nextDouble()-0.5)*2;
		double y = 2 * (rand.nextDouble()-0.5)*2;
		Point2d p = new Point2d(-14.2 + x, 10 + y);
		return getEditedSegment(motion, 1, motion.getMotionList().size()-2, getPoseAngle(base, p), p.x , p.y);
	}
	
	public ArrayList<MotionSegment> getTurnMotions(){
		Point2d base = new Point2d(-14.2, 0);
		MotionSegment motion = getTurnInPlace();
		double[] xList = { -7.5, -5, -2.5, 0, 2.5, 5, 7.5 };
		double[] yList = { -2.5, 0, 2.5, 5, 7.5 };
		ArrayList<MotionSegment> list = new ArrayList<MotionSegment>();
		for (double x : xList){
			for (double y : yList){
				Point2d p = new Point2d(-14.2 + x, 10 + y);
				MotionSegment segment = getEditedSegment(motion, 1, motion.getMotionList().size()-2, getPoseAngle(base, p), p.x , p.y);
				list.add(segment);
			}
		}
		return list;
	}
	
	public ArrayList<MotionSegment> getWalkStopMotions(){
		MotionSegment motion = getWalkStopMotion();
		MDatabase.applyDistortionForEdit(motion.getEntireMotion());
		
		ArrayList<MotionSegment> list = new ArrayList<MotionSegment>();
		int xSize = 2;
		int ySize = 4;
		double xMargin = 15;
		double yMargin = 20;
		for (int xIdx = -xSize; xIdx <= xSize; xIdx++) {
			for (int yIdx = -ySize; yIdx <= ySize; yIdx++) {
				double tx = 112 + xMargin*xIdx;
				double ty = yMargin*yIdx;
				double angle = Math.atan2(ty, tx);
				angle = angle*1.2;
				MotionSegment segment = getEditedSegment(motion, 1, motion.getMotionList().size()-2, angle, tx , ty);
				FootSlipCleanup.clean(segment);
				list.add(segment);
			}
		}
		return list;
	}
	
	public MotionSegment getRandomWalkStopMotions(){
		MotionSegment motion = getWalkStopMotion();
		MDatabase.applyDistortionForEdit(motion.getEntireMotion());
		
		double[] target = getRandomTarget(112, 30);
		MotionSegment segment = getEditedSegment(motion, 1, motion.getMotionList().size()-2, target);
		FootSlipCleanup.clean(segment);
		return segment;
	}
	
	public MotionSegment getRandomStopWalkMotions(){
		MotionSegment motion = getStopWalkMotion();
		MDatabase.applyDistortionForEdit(motion.getEntireMotion());
		
		double[] target = getRandomTarget(136, 30);
		MotionSegment segment = getEditedSegment(motion, 1, motion.getMotionList().size()-2, target);
		FootSlipCleanup.clean(segment);
		return segment;
	}
	
	public ArrayList<MotionSegment> getStopWalkMotions(){
		MotionSegment motion = getStopWalkMotion();
		MDatabase.applyDistortionForEdit(motion.getEntireMotion());
		
		ArrayList<MotionSegment> list = new ArrayList<MotionSegment>();
		int xSize = 2;
		int ySize = 4;
		double xMargin = 15;
		double yMargin = 20;
		for (int xIdx = -xSize; xIdx <= xSize; xIdx++) {
			
			if (xIdx == xSize) break;
			
			for (int yIdx = -ySize; yIdx <= ySize; yIdx++) {
				double tx = 136 + xMargin*xIdx;
				double ty = yMargin*yIdx;
				double angle = Math.atan2(ty, tx);
				angle = angle*1.2;
				MotionSegment edited = getEditedSegment(motion, 1, motion.getMotionList().size()-2, angle, tx , ty);
				FootSlipCleanup.clean(edited);
				list.add(edited);
			}
		}
		return list;
	}
	
	
	private double getPoseAngle(Point2d base, Point2d p){
		Vector2d v = MathUtil.sub(p, base);
		return Math.atan2(v.y, v.x);
	}
	
	public MotionSegment getTurnInPlace(){
		int idx1 = database.findMotion("69_16_1.bvh", 45).motionIndex;
		int idx2 = database.findMotion("69_16_1.bvh", 110).motionIndex;
		return new MotionSegment(mList, idx1, idx2);
	}
	
	private MotionSegment getWalkMotion(){
		int pivotIdx = 60;
		int matchIdx = 95;
		MotionSegment s1 = new MotionSegment(mList, pivotIdx, matchIdx - 1);
		MotionSegment s2 = new MotionSegment(mList, pivotIdx, matchIdx - 1);
		MotionSegment s3 = new MotionSegment(mList, pivotIdx, matchIdx - 1);
		s1 = MotionSegment.stitch(s1, s2, true);
		s1 = MotionSegment.stitch(s1, s3, true);
		return s1;
	}
	
	public MotionSegment getStandMotion(){
		ArrayList<Motion> stand = new ArrayList<Motion>();
		ArrayList<Motion> sample = Utils.toList(mList, 1, 18);
		stand = Utils.copy(sample);
		for (int i = 0; i < 6; i++) {
			stand = MotionSegment.stitchByInterpolation(stand, sample, false);
		}
		return new MotionSegment(MotionData.linkMotionList(stand), 18, 90);
	}
	
	public MotionSegment getStopWalkMotion(){
		ArrayList<Motion> walk = Utils.toList(mList, 1, 108);
		ArrayList<Motion> sample = Utils.toList(mList, 1, 18);
		ArrayList<Motion> result = MotionSegment.stitchByInterpolation(sample, sample, false);
		result = MotionSegment.stitchByInterpolation(result, walk, false);
		return new MotionSegment(MotionData.linkMotionList(result), 40, 115);
	}
	
	public MotionSegment getWalkStopMotion(){
		ArrayList<Motion> walk = Utils.toList(mList, 40, 114);
//		ArrayList<Motion> walk = Utils.toList(mList, 40, 78);
		ArrayList<Motion> stop = getStandMotion().getMotionList();
		ArrayList<Motion> result = MotionSegment.stitchByInterpolation(walk, stop, true, 6, 0);
		for (int i = 0; i < result.size(); i++) {
			result.get(i).knot = i;
		}
		
		double ratio = 0.6;
		double[] knotList = new double[(int)(12/ratio)];
		for (int i = 0; i < knotList.length; i++) {
			knotList[i] = walk.size()-10 + i*ratio;
		}
		ArrayList<Motion> timeArranged = MotionData.timeInterpolation(result, knotList);
		int idx1 = (int)timeArranged.get(0).knot-1;
		int idx2 = (int)Utils.last(timeArranged).knot+1;
		
		ArrayList<Motion> arranged = new ArrayList<Motion>();
		arranged.addAll(Utils.cut(result, 0, idx1));
		arranged.addAll(timeArranged);
		arranged.addAll(Utils.cut(result, idx2, result.size()-1));
		System.out.println("size :: " + Utils.toString(walk.size(), stop.size(), result.size(), idx1, idx2));
		for (int i = 70; i < 78; i++) {
			arranged.get(i).isLeftFootContact = false;
		}
		return new MotionSegment(MotionData.linkMotionList(arranged), 38, 100);
//		return result;
	}
	
	
	private static MotionSegment getEditedSegment(MotionSegment segment, int f1, int f2, double[] target){
		return getEditedSegment(segment, f1, f2, target[0], target[1], target[2]);
	}
	
	private static MotionSegment getEditedSegment(MotionSegment segment, int f1, int f2, double rot, double tx, double ty){
		return getEditedSegment(segment, f1, f2, Pose2d.byBase(new Matrix2d(rot, tx, ty)));
	}
	
	public static MotionSegment getEditedSegment(MotionSegment segment, int f1, int f2, Pose2d p2){
		return getEditedSegment(segment, f1, f2, p2, -1);
	}
	public static MotionSegment getEditedSegment(MotionSegment segment, int f1, int f2, Pose2d p2, int timeLen){
		Pose2d p1 = Pose2d.BASE;
		
		ArrayList<Constraint> constraints = new ArrayList<Constraint>();
		Constraint c1 = new Constraint();
		c1.constraintPersons = new int[]{ 0, -1 };
		c1.constraintFrames = new int[]{ f1, (DO_TIME_EDIT && timeLen >= 0) ? 0 : -100000 };
		c1.posConstraint = getPosConstraint(segment, f1, p1);
		constraints.add(c1);
		Constraint c2 = new Constraint();
		c2.constraintPersons = new int[]{ 0, -1 };
		c2.constraintFrames = new int[]{ f2, (DO_TIME_EDIT && timeLen >= 0) ? timeLen : -100000 };
		c2.posConstraint = getPosConstraint(segment, f2, p2);
		constraints.add(c2);
		
		return editByRoot(segment, constraints);
	}
	
	private static MotionSegment editByRoot(MotionSegment segment, ArrayList<Constraint> constraints){
		segment = new MotionSegment(segment);
		RootTrajectory rt = new RootTrajectory(segment, true);
		for (Constraint c : constraints){
			c.constraintFrames[0] += Configuration.BLEND_MARGIN;
			c.constraintFrames[1] += Configuration.BLEND_MARGIN;
		}
		RootTrajectory[] editedRT = MotionEditJNI.instance.editRootTrajectory(new RootTrajectory[]{ rt }, constraints);
		ArrayList<Motion> smList = segment.getEntireMotion();
		for (int i = 0; i < smList.size(); i++) {
			RootInfo root = editedRT[0].getMotionList().get(i);
			Matrix4d m = root.root();
			smList.get(i).root().set(m);
			smList.get(i).knot = root.knot - Configuration.BLEND_MARGIN;
		}
		segment.updateNotBlendedAsCurrent();
		return segment;
	}
	
	private static double[] getPosConstraint(MotionSegment s, int index, Pose2d pose){
		ArrayList<Motion> mList = s.getEntireMotion();
		Motion m = mList.get(MotionSegment.BLEND_MARGIN() + index);
		Vector3d tPrev = MathUtil.getTranslation(mList.get(MotionSegment.BLEND_MARGIN() + index-1).root());
		Vector3d tNext = MathUtil.getTranslation(mList.get(MotionSegment.BLEND_MARGIN() + index+1).root());
		Vector3d v = MathUtil.sub(tNext, tPrev);
		Pose2d originPose = PositionMotion.getPose(m);
		Matrix4d t = Pose2d.globalTransform(originPose, pose).to3d();
		t.transform(v);
		Point3d translation = pose.position3d();
		return new double[]{ translation.x, translation.z, v.x, v.z };
	}
}
