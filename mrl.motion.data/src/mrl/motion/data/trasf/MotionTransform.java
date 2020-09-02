package mrl.motion.data.trasf;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.motion.data.parser.BVHParser;
import mrl.util.Configuration;
import mrl.util.MathUtil;


public class MotionTransform {
	
	public static final Quat4d baseOrientation = new Quat4d(0, 0, 0, 1);
	
	public SkeletonData skeletonData;
	public Motion sampleMotion;
	
	public double rootPosWeight;
	public double rootOriWeight;
	public double posToOriRatio;
	public double[] weightList;
	public String[] keyList;
	
	
	public int frameRate;
	public double maxJointLength;
	public double footYLimit;

	public MotionTransform() {
		this(new BVHParser().parse(new File(Configuration.BASE_MOTION_FILE)));
	}
	
	public MotionTransform(MotionData sampleMotionData) {
		skeletonData = sampleMotionData.skeletonData;
		frameRate = sampleMotionData.framerate;
		Configuration.DEFAULT_FPS = frameRate;
		
		sampleMotion = sampleMotionData.motionList.get(0);
		
		maxJointLength = 0;
		for (Joint j : skeletonData.values()){
			if (j != skeletonData.root){
				maxJointLength = Math.max(maxJointLength, j.length);
			}
		}
		footYLimit = maxJointLength/3;
		
		ArrayList<String> vKeyList = new ArrayList<String>();
		for (String j : skeletonData.keySet()){
			if (sampleMotion.containsKey(j) && !skeletonData.root.name.equals(j)){
				vKeyList.add(j);
			}
		}
		keyList = vKeyList.toArray(new String[vKeyList.size()]);
		Arrays.sort(keyList);
		
		weightList = new double[vKeyList.size()];
		for (int i = 0; i < weightList.length; i++) {
			weightList[i] = 0.1;
		}
		
		setWeight(5, "Spine");
		setWeight(0.2, "Spine1", "Spine2");
		setWeight(5, "RightUpLeg", "RightLeg", "LeftUpLeg", "LeftLeg");
		setWeight(1, "RightArm", "RightForeArm", "LeftArm", "LeftForeArm");
//		setWeight(0.6, "RightArm", "RightForeArm", "LeftArm", "LeftForeArm");
		setWeight(0.05, "RightHand", "RightFoot", "RightToe", "LeftHand", "LeftFoot", "LeftToe");
		
		
		
		double sum = 0;
		for (int i = 0; i < weightList.length; i++) {
			sum += weightList[i];
		}
		rootPosWeight = 1;
		rootOriWeight = 5;
		sum += rootPosWeight + rootOriWeight;
		
		
		for (int i = 0; i < weightList.length; i++) {
			weightList[i] = weightList[i] / sum;
		}
		
		rootPosWeight = rootPosWeight / sum;
		rootOriWeight = rootOriWeight / sum;
		
		Matrix4d m = sampleMotion.get(skeletonData.root.name);
		Vector3d v = MathUtil.getTranslation(m);
		posToOriRatio = Math.PI * 2 / v.y; 
	}
	
	public int getIndex(String key){
		for (int i = 0; i < keyList.length; i++) {
			if (keyList[i].equals(key)) return i;
		} 
		return -1;
	}
	
	private void setWeight(double weight, String... keys){
		for (String key : keys){
			weightList[getIndex(key)] = weight;
		}
	}

	public double[] getWeightList() {
		return weightList;
	}

	public String[] getKeyList() {
		return keyList;
	}

	public SkeletonData getSkeletonData() {
		return skeletonData;
	}
	
	public void setIgnoreJoint(String joint){
		for (int i = 0; i < keyList.length; i++) {
			if (keyList[i].equals(joint)){
				weightList[i] = 0;
			}
		}
	}

	public MotionVector toVector(Motion motion){
		MotionVector vector = new MotionVector();
		
		motion = getAlignedMotion(motion);
		
		Matrix4d rootM = motion.root();
		vector.rootTranslation = MathUtil.getTranslation(rootM);
		vector.rootOrientation = MathUtil.quat(rootM);
		
		vector.jointRotations = new Quat4d[keyList.length];
		for (int i = 0; i < keyList.length; i++) {
			Matrix4d m = motion.get(keyList[i]);
			vector.jointRotations[i] = MathUtil.quat(m);
		}
		vector.isLeftFootContact = motion.isLeftFootContact;
		vector.isRightFootContact = motion.isRightFootContact;
		{
			HashMap<String, Point3d> pointData = Motion.getPointData(skeletonData, motion);
			vector.leftFootPosition = pointData.get(FootContactDetection.leftFootJoints[1]);
			vector.rightFootPosition = pointData.get(FootContactDetection.rightFootJoints[1]);
		}
		return vector;
	}
	
	
	
	public static Matrix4d getAlignTransform(Quat4d rootOrientation, Vector3d rootTranslation){
		Matrix4d base = new Matrix4d();
		base.setIdentity();
		Quat4d baseOrientation = MathUtil.quat(base);
		Vector3d baseTranslation = MathUtil.getTranslation(base);
		
		double angle = alignAngle(baseOrientation, rootOrientation);
		Matrix4d graphTransform = new Matrix4d();
		graphTransform.rotY(angle);
		
		Vector3d translation = new Vector3d(rootTranslation);
		graphTransform.transform(translation);
		translation.sub(baseTranslation, translation);
		translation.y = 0;
		graphTransform.setTranslation(translation);
		return graphTransform;
	}
	
	public static Matrix4d getAlignTransform(Motion motion){
		return getAlignTransform(motion.root());
	}
	
	public static Matrix4d getAlignTransform(Matrix4d root){
		Quat4d rootOrientation = MathUtil.quat(root);
		Vector3d rootTranslation = MathUtil.getTranslation(root);
		return getAlignTransform(rootOrientation, rootTranslation);
	}
	
	public static Motion getAlignedMotion(Motion motion){
		motion = new Motion(motion);
		Matrix4d root = motion.root();
		root.mul(getAlignTransform(motion), root);
		return motion;
	}
	
	public static Matrix4d getPlaneTransform(Matrix4d m){
		Matrix4d transform = new Matrix4d();
		transform.rotY(-alignAngle(m));
		Vector3d t = MathUtil.getTranslation(m);
		t.y = 0;
		transform.setTranslation(t);
		return transform;
	}
	
	public static double alignAngle(Quat4d base, Quat4d toAlign){
		Matrix4d m1 = new Matrix4d();
		Matrix4d m2 = new Matrix4d();
		m1.set(base);
		m2.set(toAlign);
		double angle1 = alignAngle(m1);
		double angle2 = alignAngle(m2);
		double angle = angle2 - angle1;
		return angle;
	}
	
	public static double alignAngle(Matrix4d m){
		Vector3d xAxis = new Vector3d(1, 0, 0);
		Vector3d zAxis = new Vector3d(0, 0, 1);
		
		m.transform(xAxis);
		m.transform(zAxis);
		
		xAxis.y = 0;
		zAxis.y = 0;
		
		double angle;
		if (xAxis.length() > zAxis.length()){
			Vector3d origin = new Vector3d(1, 0, 0);
			angle = xAxis.angle(origin);
			if (MathUtil.cross(origin, xAxis).y > 0){
				angle *= -1;
			}
		} else {
			Vector3d origin = new Vector3d(0, 0, 1);
			angle = zAxis.angle(origin);
			if (MathUtil.cross(origin, zAxis).y > 0){
				angle *= -1;
			}
		}
		return angle;
	}
	
	public static Matrix4d getAlignTransform(Matrix4d base, Matrix4d toAlign){
		Vector3d t1 = MathUtil.getTranslation(base);
		Vector3d t2 = MathUtil.getTranslation(toAlign);
		
		double angle1 = alignAngle(base);
		double angle2 = alignAngle(toAlign);
		double angle = angle2 - angle1;
		
		Matrix4d transform = new Matrix4d();
		transform.rotY(angle);
		
		Vector3d translation = new Vector3d(t2);
		transform.transform(translation);
		translation.sub(t1, translation);
		translation.y = 0;
		transform.setTranslation(translation);
		return transform;
	}
	
	public static double getSpatioTemporalError(Matrix4d originBase, Matrix4d originTarget, Matrix4d editBase, Matrix4d editTarget, double t1, double t2){
		Matrix4d transform = new Matrix4d();
		transform.mul(MathUtil.invert(originBase), originTarget);
		transform.mul(MathUtil.invert(editTarget));
		transform.mul(editBase);
		return getSpatioTemporalError(transform, Math.abs(t1 - t2));
	}
	public static double getSpatioTemporalError(Matrix4d transform, double timeDiff){
		double positionWeight = 0.01; // to meter
		double orientationWeight = 1.12; // radian
		double timeWeight = 0.38*(1/30d); // to second
		
		double pd = MathUtil.getTranslation(transform).length() * positionWeight;
		AxisAngle4d angle = new AxisAngle4d();
		angle.set(transform);
		double od = angle.angle * orientationWeight;
		double td = timeDiff*timeWeight;
		
		double distance = pd*pd + od*od + td*td;
		return distance;
	}
}
