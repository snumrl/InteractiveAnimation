package mrl.motion.data;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.SkeletonData.Joint;
import mrl.util.MathUtil;
import mrl.util.Pair;

public class Motion {

	public boolean isRotateFirst;

	public Contact ballContact = new Contact(false, false);
	
	public boolean isLeftFootContact;
	public boolean isRightFootContact;
	public boolean[] leftFootContact;
	public boolean[] rightFootContact;
	public boolean isMirrored = false;

	public double knot = Double.NaN;

	public int type = -1;
	public int motionIndex = -1;
	public int frameIndex = -1;

	public Motion prev;
	public Motion next;
	public MotionData motionData;

	private Matrix4d[] transforms;
	
	public double[] scales;

	public Motion(boolean isRotateFirst) {
		this.isRotateFirst = isRotateFirst;
		transforms = new Matrix4d[SkeletonData.keyList.length];
	}

	public Motion(Motion copy) {
		this(copy.isRotateFirst);
		this.isLeftFootContact = copy.isLeftFootContact;
		this.isRightFootContact = copy.isRightFootContact;
		this.leftFootContact = copy.leftFootContact;
		this.rightFootContact = copy.rightFootContact;
		this.knot = copy.knot;
		this.motionIndex = copy.motionIndex;
		this.motionData = copy.motionData;
		this.frameIndex = copy.frameIndex;
		this.ballContact = copy.ballContact;
		this.isMirrored = copy.isMirrored; 
		for (int i = 0; i < transforms.length; i++) {
			if (copy.transforms[i] != null) {
				transforms[i] = new Matrix4d(copy.transforms[i]);
			}
		}
	}

	public Matrix4d get(String key) {
		return transforms[SkeletonData.keyMap.get(key)];
	}

	public void put(String key, Matrix4d m) {
		transforms[SkeletonData.keyMap.get(key)] = m;
	}

	public Matrix4d root() {
		return get(SkeletonData.defaultRoot);
	}

	public Point3d getPosition(SkeletonData skeletonData, String key) {
		return getPointData(skeletonData, this).get(key);
	}

	public boolean containsKey(String key) {
		int idx = SkeletonData.keyMap.get(key);
		return idx >= 0 && transforms[idx] != null;
	}

	public ArrayList<Pair<String, Matrix4d>> entrySet() {
		ArrayList<Pair<String, Matrix4d>> list = new ArrayList<Pair<String, Matrix4d>>(
				transforms.length);
		for (int i = 0; i < transforms.length; i++) {
			if (transforms[i] != null) {
				list.add(new Pair<String, Matrix4d>(SkeletonData.keyList[i],
						transforms[i]));
			}
		}
		return list;
	}

	@Override
	public String toString(){
		String fName = "";
		if (motionData != null && motionData.file != null){
			fName = motionData.file.getName().replace(".bvh", "") + ":";
		}
		return fName + frameIndex; 
	}
	
	
	public static Motion mirroredMotion(Motion motion){
		Motion mirror = new Motion(motion);
		Matrix4d m = mirror.root();
		m.m20 *= -1;
		m.m21 *= -1;
		m.m22 *= -1;
		m.m23 *= -1;
		
		mirror.isLeftFootContact = motion.isRightFootContact;
		mirror.isRightFootContact = motion.isLeftFootContact;
		mirror.leftFootContact = motion.rightFootContact;
		mirror.rightFootContact = motion.leftFootContact;
		mirror.isMirrored = true;
		return mirror;
	}

	public static HashMap<String, Point3d> getPointData(
			SkeletonData skeletonData, Motion motion) {
		HashMap<String, Point3d> pointData = new HashMap<String, Point3d>();
		Matrix4d matrix = new Matrix4d();
		matrix.setIdentity();
		calcJointInformation(skeletonData.root, skeletonData, motion, matrix,
				pointData);
		
		if (motion.isMirrored){
			HashMap<String, Point3d> mirrored = new HashMap<String, Point3d>();
			String[][] leftToRight = {
					{ "Left", "Right" },
					{ "LHip", "RHip" },
					{ "LThumb", "RThumb" }
			};
			for (Entry<String, Point3d> entry : pointData.entrySet()){
				String key = entry.getKey();
				for (String[] pair :  leftToRight){
					if (key.contains(pair[0])){
						key = key.replace(pair[0], pair[1]);
					} else if (key.contains(pair[1])){
						key = key.replace(pair[1], pair[0]);
					}
				}
				mirrored.put(key, entry.getValue());
			}
			pointData = mirrored;
		}
		
		return pointData;
	}

	private static void calcJointInformation(Joint joint,
			SkeletonData skeletonData, Motion motion, Matrix4d matrix,
			HashMap<String, Point3d> jointMap) {
		matrix = new Matrix4d(matrix);

		Matrix4d m = motion.get(joint.name);

		if (motion.isRotateFirst && m != null) {
			matrix.mul(m);
		}

		boolean pass = (joint == skeletonData.root && joint.length > 0)
				|| (joint.name.equals("pelvis") && joint.length > 0);
		if (!pass && joint.length != 0) {
			Matrix4d transMatrix = new Matrix4d();
			transMatrix.setIdentity();
			transMatrix.setTranslation(joint.transition);
			matrix.mul(transMatrix);
		}

		Point3d p = new Point3d();
		matrix.transform(p);
		jointMap.put(joint.name, p);

		if (!motion.isRotateFirst && m != null) {
			if (joint == skeletonData.root || joint.name.equals("pelvis")) {
				matrix.mul(m);
				p = new Point3d();
				matrix.transform(p);
				jointMap.put(joint.name, p);
			} else {
				m = new Matrix4d(m);
				m.setTranslation(new Vector3d());
				matrix.mul(m);
			}
		}

		for (Joint child : joint.children) {
			calcJointInformation(child, skeletonData, motion, matrix, jointMap);
		}
	}

	public static Motion interpolateMotion(Motion motion1, Motion motion2,
			double ratio) {
		Motion newMotion;
		if (ratio <= 0.5) {
			newMotion = new Motion(motion1);
		} else {
			newMotion = new Motion(motion2);
		}
//		Motion newMotion = new Motion(motion1.isRotateFirst);
//		if (ratio <= 0.5) {
//			newMotion.type = motion1.type;
//			newMotion.motionIndex = motion1.motionIndex;
//			newMotion.isLeftFootContact = motion1.isLeftFootContact;
//			newMotion.isRightFootContact = motion1.isRightFootContact;
//		} else {
//			newMotion.type = motion2.type;
//			newMotion.motionIndex = motion2.motionIndex;
//			newMotion.isLeftFootContact = motion2.isLeftFootContact;
//			newMotion.isRightFootContact = motion2.isRightFootContact;
//		}

		if (!Double.isNaN(motion1.knot) && !Double.isNaN(motion2.knot)) {
			newMotion.knot = (motion1.knot + motion2.knot) / 2;
		}
		
		for (Pair<String, Matrix4d> entry : motion1.entrySet()) {
			Matrix4d m1 = entry.getValue();
			Matrix4d m2 = motion2.get(entry.getKey());
			newMotion.put(entry.getKey(), MathUtil.interpolate(m1, m2, ratio));
		}
		return newMotion;
	}


	
	public static HashMap<String, Matrix4d> getTransformData(
			SkeletonData skeletonData, Motion motion) {
		HashMap<String, Matrix4d> transformData = new HashMap<String, Matrix4d>();
		Matrix4d matrix = new Matrix4d();
		matrix.setIdentity();
		calcJointTransform(skeletonData.root, skeletonData, motion, matrix, transformData);
		
		if (motion.isMirrored){
			HashMap<String, Matrix4d> mirrored = new HashMap<String, Matrix4d>();
			String[][] leftToRight = {
					{ "Left", "Right" },
					{ "LHip", "RHip" },
					{ "LThumb", "RThumb" }
			};
			for (Entry<String, Matrix4d> entry : transformData.entrySet()){
				String key = entry.getKey();
				for (String[] pair :  leftToRight){
					if (key.contains(pair[0])){
						key = key.replace(pair[0], pair[1]);
					} else if (key.contains(pair[1])){
						key = key.replace(pair[1], pair[0]);
					}
				}
				mirrored.put(key, entry.getValue());
			}
			transformData = mirrored;
		}
		return transformData;
	}
	
	private static void calcJointTransform(Joint joint,
			SkeletonData skeletonData, Motion motion, Matrix4d matrix,
			HashMap<String, Matrix4d> jointMap) {
		matrix = new Matrix4d(matrix);

		Matrix4d m = motion.get(joint.name);

		if (motion.isRotateFirst && m != null) {
			matrix.mul(m);
		}

		boolean pass = (joint == skeletonData.root && joint.length > 0)
				|| (joint.name.equals("pelvis") && joint.length > 0);
		if (!pass && joint.length != 0) {
			Matrix4d transMatrix = new Matrix4d();
			transMatrix.setIdentity();
			transMatrix.setTranslation(joint.transition);
			matrix.mul(transMatrix);
		}

		

		if (!motion.isRotateFirst && m != null) {
			if (joint == skeletonData.root || joint.name.equals("pelvis")) {
				matrix.mul(m);
			} else {
				m = new Matrix4d(m);
				m.setTranslation(new Vector3d());
				matrix.mul(m);
			}
		}
		jointMap.put(joint.name, new Matrix4d(matrix));

		for (Joint child : joint.children) {
			calcJointTransform(child, skeletonData, motion, matrix, jointMap);
		}
	}

}
