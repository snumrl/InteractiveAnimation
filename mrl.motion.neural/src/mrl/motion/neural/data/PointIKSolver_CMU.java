package mrl.motion.neural.data;

import static mrl.util.MathUtil.getTranslation;
import static mrl.util.MathUtil.sub;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.position.PositionMotion;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.position.PositionResultMotion.PositionFrame;
import mrl.util.MathUtil;

public class PointIKSolver_CMU {
	
	private static final double eps = 0.000001;
	
	private SkeletonData skeleton;
	private Motion sample;
	
	private Motion motion;
	private HashMap<String, Point3d> map;

	public PointIKSolver_CMU(SkeletonData skeleton, Motion sample) {
		sample = new Motion(sample);
		sample = PositionMotion.getAlignedMotion(sample);
		this.skeleton = skeleton;
		this.sample = sample;
	}
	

//	ton 
//	get skeleton Finished 26
//	createEntity Finished 
//	0	BVH:Hips	Vector3(0, 97.1973, 0)	97.1973
//	1	BVH:LHipJoint	Vector3(0, 0, 0)	0
//	2	BVH:LeftUpLeg	Vector3(8.17836, -10.7678, 5.03574)	14.4288
//	3	BVH:LeftLeg	Vector3(14.6887, -40.3568, 0)	42.9468
//	4	BVH:LeftFoot	Vector3(15.3732, -42.2375, 0)	44.9482
//	5	BVH:LeftToeBase	Vector3(0.94584, -2.59866, 13.9353)	14.207
//	6	BVH:RHipJoint	Vector3(0, 0, 0)	0
//	7	BVH:RightUpLeg	Vector3(-7.83312, -10.7678, 5.03574)	14.2359
//	8	BVH:RightLeg	Vector3(-15.2552, -41.9133, 0)	44.6032
//	9	BVH:RightFoot	Vector3(-15.4096, -42.3374, 0)	45.0545
//	10	BVH:RightToeBase	Vector3(-0.98838, -2.71554, 14.1789)	14.4704
//	11	BVH:LowerBack	Vector3(0, 0, 0)	0
//	12	BVH:Spine	Vector3(0.16962, 12.2135, -1.16028)	12.2697
//	13	BVH:Spine1	Vector3(0.34032, 23, -0.2565)	23.0039
//	14	BVH:Neck	Vector3(0, 0, 0)	0
//	15	BVH:Head	Vector3(0.62442, 10.5682, -0.74382)	10.6127
//	16	BVH:LeftShoulder	Vector3(0, 0, 0)	0
//	17	BVH:LeftArm	Vector3(20.1745, 0, 0)	20.1745
//	18	BVH:LeftForeArm	Vector3(29.898, 0, 0)	29.898
//	19	BVH:LeftHand	Vector3(20.9014, 0, 0)	20.9014
//	20	BVH:LeftHandIndex1	Vector3(4.29156, 0, 0)	4.29156
//	21	BVH:RightShoulder	Vector3(0, 0, 0)	0
//	22	BVH:RightArm	Vector3(-18.8196, 0, 0)	18.8196
//	23	BVH:RightForeArm	Vector3(-31.4514, 0, 0)	31.4514
//	24	BVH:RightHand	Vector3(-20.665, 0, 0)	20.665
//	25	BVH:RightHandIndex1	Vector3(-3.73518, 0, 0)	3.73518
//	finished Finished 


	
	//###############################################
//	"Head_End",
//	"LeftHand",
//	"LeftFoot",
//	"LeftToeBase",
//	"RightHand",
//	"RightFoot",
//	"RightToeBase",
//	
//	"LeftArm",
//	"RightArm",
//	
//	"LeftForeArm",
//	"LeftLeg",
//	"RightForeArm",
//	"RightLeg",
//	
//	// added
//	"Spine",
//	"LeftHandIndex1",
//	"RightHandIndex1",
//	"Neck1",
//	"LeftUpLeg",
//	"RightUpLeg",
	
	int count = 0;
	boolean print;
	
	private HashMap<String, Point3d> frameToMap(PositionFrame frame){
		HashMap<String, Point3d> map = new HashMap<String, Point3d>();
		int fIdx = 0;
		for (String[] pair : MotionDataConverter.jointPairs){
			for (int i = 0; i < pair.length-1; i++) {
				Point3d[] pointPair = frame.get(fIdx);
				fIdx++;
				map.put(pair[i], pointPair[0]);
				map.put(pair[i+1], pointPair[1]);
			}
		}
		return map;
	}
	
	public MotionData solve(PositionResultMotion prm){
		ArrayList<Motion> mList = new ArrayList<Motion>();
		for (PositionFrame f : prm){
			mList.add(solve(f));
		}
		return new MotionData(mList);
	}
	
	public Motion solve(PositionFrame f){
		map = frameToMap(f);
		motion = new Motion(sample);
		motion.scales = new double[SkeletonData.keyList.length];
		for (int i = 0; i < motion.scales.length; i++) {
			motion.scales[i] = 1;
		}
		
		motion.root().setTranslation(new Vector3d(map.get("Hips")));
		
		alignAxis("LHipJoint", "LeftUpLeg", vec("Hips", "LeftUpLeg"));
		alignAxis("RHipJoint", "RightUpLeg", vec("Hips", "RightUpLeg"));
		alignAxis("LowerBack", "Spine", vec("Hips", "Spine"));
		
		alignAxis("Spine", "Spine1", vec("Spine", "Neck1"));
		alignAxis("Neck", "Neck_End", vec("Neck1", "Head_End"));
		
		alignAxis("RightUpLeg", "RightLeg", null);
		alignAxis("RightLeg", "RightFoot", null);
		alignAxis("RightFoot", "RightFoot_End", vec("RightFoot", "RightToeBase"));
		alignAxis("LeftUpLeg", "LeftLeg", null);
		alignAxis("LeftLeg", "LeftFoot", null);
		alignAxis("LeftFoot", "LeftFoot_End", vec("LeftFoot", "LeftToeBase"));
		
		alignAxis("LeftShoulder", "LeftArm", vec("Neck", "LeftArm"));
		alignAxis("LeftArm", "LeftForeArm", null);
		alignAxis("LeftForeArm", "LeftHand", null);
		alignAxis("LeftHand", "LeftHand_End", vec("LeftHand", "LeftHandIndex1"));
		
		alignAxis("RightShoulder", "RightArm", vec("Neck", "RightArm"));
		alignAxis("RightArm", "RightForeArm", null);
		alignAxis("RightForeArm", "RightHand", null);
		alignAxis("RightHand", "RightHand_End", vec("RightHand", "RightHandIndex1"));
		count++;
		return motion;
	}
	
	public Motion solve(HashMap<String, Point3d> map, Pose2d p){
		this.map = map;
		motion = new Motion(sample);
		motion.scales = new double[SkeletonData.keyList.length];
		for (int i = 0; i < motion.scales.length; i++) {
			motion.scales[i] = 1;
		}
		
		double rootHeight = map.get("Hips").y;
		motion.root().setTranslation(new Vector3d(0, rootHeight, 0));
		
		alignAxis("LHipJoint", "LeftUpLeg", vec("Hips", "LeftUpLeg"));
		alignAxis("RHipJoint", "RightUpLeg", vec("Hips", "RightUpLeg"));
		alignAxis("LowerBack", "Spine", vec("Hips", "Spine"));
		
		alignAxis("Spine", "Spine1", vec("Spine", "Neck1"));
		alignAxis("Neck", "Neck_End", vec("Neck1", "Head_End"));
		
		alignAxis("RightUpLeg", "RightLeg", null);
		alignAxis("RightLeg", "RightFoot", null);
		alignAxis("RightFoot", "RightFoot_End", vec("RightFoot", "RightToeBase"));
		alignAxis("LeftUpLeg", "LeftLeg", null);
		alignAxis("LeftLeg", "LeftFoot", null);
		alignAxis("LeftFoot", "LeftFoot_End", vec("LeftFoot", "LeftToeBase"));
		
		alignAxis("LeftShoulder", "LeftArm", vec("Neck", "LeftArm"));
		alignAxis("LeftArm", "LeftForeArm", null);
		alignAxis("LeftForeArm", "LeftHand", null);
		alignAxis("LeftHand", "LeftHand_End", vec("LeftHand", "LeftHandIndex1"));
		
		alignAxis("RightShoulder", "RightArm", vec("Neck", "RightArm"));
		alignAxis("RightArm", "RightForeArm", null);
		alignAxis("RightForeArm", "RightHand", null);
		alignAxis("RightHand", "RightHand_End", vec("RightHand", "RightHandIndex1"));
		
		Matrix4d globalM = Pose2d.globalTransform(Pose2d.BASE, p).to3d();
		motion.root().mul(globalM, motion.root());
		Vector3d rootTranslation = new Vector3d(Pose2d.to3d(p.position));
		rootTranslation.y = rootHeight;
		motion.root().setTranslation(rootTranslation);
		count++;
		return motion;
	}
	
	
	public Motion solveFoot(Motion base, HashMap<String, Point3d> map){
		this.map = map;
		motion = PositionMotion.getAlignedMotion(base);
		alignLegWithoutRotation("Left");
		alignLegWithoutRotation("Right");
		motion.root().set(base.root());
		count++;
		return motion;
	}
	
	private void alignArm(String prefix){
		alignAxis(prefix + "Shoulder", prefix + "Arm", null);
		updateJoint(prefix + "Arm");
		bendArm(prefix + "ForeArm", prefix + "Hand", vec(prefix + "Arm", prefix + "Hand"));
		alignAxis(prefix + "Arm", prefix + "Hand", null);
		rotateAxis(prefix + "Arm", prefix + "ForeArm", null, null);
		
		updateJoint(prefix + "Hand");
		alignAxis(prefix + "Hand", prefix + "HandIndex1", null);
//		alignAxis(prefix + "Hand", prefix + "Hand_End", null);
	}
	private void alignLeg(String prefix){
		updateJoint(prefix + "UpLeg");
		bendArm(prefix + "Leg", prefix + "Foot", vec(prefix + "UpLeg", prefix + "Foot"));
		alignAxis(prefix + "UpLeg", prefix + "Foot", null);
		rotateAxis(prefix + "UpLeg", prefix + "Leg", null, null);
		
		updateJoint(prefix + "Foot");
		alignAxis(prefix + "Foot", prefix + "ToeBase", null);
	}
	
	private void alignLegWithoutRotation(String prefix){
		updateJoint(prefix + "UpLeg");
		bendArm(prefix + "Leg", prefix + "Foot", vec(prefix + "UpLeg", prefix + "Foot"));
		alignAxis(prefix + "UpLeg", prefix + "Foot", null);
//		rotateAxis(prefix + "UpLeg", prefix + "Leg", null, null);
		
		updateJoint(prefix + "Foot");
//		alignAxis(prefix + "Foot", prefix + "Toe_End", null);
	}
	
	private Vector3d vec(String j1, String j2, String j3){
		Vector3d v = vec(j1, j2);
		v.add(vec(j3, j1));
		v.scale(0.5);
		return v;
	}
	private Vector3d vec(String j1, String j2){
		Point3d p1 = map.get(j1);
		Point3d p2 = map.get(j2);
		if (p1 == null) p1 = currentP(j1);
		if (p2 == null) p2 = currentP(j2);
		if (p1 == null) System.out.println("null1 : " + j1);
		if (p2 == null) System.out.println("null2 : " + j2);
		return sub(p2, p1);
	}
	
	private void updateJoint(String joint){
		map.put(joint, currentP(joint));
	}
	
	private Point3d currentP(String joint){
		return Motion.getPointData(skeleton, motion).get(joint);
	}
	
	private void bendArm(String j1, String j2, Vector3d v){
		double l1 = skeleton.get(j1).length;
		double l2 = skeleton.get(j2).length;
		double L = v.length();
		double angle = ACOS((l1*l1 + l2*l2 - L*L)/(2*l1*l2));
		if (print){
			System.out.println("angle :: " + count + " : " + angle + " : " + Math.toDegrees(angle));
		}
		motion.get(j1).rotZ(-(Math.PI - angle));
	}
	
	private double ACOS(double x){
		if (x > 1) return 0;
		if (x < -1) return Math.PI;
		return Math.acos(x);
	}
	
	private void rotateAxis(String base, String target, Vector3d v, Vector3d axis){
		if (v == null) v = vec(base, target);
		if (axis == null) axis = prevAxis;
		HashMap<String, Matrix4d> tData = Motion.getTransformData(skeleton, motion);
		
		Matrix4d baseM = tData.get(base);
		Matrix4d targetM = tData.get(target);
		Vector3d currentV = sub(getTranslation(targetM), getTranslation(baseM));
		
//		axis = new Vector3d(axis);
//		axis.normalize();
//		Vector3d axisY = MathUtil.cross(axis, v);
//		axisY.normalize();
//		Vector3d axisX = MathUtil.cross(axisY, axis);
//		axisX.normalize();
//		
//		Vector2d v1 = to2d(v, axisX, axisY);
//		Vector2d v2 = to2d(currentV, axisX, axisY);
//		double angle = MathUtil.directionalAngle(v2, v1);
//		AxisAngle4d rot = new AxisAngle4d(axis, angle);
		
		axis = new Vector3d(axis);
		axis.normalize();
		Vector3d v1 = MathUtil.cross(axis, v);
		Vector3d v2 = MathUtil.cross(axis, currentV);
		if (print){
			System.out.println("vv : " + count + " : " + v1.length() + " , " + v2.length());
		}
		if (v1.length() < 0.5) return;
		if (v2.length() < eps) return;
		Vector3d ax = MathUtil.cross(v1,  v2);
		ax.normalize();
		double angle = v1.angle(v2);
		AxisAngle4d rot = new AxisAngle4d(ax, -angle);
		
		
		Matrix4d rotM = new Matrix4d();
		rotM.set(rot);
		rotM.mul(rotM, baseM);
		Joint bJoint = skeleton.get(base);
		if (bJoint.parent != null){
			Matrix4d pp = new Matrix4d(tData.get(bJoint.parent.name));
			pp.invert();
			motion.get(base).mul(pp, rotM);
		} else {
			motion.get(base).set(rotM);
		}
	}
	
	private Vector3d prevAxis;
	private void alignAxis(String base, String target, Vector3d v){
		if (v == null) v = vec(base, target);
		HashMap<String, Matrix4d> tData = Motion.getTransformData(skeleton, motion);
		
		Matrix4d baseM = tData.get(base);
		Matrix4d targetM = tData.get(target);
		if (targetM == null) System.out.println("target :: "+ target);
		if (baseM == null) System.out.println("baseM :: "+ baseM);
		Vector3d currentV = sub(getTranslation(targetM), getTranslation(baseM));
		
		
		Vector3d cross = MathUtil.cross(currentV, v);
		if (cross.length() > eps){
			cross.normalize();
			double angle = currentV.angle(v);
			AxisAngle4d rot = new AxisAngle4d(cross, angle);
			
			Matrix4d rotM = new Matrix4d();
			rotM.set(rot);
			
			rotM.mul(rotM, baseM);
			
			Point3d origin = currentP(target);
			Joint bJoint = skeleton.get(base);
			if (bJoint.parent != null){
				Matrix4d pp = new Matrix4d(tData.get(bJoint.parent.name));
				pp.invert();
				motion.get(base).mul(pp, rotM);
			} else {
				motion.get(base).set(rotM);
			}
		}
		motion.scales[SkeletonData.keyMap.get(base)] = v.length()/currentV.length();
		prevAxis = v;
	}
}
