package mrl.motion.neural.loco;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.motion.data.parser.BVHParser;
import mrl.util.Pair;

public class CopyOfCMUMayaSkeletonGenerator {

	
	public static void main(String[] args) throws IOException {
		
		
		
		MotionData mData = new BVHParser().parse(new File("locomotion\\zombie\\motion\\104_41_1.bvh"));
		Motion motion = new Motion(mData.motionList.get(0));
		for (Pair<String, Matrix4d> entry : motion.entrySet()){
			entry.second.setIdentity();
		}
		motion.root().setTranslation(new Vector3d(0, 107.866, 0));
		HashMap<String, Point3d> points = Motion.getPointData(mData.skeletonData, motion);
		ArrayList<String> createLines = new ArrayList<String>();
		for (Joint j : mData.skeletonData.values()){
			if (!j.name.equals("Hips") && j.transition.length() < 0.00001) continue;
			createLines.add(String.format("cmds.polySphere(n=(\"ps_%s\"), sx=15, sy=15, r=5)", j.name));
			
			Point3d t = points.get(j.name);
			setTranslation("ps_" + j.name, t.x, t.y, t.z);
			
			
			if (j.transition.length() < 0.00001) continue;
//			createLines.add(String.format("cmds.polyCylinder(n=(\"pc_%s\"), sx=20, sy=15, sz=20, r=3, h=%.5f)", j.name, j.transition.length()));
			createLines.add(String.format("cmds.polyCylinder(n=\"pc_%s\", sx=20, sy=15, sz=20, r=4, h=%.5f)", j.name, j.transition.length()));
			
			Point3d prev = points.get(j.parent.name);
			Point3d mid = new Point3d();
			mid.interpolate(t, prev, 0.5);
			setTranslation("pc_" + j.name, mid.x, mid.y, mid.z);
			
			Vector3d rv = new Vector3d(j.transition);
			rv.scale(-0.5);
			Vector3d r = getRotation(j);
			setRotation("pc_" + j.name, r.x, r.y, r.z);
			
		}
		System.out.println();
		System.out.println("####################");
		System.out.println();
		System.out.println("import maya.cmds as cmds");
		for (String line : createLines){
			System.out.println(line);
		}
		System.exit(0);
		
		for (Joint j : mData.skeletonData.values()){
			if (j == mData.skeletonData.root) continue;
			if (j.transition.length() < 0.00001) continue;
//			if (!j.name.equals("RightLeg")) continue;
			
//			makeCylinder(j);
			attachCylinder(j);
		}
	}
	
	private static Vector3d getRotation(Joint j){
		Vector3d v = new Vector3d(j.transition);
		Vector3d base = new Vector3d(0, 1, 0);
		if (j.name.contains("UpLeg")){
			base = new Vector3d(0, -1, 0);
		}
		Vector3d cross = new Vector3d();
		
		cross.cross(base, v);
		cross.normalize();
		AxisAngle4d aa = new AxisAngle4d(cross, v.angle(base));
		Matrix3d m = new Matrix3d();
		m.set(aa);
		m.transform(base);
		Vector3d e = BVHCMUHandAdjust.matrixToEulerXYZ(m);
		return e;
	}
	
	private static void attachCylinder(Joint j){
		String name = "pc_" + j.name;
		System.out.println(String.format("parent pc_%s BVH:%s;", j.name, j.name));
		Vector3d t = new Vector3d(j.transition);
		t.scale(-0.5);
		setTranslation(name, t.x, t.y, t.z);
		Vector3d r = getRotation(j);
		setRotation(name, r.x, r.y, r.z);
	}
	
	private static void setTranslation(String name, double x, double y, double z){
		System.out.println(String.format("setAttr \"%s.translateX\" %.5f;", name, x));
		System.out.println(String.format("setAttr \"%s.translateY\" %.5f;", name, y));
		System.out.println(String.format("setAttr \"%s.translateZ\" %.5f;", name, z));
	}
	private static void setRotation(String name, double x, double y, double z){
		System.out.println(String.format("setAttr \"%s.rotateX\" %.5f;", name, x));
		System.out.println(String.format("setAttr \"%s.rotateY\" %.5f;", name, y));
		System.out.println(String.format("setAttr \"%s.rotateZ\" %.5f;", name, z));
	}
	
	private static void makeCylinder(Joint j){
		System.out.println(String.format("cmds.polyCylinder(n=\"pc_%s\", sx=20, sy=15, sz=20, r=3, h=%.5f)", j.name, j.transition.length()));
	}
}
