package mrl.motion.neural.loco;

import static java.lang.Math.asin;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.motion.data.cleanup.BVHScale;
import mrl.motion.data.cleanup.FPSReducer;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.parser.BVHWriter;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class BVHCMUHandAdjust {

	public static Matrix4d eulerToMatrixZXY(Vector3d euler){
		euler = new Vector3d(euler);
		euler.scale(Math.PI/180);
		Matrix4d mx = new Matrix4d();
		mx.rotX(euler.x);
		Matrix4d my = new Matrix4d();
		my.rotY(euler.y);
		Matrix4d mz = new Matrix4d();
		mz.rotZ(euler.z);
		
		Matrix4d m = new Matrix4d();
		m.set(mz);
		m.mul(mx);
		m.mul(my);
		return m;
	}
	
	public static Vector3d matrixToEulerZXY(Matrix4d m){
		Vector3d euler = new Vector3d();
		euler.x = Math.asin(m.m21);
		euler.y = atan2(-m.m20, m.m22);
		euler.z = atan2(-m.m10, m.m11);
		euler.scale(180/Math.PI);
		return euler;
	}
	
	public static Vector3d matrixToEulerZYX(Matrix3d m){
		Vector3d euler = new Vector3d();
		euler.y = asin(-m.m20);
		euler.x = atan2(m.m21, m.m22);
		euler.z = atan2(m.m10, m.m00);
		euler.scale(180/Math.PI);
		return euler;
	}
	public static Matrix3d eulerToMatrixZYX(Vector3d euler){
		euler = new Vector3d(euler);
		euler.scale(Math.PI/180);
		Matrix3d mx = new Matrix3d();
		mx.rotX(euler.x);
		Matrix3d my = new Matrix3d();
		my.rotY(euler.y);
		Matrix3d mz = new Matrix3d();
		mz.rotZ(euler.z);
		
		Matrix3d m = new Matrix3d();
		m.set(mz);
		m.mul(my);
		m.mul(mx);
		return m;
	}
	
	public static Vector3d matrixToEulerYZX(Matrix3d m){
		Vector3d euler = new Vector3d();
		euler.x = atan2(-m.m12, m.m11);
		euler.y = atan2(-m.m20, m.m00);
		euler.z = asin(m.m10);
		return euler;
	}
	public static Matrix3d eulerToMatrixYZX(Vector3d euler){
		euler = new Vector3d(euler);
		Matrix3d mx = new Matrix3d();
		mx.rotX(euler.x);
		Matrix3d my = new Matrix3d();
		my.rotY(euler.y);
		Matrix3d mz = new Matrix3d();
		mz.rotZ(euler.z);
		
		Matrix3d m = new Matrix3d();
		m.set(my);
		m.mul(mz);
		m.mul(mx);
		return m;
	}
	
	public static Vector3d matrixToEulerZXY(Matrix3d m){
		Vector3d euler = new Vector3d();
		euler.x = asin(m.m21);
		euler.y = atan2(-m.m20, m.m22);
		euler.z = atan2(-m.m01, m.m11);
		euler.scale(180/Math.PI);
		return euler;
	}
	
	public static Matrix3d eulerToMatrixZXY3d(Vector3d euler){
		euler = new Vector3d(euler);
		euler.scale(Math.PI/180);
		Matrix3d mx = new Matrix3d();
		mx.rotX(euler.x);
		Matrix3d my = new Matrix3d();
		my.rotY(euler.y);
		Matrix3d mz = new Matrix3d();
		mz.rotZ(euler.z);
		
		Matrix3d m = new Matrix3d();
		m.set(mz);
		m.mul(mx);
		m.mul(my);
		return m;
	}
	
	public static Vector3d matrixToEulerXYZ(Matrix3d m){
		Vector3d euler = new Vector3d();
		
		euler.x = atan2(-m.m12, m.m22);
		euler.y = asin(m.m02);
		euler.z = atan2(-m.m01, m.m00);
		
//		euler.x = asin(m.m21);
//		euler.y = atan2(-m.m20, m.m22);
//		euler.z = atan2(-m.m01, m.m11);
		euler.scale(180/Math.PI);
		return euler;
	}
	
	public static Matrix3d eulerToMatrixXYZ(Vector3d euler){
		euler = new Vector3d(euler);
		euler.scale(Math.PI/180);
		Matrix3d mx = new Matrix3d();
		mx.rotX(euler.x);
		Matrix3d my = new Matrix3d();
		my.rotY(euler.y);
		Matrix3d mz = new Matrix3d();
		mz.rotZ(euler.z);
		
		Matrix3d m = new Matrix3d();
		m.set(mx);
		m.mul(my);
		m.mul(mz);
		return m;
	}
	
	static void transform(File file) throws IOException{
		BVHParser parser = new BVHParser();
		ArrayList<Motion> motionList = parser.parse(file).motionList;
		int left = parser.jointIndex("LeftHand");
		int right = parser.jointIndex("RightHand");
//		int left = parser.jointIndex("LeftFingerBase");
//		int right = parser.jointIndex("RightFingerBase");
		System.out.println("left right :: " + left + " , " + right);
		System.out.flush();
//		System.exit(0);
		
//		File tempFile = new File("salsa\\export.bvh");
		File tempFile = new File(file.getAbsoluteFile().getParent() + "\\" + "_" + file.getName());
		BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
		BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(tempFile)));
		System.out.println("file : " + file.getName());
		String line;
		int jointIndex = 0;
		boolean isStarted = false;
		int mIndex = 0;
		while ((line = br.readLine()) != null) {
			if (line.trim().length() == 0) continue;
			if (line.trim().contains("JOINT LeftFingerBase")
					|| line.trim().contains("JOINT RightFingerBase")){
//				bw.write(line + "\r\n");
				
//				{
//				OFFSET 0.000000 0.000000 0.000000
//				CHANNELS 3 Zrotation Yrotation Xrotation
//				JOINT LeftHandIndex1
				for (int i = 0; i < 3; i++) { 
					br.readLine();
				}
//				{
//					OFFSET 4.291560 -0.000000 -0.000000
//					CHANNELS 3 Zrotation Yrotation Xrotation
//					End Site
//					{
//					OFFSET 0.000000 0.000000 0.000000
//					}
				for (int i = 0; i < 8; i++) { 
					line = br.readLine();
					bw.write(line + "\r\n");
				}
//				}
				br.readLine();
				br.readLine();
				
			}
			
			if (isStarted){
				String[] tokens = line.split(" ");
				double[] values = new double[tokens.length];
				for (int i = 0; i < values.length; i++) {
					values[i] = Double.parseDouble(tokens[i]);
				}
				
//				double[] transformed = new double[values.length - 3];
				double[] transformed = new double[values.length - 3*2];
				int i1 = left*3 + 3;
				System.arraycopy(values, 0, transformed, 0, i1);
				
				{
					Matrix3d m1 = eulerToMatrixZYX(new Vector3d(values[i1+0], values[i1+1], values[i1+2]));
					Matrix3d m2 = eulerToMatrixZYX(new Vector3d(values[i1+3], values[i1+4], values[i1+5]));
	//				m1.mul(m1, m2);
					m1.mul(m2, m1);
					Vector3d e = matrixToEulerZYX(m1);
					transformed[i1+0] = e.x;
					transformed[i1+1] = e.y;
					transformed[i1+2] = e.z;
				}
				
				
				int i2 = right*3 + 3;
				System.arraycopy(values, i1+6, transformed, i1+3, i2 - (i1+6));
				{
					Matrix3d m1 = eulerToMatrixZYX(new Vector3d(values[i2+0], values[i2+1], values[i2+2]));
					Matrix3d m2 = eulerToMatrixZYX(new Vector3d(values[i2+3], values[i2+4], values[i2+5]));
//					m1.mul(m1, m2);
					m1.mul(m2, m1);
					Vector3d e = matrixToEulerZYX(m1);
					transformed[i2+0-3] = e.x;
					transformed[i2+1-3] = e.y;
					transformed[i2+2-3] = e.z;
				}
				
				
				
				System.arraycopy(values, i2+6, transformed, i2, values.length - (i2+6));
				
				
				StringBuilder sb = new StringBuilder();
				for (int i = 0; i < transformed.length; i++) {
					if (i != 0) sb.append(" ");
					sb.append(String.format("%.5f", transformed[i]));
				}
				line = sb.toString();
				
				mIndex++;
			}
			
			bw.write(line + "\r\n");
			
			if (line.startsWith("Frame Time:")) isStarted = true;
		}
		bw.close();
		br.close();
		
		file.delete();
		tempFile.renameTo(file);
	}
	
	private static void compare(String f1, String f2){
		MotionData m1 = new BVHParser().parse(new File(f1));
		ArrayList<HashMap<String, Point3d>> dataList1 = new ArrayList<HashMap<String,Point3d>>();
		for (Motion m : m1.motionList){
			HashMap<String, Point3d> points = Motion.getPointData(m1.skeletonData, m);
			dataList1.add(points);
		}
		SkeletonData.instance = null;
		MotionData m2 = new BVHParser().parse(new File(f2));
		ArrayList<HashMap<String, Point3d>> dataList2 = new ArrayList<HashMap<String,Point3d>>();
		for (Motion m : m2.motionList){
			HashMap<String, Point3d> points = Motion.getPointData(m2.skeletonData, m);
			dataList2.add(points);
		}
		
		for (int i = 0; i < dataList1.size(); i++) {
			HashMap<String, Point3d> map1 = dataList1.get(i);
			HashMap<String, Point3d> map2 = dataList2.get(i);
			for (Entry<String, Point3d> entry : map2.entrySet()){
				String key = entry.getKey();
				if (key.equals("LeftFingerBase_End")){
					key = "LeftHandIndex1_End";
				}
				Point3d p1 = map1.get(key);
				Point3d p2 = entry.getValue();
				if (p1.distance(p2) > 0.001){
					System.out.println("pp : " +  entry.getKey() + " : " + p1 + " : "  + p2 + " : " + p1.distance(p2));
				}
			}
		}
		System.exit(0);
	}
	
	public static void main(String[] args) throws IOException {
		
		File folder = new File("C:\\Users\\khlee\\git\\MotionGAN\\mrl.motion.neural\\salsa\\motion");
		for (File file : folder.listFiles()){
			if (file.isDirectory()) continue;
			if (file.getName().startsWith("_")) continue;
			if (!file.getName().endsWith("bvh")) continue;
			transform(file);
		}
//		transform(new File("salsa\\motion\\60_01_1.bvh"));
		
//		compare("salsa\\motion\\60_01_1.bvh", "salsa\\export.bvh");
		
//		BVHWriter bw = new BVHWriter();
//		MotionData motionData = new BVHParser().parse(new File("salsa\\motion\\60_02_1.bvh"));
//		bw.write(new File("salsa\\export.bvh"), motionData);
		
	}
}
