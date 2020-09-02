package mrl.motion.data.cleanup;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.SkeletonData.Joint;
import mrl.motion.data.parser.BVHParser;
import mrl.util.MathUtil;

public class BVHMerge {
	
	public static class Person{
		public int index;
		public SkeletonData skeletonData;
		public File seedFile;
		public ArrayList<File> fileList = new ArrayList<File>();
		public int motionCounts = 0;
		public ArrayList<Integer> sizeList = new ArrayList<Integer>();
		
		public Person(int index, SkeletonData skeletonData, File seedFile) {
			this.index = index;
			this.skeletonData = skeletonData;
			this.seedFile = seedFile;
		}
		
		static double getRootHeight(File seedFile){
			MotionData motionData = new BVHParser().parse(seedFile);
			Joint toeEnd = motionData.skeletonData.get("RightToeEnd");
			if (toeEnd == null){
				toeEnd = motionData.skeletonData.get("RightToe" + BVHParser.END_POSTFIX);
			}
			Vector3d translation = new Vector3d();
			while (toeEnd != motionData.skeletonData.root){
				translation.add(toeEnd.transition);
				toeEnd = toeEnd.parent;
			}
			System.out.println("tt : " + translation);
			return Math.abs(translation.y);
//			return Math.abs(translation.z);
		}
		static double getRootHeight2(File seedFile){
			MotionData motionData = new BVHParser().parse(seedFile);
			return MathUtil.getTranslation(motionData.motionList.get(0).get(motionData.skeletonData.root.name)).y;
		}
		
		String getTPose(File seedFile, int seedFrame){
			BVHParser parser = new BVHParser();
			MotionData motionData = parser.parse(seedFile);
			Motion motion = motionData.motionList.get(seedFrame);
//			HashMap<String, Point3d> pointData = Motion.getPointData(motionData.skeletonData, motion);
			String line = "";
			for (int i = 0; i < parser.getJointList().size(); i++) {
				Joint joint = parser.getJointList().get(i);
				Matrix4d m = motion.get(joint.name);
				if (m == null) continue;
				
//				line = getTposeJoint(i, joint, m, line);
				line = getCMUTpose(i, joint, m, line);
			}
			System.out.println("tpose : " + line);
			return line;
		}
		
		private String getCMUTpose(int i, Joint joint, Matrix4d m, String line){
			if (i == 0){
				// Hips
				line += "0 " + MathUtil.getTranslation(m).y + " 0 0 0 0";
//				line += "0 " + MathUtil.getTranslation(m).y + " 0 0 0 0";
			} else {
				if (joint.name.equals("RightUpLeg")){
					line += " 18 0 0";
				} else if (joint.name.equals("LeftUpLeg")){
					line += " -19 0 0";
				} else {
					line += " 0 0 0";
				}
			}
			return line;
		}
		
		private String getTposeJoint(int i, Joint joint, Matrix4d m, String line){
			int lToeOffset = 0;
			int rToeOffset = 0;
			int hipOffset = 0;
			int heightOffset = 0;
			if (index == 1 || index == 3){
				hipOffset = 5;
			}
			if (index == 1){
				rToeOffset = lToeOffset = 10;
			}
			if (index == 4){
				rToeOffset = lToeOffset = 15;
			}
			if (index == 3){
				rToeOffset = -1;
				lToeOffset = 1;
				heightOffset = -2;
			}
			
			if (i == 0){
				// Hips
				line += "0 " + (MathUtil.getTranslation(m).y + heightOffset) + " 0 0 " + hipOffset + " 0";
//				line += "0 " + MathUtil.getTranslation(m).y + " 0 0 0 0";
			} else {
//				CHANNELS 3 Zrotation Xrotation Yrotation
				if (joint.name.equals("Head")){
					line += " 0 -30 0";
				} else if (joint.name.equals("LeftArm")){
					line += " 90 0 0";
				} else if (joint.name.equals("RightArm")){
					line += " -90 0 0";
				} else if (joint.name.equals("LeftFoot")){
					line += " 0 " + lToeOffset + " 0";
				} else if (joint.name.equals("RightFoot")){
	//				if (index == 3){
	//					line += " 22 11 25";
	//				} else {
						line += " 0 " + rToeOffset + " 0";
	//				}
					
				} else if (joint.name.endsWith("UpLeg") || joint.name.equals("Spine")){
					line += " 0 -" + hipOffset + " 0";
				} else {
					line += " 0 0 0";
				}
			}
			return line;
		}
		
		static double[] getToeAngles(String prefix, HashMap<String, Point3d> pointData, SkeletonData skeletonData){
			String j0 = prefix + "Foot";
			String j1 = prefix + "Toe";
			String j2 = prefix + "Toe" + BVHParser.END_POSTFIX;
			Point3d p0 = pointData.get(j0);
			Point3d p1 = pointData.get(j1);
			Point3d p2 = pointData.get(j2);
			Vector3d v1 = skeletonData.get(j1).transition;
			
			Vector3d d1 = new Vector3d();
			d1.sub(p2, p1);
			
			Vector3d d0 = new Vector3d();
			d0.sub(p1, p0);
			
			
			Vector3d upVector = new Vector3d(0, 1, 0);
			double angle1 = Math.toDegrees(upVector.angle(v1));
			double desire1 = Math.toDegrees(upVector.angle(d0));
			return new double[] { desire1 - angle1 };
		}
		
		
		
		private void write(){
			try {
				BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("merged_" + index + ".bvh")));
				BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(seedFile)));
				String line;
				while ((line = br.readLine()) != null) {
					if (line.startsWith("Frames:")){
						line = "Frames: " + (motionCounts + 1);
					}
					bw.write(line + "\r\n");
					
					if (line.startsWith("Frame Time")){
						bw.write(getTPose(seedFile, 0) + "\r\n");
						break;
					}
				}
				
				int startLen = -1;
				for (File file : fileList) {
					br = new BufferedReader(new InputStreamReader(new FileInputStream(file)));
					boolean isStarted = false;
					int thisLen = -1;
					while ((line = br.readLine()) != null) {
						if (line.trim().length() == 0) continue;
						if (isStarted){
							if (startLen < 0){
								startLen = line.split(" ").length;
							}
							if (thisLen < 0){
								thisLen = line.split(" ").length;
								if (startLen != thisLen){
									System.out.println(index + " : " + seedFile.getName() + " : " + file.getName() + " : " +startLen + " : " + thisLen);
								}
							}
							bw.write(line + "\r\n");
						}
						if (line.startsWith("Frame Time")) isStarted = true;
					}
				}
				br.close();
				bw.close();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		
	}
	
	public static HashMap<Vector3d, Person> loadPersonMap(String folderName, boolean singlePerson){
		File folder = new File(folderName);
		HashMap<Vector3d, Person> personMap = new HashMap<Vector3d, Person>();
		for (File file : folder.listFiles()) {
			String name = file.getName();
			if (!name.endsWith("bvh")) continue;
			
			MotionData motionData = new BVHParser().parse(file);
			SkeletonData skeletonData = motionData.skeletonData;
			try{
				Vector3d v;
				if (singlePerson){
					v = new Vector3d();
				} else {
					v = new Vector3d(skeletonData.get("RightLeg").length, skeletonData.get("RightFoot").length, skeletonData.get("RightToe").length);
				}
				Person person = personMap.get(v);
				if (person == null){
					person = new Person(personMap.size() + 1, skeletonData, file);
					personMap.put(v, person);
				}
				person.fileList.add(file);
				person.motionCounts += motionData.motionList.size();
				person.sizeList.add(motionData.motionList.size());
				System.out.println(file.getName() + "\t" + person.index);
			} catch (Exception e){
				System.out.println("file : " + file.getName());
				e.printStackTrace();
			}
		}
		return personMap;
	}

	public static void main(String[] args) {
//		{
//			Matrix3d m1 = new Matrix3d();
//			m1.rotY(Math.toRadians(28));
//			Matrix3d m2 = new Matrix3d();
//			m2.rotZ(Math.toRadians(25));
//			m1.mul(m1, m2);
//			
//			Vector3d e = BVHValidator.matrixToEulerZXY(m1);
//			System.out.println(e);
//			System.exit(0);
//		}
		
		
//		HashMap<Vector3d, Person> personMap = loadPersonMap("D:\\data\\Tennis\\CMU_run_motions\\motion", true);
		
//		HashMap<Vector3d, Person> personMap = loadPersonMap("C:\\Users\\khlee\\git\\MotionGAN\\mrl.motion.neural\\salsa\\motion", true);
//		HashMap<Vector3d, Person> personMap = loadPersonMap("D:\\data\\motionsynth\\walkMotion\\69", true);
		HashMap<Vector3d, Person> personMap = loadPersonMap("C:\\workspace\\MotionData\\walkMotion\\69", true);
		
		for (Person person : personMap.values()){
			person.write();
		}
	}
}
