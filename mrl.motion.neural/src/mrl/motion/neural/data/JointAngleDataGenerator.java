package mrl.motion.neural.data;

import java.io.DataInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.util.Pair;
import mrl.util.Utils;

public class JointAngleDataGenerator {
	
	public static String[] AngleJointList = {
		"Head",
		"Hips",
		"LeftArm",
		"LeftFoot",
		"LeftForeArm",
		"LeftHand",
		"LeftLeg",
		"LeftShoulder",
		"LeftToe",
		"LeftUpLeg",
		"Neck",
		"RightArm",
		"RightFoot",
		"RightForeArm",
		"RightHand",
		"RightLeg",
		"RightShoulder",
		"RightToe",
		"RightUpLeg",
		"Spine",
		"Spine1",
		"Spine2",
	};
	
	private static MotionTransform t = new MotionTransform();
	
//	public static Motion

	public void generate(MotionSegment segment){
		ArrayList<double[]> mDataList = MotionDataConverter.motionToData(segment);
		ArrayList<double[]> jointData = getJointAngleData(Utils.toArray(segment.getMotionList()));
		
		DataExtractor.writeNormalizeInfo("xNormal.dat", mDataList);
		DataExtractor.writeDataWithNormalize("xData.dat", mDataList, "xNormal.dat");
		// foot contact
		DataExtractor.writeNormalizeInfo("yNormal.dat", jointData);
		DataExtractor.writeDataWithNormalize("yData.dat", jointData, "yNormal.dat");
	}
	
	
	public static ArrayList<double[]> getJointAngleData(Motion[] mList){
		ArrayList<double[]> list = new ArrayList<double[]>();
		for (Motion motion : mList){
			double[] data = new double[AngleJointList.length*3+1];
			boolean error = false;
			for (int i = 0; i < AngleJointList.length; i++) {
				Matrix4d matrix = motion.get(AngleJointList[i]);
				Matrix4d m = new Matrix4d(t.sampleMotion.get(AngleJointList[i]));
				m.invert();
				m.mul(matrix);
				matrix = m;
				
				Vector3d v = MathUtil.toVector(matrix);
				if (v.length() > Math.PI - 0.1 && AngleJointList[i].equals("RightShoulder")){
					AxisAngle4d a = new AxisAngle4d();
					a.set(matrix);
					System.out.println(motion.motionData.file.getName() + " : " + motion.frameIndex + " : " + list.size() + " : " + AngleJointList[i] + "\t\t" + v.length() + " : " + v);
					throw new RuntimeException();
//					error =  true;
//					break;
				}
				data[i*3 + 0] = v.x;
				data[i*3 + 1] = v.y;
				data[i*3 + 2] = v.z;
			}
			data[data.length-1] = motion.root().m13;
//			if (error) continue;
			list.add(data);
		}
		return list;
	}
	
	public static Motion toMotion(double[] data){
		Motion motion = new Motion(t.sampleMotion);
		for (int j = 0; j < AngleJointList.length; j++) {
			Vector3d v = new Vector3d(data[j*3 + 0], data[j*3 + 1], data[j*3 + 2]);
			Matrix4d matrix = MathUtil.toMatrix(v);
			
			Matrix4d m = new Matrix4d(t.sampleMotion.get(AngleJointList[j]));
			m.mul(matrix);
			matrix = m;
			
			motion.put(AngleJointList[j], matrix);
		}
		motion.get(t.skeletonData.root.name).setTranslation(new Vector3d(0, 80, 0));
		return motion;
	}
	
	static void printWeights(){
		HashMap<String, Double> weightMap  = new HashMap<String, Double>();
		for (int i = 0; i < t.weightList.length; i++) {
			weightMap.put(t.keyList[i], t.weightList[i]);
		}
		weightMap.put("Hips", t.rootOriWeight);
		double[] data = new double[AngleJointList.length*3+1];
		for (int i = 0; i < AngleJointList.length; i++) {
			double w = weightMap.get(AngleJointList[i]);
			data[i*3 + 0] = w;
			data[i*3 + 1] = w;
			data[i*3 + 2] = w;
		}
		data[data.length-1] = t.rootOriWeight;
		
		Normalizer normal = new Normalizer("joint_d");
		Pair<boolean[], Integer> pair = DataExtractor.checkStdValid(normal.yMeanAndStd[1]);
		System.out.println("len : " + pair.second);
		boolean[] isValid = pair.first;
		for (int i = 0; i < isValid.length; i++) {
			if (isValid[i]){
				System.out.print(String.format("%.4f, ", data[i]*20));
			}
		}
		System.out.println();
//		normal.yMeanAndStd
		
		System.exit(0);
	}
	
	public static void main(String[] args) {
		{
			MDatabase database = BasketDataGenerator.loadBasketData();
			MotionData motionData = database.getMotionDataList()[0];
			FootSlipCleanup.clean(motionData);
			MainViewerModule.run(motionData);
			System.exit(0);
			
			ArrayList<Motion> mList = database.getMotionDataList()[0].motionList;
			for (Motion m : mList){
				AxisAngle4d a = new AxisAngle4d();
				a.set(m.get("RightForeArm"));
//				a.set(m.get("LeftForeArm"));
				System.out.println(a);
			}
			System.exit(0);
		}
		
		
		printWeights();

		MotionSegment segment = new DribbleGraphDataGenerator().generateMotion(200000);
		new JointAngleDataGenerator().generate(segment);
	}
}
