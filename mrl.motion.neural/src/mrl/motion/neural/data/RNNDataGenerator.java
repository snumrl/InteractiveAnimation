package mrl.motion.neural.data;

import java.util.ArrayList;
import java.util.Arrays;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.graph.MotionSegment;
import mrl.util.Configuration;
import mrl.util.MathUtil;

public class RNNDataGenerator {
	
	public static boolean NO_INPUT = false;
	public static boolean APPEND_POSE_AS_INPUT = false;
	public static boolean USE_RESIDUAL = false;
	public static boolean USE_VELOCITY = false;
	
	public static void setResidualWithInput(){
		USE_RESIDUAL = true;
		APPEND_POSE_AS_INPUT = true;
	}

	public static void generate(MotionSegment segment, ControlDataGenerator c){
		generate(segment, c, false);
	}
	public static void generate(MotionSegment segment, ControlDataGenerator c, boolean append){
		ArrayList<Motion> mList = segment.getMotionList();
		ArrayList<double[]> mDataList = MotionDataConverter.motionToData(segment);
		
		c.setData(mList, mDataList);
		
		ArrayList<double[]> xList = new ArrayList<double[]>();
		ArrayList<double[]> yList = new ArrayList<double[]>();
		for (int i = 0; i < mList.size(); i++) {
			double[] x = c.getControl(i);
			if (x == null) break;
			double[] y = mDataList.get(i+1);
			
			if (APPEND_POSE_AS_INPUT){
				if (NO_INPUT){
					x = mDataList.get(i);
				} else {
					x = MathUtil.concatenate(x, mDataList.get(i));
				}
			}
			
			if (USE_RESIDUAL){
				y = MathUtil.copy(y);
				int poseStart = MotionDataConverter.ROOT_OFFSET;
				double[] prevY = mDataList.get(i);
				for (int j = poseStart; j < y.length; j++) {
					y[j] = y[j] - prevY[j];
				}
			}
			xList.add(x);
			
			if (USE_VELOCITY){
				int poseStart = MotionDataConverter.ROOT_OFFSET;
				int vLen = y.length - poseStart;
				double[] velocity = new double[vLen];
				double[] prevY = mDataList.get(i);
				for (int j = 0; j < velocity.length; j++) {
					int idx = poseStart + j;
					velocity[j] = y[idx] - prevY[idx];
				}
				y = MathUtil.concatenate(y, velocity);
			}
			
			double[] prediction = c.getPrediction(i+1);
			if (prediction != null){
				y = MathUtil.concatenate(y, prediction);
			}
			
			double[] add = c.getHasBall(i+1);
			if (add != null){
				y = MathUtil.concatenate(y, add);
				if (MotionDataConverter.includeBall && add[0] < 0.5){
					for (int j = 0; j < 8; j++) {
						y[j] = Double.NaN;
					}
				}
			}
			yList.add(y);
		}
		
		if (append){
			double[][] xNormal = DataExtractor.readNormalizeInfo("xNormal.dat");
			double[][] yNormal = DataExtractor.readNormalizeInfo("yNormal.dat");
			
			ArrayList<double[]> xData = DataExtractor.readData("xData.dat");
			for (double[] data : xList){
				data = DataExtractor.getNormalizedData(data, xNormal);
				xData.add(data);
			}
			ArrayList<double[]> yData = DataExtractor.readData("yData.dat");
			for (double[] data : yList){
				data = DataExtractor.getNormalizedData(data, yNormal);
				yData.add(data);
			}
			DataExtractor.writeData("xData.dat", xData);
			DataExtractor.writeData("yData.dat", yData);
		} else {
			DataExtractor.writeNormalizeInfo("xNormal.dat", xList, c.getNormalMarking());
			DataExtractor.writeDataWithNormalize("xData.dat", xList, "xNormal.dat");
			
			// foot contact
			DataExtractor.writeNormalizeInfo("yNormal.dat", yList, MotionDataConverter.getNormalMarking());
			DataExtractor.writeDataWithNormalize("yData.dat", yList, "yNormal.dat");
		}
	}
	
	public static void generate(ArrayList<MotionSegment> segmentList, ControlDataGenerator c){
		ArrayList<ArrayList<double[]>> xData = new ArrayList<ArrayList<double[]>>();
		ArrayList<ArrayList<double[]>> yData = new ArrayList<ArrayList<double[]>>();
		ArrayList<double[]> xTotal = new ArrayList<double[]>();
		ArrayList<double[]> yTotal = new ArrayList<double[]>();
		for (MotionSegment segment : segmentList){
			ArrayList<Motion> mList = segment.getMotionList();
			ArrayList<double[]> mDataList = MotionDataConverter.motionToData(segment);
			
			c.setData(mList, mDataList);
			
			ArrayList<double[]> xList = new ArrayList<double[]>();
			ArrayList<double[]> yList = new ArrayList<double[]>();
			for (int i = 0; i < mList.size(); i++) {
				double[] x = c.getControl(i);
				if (x == null) break;
				double[] y = mDataList.get(i+1);
				
				if (APPEND_POSE_AS_INPUT){
					if (NO_INPUT){
						x = mDataList.get(i);
					} else {
						x = MathUtil.concatenate(x, mDataList.get(i));
					}
				}
				
				if (USE_RESIDUAL){
					y = MathUtil.copy(y);
					int poseStart = MotionDataConverter.ROOT_OFFSET;
					double[] prevY = mDataList.get(i);
					for (int j = poseStart; j < y.length; j++) {
						y[j] = y[j] - prevY[j];
					}
				}
				xList.add(x);
				
				if (USE_VELOCITY){
					int poseStart = MotionDataConverter.ROOT_OFFSET;
					int vLen = y.length - poseStart;
					double[] velocity = new double[vLen];
					double[] prevY = mDataList.get(i);
					for (int j = 0; j < velocity.length; j++) {
						int idx = poseStart + j;
						velocity[j] = y[idx] - prevY[idx];
					}
					y = MathUtil.concatenate(y, velocity);
				}
				
				double[] prediction = c.getPrediction(i+1);
				if (prediction != null){
					y = MathUtil.concatenate(y, prediction);
				}
				
				double[] add = c.getHasBall(i+1);
				if (add != null){
					y = MathUtil.concatenate(y, add);
					if (MotionDataConverter.includeBall && add[0] < 0.5){
						for (int j = 0; j < 8; j++) {
							y[j] = Double.NaN;
						}
					}
				}
				yList.add(y);
			}
			
			
			xTotal.addAll(xList);
			yTotal.addAll(yList);
			xData.add(xList);
			yData.add(yList);
		}
		
		double[][] xNormal = DataExtractor.writeNormalizeInfo("xNormal.dat", xTotal, c.getNormalMarking());
		for (int i = 0; i < xData.size(); i++) {
			xData.set(i, DataExtractor.getNormalizedData(xData.get(i), xNormal));
		}
//		("xData.dat", xData, "xNormal.dat");
		DataExtractor.writeListData("xData.dat", xData);
		
		// foot contact
		double[][] yNormal = DataExtractor.writeNormalizeInfo("yNormal.dat", yTotal, MotionDataConverter.getNormalMarking());
		for (int i = 0; i < yData.size(); i++) {
			yData.set(i, DataExtractor.getNormalizedData(yData.get(i), yNormal));
		}
		DataExtractor.writeListData("yData.dat", yData);
	}
	
	
	public static MDatabase loadCMUDatabase(String folder){
		Configuration.MOTION_FOLDER = folder;
		MDatabase database = MDatabase.load();
		
//		FootContactDetection.CONTACT_MARGIN = 2;
//		FootContactDetection.rightFootJoints = new String[]{ "RightFoot", "RightToeBase", "RightToeBase_End" } ;
//		FootContactDetection.leftFootJoints = new String[]{ "LeftFoot", "LeftToeBase", "LeftToeBase_End" } ;
//		MotionDataConverter.KeyJointList = new String[]{
//				"Head_End",
////				"Hips",
//				
//				"LeftHand",
////				"LeftHandIndex1",
////				"LThumb_End",
//				"LeftFoot",
//				"LeftToeBase",
//				"RightHand",
////				"RightHandIndex1",
////				"RThumb_End",
//				"RightFoot",
//				"RightToeBase",
//				
//				"LeftArm",
//				"RightArm",
//				
//				"LeftForeArm",
//				"LeftLeg",
//				"RightForeArm",
//				"RightLeg",
//			};
		
		MotionDataConverter.setCMUAllJoints();
		
		return database;
	}
	
	
	
}
