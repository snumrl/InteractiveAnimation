package mrl.motion.neural.predict;

import java.util.ArrayList;

import mrl.motion.neural.data.Normalizer;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class MotionOutputEvaluator {

	public static int WINDOW_SIZE = 20;
	public ArrayList<double[]> totalData;
	private int wIndex;

	public MotionOutputEvaluator(Normalizer normalizer) {
		totalData = new ArrayList<double[]>();
		for (double[] data : normalizer.yList){
			totalData.add(normalizer.deNormalizeY(data));
		}
	}
	
	public ArrayList<MOEvalMatch> evaluate(ArrayList<double[]> output){
		ArrayList<MOEvalMatch> matchList = new ArrayList<MotionOutputEvaluator.MOEvalMatch>();
		for (int i = 0; i < output.size()-WINDOW_SIZE; i+=WINDOW_SIZE/2) {
			wIndex = i;
			System.out.println("progress :: " + wIndex + " / " + output.size());
			ArrayList<double[]> window = Utils.cut(output, i, i+WINDOW_SIZE - 1);
			MOEvalMatch match = findMatch(window);
			match.windowStart = i;
			matchList.add(match);
		}
		return matchList;
	}
	
	private MOEvalMatch findMatch(ArrayList<double[]> window){
		double minDist = Integer.MAX_VALUE;
		int matchIdx = -1;
		for (int i = 0; i < totalData.size() - window.size(); i+=2) {
			double d = distance(window, totalData, i);
			if (d < minDist){
				minDist = d;
				matchIdx = i;
			}
		}
		MOEvalMatch match = new MOEvalMatch();
		match.matchFrame = matchIdx;
		match.distance = minDist;
		return match;
	}
	
	private double distance(ArrayList<double[]> window, ArrayList<double[]> totalList, int index){
		double dSum = 0;
		for (int i = 0; i < window.size(); i++) {
			dSum += MathUtil.distance(window.get(i), totalList.get(index + i));
		}
		return dSum/window.size();
	}
	
	public static class MOEvalMatch{
		public int matchFrame;
		public double distance;
		public int windowStart;
		
	}
}
