package mrl.motion.neural.predict;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;

import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.data.DataExtractor;
import mrl.motion.neural.data.Normalizer;
import mrl.motion.neural.run.MultiPersonRNNPython;
import mrl.motion.neural.run.PythonRuntime;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.Pair;
import mrl.util.Utils;

public class SyncPredictManager {

	private MultiPersonRNNPython python;
	private ArrayList<PersonPredicter> personList;
	private String name;
	
	public SyncPredictManager(String name) {
		this.name = name;
	}
	
	public Normalizer getNormalizer(){
		return python.normal;
	}
	
	public void init(ArrayList<PersonPredicter> personList){
		this.personList = personList;
		if (python == null){
			python = new MultiPersonRNNPython(name, personList.size());
		}
		for (PersonPredicter p : personList){
			p.normal = python.normal;
		}
		
		double[][] startMotions = new double[personList.size()][];
		for (int i = 0; i < startMotions.length; i++) {
			Pair<boolean[], Integer> valid = DataExtractor.checkStdValid(python.normal.yMeanAndStd[1]);
			startMotions[i] = new double[valid.second];
			if (i == 0){
				startMotions[i] = python.normal.yList.get(1000);
			}
		}
		python.model.setStartMotion(startMotions);
	}
	
	private int updatePassCount = 0;
	public boolean iteration(){
		double[][] eInput = new double[personList.size()][];
		for (int i = 0; i < eInput.length; i++) {
			eInput[i] = personList.get(i).getEstimationGoal();
		}
		
		for (int i = 0; i < personList.size(); i++) {
//			personList.get(i).updateRemainTime(personList, unNormaledEstimated);
		}
		
		double[][] pInput = new double[personList.size()][];
		for (int i = 0; i < pInput.length; i++) {
			pInput[i] = personList.get(i).getPredictGoal();
		}
		
		double[][] predicted = python.model.predict(pInput);
		for (int i = 0; i < personList.size(); i++) {
			double[] output = python.normal.deNormalizeY(predicted[i]);
			personList.get(i).updateMotion(output);
		}
		
		for (PersonPredicter p : personList){
			if (p.isFinished) return true;
		}
		return false;
	}
	
	private boolean isAnyUpdateRequired(){
		for (PersonPredicter p : personList){
			if (p.isTimeUpdateRequired()) return true;
		}
		return false;
	}
	
	public TimeBasedList<Point3d> getBallTrajectory(){
		BallTrajectoryGenerator g = new BallTrajectoryGenerator();
		int frames = personList.get(0).getMotion().size();
		TimeBasedList<Point3d> trajectory = new TimeBasedList<Point3d>();
		for (int i = 0; i < frames; i++) {
			trajectory.add(getBall(i));
		}
		
		ArrayList<int[]> bounceIntervals = Utils.getNullIntervals(trajectory, true);
		if (bounceIntervals.get(0)[0] < 2) bounceIntervals.remove(0);
		for (int i = 0; i < bounceIntervals.size(); i++) {
			int[] interval = bounceIntervals.get(i);
			int sIdx = interval[0]-1;
			int eIdx = interval[1]+1;
			if (eIdx >= trajectory.size() - 10) break;
			Point3d start = trajectory.get(sIdx - 1);
			Point3d end = trajectory.get(eIdx + 1);
			ArrayList<Point3d> t = g.getBounceTrajectory(start, end, sIdx, eIdx);
			for (int j = 0; j < t.size(); j++) {
				trajectory.set(sIdx+j, t.get(j));
			}
		}
		
		
		int[] last = Utils.last(bounceIntervals);
		if (trajectory.size() - last[1] < 10){
			int end = last[0] - 1;
			Point3d start = trajectory.get(end - 1);
			ArrayList<Point3d> t = g.getShootAndFall(start, 50);
			for (int i = 0; i < t.size(); i++) {
				int idx = end + i;
				if (idx >= trajectory.size()) break;
				trajectory.set(idx, t.get(i));
			}
		}
		
		return trajectory;
	}
	
	private Point3d getBall(int fIndex){
		Point3d ball = null;
		for (PersonPredicter person : personList){
			Point3d b = person.getMotionGenerator().ballList.get(fIndex);
			if (b == null) continue;
			if (ball != null){
				System.out.println("both ball!! :: " + fIndex);
//				throw new RuntimeException();
			}
			ball = b;
		}
		return ball;
	}
}
