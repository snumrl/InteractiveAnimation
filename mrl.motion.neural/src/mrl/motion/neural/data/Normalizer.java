package mrl.motion.neural.data;

import java.util.ArrayList;

public class Normalizer {
	
	public double[][] xMeanAndStd;
	public double[][] yMeanAndStd;
	public ArrayList<double[]> xList;
	public ArrayList<double[]> yList;

	public Normalizer(String folder){
		if (folder != null){
			folder = "neuralData" + "\\" + folder + "\\data";
		} else {
			folder = ".";
		}
		xMeanAndStd = DataExtractor.readNormalizeInfo(folder + "\\xNormal.dat");
		yMeanAndStd = DataExtractor.readNormalizeInfo(folder + "\\yNormal.dat");
		xList = DataExtractor.readData(folder + "\\xData.dat");
		yList = DataExtractor.readData(folder + "\\yData.dat");
	}
	
	
	public Normalizer(double[][] xMeanAndStd, double[][] yMeanAndStd,
			ArrayList<double[]> xList, ArrayList<double[]> yList) {
		this.xMeanAndStd = xMeanAndStd;
		this.yMeanAndStd = yMeanAndStd;
		this.xList = xList;
		this.yList = yList;
	}


	public double[] normalizeX(double[] x){
//		return normalizeX(x, 0);
		return DataExtractor.getNormalizedData(x, xMeanAndStd);
	}
	public double[] normalizeX(double[] x, int offset){
		return DataExtractor.getNormalizedData(x, xMeanAndStd, offset);
	}
	public double[] deNormalizeX(double[] x){
		return deNormalizeX(x, 0);
	}
	public double[] deNormalizeX(double[] x, int offset){
		return DataExtractor.getUnnormalizedOutput(x, xMeanAndStd, offset);
	}
	public double[] normalizeY(double[] y){
		return DataExtractor.getNormalizedData(y, yMeanAndStd);
	}
	public double[] deNormalizeY(double[] y){
		return DataExtractor.getUnnormalizedOutput(y, yMeanAndStd);
	}
	
	public static void printDataInfo(Object[] config, double[][] meanAndStd){
		double[] mean = meanAndStd[0];
		double[] std = meanAndStd[1];
		int idx = 0;
		for (int i = 0; i < config.length; i+=2) {
			String label = (String)config[i];
			int offset = (int)config[i + 1];
			for (int axis = 1; axis <= offset; axis++) {
				System.out.println(String.format("%s : %.5f / %.5f", label + axis, mean[idx], std[idx]));
				idx++;
			}
		}
	}
}
