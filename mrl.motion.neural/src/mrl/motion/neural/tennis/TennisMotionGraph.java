package mrl.motion.neural.tennis;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MGraph;
import mrl.motion.graph.MGraphExplorer;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.ControlDataGenerator.TimeIntervalControl;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.FileUtil;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class TennisMotionGraph {

	public static void main(String[] args) {
//		Configuration.setDataFolder("tennisData");
//		MotionDataConverter.setCMUJointSet();
//		MDatabase database = MDatabase.load();
		
		MDatabase database = TennisDataGenerator.loadTennisData("tennisData");
		
		// generate new motion graph and save to files.
//		MGraphGenerator g = new MGraphGenerator(database);
//		g.saveResult();
		
		RNNDataGenerator.USE_RESIDUAL = true;
		String targetFolder = "tnr_d01";
		
		// load motion graph from saved files.
		MGraph graph = new MGraph(database);
		MathUtil.random = new Random();
		MathUtil.random = new Random(5712);
		MGraphExplorer exp = new MGraphExplorer(graph);
		
		String size = targetFolder.substring("tnr_d".length(), "tnr_d01".length());
		if (size.startsWith("0")) size = size.substring(1);
		ArrayList<int[]> segments = exp.explore(Integer.parseInt(size)*10000);
//		ArrayList<int[]> segments = exp.explore(1000);
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
		if (targetFolder.endsWith("e")){
			segment = tEdit.edit(segment);
		}
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		
		// show generated motion
//		MotionData mData = new MotionData(segment.getMotionList());
//		MainViewerModule.run(mData);
		RNNDataGenerator.generate(segment, new TimeIntervalControl(30, false));
		
		
		targetFolder = "neuralData\\" + targetFolder + "\\data";
		new File(targetFolder).mkdirs();
		
		String[] files = {
				"xData.dat",
				"xNormal.dat",
				"yData.dat",
				"yNormal.dat"
		};
		for (String file : files){
			File oFile = new File(file);
			File tFile = new File(targetFolder + "\\" + file);
			oFile.renameTo(tFile);
		}
		
	}
}
