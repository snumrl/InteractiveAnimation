package mrl.motion.neural.loco;

import java.util.ArrayList;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MGraph;
import mrl.motion.graph.MGraphExplorer;
import mrl.motion.graph.MGraphGenerator;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.neural.walk.WalkControl;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.Configuration;
import mrl.util.Utils;

public class LocomotionGraph {
	
	private static void init(String folder){
		Configuration.setDataFolder("locomotion\\" + folder);
		Configuration.MGRAPH_NODE_CACHE_FILE = "locomotion\\graphs\\" + folder + "_" + Configuration.MGRAPH_NODE_CACHE_FILE;
		Configuration.MGRAPH_EDGE_CACHE_FILE = "locomotion\\graphs\\" + folder + "_" + Configuration.MGRAPH_EDGE_CACHE_FILE;
	}

//	static boolean showMotion = true;
//	static boolean makeGraph = true;
	static boolean showMotion = false;
	static boolean makeGraph = false;
	
	public static void main(String[] args) {

		
		String folder;
		double edgeLimit;
		
//		folder = "gorilla";edgeLimit = 40;
//		FootContactDetection.heightLimit = new double[]{ 20, 13, 11 };
//		FootContactDetection.velocityLimit = new double[]{ 4, 4, 4 };
		
//		folder = "drunk";edgeLimit = 30;
//		FootContactDetection.heightLimit = new double[]{ 15, 11, 9 };
//		FootContactDetection.velocityLimit = new double[]{3 , 3, 3 };
		
//		folder = "walk";edgeLimit = 10;
		
//		folder = "zombie";edgeLimit = 10;
//		FootContactDetection.heightLimit = new double[]{ 15, 11, 9 };
//		FootContactDetection.velocityLimit = new double[]{3 , 3, 3 };
		
//		FootContactDetection.heightLimit = new double[]{ 20, 13, 11 };
//		FootContactDetection.velocityLimit = new double[]{ 8, 8, 8 };
//		folder = "indianDance";edgeLimit = 40;
		
		FootContactDetection.heightLimit = new double[]{ 15, 11, 9 };
		FootContactDetection.velocityLimit = new double[]{3 , 3, 3 };
		folder = "edin_loco";edgeLimit = 20;
		
		folder = "edin_loco";
		
		init(folder);
		Configuration.MGRAPH_EDGE_WEIGHT_LIMIT = edgeLimit;
		
		
		MotionDataConverter.setCMUJointSet();
		MDatabase database = MDatabase.load();
		int len = database.getMotionList().length;
		int sec = (len/30);
		System.out.println(folder + " : " + sec);
		System.exit(0);
//		database.addMirroredData();
		if (makeGraph){
			// generate new motion graph and save to files.
			MGraphGenerator g = new MGraphGenerator(database);
			g.saveResult();
		}
		
		// load motion graph from saved files.
		MGraph graph = new MGraph(database);
		MGraphExplorer exp = new MGraphExplorer(graph);
		ArrayList<int[]> segments = exp.explore(showMotion ? 3000 : 300000);
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam();
		tEdit.timeOffset = 0;
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		
		// show generated motion
		if (showMotion){
			MotionData mData = new MotionData(segment.getMotionList());
			MainViewerModule.run(mData);
		} else {
//			RNNDataGenerator.generate(segment, new PredictiveParameterControl(false));
//			RNNDataGenerator.generate(segment, new IntervalMeanControl());
//			RNNDataGenerator.generate(segment, new SelectiveDirectionControl());
			RNNDataGenerator.generate(segment, new WalkControl());
		}
	}
}
