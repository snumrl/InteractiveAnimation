package mrl.motion.neural.predict;

import static mrl.motion.neural.play.OgreRecorder.BALL_FILE;
import static mrl.motion.neural.play.OgreRecorder.motionFile;

import java.io.File;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.parser.BVHWriter;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.basket.sketch.SketchData;
import mrl.motion.neural.basket.sketch.SketchDrawModule;
import mrl.motion.neural.data.BasketTimeControl;
import mrl.motion.neural.data.BasketTimeControl2;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.predict.MotionOutputEvaluator.MOEvalMatch;
import mrl.motion.neural.run.RuntimeMotionGenerator;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.Configuration;
import mrl.util.FileUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.ListViewerModule;
import mrl.widget.app.SliderModule;
import mrl.widget.app.ListViewerModule.ListListener;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

import org.eclipse.swt.SWT;

public class SketchTestModule extends Module{

	private SketchDrawModule sModule;
	private SyncPredictManager manager;
	
	private ArrayList<ArrayList<double[]>> resultDataList;
	private ArrayList<RuntimeMotionGenerator> generators;
	
	private Preset preset;
	private int[] timeOffset;
	
	@Override
	protected void initializeImpl() {
		getModule(MainViewerModule.class);
		
		
		OgreJNI.open(OgreJNI.courtParam());
		MotionDataConverter.setUseOrientation();
		BasketTimeControl2.includeNextTarget = true;
		BasketTimeControl.useDoubleAction = true;
		manager = new SyncPredictManager("all_time_da_ori");
//		manager = new SyncPredictManager("all_time_ip");
//		manager = new SyncPredictManager("all_time3");
		
		int po = 22;
		
		
//		int t1 = 50;
//		int t2 = 100;
//		int t3 = 150;
//		int t4 = 220;
//		Preset preset = new Preset("giveAndGo.sk", new int[][]{ 
//				{t1, t3+po, t4 },
//				{t1+po, t2, t4 },
//				{t2+po, t3, t4},
//		} );
//		timeOffset = new int[]{ 55, 50, 50, 70 };
//		preset = new Preset("giveAndGo.sk", getFixedTime());
		
		
		timeOffset = new int[]{ 72, 100, 80 };
		preset = new Preset("sketch2.sk", getFixedTime());
//		preset = new Preset("sketch_full2.sk", getFixedTime());
//		
		
		sModule = getModule(SketchDrawModule.class);
		sModule.loadSketch(preset.sketch);
		
		getModule(SliderModule.class).setSliderValues(timeOffset);
		
		
		addMenu("&Menu", "Test &Target\tCtrl+T", SWT.MOD1 + 'T', new Runnable() {
			@Override
			public void run() {
				timeOffset = getModule(SliderModule.class).getSliderValues();
				preset.fixedTime = getFixedTime();
				test(preset);
			}
		});
		addMenu("&Menu", "Test &E\tCtrl+E", SWT.MOD1 + 'E', new Runnable() {
			@Override
			public void run() {
				evaluate();

			}
		});
	}
	
	private int[][] getFixedTime(){
		int po = 22;
		int[] t = new int[timeOffset.length];
		for (int i = 0; i < t.length; i++) {
			if (i > 0) t[i] = t[i-1];
			t[i] += timeOffset[i];
		}
		
		if (timeOffset.length == 3){
			return new int[][]{
					{t[0] + po, t[1], t[2] },
					{t[0], t[1] + po, t[2] },
			};
		} else {
			return new int[][]{
					{t[0], t[2]+po, t[3] },
					{t[0]+po, t[1], t[3] },
					{t[1]+po, t[2], t[3] },
			};
		}
	}
	
	private void evaluate(){
		final MotionOutputEvaluator evaluator = new MotionOutputEvaluator(manager.getNormalizer());
		ArrayList<MOEvalMatch> matchList = evaluator.evaluate(resultDataList.get(0));
		getModule(ListViewerModule.class).setItems(matchList, new ListListener<MOEvalMatch>() {
			@Override
			public String[] getColumnHeaders() {
				return new String[]{ "frame", "distance" };
			}

			@Override
			public String[] getTableValues(MOEvalMatch item) {
				return Utils.toStringArrays(item.windowStart, item.distance);
			}

			@Override
			public void onItemSelection(MOEvalMatch item) {
				Pose2d pose = generators.get(0).poseList.get(item.windowStart);
				RuntimeMotionGenerator g = new RuntimeMotionGenerator();
				g.pose = pose;
				for (int i = 0; i < MotionOutputEvaluator.WINDOW_SIZE; i++) {
					g.update(evaluator.totalData.get(item.matchFrame + i));
				}
				ArrayList<Motion> motionList = new ArrayList<Motion>();
				for (int i = 0; i < item.windowStart; i++) {
					motionList.add(null);
				}
				motionList.addAll(g.motionList);
				getModule(ItemListModule.class).addSingleItem("Match", new MotionData(motionList), new ItemDescription(new Vector3d(0, 1, 0)));
				getModule(MainViewerModule.class).setTimeIndex(item.windowStart);
			}
		});
	}

	private void test(Preset preset){
//		ArrayList<Integer> timeList = poseModule.getTimeList();
//		int[] fixedTime = new int[timeList.size()];
//		for (int i = 0; i < fixedTime.length; i++) {
//			fixedTime[i] = timeList.get(i);
//			if (i > 0) fixedTime[i] += fixedTime[i-1];
//		}
//		PersonPredicter.fixedTime = fixedTime;
		
		SketchData sketch = sModule.getSketchDrawer().getSketchData();
		SketchToPredicter sToP = new SketchToPredicter(sketch, preset.fixedTime);
		
		
		manager.init(sToP.personList);
		System.out.println("## initialized.");
		int fCount = 0;
		for (int i = 0; i < 400; i++) {
			if (manager.iteration()){
				fCount++;
			}
			if (fCount > 30) break;
		}
		
		resultDataList = new ArrayList<ArrayList<double[]>>();
		generators = new ArrayList<RuntimeMotionGenerator>();
		MotionData[] mDataList = new MotionData[sToP.personList.size()];
		for (int i = 0; i < sToP.personList.size(); i++) {
			PersonPredicter p = sToP.personList.get(i);
			mDataList[i] = new MotionData(p.getMotionGenerator().motionList);
			generators.add(p.getMotionGenerator());
			resultDataList.add(p.getMotionGenerator().dataList);
			getModule(ItemListModule.class).addSingleItem("Motion" + i, mDataList[i]);
			getModule(ItemListModule.class).addSingleItem("Pose" + i, p.poseList, new ItemDescription(new Vector3d(0, 1, 0)));
		}
		TimeBasedList<Point3d> ball = manager.getBallTrajectory();
		getModule(ItemListModule.class).addSingleItem("Ball", ball, BallTrajectoryGenerator.ballDescription());
		
		if (OgreJNI.isOpened()){
			OgreJNI.setMotion(mDataList);
			OgreJNI.setBall(ball);
		}
		
		save(preset.sketch, mDataList, ball);
		
		System.out.println("Generation Finished.");
	}
	
	private void save(String name, MotionData[] mDataList, ArrayList<Point3d> ball){
		String folder = "output\\" + name.substring(0, name.length()-".sk".length());
		new File(folder).mkdirs();
		
		BVHWriter writer = new BVHWriter(new File(Configuration.BASE_MOTION_FILE));
		for (int i = 0; i < mDataList.length; i++) {
			writer.write(new File(folder + "\\" + motionFile(i)), mDataList[i]);
		}
		
		String[] ballTrajectory = new String[ball.size()];
		for (int i = 0; i < ballTrajectory.length; i++) {
			Point3d p = ball.get(i);
			ballTrajectory[i] = Utils.toString(p.x, p.y, p.z);
		}
		FileUtil.writeAsString(ballTrajectory, folder + "\\" + BALL_FILE);
		
	}
	
	
	private static class Preset{
		String sketch;
		int[][] fixedTime;
		
		public Preset(String sketch, int[][] fixedTime) {
			this.sketch = sketch;
			this.fixedTime = fixedTime;
		}
	}

	public static void main(String[] args) {
//		BasketDataGenerator.loadBasketData();
		
		MotionDataConverter.setAllJoints();
//		MotionDataConverter.setNoBall();
		MainApplication.run(new SketchTestModule());
		OgreJNI.close();
	}
}
