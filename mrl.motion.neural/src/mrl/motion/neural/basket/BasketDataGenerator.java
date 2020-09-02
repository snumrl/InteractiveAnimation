package mrl.motion.neural.basket;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.clip.MClip;
import mrl.motion.data.clip.MClipDistance;
import mrl.motion.data.clip.MClipManager;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.BasketActivationControl;
import mrl.motion.neural.data.BasketGameControl;
import mrl.motion.neural.data.BasketGameControl2;
import mrl.motion.neural.data.BasketGameControl3;
import mrl.motion.neural.data.BasketPassControl;
import mrl.motion.neural.data.BasketPickupControl;
import mrl.motion.neural.data.BasketTimeControl;
import mrl.motion.neural.data.BasketTimeControl2;
import mrl.motion.neural.data.ControlDataGenerator;
import mrl.motion.neural.data.DribbleGraphDataGenerator;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.param.MotionEditParam;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.Configuration;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;

public class BasketDataGenerator {

	public static String CLIP_FILE = "basekClip.dat";
	public static double TRANSITIONABLE_DISTANCE = 10;
	
	public static int MAX_TIME = 30*4;
	public static double MAX_TARGET_DISTANCE = Integer.MAX_VALUE;
	public static double MAX_CONTROL_DISTANCE = 100;
	public static boolean doEdit = true;
	
	public static MDatabase loadBasketData(){
		Configuration.setDataFolder("basketData");
		MDatabase database = MDatabase.load();
		BallTrajectoryGenerator.updateBallContacts(database);
		return database;
	}
	
	
	public MDatabase database;
	private MClipManager cManager;
	
	public MotionSegment segment;
	public ArrayList<ControlTarget> targetList;
	
	public BasketDataGenerator(){
		database = loadBasketData();
		cManager = new MClipManager(database, CLIP_FILE);
		System.exit(0);
	}
	
	public TrajectoryEditParam generateDribble(){
		BasketSequenceGenerator g = new BasketSequenceGenerator(cManager);
		MClipManager cManager = new MClipManager(database, BasketDataGenerator.CLIP_FILE);
		g.dribbleOnly = true;
		ArrayList<MClip> clipList = g.generate(25);
		clipList = Utils.cut(clipList, 8, -1);
		segment = cManager.generateMotion(clipList);
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam(); 
		tEdit.preOffset = new double[][]{
				{ 0.8, 25, -0.10 },	
				{ 0.9, 30, 0.2 },	
				{ 0.8, -30, -0.15 },	
		};
		segment = tEdit.edit(segment);
		return tEdit;
	}
	
	
	private ArrayList<MClip> clipSequence(MClip clip, int size){
		int len = clip.length();
		int sum = 0;
		ArrayList<MClip> list = new ArrayList<MClip>();
		while (true){
			list.add(clip);
			sum += len;
			if (sum > size) break;
		}
		return list;
	}
	
	static int basicSize = 0;
	static boolean basicFromGraph = false;
	public MotionSegment generateMotion(int size){
		BasketSequenceGenerator g = new BasketSequenceGenerator(cManager);
		MClipManager cManager = new MClipManager(database, BasketDataGenerator.CLIP_FILE);
		ArrayList<MClip> clipList = new ArrayList<MClip>(); 
		MotionSegment prefix = null;
		if (basicSize > 0 && !basicFromGraph){
			MClip walk = cManager.findClip("s_008_4_1.bvh", 1487);
			MClip run = cManager.findClip("s_006_7_1.bvh", 1020);
			MClip dribble = cManager.findClip("s_003_1_1.bvh", 281);
			clipList.addAll(clipSequence(walk, basicSize));
			clipList.addAll(clipSequence(run, basicSize));
			if (BasketGraph.PICKUP_MODE){
				clipList.add(cManager.labelMap.get("pickup").get(0));
			} else {
				clipList.add(cManager.labelMap.get("pass'").get(0));
			}
			clipList.addAll(clipSequence(dribble, basicSize));
		}
		clipList.addAll(g.generate(size));
//		System.exit(0);
		
//		try {
//			System.out.println("Sleppp--------");
//			System.out.flush();
//			Thread.sleep(100000000);
//		} catch (InterruptedException e) {
//			e.printStackTrace();
//		}
		
		segment = cManager.generateMotion(clipList);
		
		if (basicSize > 0 && basicFromGraph){
			DribbleGraphDataGenerator.isWalk = true;
			DribbleGraphDataGenerator wg = new DribbleGraphDataGenerator(database);
			MotionSegment walk = wg.generateMotion(basicSize, true);
			
			DribbleGraphDataGenerator.isWalk = false;
			DribbleGraphDataGenerator dg = new DribbleGraphDataGenerator(database);
			MotionSegment dribble = dg.generateMotion(basicSize, true);
			
			MClip clip;
			if (BasketGraph.PICKUP_MODE){
				clip = cManager.labelMap.get("pickup").get(0);
			} else {
				clip = cManager.labelMap.get("pass'").get(0);
			}
			MotionSegment middle = new MotionSegment(database.getMotionList(), clip.startMotionIndex, clip.lastMotionIndex);
			
			prefix = MotionSegment.stitch(walk,  middle, true);
			prefix = MotionSegment.stitch(prefix,  dribble, true);
			segment = MotionSegment.stitch(prefix,  segment, false);
			
			ArrayList<MClip> newClipList = new ArrayList<MClip>();
			newClipList.add(clip);
			newClipList.addAll(clipList);
			clipList = newClipList;
		}
		
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam(0.2, 45, 0.08);
		if (doEdit){
			segment = tEdit.edit(segment);
		}
		FootSlipCleanup.clean(segment);
		targetList = getControlTargets(segment, clipList);
		return segment;
	}
	
	public void generateActivationData(int size){
		generateMotion(size);
		BasketActivationControl control = new BasketActivationControl(database, targetList);
		RNNDataGenerator.generate(segment, control);
	}
	
	public void generatePassData(int size, boolean includeRootMove){
		generateMotion(size);
		BasketPassControl control = new BasketPassControl(database, targetList);
		control.includeRootMove = includeRootMove;
		RNNDataGenerator.generate(segment, control, false);
	}
	
	public void generateTimeData(int size, boolean includeRootMove, boolean append){
		generateMotion(size);
//		BasketTimeControl2 control = new BasketTimeControl2(database, targetList);
		BasketTimeControl control = new BasketTimeControl(database, targetList);
		control.includeRootMove = includeRootMove;
		RNNDataGenerator.generate(segment, control, append);
	}
	public void generateGameData(int size,boolean append){
		generateMotion(size);
//		BasketGameControl3 control = new BasketGameControl3(database, targetList);
//		BasketGameControl2 control = new BasketGameControl2(database, targetList);
		BasketGameControl control = new BasketGameControl(database, targetList);
		RNNDataGenerator.generate(segment, control, append);
	}
	public void generatePickupData(int size){
		BasketGraph.PICKUP_MODE = true;
		generateMotion(size);
		BasketPickupControl control = new BasketPickupControl(database, targetList);
		RNNDataGenerator.generate(segment, control, false);
	}
	
	void showMotion(){
		ArrayList<Motion> motion = segment.getMotionList();
		BallTrajectoryGenerator g = new BallTrajectoryGenerator();
		TimeBasedList<Point3d> ballTrajectory = new TimeBasedList<Point3d>(); 
		ballTrajectory.addAll(g.generate(motion));
		MainViewerModule.runWithDescription(new Object[]{
					new MotionData(motion), ballTrajectory
			}, new ItemDescription[]{
					null, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS)
		});
		System.exit(0);
	}
	
	public static double[] getControlData(Pose2d targetPose){
		targetPose = new Pose2d(targetPose);
		if (MathUtil.length(targetPose.position) > MAX_TARGET_DISTANCE){
			targetPose.position.scale(MAX_TARGET_DISTANCE/MathUtil.length(targetPose.position));
		}
		double[] control = targetPose.toArray();
		Vector2d normalP = new Vector2d(targetPose.position);
		if (normalP.length() > MAX_CONTROL_DISTANCE){
			normalP.scale(MAX_CONTROL_DISTANCE/normalP.length());
		}
		control = MathUtil.concatenate(control, new double[]{ normalP.x, normalP.y });
		return control;
	}
	
	public static double[] getControlData(Pose2d targetPose, int remainTime){
		double[] control = getControlData(targetPose);
		remainTime = Math.min(remainTime, MAX_TIME);
		control = MathUtil.concatenate(control, new double[]{ remainTime });
		return control;
	}
	
	public static double[] getControlData(Pose2d targetPose, int remainTime, double rootMove){
		double[] control = getControlData(targetPose);
		remainTime = Math.min(remainTime, MAX_TIME);
		control = MathUtil.concatenate(control, new double[]{ rootMove, remainTime });
		return control;
	}
	
	public static double[] getActionType(String type){
		double[] data = new double[3];
		int index = -1;
		if (type.equals("pass'")){
			index = 1;
		} else if (type.equals("shoot")){
			index = 2;
		} else {
			index = 0;
		}
		data[index] = 1;
//		System.out.println("action :: " + type + " : " + index);
		return data;
	}
	
	private static boolean isActionClip(String type){
		if (type.contains("pass")) return true;
		if (type.equals("shoot")) return true;
		if (type.equals("pickup")) return true;
		
		if (type.equals("catch")) return true;
		if (type.equals("shoot_near")) return true;
		return false;
	}
	
	private ArrayList<ControlTarget> getControlTargets(MotionSegment segment, ArrayList<MClip> clipList){
		ArrayList<Motion> mList = segment.getMotionList();
		ArrayList<ControlTarget> targetList = new ArrayList<ControlTarget>();
		
		int mIndex = 0;
		int cIndex = 0;
		while (true){
			MClip interactionClip = null;
			for (; cIndex < clipList.size(); cIndex++) {
				if (isActionClip(clipList.get(cIndex).type)){
					interactionClip = clipList.get(cIndex);
					break;
				}
			}
			if (interactionClip == null) break;
			System.out.println(mIndex + " : " + interactionClip);
			int iMotionIndex = interactionClip.interactionMotionIndex();
			int matchIndex = -1;
			int startMIndex = mIndex;
			for (; mIndex < mList.size(); mIndex++) {
				if (mList.get(mIndex).motionIndex == iMotionIndex || mList.get(mIndex).motionIndex == (iMotionIndex+1)){
					matchIndex = mIndex;
					break;
				}
			}
			if (matchIndex < 0){
				System.out.println("no matcing :: " + cIndex+ " : " + clipList.get(cIndex) + " : " + iMotionIndex + " : " + startMIndex);
				throw new RuntimeException();
			}
			
			ControlTarget target = new ControlTarget(interactionClip, cIndex, mIndex);
			if (cIndex > 0){
				target.prevClip = clipList.get(cIndex-1);
			}
			targetList.add(target);
			cIndex++;
			mIndex+=2;
		}
		return targetList;
	}
	
	public static class ControlTarget{
		public MClip clip;
		public int cIndex;
		public int mIndex;
		
		public int prevMargin = -1;
		public int postMargin = -1;
		
		public MClip prevClip;
		
		public ControlTarget(MClip clip, int cIndex, int mIndex) {
			this.clip = clip;
			this.cIndex = cIndex;
			this.mIndex = mIndex;
		}
		
		public Pose2d getInteractionPose(ArrayList<Motion> generatedMotion, MDatabase database){
			int interMotionIndex = clip.interactionMotionIndex();
			Motion originMotion = database.getMotionList()[interMotionIndex];
//			Motion interactMotion = segment.getMotionList().get(interactionIndex);
			Matrix4d m = new Matrix4d();
			m.rotY(clip.angleOffset);
			Vector3d vX = new Vector3d(1, 0, 0);
			m.transform(vX);
			Vector3d interactionDirection = new Vector3d(vX);
			Point3d interactionPosition = new Point3d(MathUtil.getTranslation(originMotion.root()));
			interactionPosition.y = 0;
			if (Double.isNaN(interactionDirection.x)){
				System.out.println("invalid clip :: " + clip + " : " + clip.angleOffset);
				throw new RuntimeException();
			}
			Pose2d interactionPose = new Pose2d(interactionPosition, interactionDirection);
			Pose2d iMotionPose = PositionMotion.getPose(originMotion);
			Pose2d relativePose = Pose2d.relativePose(iMotionPose, interactionPose);
			
			Motion gMotion = generatedMotion.get(mIndex);
			if (gMotion.motionIndex != clip.interactionMotionIndex()) throw new RuntimeException();
			return PositionMotion.getPose(gMotion).localToGlobal(relativePose);
		}
	}

	
	static void calcClipDistance(){
		MDatabase database = loadBasketData();
		MClipDistance.calcClipDistance(database, "basekClip.dat");
		System.exit(0);
	}
	
	public static void main(String[] args) {
//		calcClipDistance();
//		{
//			BasketDataGenerator g = new BasketDataGenerator();
//			ArrayList<Integer> oList = new ArrayList<Integer>();
//			for (MClip c : g.cManager.totalClips){
//				if (isActionClip(c.type)){
//					oList.add(c.interactionOffset);
//					if (c.interactionOffset >= 30){
//						System.out.println(c.interactionOffset + " \t" + c);
//					}
//				}
//			}
//			double[] s = MathUtil.getStatistics(MathUtil.toDouble(Utils.toIntArray(oList)));
//			System.out.println(Arrays.toString(s));
//			System.exit(0);;
//		}
//		calcClipDistance();
		
//		new BasketDataGenerator().generateDribble();
//		new BasketDataGenerator().generatePass(1000);
//		MathUtil.random.setSeed(6622354);
//		new BasketDataGenerator().generateTimeData(20000, false);
		
		MotionDataConverter.setAllJoints();
		MathUtil.random.setSeed(6254);
//		MathUtil.random.setSeed(72723);
//		MathUtil.random.setSeed(29132);
		
//		basicSize = 200;
//		ArrayList<Motion> motion = new BasketDataGenerator().generateMotion(100).getMotionList();
//		BallTrajectoryGenerator g = new BallTrajectoryGenerator();
//		TimeBasedList<Point3d> ballTrajectory = new TimeBasedList<Point3d>(); 
//		ballTrajectory.addAll(g.generate(motion));
//		MainViewerModule.runWithDescription(new Object[]{
//					new MotionData(motion), ballTrajectory
//			}, new ItemDescription[]{
//					null, new ItemDescription(BallTrajectoryGenerator.BALL_RADIUS)
//		});
//		
//		System.exit(0);
		
		MotionEditParam.DO_TIME_EDIT = false;
		
		basicSize = 0;
		BasketGraph.PASS_MODE = true;
		MotionDataConverter.setUseOrientation();
		BasketPassControl.useDoubleAction = true;
		BasketPassControl.includeActivation = false;
		BasketPassControl.useBoundedParam = false;
		new BasketDataGenerator().generatePassData(10000, false);
		System.exit(0);
		
//		// game
//		basicFromGraph = true;
////		BasketSequenceGenerator.PICK_RANDOM = true;
//		BasketGraph.INCLUDE_CATCH = true;
//		basicSize = 60000;
//		MotionDataConverter.setUseOrientation();
////		RNNDataGenerator.USE_VELOCITY = true;
//		doEdit = true;
////		ControlDataGenerator.includePrediction = true;
//		BasketGameControl.useDoubleAction = true;
//		new BasketDataGenerator().generateGameData(15000, false);
//		System.exit(0);
		
		basicSize = 60000;
		BasketGraph.ALL_TIME = true;
//		MotionDataConverter.setNoBallVelocity();
//		BasketTimeControl2.includeActivation = false;
//		MathUtil.random.setSeed(6254);
		BasketTimeControl2.includeNextTarget = true;
		BasketTimeControl.useDoubleAction = true;
		MotionDataConverter.setUseOrientation();
//		ControlDataGenerator.includePrediction = true;
//		new BasketDataGenerator().generateTimeData(300, false, false);
		new BasketDataGenerator().generateTimeData(15000, false, false);
//		MathUtil.random.setSeed(32391);
//		MathUtil.random.setSeed(542354);
//		new BasketDataGenerator().generateTimeData(15000, true, true);
		System.exit(0);
		
		//  pickup ( shoot )
//		basicFromGraph = true;
//		BasketGraph.INCLUDE_CATCH = true;
//		BasketSequenceGenerator.PICK_RANDOM = true;
//		basicSize = 90000;
//		new BasketDataGenerator().generatePickupData(7000);
		
		MathUtil.random.setSeed(662234);
		new BasketDataGenerator().generateDribble();
	}
}
