package mrl.motion.neural.basket;

import static mrl.motion.neural.basket.BasketMotionGraph.BFrameType.After;
import static mrl.motion.neural.basket.BasketMotionGraph.BFrameType.Before;
import static mrl.motion.neural.basket.BasketMotionGraph.BFrameType.Interaction;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Catch;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Dribble;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Pass;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Pass_;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Pickup;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Shoot;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Shoot_Near;
import static mrl.motion.neural.basket.BasketMotionGraph.BasketType.Walk;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;

import mrl.motion.data.Contact;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.motion.data.MotionData;
import mrl.motion.data.trasf.FootSlipCleanup;
import mrl.motion.graph.MGraph;
import mrl.motion.graph.MGraphExplorer;
import mrl.motion.graph.MGraphGenerator;
import mrl.motion.graph.MotionSegment;
import mrl.motion.neural.data.RNNDataGenerator;
import mrl.motion.neural.param.TrajectoryEditParam;
import mrl.motion.neural.walk.WalkControl;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.motion.viewer.module.TimeBasedList;
import mrl.util.Configuration;
import mrl.util.MathUtil;
import mrl.util.Utils;
import mrl.widget.app.Item.ItemDescription;

public class BasketMotionGraph {

	public enum BasketType{
		Dribble, Walk, Shoot, Shoot_Near, Pass, Pass_, Pickup, Catch; 
		
		public String actionName(){
			if (this == Pass_) return "pass'";
			return name().toLowerCase();
		}
		
		public boolean isShoot(){
			return this == Shoot || this == Shoot_Near;
		}
		
		public boolean isAction(){
			return !(this == Dribble || this == Walk);
		}
	}
	public enum BFrameType{
		Before, Interaction, After, None
	}
	
	public static int INTERACTION_MARGIN = 20;
	public static boolean includePickup = false;
	public static boolean includeCatch = true;
	private static int BALL_MIN_MARGIN = 6;
	
	public MDatabase database;
	public Motion[] mList;
	private BasketType[] bTypeList;
	private BFrameType[] frameTypes;
	
	private int[] prevBallMargin;
	private int[] postBallMargin;
	private int[] ballLeftOffset;
	
	public BasketMotionGraph(MDatabase database) {
		this.database = database;
		mList = database.getMotionList();
		
		bTypeList = new BasketType[mList.length];
		frameTypes = new BFrameType[bTypeList.length];
		for (MotionAnnotation ann : database.getTransitionAnnotations()){
			BasketType type = null;
			for (BasketType t : BasketType.values()){
				if (t.toString().toLowerCase().equals(ann.type.toLowerCase())){
					type = t;
					break;
				}
			}
			if (type == null) throw new RuntimeException(ann.toString());
			int start = database.findMotion(ann.file, ann.startFrame).motionIndex;
			int interFrame = start + ann.interactionFrame - ann.startFrame;
			int end = start + ann.endFrame - ann.startFrame;
			for (int i = start; i <= end; i++) {
				bTypeList[i] = type;
				if (ann.interactionFrame < 0){
					frameTypes[i] = BFrameType.None;
				} else if (i < interFrame - INTERACTION_MARGIN){
					frameTypes[i] = BFrameType.Before;
				} else if (i > interFrame + INTERACTION_MARGIN){
					frameTypes[i] = BFrameType.After;
				} else {
					frameTypes[i] = BFrameType.Interaction;
				}
			}
		}
		
		prevBallMargin = new int[mList.length];
		postBallMargin = new int[mList.length];
		ballLeftOffset = new int[mList.length];
		for (int i = 0; i < prevBallMargin.length; i++) {
			Contact contact = mList[i].ballContact;
			int margin = 0;
			for (int m = 1; m <= BALL_MIN_MARGIN; m++) {
				int idx = i - m;
				if (idx < 0) break;
				if (contact.equals(mList[idx].ballContact)){
					margin++;
				}
			}
			prevBallMargin[i] = margin;
			
			margin = 0;
			for (int m = 1; m <= BALL_MIN_MARGIN; m++) {
				int idx = i + m;
				if (idx >= mList.length) break;
				if (contact.equals(mList[idx].ballContact)){
					margin++;
				}
			}
			postBallMargin[i] = margin;
			
			ballLeftOffset[i] = ballLeftOffset(i, 10);
		}
		
		try{
			MGraphGenerator mg = new MGraphGenerator(database){
				protected boolean isValid(MGGNode source, MGGNode target){
					boolean valid = isValidEdge(source.motion.motionIndex, target.motion.motionIndex);
					if (!valid) return false;
					return super.isValid(source, target);
				}
				
				protected double getEdgeWeightLimit(MGGNode source, MGGNode target){
					return edgeLimit(source.motion.motionIndex, target.motion.motionIndex);
				}
				
				protected boolean isValidMotion(Motion m){
					BasketType type = bTypeList[m.motionIndex];
					if (!includePickup && type == Pickup) return false;
					if (!includeCatch && type == Catch) return false;
					return super.isValidMotion(m);
				}
			};
			mg.saveResult();
		} catch (RuntimeException e){
			e.printStackTrace();
			System.out.flush();
			System.err.flush();
			System.exit(0);
		}
	}

	private int ballLeftOffset(int index, int margin){
		if (mList[index].ballContact.isNoContact()) return margin;
		for (int i = 1; i < margin; i++) {
			int idx = index + i;
			if (idx < 0 || idx >= mList.length) continue;
			if (mList[idx].ballContact.isNoContact()) return i;
		}
		return margin;
	}
	
	private boolean isValidEdge(int source, int target){
		Motion m1 = mList[source+1];
		Motion m2 = mList[target];
		if (!m1.ballContact.equals(m2.ballContact)) return false;
		if (ballLeftOffset[source] <= 4 && ballLeftOffset[target] >= 8) return false;
		if (ballLeftOffset[target] <= 4 && ballLeftOffset[source] >= 8) return false;
		if (prevBallMargin[source+1] + postBallMargin[target] < BALL_MIN_MARGIN) return false;
		if (m1.isLeftFootContact != m2.isLeftFootContact) return false;
		if (m1.isRightFootContact != m2.isRightFootContact) return false;
		
		BasketType sType = bTypeList[source];
		BasketType tType = bTypeList[target];
		BFrameType sFrame = frameTypes[source];
		BFrameType tFrame = frameTypes[target];
		if (sFrame == Interaction || tFrame == Interaction) return false;
		if (tFrame == After){
			if (sType != tType) return false;
			if (sFrame != After) return false;
			return true;
		}
		if (sFrame == Before){
			if (sType != tType){
				if (tType == Dribble && isMatch(sType,  Shoot, Shoot_Near, Pass)){
				} else if (tType == Walk && isMatch(sType, Pass_, Pickup, Catch)){
				} else {
					return false;
				}
			}
			if (tFrame != Before) return false;
			return true;
		}

//		Dribble, Walk, Shoot, Shoot_Near, Pass, Pass_, Pickup, Catch; 
		if (sType == Dribble){
			return isMatch(tType, Dribble, Shoot, Shoot_Near, Pass);
		} else if (sType == Walk){
			return isMatch(tType, Walk, Pass_, Pickup, Catch);
		} else if (sType == Shoot || sType == Shoot_Near || sType == Pass){
			return isMatch(tType, Walk, Pass_, Pickup, Catch);
		} else if (sType == Pass_ || sType == Pickup || sType == Catch){
			return isMatch(tType, Dribble, Shoot, Shoot_Near, Pass);
		}
		throw new RuntimeException();
	}
	
	private boolean isMatch(BasketType pivot, BasketType... types){
		for (BasketType t : types){
			if (pivot == t) return true;
		}
		return false;
	}
	
	private double edgeLimit(int source, int target){
		BasketType sType = bTypeList[source];
		BasketType tType = bTypeList[target];
		if (sType == null) System.out.println(database.getMotionList()[source] + " s: " + database.getTypeList()[source]);
		if (tType == null) System.out.println(database.getMotionList()[target] + " s: " + database.getTypeList()[target]);
		if (tType.isAction() || sType.isAction()){
			return Configuration.MGRAPH_EDGE_WEIGHT_LIMIT*4;
		}
		return Configuration.MGRAPH_EDGE_WEIGHT_LIMIT;
	}


	static boolean showMotion = true;
//	static boolean makeGraph = true;
//	static boolean showMotion = false;
	static boolean makeGraph = false;
	public static void main(String[] args) {
		Configuration.setDataFolder("basketData");
//		Configuration.ANNOTATION_FOLDER = "dribbleAnnotation";
//		Configuration.ANNOTATION_FOLDER = "basketData\\empty_ann";
		Configuration.TRANSITION_FOLDER = "dribbleAnnotation";
		
		Configuration.MGRAPH_EDGE_WEIGHT_LIMIT = 40;
		MDatabase database = MDatabase.load();
		BallTrajectoryGenerator.updateBallContacts(database);
		if (makeGraph){
			includeCatch = false;
			// generate new motion graph and save to files.
			new BasketMotionGraph(database);
		}
		
		// load motion graph from saved files.
		MGraph graph = new MGraph(database);
		MGraphExplorer exp = new MGraphExplorer(graph);
		ArrayList<Integer> interactionFrames = new ArrayList<Integer>();
		for (MotionAnnotation ann : database.getTransitionAnnotations()){
			if (ann.interactionFrame < 0) continue;
			interactionFrames.add(database.findMotion(ann.file, ann.interactionFrame).motionIndex);
		}
		exp.setInteractionFrames(interactionFrames, 60);
		MathUtil.random = new Random();
		ArrayList<int[]> segments = exp.explore(showMotion ? 3000 : 300000);
		
		MotionSegment segment = MotionSegment.getPathMotion(database.getMotionList(), Utils.toArray(segments));
		TrajectoryEditParam.CUT_MIN = 90;
		TrajectoryEditParam.CUT_MAX = 180;
		TrajectoryEditParam tEdit = new TrajectoryEditParam(0.3, 45, 0.1);
		segment = tEdit.edit(segment);
		FootSlipCleanup.clean(segment);
		ArrayList<Motion> mList = MotionData.divideByKnot(segment.getEntireMotion());
		segment = new MotionSegment(Utils.toArray(mList), MotionSegment.BLEND_MARGIN(), mList.size() - MotionSegment.BLEND_MARGIN() - 1, true);
		
		// show generated motion
		if (showMotion){
			MotionData mData = new MotionData(segment.getMotionList());
			TimeBasedList<Point3d> ballList = new TimeBasedList<Point3d>(new BallTrajectoryGenerator().generate(mData.motionList));
			
			MainViewerModule.runWithDescription(new Object[]{ mData, ballList} , new ItemDescription[]{ null, BallTrajectoryGenerator.ballDescription() });
		} else {
//			RNNDataGenerator.generate(segment, new PredictiveParameterControl());
//			RNNDataGenerator.generate(segment, new IntervalMeanControl());
//			RNNDataGenerator.generate(segment, new SelectiveDirectionControl());
			RNNDataGenerator.generate(segment, new WalkControl());
		}
	}
}
