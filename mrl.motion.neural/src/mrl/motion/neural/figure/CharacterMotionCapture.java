package mrl.motion.neural.figure;

import java.io.File;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.MathUtil;
import mrl.util.Utils;

public class CharacterMotionCapture {

	
	static void grammar_expansion_png(){
		MDatabase database = BasketDataGenerator.loadBasketData();
		Motion[] mList = database.getMotionList();
		int mIndex = database.findMotion("s_007_4_1.bvh", 626).motionIndex;
//		int mIndex = database.findMotion("s_003_1_1.bvh", 409).motionIndex;
//		int mIndex = database.findMotion("s_007_1_1.bvh", 181).motionIndex;
//		int mIndex = database.findMotion("s_004_6_2.bvh", 330).motionIndex;
//		int mIndex = database.findMotion("s_004_1_2.bvh", 363).motionIndex;
//		int mIndex = database.findMotion("s_006_7_1.bvh", 1031).motionIndex;
		OgreJNI.open(new double[]{ -1 });
//		int[] frames = {1, 6, 18};
//		int[] frames = {1, 8, 16};
//		int[] frames = {1, 7, 13};
		int[] frames = {0};
		double posOffset = 70;
//		double posOffset = 80;
		MotionData[] mDataList = new MotionData[frames.length];
		
		Vector3d v = Pose2d.to3d(PositionMotion.getPose(mList[mIndex]).direction);
		v.normalize();
		
		BallTrajectoryGenerator g = new BallTrajectoryGenerator();
		for (int i = 0; i < mDataList.length; i++) {
			Motion m = new Motion(mList[mIndex + frames[i]]);
			Vector3d mv = new Vector3d(v);
			mv.scale(i*posOffset);
			mv.add(MathUtil.getTranslation(m.root()));
			m.root().setTranslation(mv);
			mDataList[i] = new MotionData(Utils.singleList(m));
		}
		
		ArrayList<Point3d> ballTrajectory = g.generate(mList[mIndex].motionData.motionList);
		OgreJNI.setBall(Utils.singleList(ballTrajectory.get(mList[mIndex].frameIndex)));
		OgreJNI.setMotion(mDataList);
	}
	
	static void quality1(){
		OgreJNI.open(new double[]{ 0, 1 });
		
		MotionData mData = new BVHParser().parse(new File("output\\dribble_quality\\motion1.bvh"));
		ArrayList<MotionData> mDataList = new ArrayList<MotionData>();
		int interval = 16;
		for (int i = 0; i < mData.motionList.size(); i+=interval) {
			int idx = i/interval;
//			if (idx == 4 || idx == 5 || idx == 7) continue;
			Motion motion = mData.motionList.get(i);
			mDataList.add(new MotionData(Utils.singleList(motion)));
		}
		mDataList.add(new MotionData(Utils.singleList(Utils.last(mData.motionList))));
		OgreJNI.setMotion(Utils.toArray(mDataList));
		OgreJNI.waitForOgreClose();
	}
	
	public static void main(String[] args) {
		
		quality1();
	}
}
