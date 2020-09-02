package mrl.motion.neural.tennis;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.parser.BVHWriter;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.neural.basket.BallTrajectoryGenerator;
import mrl.motion.neural.basket.BasketDataGenerator;
import mrl.motion.neural.data.PointIKSolver;
import mrl.motion.neural.tennis.TennisRacketData.TennisRacket;
import mrl.motion.position.PositionMotion;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.Utils;

public class TennisOgreViewer {

	public static void main(String[] args) {
		
//		String folder = "basketData\\motion\\";
//		String file = "s_004_3_2.bvh";
//		MotionData motionData = new BVHParser().parse(new File(folder + file));
		
//		MotionData motionData = new BVHParser().parse(new File("tennisData\\backhand_slice_kaiwa.bvh"));
//		BVHWriter bw = new BVHWriter(new File("basketData\\motion\\s_003_1_1.bvh"));
//		bw.write(new File("tennisData\\test.bvh"), motionData);
//		System.exit(0);
		
//		MotionData motionData = new BVHParser().parse(new File("basketData\\motion\\s_003_1_1.bvh"));
//		MotionData motionData = new BVHParser().parse(new File("tennisData\\test.bvh"));
//		motionData.motionList = Utils.cut(motionData.motionList, 300, 310);
		
		MDatabase database = BasketDataGenerator.loadBasketData();
//		MotionData motionData = database.getMotionDataList()[database.findMotionDataIndex("s_010_4_2.bvh")];
		MotionData motionData = database.getMotionDataList()[database.findMotionDataIndex("s_004_3_2.bvh")];
//		MotionData motionData = database.getMotionDataList()[1];
		
//		BasketDataGenerator.loadBasketData();
//		MotionData motionData = new BVHParser().parse(new File("tennisData\\merged_1.bvh"));
//		
//		MotionTransform t = new MotionTransform();
//		PointIKSolver solver = new PointIKSolver(t.skeletonData, t.sampleMotion);
//		ArrayList<Motion> solved =new ArrayList<Motion>();
//		for (Motion motion : motionData.motionList){
//			PositionMotion pm = new PositionMotion(motion);
//			solved.add(solver.solve(pm.pointData, pm.pose));
//		}
//		motionData.motionList = solved;
		
		BallTrajectoryGenerator g = new BallTrajectoryGenerator();
		ArrayList<Point3d> ballList = g.generate(motionData.motionList);
		
		ArrayList<Matrix4d> racketList = new ArrayList<Matrix4d>();
		for (Motion motion : motionData.motionList){
			Matrix4d hand = Motion.getTransformData(motionData.skeletonData, motion).get("RightHand");
			Matrix4d rot = new Matrix4d();
			rot.rotX(Math.PI);
			rot.setTranslation(new Vector3d(0, 0, 100));
			hand.mul(hand, rot);
			racketList.add(hand);
		}
		
		OgreJNI.setConfig(OgreJNI.CONFIG_IS_TENNIS, 1);
//		OgreJNI.setConfig(OgreJNI.CONFIG_USE_ARROW, 5);
		
		OgreJNI.open(new double[]{ 0,1,1 });
//		OgreJNI.setRacket(Utils.singleList(racketList));
		OgreJNI.setBall(ballList);
//		OgreJNI.setBall2(ballList, racketList);
		OgreJNI.setMotion(new MotionData[] { motionData });
		OgreJNI.waitForOgreClose();
	}
}
