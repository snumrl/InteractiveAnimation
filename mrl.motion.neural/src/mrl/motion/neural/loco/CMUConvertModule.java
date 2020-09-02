package mrl.motion.neural.loco;

import java.io.File;
import java.util.HashMap;

import javax.vecmath.Point3d;

import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.SkeletonData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.motion.neural.data.PointIKSolver_CMU;
import mrl.motion.position.PositionMotion;
import mrl.motion.position.PositionResultMotion;
import mrl.motion.position.PositionResultMotion.PositionFrame;
import mrl.motion.viewer.module.MainViewerModule;
import mrl.util.Configuration;
import mrl.util.FileUtil;
import mrl.util.Utils;
import mrl.widget.app.ItemListModule;
import mrl.widget.app.MainApplication;
import mrl.widget.app.Module;

public class CMUConvertModule extends Module{

	@Override
	protected void initializeImpl() {
		getModule(MainViewerModule.class);
		
		MotionDataConverter.setCMUJointSet();
		MotionData mData = new BVHParser().parse(new File("locomotion\\edin_loco\\motion\\locomotion_jog_sidestep_000_005.bvh"));
//		MotionData mData = new BVHParser().parse(new File("locomotion\\edin_loco\\motion\\locomotion_jog_000_000.bvh"));
//		HashMap<String, Point3d> map = Motion.getPointData(mData.skeletonData, mData.motionList.get(0));
		PositionMotion pppm = new PositionMotion(mData.motionList.get(0));
		HashMap<String, Point3d> map = pppm.pointData;
		PositionFrame frame = MotionDataConverter.getJointPositions(Motion.getPointData(mData.skeletonData, mData.motionList.get(0)));
		PositionResultMotion pm = new PositionResultMotion();
		pm.add(frame);
		getModule(ItemListModule.class).addSingleItem("Motion", pm);
		SkeletonData.instance = null;
		
		Configuration.setDataFolder("locomotion\\edin_loco");
		MDatabase database = MDatabase.load();
		Motion motion = database.getMotionList()[0];
		motion = PositionMotion.getAlignedMotion(motion);
		PointIKSolver_CMU s = new PointIKSolver_CMU(SkeletonData.instance, motion);
		Motion m = s.solve(map, pppm.pose);
		System.out.println("ppp : " + pppm.pose);
		System.out.println(frame.get(0)[0]);
		getModule(ItemListModule.class).addSingleItem("Solved", new MotionData(Utils.singleList(m)));
	}
	
	private HashMap<String, Point3d> frameToMap(PositionFrame frame){
		HashMap<String, Point3d> map = new HashMap<String, Point3d>();
		int fIdx = 0;
		for (String[] pair : MotionDataConverter.jointPairs){
			for (int i = 0; i < pair.length-1; i++) {
				Point3d[] pointPair = frame.get(fIdx);
				fIdx++;
				map.put(pair[i], pointPair[0]);
				map.put(pair[i+1], pointPair[1]);
			}
		}
		return map;
	}
	
	
	public static void main(String[] args) {
		MotionDataConverter.setCMUJointSet();
//		Configuration.setDataFolder("salsa");
//		Configuration.setDataFolder("locomotion\\edin_loco");
//		MDatabase database = MDatabase.load();
		MotionData mData = new BVHParser().parse(new File("cmu.bvh"));
		Motion motion = mData.motionList.get(0);
		System.out.println(SkeletonData.keyList.length);
		for (String joint : SkeletonData.keyList){
			if (motion.get(joint) == null) continue;
//			if (SkeletonData.instance.get(joint).transition.length() < 0.00001) continue;
			System.out.println(joint);
		}
		System.exit(0);
		
		MainApplication.run(new CMUConvertModule());
	}

}
