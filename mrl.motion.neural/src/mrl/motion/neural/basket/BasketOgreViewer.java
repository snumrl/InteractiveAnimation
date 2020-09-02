package mrl.motion.neural.basket;

import java.io.File;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.util.Utils;

public class BasketOgreViewer {

	public static void main(String[] args) {
//		String file = "basketData\\motion\\s_007_1_1.bvh";
		String file = "locomotion\\edin_loco\\motion\\locomotion_jog_sidestep_000_005.bvh";
		
		MotionData motionData = new BVHParser().parse(new File(file));
//		String[] keyList = motionData.skeletonData.keyList;
//		for (int i = 0; i < keyList.length; i++) {
//			System.out.println(i + "\t" + keyList[i]);
//		}
//		Motion motion = motionData.motionList.get(0);
//		for (String key : OgreJNI.JointList){
//			System.out.println(key + " : " + (motion.get(key) != null));
//		}
//		System.exit(0);
		
//		OgreJNI.setConfig(OgreJNI.CONFIG_USE_ARROW, 10);
//		OgreJNI.open(OgreJNI.courtParam());
		OgreJNI.setUseCMUMotion();
		OgreJNI.open(new double[]{ 0 });
		OgreJNI.setMotion(new MotionData[] { motionData });
		OgreJNI.waitForOgreClose();
	}
}
