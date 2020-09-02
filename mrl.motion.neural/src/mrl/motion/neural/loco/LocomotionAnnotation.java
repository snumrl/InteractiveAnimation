package mrl.motion.neural.loco;

import mrl.motion.annotation.MotionAnnotationHelper;
import mrl.util.Configuration;

public class LocomotionAnnotation {

	public static void main(String[] args) {
//		Configuration.setDataFolder("salsa");
		Configuration.setDataFolder("locomotion\\edin_loco");
//		Configuration.MOTION_FOLDER = "salsa\\_motion";
		String motionFolder = Configuration.MOTION_FOLDER;
		String annFolder;
		annFolder = Configuration.TRANSITION_FOLDER;
		MotionAnnotationHelper.open(motionFolder, annFolder);
	}
}
