package mrl.motion.neural.walk;

import mrl.motion.annotation.MotionAnnotationHelper;
import mrl.util.Configuration;

public class WalkAnnotation {

	public static void main(String[] args) {
//		Configuration.setDataFolder("walk16");
		Configuration.setDataFolder("walkTest");
//		Configuration.setDataFolder("salsa");
		String motionFolder = Configuration.MOTION_FOLDER;
		String annFolder;
//		annFolder = Configuration.TRANSITION_FOLDER;
		annFolder = Configuration.ANNOTATION_FOLDER;
		MotionAnnotationHelper.open(motionFolder, annFolder);
	}
}
