package mrl.motion.neural.basket;

import java.io.File;
import java.util.ArrayList;

import mrl.motion.annotation.MotionAnnotationHelper;
import mrl.motion.data.MDatabase;
import mrl.motion.data.Motion;
import mrl.motion.data.MotionAnnotation;
import mrl.util.Configuration;
import mrl.util.ObjectSerializer;

public class BasketAnnotationRun {
	
	static void updatePassOffset(String annFolder){
		MDatabase database = BasketDataGenerator.loadBasketData();
		Motion[] mList = database.getMotionList();
		File folder = new File(annFolder);
		for (File file : folder.listFiles()){
			if (file.isDirectory() || !file.getName().endsWith(".lab")) continue;
			ArrayList<MotionAnnotation> list = ObjectSerializer.load(MotionAnnotation.class, file);
			if (list.size() == 0) continue;
			for (MotionAnnotation ann : list){
				if (ann.startFrame == 0) throw new RuntimeException();
				if (!ann.type.startsWith("pass")) continue;
				int interMotion = database.findMotion(ann.file, ann.interactionFrame).motionIndex;
				System.out.println(mList[interMotion].ballContact.isNoContact());
//					if (ann.type.equals("pass'")){
//						int offset = 0;
//						for (offset = -12; offset < 20; offset++) {
//							if (!mList[interMotion + offset].ballContact.isNoContact()){
//								break;
//							}
//						}
//						System.out.println(ann + "\t\t" + offset);
//						ann.interactionFrame += offset;
//					} else {
//						int offset = 0;
//						for (offset = -12; offset < 20; offset++) {
//							if (mList[interMotion + offset].ballContact.isNoContact()){
//								break;
//							}
//						}
//						offset--;
//						System.out.println(ann + "\t\t" + offset);
//						ann.interactionFrame += offset;
//					}
				if (ann.interactionFrame <= ann.startFrame || ann.interactionFrame >= ann.endFrame){
					System.out.println("errorr!@$!@$");
					throw new RuntimeException();
				}
			}
//				ObjectSerializer.save(MotionAnnotation.class, list, file);
		}
		System.exit(0);
	}

	public static void main(String[] args) {
//		Configuration.setDataFolder("walkMotion");
//		Configuration.setDataFolder("walkData");
		Configuration.setDataFolder("basketData");
		String motionFolder = Configuration.MOTION_FOLDER;
		String annFolder;
		annFolder = Configuration.ANNOTATION_FOLDER;
//		annFolder = "dribbleAnnotation";
//		annFolder = "mDatabaseTypeAnns";
		
		
//		annFolder = Configuration.TRANSITION_FOLDER;
//		annFolder = Configuration.TYPE_ANNOTATION_FOLDER;
		
//		MotionAnnotation.defaultType = "pickup";
//		MotionAnnotation.defaultType = "b_right";
		MotionAnnotationHelper helper = new MotionAnnotationHelper(motionFolder, annFolder);
//		BasketAnnotationHelper helper = new BasketAnnotationHelper(motionFolder, annFolder, BallTrajectoryGenerator.BALL_ANN_FOLDER);
//		BasketAnnotationHelper helper = new BasketAnnotationHelper(motionFolder, BallTrajectoryGenerator.BALL_ANN_FOLDER, annFolder);
		helper.open();
	}
}
