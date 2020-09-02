package mrl.motion.neural.loco;

import java.io.File;

import javax.vecmath.Point3d;

import mrl.motion.data.Motion;
import mrl.motion.data.MotionData;
import mrl.motion.data.parser.BVHParser;
import mrl.motion.data.trasf.MotionTransform;
import mrl.motion.data.trasf.Pose2d;
import mrl.motion.neural.play.OgreRecorder.RecordData;
import mrl.motion.viewer.ogre.OgreJNI;
import mrl.motion.viewer.ogre.OgreJNI.OgreStatus;
import mrl.util.Pair;
import mrl.util.Utils;

public class MultiCharacterOgreViewer {
	
	static int fileIndex = 10;

	private static void updateSalsaFile(){
		String folder = "salsa\\motion\\";
		String file = "60_%02d_%d.bvh";
		MotionData[] mDataList = new MotionData[2];
		System.out.println("load file :: " + fileIndex);
		for (int i = 0; i < mDataList.length; i++) {
			mDataList[i] = new BVHParser().parse(new File(folder + String.format(file, fileIndex, i+1)));
		}
		OgreJNI.setMotion(mDataList);
	}
	
	public static void main(String[] args) {
		
		OgreJNI.setConfig(OgreJNI.CONFIG_IS_WOMAN, 1);
//		OgreJNI.open(OgreJNI.courtParam());
		OgreJNI.open(new double[]{ 0 });
		
		
		try{
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.out.println("Init Finished.");
			int prevKey = -1;
			while (true){
				if (OgreJNI.isClosed()) break;
				
				OgreStatus status = OgreJNI.getStatus();
				
				int key = status.key;
				if (key == prevKey){
					key = -1;
				} else {
					prevKey = key;
					System.out.println("key : " + key);
				}
				
				if (key == (int)'e'){
					fileIndex = Math.max(1, fileIndex - 1);
					updateSalsaFile();
				} else if (key == (int)'r'){
					fileIndex = Math.min(15, fileIndex + 1);
					updateSalsaFile();
				} else if (key == (int)'q'){
					OgreJNI.close();
					return;
				} else if (key == (int)'c'){
					break;
				}
			}
		} catch (RuntimeException e){
			e.printStackTrace();
		}
		OgreJNI.close();
	}
}
