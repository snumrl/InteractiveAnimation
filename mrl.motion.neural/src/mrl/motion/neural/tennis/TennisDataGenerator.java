package mrl.motion.neural.tennis;

import mrl.motion.data.FootContactDetection;
import mrl.motion.data.MDatabase;
import mrl.motion.neural.data.MotionDataConverter;
import mrl.util.Configuration;

public class TennisDataGenerator {
	
	public static void setTennisJointSet(){
		FootContactDetection.CONTACT_MARGIN = 2;
		FootContactDetection.rightFootJoints = new String[]{ "RightFoot", "RightToes", "RightToes_End" } ;
		FootContactDetection.leftFootJoints = new String[]{ "LeftFoot", "LeftToes", "LeftToes_End" } ;
		MotionDataConverter.KeyJointList = new String[]{
				"HEad_End",
				"LeftHand",
//				"LeftHand_End",
				"LeftFoot",
				"LeftToes",
				"RightHand",
//				"RightHand_End",
				"RightFoot",
				"RightToes",
				
				"LeftArm",
				"RightArm",
				
				"LeftForeArm",
				"LeftLeg",
				"RightForeArm",
				"RightLeg",
			};
		MotionDataConverter.setNoBall();
	}

	public static MDatabase loadTennisData(String folder){
		Configuration.MOTION_FOLDER = folder;
		setTennisJointSet();
		MDatabase database = MDatabase.load();
		return database;
	}
}
